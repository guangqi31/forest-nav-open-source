"""Isaac 6.2 内的传感器装配接口；不创建机器人、场景或仿真主循环。

宿主在 SimulationApp 初始化且机器人刚体已经创建之后调用 SensorBridge。
必须保留实例引用。模块只读取物理传感器并驱动 NVIDIA 原生 ROS 发布节点，
不导入系统 rclpy，不混用 Isaac Python 3.12 与 ROS Humble Python 3.10。
"""
from pathlib import Path
import math
import yaml


class SensorBridge:
    def __init__(self, body_prim_path, config_path=None, graph_path='/ForestSensorBridge'):
        # Isaac 模块延迟导入，使普通 ROS 环境可以读取本文件而无需启动 Kit。
        import carb
        import omni.graph.core as og
        import omni.usd
        from pxr import Gf, UsdGeom, UsdPhysics
        from isaacsim.core.experimental.utils.app import enable_extension
        from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager

        config_path = Path(config_path) if config_path else Path(__file__).resolve().parents[1] / 'config/sensors.yaml'
        with config_path.open(encoding='utf-8') as stream:
            self.config = yaml.safe_load(stream)
        stage = omni.usd.get_context().get_stage()
        body = stage.GetPrimAtPath(body_prim_path)
        # 这里核对的是调用接口是否正确，不能静默创建一个脱离机器人的假安装刚体。
        if not body.IsValid() or not body.HasAPI(UsdPhysics.RigidBodyAPI):
            raise ValueError(f'body_prim_path 必须指向已存在的物理刚体：{body_prim_path}')
        if not math.isclose(UsdGeom.GetStageMetersPerUnit(stage), 1.0):
            raise ValueError('本接口使用米制场景，请在宿主中明确单位，不静默缩放机器人')
        if stage.GetPrimAtPath(graph_path).IsValid():
            raise ValueError(f'发布图已存在，请勿重复挂接：{graph_path}')
        enable_extension('isaacsim.ros2.bridge')
        enable_extension('isaacsim.sensors.experimental.rtx')
        enable_extension('isaacsim.sensors.experimental.physics')
        from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor
        from isaacsim.sensors.experimental.physics import IMU, IMUSensor

        lidar_cfg, imu_cfg = self.config['lidar'], self.config['imu']
        self.lidar = Lidar.create(
            path=body_prim_path + '/forest_lidar', config=lidar_cfg['config'],
            tick_rate=float(lidar_cfg['rate_hz']), accumulate_outputs=True,
            aux_output_level='FULL',
            attributes={'omni:sensor:Core:outputFrameOfReference': 'SENSOR',
                        'omni:sensor:Core:scanRateBaseHz': float(lidar_cfg['rate_hz'])})
        self.imu = IMUSensor(IMU.create(body_prim_path + '/forest_imu',
            linear_acceleration_filter_size=1, angular_velocity_filter_size=1,
            orientation_filter_size=1))

        def set_sensor_pose(prim, pose):
            # 某些雷达资产在外壳下嵌套真正的 OmniLidar。配置定义的是实际采样坐标系，
            # 因此按实际传感器 prim 的父级换算局部矩阵，避免把外壳偏移误当安装外参。
            # USD 使用行向量约定：传感器到世界 = 传感器到刚体 * 刚体到世界。
            q = pose['orientation_wxyz']
            norm = math.sqrt(sum(float(v) ** 2 for v in q))
            w, x, y, z = [float(v) / norm for v in q]
            desired = Gf.Matrix4d(1.0)
            desired.SetRotate(Gf.Quatd(w, Gf.Vec3d(x, y, z)))
            desired.SetTranslateOnly(Gf.Vec3d(*pose['position']))
            cache = UsdGeom.XformCache()
            world = desired * cache.GetLocalToWorldTransform(body)
            parent_world = cache.GetLocalToWorldTransform(prim.GetParent())
            xform = UsdGeom.Xformable(prim)
            xform.ClearXformOpOrder()
            xform.AddTransformOp(opSuffix='forest_mount').Set(world * parent_world.GetInverse())

        set_sensor_pose(self.lidar.prims[0], lidar_cfg)
        set_sensor_pose(stage.GetPrimAtPath(body_prim_path + '/forest_imu'), imu_cfg)
        self.lidar_sensor = LidarSensor(self.lidar, annotators=[])
        self.lidar_sensor.attach_writer('RtxLidarROS2PublishPointCloud',
            topicName=lidar_cfg['raw_topic'], frameId=self.config['frames']['lidar'],
            outputIntensity=True, outputTimestamp=True, outputChannelId=True)

        # 使用手动触发原生节点，使 IMU 按物理采样而非界面渲染频率发布。
        # 时钟与 IMU 分图，IMU 初始化尚无有效读数时，时钟仍然正常推进。
        self._og = og
        self._manager = SimulationManager
        self._imu_period = 1.0 / float(imu_cfg['rate_hz'])
        self._last_imu_time = None
        self._last_imu_tick = None
        self._graph_path = graph_path
        self._imu_graph = self._create_publish_graph(graph_path + '/Imu',
            'isaacsim.ros2.bridge.ROS2PublishImu',
            {'topicName': imu_cfg['topic'], 'frameId': self.config['frames']['imu'],
             'publishOrientation': False, 'publishLinearAcceleration': True,
             'publishAngularVelocity': True})
        self._clock_graph = None
        if self.config['publish_clock']:
            self._clock_graph = self._create_publish_graph(graph_path + '/Clock',
                'isaacsim.ros2.bridge.ROS2PublishClock', {'topicName': self.config['clock_topic']})
        # 不修改宿主的物理步长。低于目标采样率时明确报告，不能复制读数冒充 200Hz。
        dt = SimulationManager.get_physics_dt()
        if dt > self._imu_period + 1e-9:
            carb.log_warn(f'IMU 目标 {imu_cfg["rate_hz"]}Hz，当前物理步长 {dt}s；实际频率受物理步限制')
        # 使用独立函数满足原生物理回调的 (dt, context) 签名，并保留引用。
        self._callback = lambda dt, context: self._publish_step()
        self._callback_id = SimulationManager.register_callback(
            self._callback, event=SimulationEvent.PHYSICS_POST_STEP, order=100)

    def _create_publish_graph(self, path, publisher_type, parameters):
        og = self._og
        graph, _, _, _ = og.Controller.edit(
            {'graph_path': path, 'evaluator_name': 'execution'},
            {og.Controller.Keys.CREATE_NODES: [
                ('Trigger', 'omni.graph.action.OnImpulseEvent'), ('Publish', publisher_type)],
             og.Controller.Keys.CONNECT: [('Trigger.outputs:execOut', 'Publish.inputs:execIn')],
             og.Controller.Keys.SET_VALUES: [
                 (f'Publish.inputs:{key}', value) for key, value in parameters.items()]})
        return path

    def _publish(self, graph, values):
        og = self._og
        path = graph
        for key, value in values.items():
            og.Controller.set(og.Controller.attribute(f'{path}/Publish.inputs:{key}'), value)
        og.Controller.set(og.Controller.attribute(f'{path}/Trigger.state:enableImpulse'), True)
        og.Controller.evaluate_sync(graph)

    def _publish_step(self):
        if self._clock_graph is not None:
            self._publish(self._clock_graph, {'timeStamp': self._manager.get_simulation_time()})
        # read_gravity=True 返回加速度计比力：静止时模长约 g，自由落体时约 0。
        # 不把仿真真值姿态发布成已估计姿态，也不使用世界系速度差分伪造 IMU。
        reading = self.imu.get_sensor_reading(read_gravity=True)
        if not reading.is_valid:
            return
        stamp = float(reading.time)
        # 原生采样时间存在微秒级浮点量化误差。不能用“与上一帧之差必须达到
        # 精确的 0.005 秒”来判断，否则正常的 200Hz 读数会被误判为过早而丢弃。
        # 将采样时间映射到目标频率的整数时隙，同一时隙只发布一份真实读数。
        # 只改变发布判定，消息仍使用原始采样时间；不插值、不补帧、不改 IMU 数据。
        tick = round(stamp / self._imu_period)
        if self._last_imu_time is not None:
            if stamp >= self._last_imu_time and tick <= self._last_imu_tick:
                return
        # 宿主主动重置后时间可能回退，此时接受新时间轴，不自行重置仿真。
        self._last_imu_time = stamp
        self._last_imu_tick = tick
        self._publish(self._imu_graph, {'timeStamp': stamp,
            'linearAcceleration': [reading.linear_acceleration_x, reading.linear_acceleration_y,
                                   reading.linear_acceleration_z],
            'angularVelocity': [reading.angular_velocity_x, reading.angular_velocity_y,
                                reading.angular_velocity_z]})

    def detach(self):
        """宿主主动卸载接口时调用；只解绑传感器发布，不停止或关闭仿真。"""
        import omni.usd
        self._manager.deregister_callback(self._callback_id)
        self.lidar_sensor.detach_writer('RtxLidarROS2PublishPointCloud')
        omni.usd.get_context().get_stage().RemovePrim(self._graph_path)
