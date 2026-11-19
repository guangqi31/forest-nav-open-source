"""直接把机体系 [vx, vy, wz] 送入G1原有速度指令观测。

ROS订阅使用Isaac原生通用Subscriber，直接接收TwistStamped。
没有限速、死区、滤波、超时、仲裁、摔倒判断、自动重置或退出逻辑。
首次消息前缓存为零；后续保持最新消息，只有下一条消息能改变缓存。
"""

# 配置会被IsaacLab复制，因此观测配置只保存字符串key，不保存绑定了USD资源的对象。
# 注册表仅用于把配置里的key关联到本进程实例，不选择指令、不调度控制。
_ADAPTERS = {}


class G1VelocityAdapter:
    def __init__(self, topic="/cmd_vel", bridge_key="g1_velocity", graph_path="/G1VelocityInput"):
        self.topic = topic
        self.bridge_key = bridge_key
        self.graph_path = graph_path
        self.latest = (0.0, 0.0, 0.0)
        self._og = None
        _ADAPTERS[bridge_key] = self

    def configure_observations(self, env_cfg, group_names=("policy", "critic")):
        """在创建环境前调用，只替换原velocity_commands项的取值函数。

        history_length、flatten_history_dim、观测顺序及其他项均保持原配置。
        历史仍由IsaacLab原生ObservationManager生成，不把过去帧改写成当前值。
        """
        for name in group_names:
            term = getattr(env_cfg.observations, name).velocity_commands
            term.func = velocity_commands
            term.params = {"bridge_key": self.bridge_key, "command_name": "base_velocity"}

    def connect(self):
        """在SimulationApp和环境创建后调用，连接同一ROS域中的速度话题。

        不创建仿真主循环，不加载策略，不创建额外进程。应在首次策略推理前调用。
        传输配置使用桥接①在进程启动前加载的现有配置。
        """
        import omni.graph.core as og
        from isaacsim.core.experimental.utils.app import enable_extension
        enable_extension("isaacsim.ros2.bridge")
        self._og = og
        og.Controller.edit(
            {"graph_path": self.graph_path, "evaluator_name": "execution"},
            {og.Controller.Keys.CREATE_NODES: [
                ("Trigger", "omni.graph.action.OnImpulseEvent"),
                ("Subscriber", "isaacsim.ros2.bridge.ROS2Subscriber")],
             og.Controller.Keys.CONNECT: [("Trigger.outputs:execOut", "Subscriber.inputs:execIn")],
             og.Controller.Keys.SET_VALUES: [
                ("Subscriber.inputs:messagePackage", "geometry_msgs"),
                ("Subscriber.inputs:messageSubfolder", "msg"),
                ("Subscriber.inputs:messageName", "TwistStamped"),
                ("Subscriber.inputs:topicName", self.topic),
                # 原生接收队列只留最新一条，不排队执行过时的速度指令。
                ("Subscriber.inputs:queueSize", 1)]})
        self.read_latest()

    def read_latest(self):
        """非阻塞读取原生Subscriber的最新输出，原值传递，不做数值修改。

        消息header只表示上游来源和时间；本适配器不据此拒绝、归零或转换坐标。
        接口约定vehicle和策略基座坐标轴一致：x前、y左、z上。
        """
        if self._og is not None:
            og = self._og
            og.Controller.set(og.Controller.attribute(self.graph_path + "/Trigger.state:enableImpulse"), True)
            og.Controller.evaluate_sync(self.graph_path)
            self.latest = tuple(float(og.Controller.get(og.Controller.attribute(
                self.graph_path + "/Subscriber.outputs:" + field)))
                for field in ("twist:linear:x", "twist:linear:y", "twist:angular:z"))
        return self.latest


def velocity_commands(env, bridge_key="g1_velocity", command_name="base_velocity"):
    """提供原生速度观测项需要的(num_envs,3)张量。

    每次原生观测更新时先取最新外部速度，再覆盖原有command term的三列。
    因此策略不会读到原随机命令；不修改随机命令算法或其他配置。
    观测管理器随后按原配置追加历史；策略、关节动作和推理循环保持原样。
    """
    adapter = _ADAPTERS[bridge_key]
    values = adapter.read_latest()
    target = env.command_manager.get_term(command_name).vel_command_b
    target[:] = target.new_tensor(values)
    return target
