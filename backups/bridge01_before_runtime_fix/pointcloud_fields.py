#!/usr/bin/env python3
"""只适配 PointCloud2 的字段和时间表达，不做定位、滤波或导航决策。"""
import copy
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

# Isaac 原生时间为 uint64 纳秒，ROS PointField 不支持 uint64，因此拆成两个 uint32。
# 使用显式偏移的结构化数组读取，可兼容字段排列、行末填充和大小端。
_TYPES = {PointField.INT8: 'i1', PointField.UINT8: 'u1',
          PointField.INT16: 'i2', PointField.UINT16: 'u2',
          PointField.INT32: 'i4', PointField.UINT32: 'u4',
          PointField.FLOAT32: 'f4', PointField.FLOAT64: 'f8'}
_REQUIRED = ('x', 'y', 'z', 'intensity', 'channel_id', 'timestamp_0', 'timestamp_1')
_OUTPUT = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'ring', 'timestamp'],
                    'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f8'],
                    'offsets': [0, 4, 8, 12, 16, 18], 'itemsize': 26})


def convert_cloud(message, expected_frame='lidar_link', scan_lines=32):
    """原样保留点坐标；按逐点时间排列，并用首点时间作扫描帧头。

    FAST-LIO 将帧头当作扫描起点，将每个 timestamp 与第一个 timestamp 的差
    当作去畸变的相对时间。因此不能保留扫描结束时的帧头，也不能把逐点时间填零。
    空帧返回 None；格式错误抛 ValueError，由节点报告，不能伪造一帧正常数据。
    """
    if message.header.frame_id != expected_frame:
        raise ValueError(f'点云实际 frame_id={message.header.frame_id!r}，期望 {expected_frame!r}')
    if not message.width or not message.height:
        return None
    fields = {field.name: field for field in message.fields}
    endian = '>' if message.is_bigendian else '<'
    formats, offsets = [], []
    for name in _REQUIRED:
        field = fields.get(name)
        if field is None or field.count != 1 or field.datatype not in _TYPES:
            raise ValueError(f'缺少或不支持的原生点云字段：{name}')
        if name in ('channel_id', 'timestamp_0', 'timestamp_1') and field.datatype != PointField.UINT32:
            raise ValueError(f'{name} 必须保留原生 UINT32 类型')
        dtype = np.dtype(endian + _TYPES[field.datatype])
        if field.offset < 0 or field.offset + dtype.itemsize > message.point_step:
            raise ValueError(f'字段 {name} 超出 point_step')
        formats.append(dtype)
        offsets.append(field.offset)
    if (message.row_step < message.width * message.point_step or
            len(message.data) < message.height * message.row_step):
        raise ValueError('PointCloud2 数据长度或行跨度错误')
    dtype = np.dtype({'names': _REQUIRED, 'formats': formats,
                      'offsets': offsets, 'itemsize': message.point_step})
    points = np.ndarray((message.height, message.width), dtype=dtype,
                        buffer=message.data, strides=(message.row_step, message.point_step)).reshape(-1)
    times_ns = ((points['timestamp_1'].astype(np.uint64) << np.uint64(32)) |
                points['timestamp_0'].astype(np.uint64))
    rings = points['channel_id']
    if np.any(rings >= scan_lines):
        raise ValueError('channel_id 超出配置线数；请核对雷达配置，不能截断或伪造 ring')
    # 这里只重排消息存储，不删点、不插值、不修改采样时间。
    order = np.argsort(times_ns, kind='stable')
    packed = np.empty(points.size, dtype=_OUTPUT)
    for name in ('x', 'y', 'z', 'intensity'):
        packed[name] = points[name][order]
    packed['ring'] = rings[order]
    packed['timestamp'] = times_ns[order].astype(np.float64) * 1e-9
    result = PointCloud2()
    result.header = copy.deepcopy(message.header)
    first_ns = int(times_ns[order[0]])
    result.header.stamp.sec, result.header.stamp.nanosec = divmod(first_ns, 1_000_000_000)
    result.height, result.width = 1, points.size
    result.fields = [PointField(name=name, offset=int(_OUTPUT.fields[name][1]),
                               datatype=(PointField.UINT16 if name == 'ring' else
                                         PointField.FLOAT64 if name == 'timestamp' else PointField.FLOAT32),
                               count=1) for name in _OUTPUT.names]
    result.is_bigendian = False
    result.point_step, result.row_step = _OUTPUT.itemsize, _OUTPUT.itemsize * points.size
    result.is_dense = message.is_dense
    result.data = packed.tobytes()
    return result


class PointCloudFields(Node):
    def __init__(self):
        super().__init__('forest_pointcloud_fields')
        self.declare_parameter('input_topic', '/isaac/lidar/points_raw')
        self.declare_parameter('output_topic', '/lidar/points')
        self.declare_parameter('frame_id', 'lidar_link')
        self.declare_parameter('scan_lines', 32)
        self.frame = self.get_parameter('frame_id').value
        self.lines = self.get_parameter('scan_lines').value
        self.publisher = self.create_publisher(PointCloud2,
            self.get_parameter('output_topic').value, qos_profile_sensor_data)
        self.subscription = self.create_subscription(PointCloud2,
            self.get_parameter('input_topic').value, self.on_cloud, qos_profile_sensor_data)

    def on_cloud(self, message):
        try:
            converted = convert_cloud(message, self.frame, self.lines)
        except (ValueError, TypeError, OverflowError) as error:
            # 数据错误只报告当前消息，节点继续等待后续消息，不关闭仿真或其他进程。
            self.get_logger().error(f'点云字段适配失败：{error}', throttle_duration_sec=5.0)
            return
        if converted is not None:
            self.publisher.publish(converted)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFields()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 仅清理由用户 Ctrl+C 或 ROS 上下文关闭触发的本节点资源。
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
