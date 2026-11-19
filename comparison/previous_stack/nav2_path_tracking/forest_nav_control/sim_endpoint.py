"""Isaac Python 3.12 与 ROS Python 3.10 之间的本机 UDP 接口，无 ROS 依赖。

只监听回环地址。指令按会话和序号接收，不附加速度限幅或失联停车。消息中的单调时钟仅用于同一台主机的进程间新鲜度校验。
"""
import json
import math
import os
import socket
import time
import uuid

STATE_PORT = int(os.environ.get('FOREST_NAV_STATE_PORT', '15361'))
COMMAND_PORT = int(os.environ.get('FOREST_NAV_COMMAND_PORT', '15362'))


def finite_vector(value, count):
    return isinstance(value, list) and len(value) == count and all(
        isinstance(x, (float, int)) and math.isfinite(x) for x in value)


MAX_DATAGRAM_BYTES = 8192


def path_packet(points, *, session, revision):
    """显示路线独立限长，均匀降采样保留首尾；不改变导航使用的原路径。"""
    sampled = list(points)
    while True:
        packet = dict(version=1, kind='path', session=session,
                      sent_ns=time.monotonic_ns(), path_revision=revision, path=sampled)
        encoded = json.dumps(packet, allow_nan=False).encode()
        if len(encoded) <= MAX_DATAGRAM_BYTES:
            return encoded
        if len(sampled) <= 2:
            raise ValueError('路线元数据超过UDP包上限')
        reduced = sampled[::2]
        if (len(sampled)-1) % 2:
            reduced.append(sampled[-1])
        sampled = reduced


class SimEndpoint:
    def __init__(self, state_port=STATE_PORT, command_port=COMMAND_PORT):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('127.0.0.1', command_port))
        self.socket.setblocking(False)
        self.destination = ('127.0.0.1', state_port)
        self.session = uuid.uuid4().hex
        self.state_sequence = 0
        self.command_sequence = -1
        self.command_received = None
        self.command = (0., 0., 0.)
        self.path = ()
        self.path_revision = -1
        self.path_received = None
        self.ready = False

    def publish_state(self, sim_time, pose, velocity, *, ready=True, fault='', locomotion=None):
        """pose=[地图x,y,yaw]；velocity=[机体水平vx,vy,wz]，均为实际反馈。"""
        if not finite_vector(list(pose), 3) or not finite_vector(list(velocity), 3):
            raise ValueError('仿真状态含有非有限值')
        self.ready = bool(ready and not fault)
        self.state_sequence += 1
        packet = {'version': 1, 'session': self.session, 'sequence': self.state_sequence,
                  'sent_ns': time.monotonic_ns(), 'sim_time': float(sim_time),
                  'pose': list(pose), 'velocity': list(velocity), 'ready': self.ready, 'fault': fault}
        if locomotion is not None:packet['locomotion']=locomotion
        self.socket.sendto(json.dumps(packet, allow_nan=False).encode(), self.destination)

    def read_command(self):
        """只接收本会话中的递增有限指令，不因延迟或断流清零。"""
        for _ in range(100):
            try:
                data, _ = self.socket.recvfrom(MAX_DATAGRAM_BYTES)
            except BlockingIOError:
                break
            try:
                packet = json.loads(data)
                if not isinstance(packet,dict):
                    continue
                if packet.get('kind') == 'path':
                    if (packet.get('version') == 1 and packet.get('session') == self.session
                            ):
                        self._accept_path(packet)
                    # 显示包不能刷新速度看门狗或消费速度序号。
                    continue
                if (packet.get('version') != 1 or packet.get('session') != self.session or
                    not isinstance(packet.get('sequence'), int) or
                    packet['sequence'] <= self.command_sequence):
                    continue
                values = packet['command']
                if not finite_vector(values, 3):
                    continue
                if 'path' in packet or 'path_revision' in packet:
                    if not self._accept_path(packet):
                        continue
                self.command_sequence = packet['sequence']
                self.command_received = time.monotonic()
                self.command = tuple(float(v) for v in values)
            except (ValueError, TypeError, KeyError, OverflowError):
                continue
        return self.command

    def _accept_path(self, packet):
        """校验同一会话中的路线；空路线表示导航已结束。"""
        revision = packet.get('path_revision')
        points = packet.get('path')
        if not isinstance(revision, int) or revision < self.path_revision:
            return False
        if revision == self.path_revision:
            return True
        if (not isinstance(points, list) or len(points) > 4096):
            return False
        parsed = []
        for point in points:
            if (not isinstance(point, list) or len(point) != 2
                    or not all(isinstance(value, (int, float))
                               and math.isfinite(value) for value in point)):
                return False
            parsed.append((float(point[0]), float(point[1])))
        self.path = tuple(parsed)
        self.path_revision = revision
        self.path_received = time.monotonic()
        return True

    def read_path(self):
        """返回最新路线及修订号；没有收到路线时为空。"""
        return self.path, self.path_revision

    def close(self):
        self.ready = False
        self.socket.close()
