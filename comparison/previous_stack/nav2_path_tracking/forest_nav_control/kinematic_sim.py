"""用于 ROS 闭环验收的平面运动模型；不是 G1 动力学仿真或策略替代品。

使用与 Isaac 端同一个 SimEndpoint，验证跨进程传输、时钟、反馈和超时。
导航输出由真实 DWB 计算，模型只执行加速度受限的平面速度积分。
"""
import argparse
import math
import time
from .sim_endpoint import SimEndpoint


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start',type=float,nargs=3,default=[-8.,-2.,0.])
    args = parser.parse_args()
    endpoint = SimEndpoint()
    x,y,yaw = args.start
    vx,vy,wz = 0.,0.,0.
    sim_time,dt = 1.,.02
    next_tick = time.monotonic()
    try:
        while True:
            endpoint.publish_state(sim_time,[x,y,yaw],[vx,vy,wz])
            command = endpoint.read_command()
            velocity = [vx,vy,wz]
            for index,acceleration in enumerate([.6,.6,.8]):
                velocity[index] += max(-acceleration*dt,min(acceleration*dt,command[index]-velocity[index]))
            vx,vy,wz = velocity
            x += (math.cos(yaw)*vx-math.sin(yaw)*vy)*dt
            y += (math.sin(yaw)*vx+math.cos(yaw)*vy)*dt
            yaw = math.atan2(math.sin(yaw+wz*dt),math.cos(yaw+wz*dt))
            sim_time += dt
            next_tick += dt
            time.sleep(max(0.,next_tick-time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        endpoint.close()


if __name__ == '__main__':
    main()
