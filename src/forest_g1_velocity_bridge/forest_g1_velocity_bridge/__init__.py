"""桥接③：只做速度消息到策略输入的适配。"""
from .adapter import G1VelocityAdapter, velocity_commands

__all__ = ["G1VelocityAdapter", "velocity_commands"]
