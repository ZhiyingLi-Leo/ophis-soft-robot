"""与 ROS 2 和串口无关的速度映射逻辑，便于在 macOS 上单元测试。"""

from dataclasses import dataclass


@dataclass(frozen=True)
class ControlConfig:
    max_speed: int = 3200
    deadzone: float = 0.12
    max_acceleration: float = 3200.0

    def validate(self) -> None:
        if self.max_speed <= 0:
            raise ValueError("max_speed 必须大于 0")
        if not 0.0 <= self.deadzone < 1.0:
            raise ValueError("deadzone 必须在 [0, 1) 内")
        if self.max_acceleration <= 0.0:
            raise ValueError("max_acceleration 必须大于 0")


def axis_to_speed(axis: float, config: ControlConfig) -> int:
    """将 -1..1 的摇杆量映射为带符号的步/秒，并去除回中漂移。"""
    axis = max(-1.0, min(1.0, float(axis)))
    if abs(axis) <= config.deadzone:
        return 0
    sign = -1.0 if axis < 0.0 else 1.0
    scaled = (abs(axis) - config.deadzone) / (1.0 - config.deadzone)
    return round(sign * scaled * config.max_speed)


def limit_speed_change(current: int, target: int, dt_seconds: float,
                       config: ControlConfig) -> int:
    """限制速度变化率，避免瞬时反向；急停和手柄掉线不调用此函数。"""
    if dt_seconds <= 0.0:
        return current
    max_delta = max(1, round(config.max_acceleration * dt_seconds))
    if target > current:
        return min(target, current + max_delta)
    return max(target, current - max_delta)
