#pragma once

#include <algorithm>
#include <cmath>

namespace mac_control {

struct ControlConfig {
    int max_speed_steps_per_second = 16000;
    double deadzone = 0.08;
    double max_acceleration_steps_per_second_squared = 12000.0;
};

// 将手柄的 -1~1 摇杆量转换为步进电机目标速度。
// 死区用于抑制摇杆回中时的小幅漂移。
inline int axis_to_speed(double axis, const ControlConfig &config) {
    axis = std::clamp(axis, -1.0, 1.0);
    if (std::abs(axis) <= config.deadzone) {
        return 0;
    }

    const double sign = axis < 0.0 ? -1.0 : 1.0;
    const double scaled = (std::abs(axis) - config.deadzone) / (1.0 - config.deadzone);
    return static_cast<int>(std::lround(sign * scaled * config.max_speed_steps_per_second));
}

// 对速度指令做斜坡限制，避免真机接入后瞬间从 0 跳到最高速。
inline int limit_speed_change(int current, int target, double dt_seconds,
                              const ControlConfig &config) {
    if (dt_seconds <= 0.0) {
        return current;
    }
    const int max_delta = std::max(
        1, static_cast<int>(std::lround(config.max_acceleration_steps_per_second_squared * dt_seconds)));
    if (target > current) {
        return std::min(target, current + max_delta);
    }
    return std::max(target, current - max_delta);
}

} // namespace mac_control
