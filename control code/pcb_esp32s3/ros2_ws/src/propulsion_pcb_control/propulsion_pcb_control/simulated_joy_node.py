"""发布可重复的 Joy 序列，用于没有手柄和 PCB 时验证 ROS 2 链路。"""

import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class SimulatedJoyNode(Node):
    def __init__(self) -> None:
        super().__init__("simulated_joy")
        self.declare_parameter("axis_index", 3)
        self.declare_parameter("stop_button_index", 6)
        self.axis_index = int(self.get_parameter("axis_index").value)
        self.stop_button_index = int(self.get_parameter("stop_button_index").value)
        self.publisher = self.create_publisher(Joy, "/joy", 10)
        self.started = time.monotonic()
        self.last_state = ""
        self.create_timer(0.05, self._publish_sample)

    def _publish_sample(self) -> None:
        elapsed = (time.monotonic() - self.started) % 10.0
        axis = 0.0
        emergency = 0
        state = "停止"
        if 1.0 <= elapsed < 3.0:
            axis, state = 0.6, "正向"
        elif 4.0 <= elapsed < 5.0:
            axis, state = -0.6, "反向"
        elif 5.0 <= elapsed < 5.5:
            axis, emergency, state = -0.6, 1, "反向过程中急停"

        message = Joy()
        message.header.stamp = self.get_clock().now().to_msg()
        message.axes = [0.0] * (self.axis_index + 1)
        message.buttons = [0] * (self.stop_button_index + 1)
        message.axes[self.axis_index] = axis
        message.buttons[self.stop_button_index] = emergency
        self.publisher.publish(message)

        if state != self.last_state:
            self.get_logger().info(f"仿真输入：{state} axis={axis} emergency={emergency}")
            self.last_state = state


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = SimulatedJoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
