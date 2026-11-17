"""订阅 sensor_msgs/Joy，并将推进速度发送给 PCB。"""

import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32, String

from propulsion_pcb_control.control_logic import ControlConfig, axis_to_speed, limit_speed_change
from propulsion_pcb_control.serial_transport import PCBSerial


class PropulsionNode(Node):
    def __init__(self) -> None:
        super().__init__("propulsion_pcb_controller")

        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("axis_index", 3)
        self.declare_parameter("stop_button_index", 6)
        self.declare_parameter("max_speed", 3200)
        self.declare_parameter("max_acceleration", 3200.0)
        self.declare_parameter("deadzone", 0.12)
        self.declare_parameter("joy_timeout", 0.5)
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("diagnostic", True)

        self.axis_index = int(self.get_parameter("axis_index").value)
        self.stop_button_index = int(self.get_parameter("stop_button_index").value)
        self.joy_timeout = float(self.get_parameter("joy_timeout").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.diagnostic = bool(self.get_parameter("diagnostic").value)
        if self.axis_index < 0 or self.stop_button_index < 0 or self.joy_timeout <= 0.0:
            raise ValueError("axis_index、stop_button_index 或 joy_timeout 参数无效")

        self.config = ControlConfig(
            max_speed=int(self.get_parameter("max_speed").value),
            max_acceleration=float(self.get_parameter("max_acceleration").value),
            deadzone=float(self.get_parameter("deadzone").value),
        )
        self.config.validate()

        self.serial: Optional[PCBSerial] = None
        if not self.dry_run:
            path = str(self.get_parameter("serial_port").value)
            baud = int(self.get_parameter("baud_rate").value)
            self.serial = PCBSerial(path, baud)
            self.serial.stop()
            self.get_logger().info(f"PCB 已连接：{path} @ {baud}")
        else:
            self.get_logger().info("dry-run：不打开 PCB 串口")

        joy_topic = str(self.get_parameter("joy_topic").value)
        self.create_subscription(Joy, joy_topic, self._on_joy, 10)
        self.command_publisher = self.create_publisher(Int32, "~/command_speed", 10)
        self.status_publisher = self.create_publisher(String, "~/status", 10)
        self.create_timer(0.02, self._on_timer)

        self.axis = 0.0
        self.emergency_pressed = False
        self.received_joy = False
        self.last_joy_time = time.monotonic()
        self.last_timer_time = time.monotonic()
        self.last_diagnostic_time = 0.0
        self.timeout_stopped = False
        self.emergency_stopped = False
        self.mapping_error_logged = False
        self.commanded_speed = 0
        self.last_sent_speed: Optional[int] = None
        self._send_speed(0, force=True, reason="启动停止")

        self.get_logger().info(
            f"订阅 {joy_topic}：axis[{self.axis_index}]，"
            f"急停 button[{self.stop_button_index}]，最大速度 {self.config.max_speed}")

    def _on_joy(self, message: Joy) -> None:
        axis_valid = self.axis_index < len(message.axes)
        button_valid = self.stop_button_index < len(message.buttons)
        mapping_valid = axis_valid and button_valid

        if not mapping_valid and not self.mapping_error_logged:
            self.get_logger().error(
                f"Joy 映射无效：收到 {len(message.axes)} 个轴、{len(message.buttons)} 个按键，"
                f"配置要求 axis[{self.axis_index}] 和 button[{self.stop_button_index}]；立即停止")
        self.mapping_error_logged = not mapping_valid

        self.axis = float(message.axes[self.axis_index]) if axis_valid else 0.0
        button_pressed = bool(message.buttons[self.stop_button_index]) if button_valid else False
        # 映射不完整时不能确认急停键可用，因此按急停处理。
        self.emergency_pressed = (not mapping_valid) or button_pressed

        self.received_joy = True
        self.last_joy_time = time.monotonic()
        self.timeout_stopped = False

    def _on_timer(self) -> None:
        now = time.monotonic()
        if self.serial is not None:
            self.serial.keep_alive()
        dt = now - self.last_timer_time
        self.last_timer_time = now

        timed_out = (not self.received_joy or
                     now - self.last_joy_time > self.joy_timeout)
        if timed_out:
            self.commanded_speed = 0
            self._send_speed(0, force=not self.timeout_stopped, reason="手柄超时停止")
            self.timeout_stopped = True
            return

        target = axis_to_speed(self.axis, self.config)
        if self.emergency_pressed:
            self.commanded_speed = 0
            self._send_speed(
                0, force=not self.emergency_stopped,
                reason="手柄急停或映射无效")
            self.emergency_stopped = True
        else:
            self.emergency_stopped = False
            self.commanded_speed = limit_speed_change(
                self.commanded_speed, target, dt, self.config)
            self._send_speed(self.commanded_speed, force=False, reason="摇杆")

        if self.serial is not None:
            reply = self.serial.read_available()
            if "ERR" in reply:
                self.get_logger().error(f"PCB 返回：{reply.strip()}")

        if self.diagnostic and now - self.last_diagnostic_time >= 0.5:
            self.get_logger().info(
                f"axis={self.axis:.3f} target={target} command={self.commanded_speed} "
                f"emergency={'yes' if self.emergency_pressed else 'no'}")
            self.last_diagnostic_time = now

    def _send_speed(self, speed: int, force: bool, reason: str) -> None:
        if not force and speed == self.last_sent_speed:
            return
        if self.serial is not None:
            self.serial.send_speed(speed)
        message = Int32()
        message.data = int(speed)
        self.command_publisher.publish(message)
        status = String()
        status.data = f"V{speed} {reason} {'dry-run' if self.dry_run else 'serial'}"
        self.status_publisher.publish(status)
        self.last_sent_speed = speed

    def shutdown(self) -> None:
        self.commanded_speed = 0
        try:
            self._send_speed(0, force=True, reason="节点退出")
        finally:
            if self.serial is not None:
                self.serial.close()
                self.serial = None


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[PropulsionNode] = None
    try:
        node = PropulsionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
