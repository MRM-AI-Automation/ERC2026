#!/usr/bin/env python3
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Bool


class ModeToggle(Node):
    def __init__(self):
        super().__init__("keyboard_toggle_node")

        self.current_mode = False
        self.last_mode = False

        self.cli = self.create_client(Trigger, "/toggle_autonomous")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /toggle_autonomous service...")

        self.subscription = self.create_subscription(
            Bool,
            "/autonomous_mode_state",
            self.mode_callback,
            10
        )

        self.get_logger().info("Press Z to toggle mode. Press Q to quit.")
        self.settings = termios.tcgetattr(sys.stdin)

    def mode_callback(self, msg):
        self.last_mode = self.current_mode
        self.current_mode = msg.data

        prev = "AUTONOMOUS" if self.last_mode else "MANUAL"
        new = "AUTONOMOUS" if self.current_mode else "MANUAL"

        if self.last_mode != self.current_mode:
            self.get_logger().info(f"Mode changed: {prev} → {new}")
        else:
            self.get_logger().info(f"Mode reaffirmed: {new}")

    def read_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def loop(self):
        while rclpy.ok():
            key = self.read_key().lower()

            if key == "z":
                self.last_mode = self.current_mode
                req = Trigger.Request()
                future = self.cli.call_async(req)
                rclpy.spin_until_future_complete(self, future)

                if future.result():
                    self.get_logger().info(
                        f"Toggle requested. Waiting for updated mode..."
                    )

            elif key == "q":
                self.get_logger().info("Exiting keyboard control")
                break


def main():
    rclpy.init()
    node = ModeToggle()
    node.loop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
