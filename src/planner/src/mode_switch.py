#!/usr/bin/env python3

import sys
import termios
import tty
import select
import signal

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Bool


class ModeToggle(Node):

    def __init__(self):
        super().__init__("keyboard_toggle_node")

        self.current_mode = False
        self.running = True

        self.cli = self.create_client(
            Trigger,
            "/toggle_autonomous"
        )

        self.get_logger().info(
            "Waiting for /toggle_autonomous service..."
        )

        while rclpy.ok() and not self.cli.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info(
                "Waiting for /toggle_autonomous service..."
            )

        self.subscription = self.create_subscription(
            Bool,
            "/autonomous_mode_state",
            self.mode_callback,
            10
        )

        self.settings = None

        if sys.stdin.isatty():
            self.settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())

        signal.signal(
            signal.SIGINT,
            self.signal_handler
        )

        self.timer = self.create_timer(
            0.05,
            self.keyboard_callback
        )

        self.get_logger().info(
            "================================================="
        )
        self.get_logger().info(
            "Keyboard mode control started"
        )
        self.get_logger().info(
            "Press Z = toggle autonomous/manual"
        )
        self.get_logger().info(
            "Press Q = quit"
        )
        self.get_logger().info(
            "Press Ctrl+C = quit"
        )
        self.get_logger().info(
            "================================================="
        )

    def mode_callback(self, msg):
        new_mode = msg.data

        if new_mode != self.current_mode:

            old_name = (
                "AUTONOMOUS"
                if self.current_mode
                else "MANUAL"
            )

            new_name = (
                "AUTONOMOUS"
                if new_mode
                else "MANUAL"
            )

            self.current_mode = new_mode

            self.get_logger().info(
                f"Mode changed externally: "
                f"{old_name} -> {new_name}"
            )

        else:
            self.current_mode = new_mode

    def keyboard_callback(self):

        if not self.running:
            return

        if not sys.stdin.isatty():
            return

        readable, _, _ = select.select(
            [sys.stdin],
            [],
            [],
            0
        )

        if not readable:
            return

        key = sys.stdin.read(1).lower()

        if key == "z":

            self.get_logger().info(
                f"Z pressed. Requesting toggle "
                f"(current state: "
                f"{'AUTONOMOUS' if self.current_mode else 'MANUAL'})"
            )

            request = Trigger.Request()

            future = self.cli.call_async(request)

            future.add_done_callback(
                self.toggle_response_callback
            )

        elif key == "q":

            self.get_logger().info(
                "Q pressed. Stopping node."
            )

            self.shutdown()

    def toggle_response_callback(self, future):

        try:
            response = future.result()

            if response.success:
                self.get_logger().info(
                    f"Toggle service accepted: "
                    f"{response.message}"
                )
            else:
                self.get_logger().warn(
                    f"Toggle service rejected: "
                    f"{response.message}"
                )

        except Exception as e:

            self.get_logger().error(
                f"Toggle service call failed: {e}"
            )

    def signal_handler(self, signum, frame):

        self.get_logger().info(
            "Ctrl+C detected. Stopping node."
        )

        self.shutdown()

    def shutdown(self):

        if not self.running:
            return

        self.running = False

        self.get_logger().info(
            "Shutting down keyboard control..."
        )

        if self.timer is not None:
            self.timer.cancel()

        self.restore_terminal()

        if rclpy.ok():
            rclpy.shutdown()

    def restore_terminal(self):

        if self.settings is not None and sys.stdin.isatty():

            try:
                termios.tcsetattr(
                    sys.stdin,
                    termios.TCSADRAIN,
                    self.settings
                )
            except Exception:
                pass

    def destroy_node(self):

        self.restore_terminal()

        super().destroy_node()


def main():

    rclpy.init()

    node = None

    try:

        node = ModeToggle()

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        if node is not None:

            node.restore_terminal()

            if node.running:
                node.running = False

            try:
                node.destroy_node()
            except Exception:
                pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
