#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JawZeroPublisher(Node):
    def __init__(self):
        super().__init__('jaw_zero_publisher')
        # Launch parameter typing can vary (string vs int); normalize to string.
        instrument = str(self.declare_parameter('instrument', '').value).strip()
        self._active = instrument.endswith('183') or instrument.endswith('184')

        if not self._active:
            self.get_logger().info(
                f'Jaw zero clamp disabled for instrument "{instrument}"')
            return

        self._publisher = self.create_publisher(JointState, 'jaw_zero_js', 10)
        self._subscription = self.create_subscription(
            JointState,
            'jaw/measured_js',
            self._handle_jaw_state,
            10)
        self.get_logger().info(
            f'Jaw zero clamp enabled for instrument "{instrument}"')

    def _handle_jaw_state(self, message: JointState) -> None:
        clamped = JointState()
        clamped.header = message.header
        clamped.name = list(message.name) if message.name else ['jaw']

        if message.position:
            clamped.position = [0.0 for _ in message.position]
        else:
            clamped.position = [0.0]

        if message.velocity:
            clamped.velocity = [0.0 for _ in message.velocity]
        if message.effort:
            clamped.effort = [0.0 for _ in message.effort]

        self._publisher.publish(clamped)


def main(args=None):
    rclpy.init(args=args)
    node = JawZeroPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
