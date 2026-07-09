#!/usr/bin/env python3

import math

import rclpy
from lars_msgs.msg import Float32Stamped
from rclpy.node import Node
from sensor_msgs.msg import JointState


class SteeringJointStateBridge(Node):
    def __init__(self):
        super().__init__("lars_steering_joint_state_bridge")
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.subscription = self.create_subscription(
            Float32Stamped,
            "/hardware/measure/steering_angle_front",
            self.on_steering_angle,
            10,
        )

    def on_steering_angle(self, message):
        angle_rad = math.radians(message.data)

        joint_state = JointState()
        joint_state.header = message.header
        joint_state.name = [
            "front_left_wheel_joint",
            "front_right_wheel_joint",
        ]
        joint_state.position = [
            angle_rad,
            angle_rad,
        ]

        self.publisher.publish(joint_state)


def main():
    rclpy.init()
    node = SteeringJointStateBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
