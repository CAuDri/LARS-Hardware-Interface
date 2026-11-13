#!/usr/bin/env python3

import math

import rclpy
from lars_msgs.msg import Float32Stamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class SteeringJointStateBridge(Node):
    def __init__(self):
        super().__init__("lars_steering_joint_state_bridge")
        self.declare_parameter("steering_topic", "/hardware/measure/steering_angle")
        self.angle_rad = 0.0
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.subscription = self.create_subscription(
            Float32Stamped,
            self.get_parameter("steering_topic").value,
            self.on_steering_angle,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(0.05, self.publish_joint_state)

    def on_steering_angle(self, message):
        if math.isfinite(message.data):
            self.angle_rad = math.radians(message.data)

    def publish_joint_state(self):
        # Neutral display before the first feedback; hold the latest angle after.
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [
            "front_left_wheel_joint",
            "front_right_wheel_joint",
        ]
        joint_state.position = [
            self.angle_rad,
            self.angle_rad,
        ]

        self.publisher.publish(joint_state)


def main():
    rclpy.init()
    node = SteeringJointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
