#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


class ImuOrientationTfBridge(Node):
    def __init__(self):
        super().__init__("lars_imu_orientation_tf_bridge")
        self.declare_parameter("imu_topic", "/hardware/measure/imu")
        self.declare_parameter("world_frame", "imu_visualization_world")
        self.declare_parameter("vehicle_frame", "vehicle_visualization_frame")

        self.world_frame = self.get_parameter("world_frame").value
        self.vehicle_frame = self.get_parameter("vehicle_frame").value
        self.broadcaster = TransformBroadcaster(self)
        self.invalid_orientation_reported = False
        self.subscription = self.create_subscription(
            Imu,
            self.get_parameter("imu_topic").value,
            self.on_imu,
            qos_profile_sensor_data,
        )

    def on_imu(self, message):
        quaternion = message.orientation
        components = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        norm_squared = sum(component * component for component in components)
        if not all(math.isfinite(component) for component in components) or norm_squared < 1e-12:
            if not self.invalid_orientation_reported:
                self.get_logger().warning("Ignoring invalid IMU orientation quaternion")
                self.invalid_orientation_reported = True
            return

        self.invalid_orientation_reported = False
        inverse_norm = 1.0 / math.sqrt(norm_squared)

        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = self.world_frame
        transform.child_frame_id = self.vehicle_frame
        transform.transform.rotation.x = quaternion.x * inverse_norm
        transform.transform.rotation.y = quaternion.y * inverse_norm
        transform.transform.rotation.z = quaternion.z * inverse_norm
        transform.transform.rotation.w = quaternion.w * inverse_norm
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = ImuOrientationTfBridge()
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
