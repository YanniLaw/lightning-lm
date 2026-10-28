#!/usr/bin/env python3
"""Forward bag TF except legacy map corrections and frames owned by SLAM."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster
import yaml


class BagTfFilter(Node):
    def __init__(self):
        super().__init__("lightning_bag_tf_filter")
        config_path = self.declare_parameter("config_path", "").value
        system = {}
        if config_path:
            with open(config_path, encoding="utf-8") as config_file:
                system = yaml.safe_load(config_file).get("system", {})
        self.owned_children = set()
        if system.get("pub_tf", True):
            published_frame = system.get("published_frame", "imu_frame")
            self.owned_children.add(published_frame)
            if system.get("provide_odom_frame", False):
                self.owned_children.add(system.get("odom_frame", "odom"))
            if published_frame != "imu_frame":
                self.owned_children.add("imu_frame")
        self.publisher = self.create_publisher(TFMessage, "/tf", 100)
        self.subscription = self.create_subscription(
            TFMessage, "/lightning/bag_tf", self.handle_tf, 100
        )
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.static_subscription = self.create_subscription(
            TFMessage, "/lightning/bag_tf_static", self.handle_static_tf,
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE),
        )

    def filter_transforms(self, message):
        return [
            transform
            for transform in message.transforms
            if (transform.header.frame_id, transform.child_frame_id)
            not in {("map", "odom"), ("map", "local_map")}
            and transform.child_frame_id not in self.owned_children
        ]

    def handle_tf(self, message):
        transforms = self.filter_transforms(message)
        if transforms:
            self.publisher.publish(TFMessage(transforms=transforms))

    def handle_static_tf(self, message):
        transforms = self.filter_transforms(message)
        if transforms:
            # Accumulate all static batches for subscribers joining later.
            self.static_broadcaster.sendTransform(transforms)


def main():
    rclpy.init()
    node = BagTfFilter()
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
