#!/usr/bin/env python3
"""
MID-360 mapping pre-filter for ROS 2 PointCloud2.

Filters points in the LiDAR frame before they reach GLIM:
  1) minimum radial range
  2) maximum radial range (optional)
  3) self/body exclusion box (optional)
  4) companion-person exclusion box (optional)

The output preserves the original PointCloud2 field layout, including Livox
fields such as intensity/tag/line/timestamp.

Example:
  python3 mid360_mapping_filter.py --ros-args \
    -p input_topic:=/livox/lidar \
    -p output_topic:=/livox/lidar_mapping \
    -p min_range_m:=0.5 \
    -p companion_box_enabled:=true \
    -p companion_min_x:=-1.5 -p companion_max_x:=0.5 \
    -p companion_min_y:=-1.5 -p companion_max_y:=-0.3 \
    -p companion_min_z:=-1.5 -p companion_max_z:=1.0
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class Mid360MappingFilter(Node):
    def __init__(self):
        super().__init__("mid360_mapping_filter")

        self.declare_parameter("input_topic", "/livox/lidar")
        self.declare_parameter("output_topic", "/livox/lidar_mapping")

        self.declare_parameter("min_range_m", 0.0)
        self.declare_parameter("max_range_m", 0.0)

        self.declare_parameter("self_box_enabled", False)
        self.declare_parameter("self_min_x", -0.5)
        self.declare_parameter("self_max_x", 0.5)
        self.declare_parameter("self_min_y", -0.5)
        self.declare_parameter("self_max_y", 0.5)
        self.declare_parameter("self_min_z", -2.0)
        self.declare_parameter("self_max_z", 0.2)

        self.declare_parameter("companion_box_enabled", False)
        self.declare_parameter("companion_min_x", -1.5)
        self.declare_parameter("companion_max_x", 0.5)
        self.declare_parameter("companion_min_y", -1.5)
        self.declare_parameter("companion_max_y", -0.3)
        self.declare_parameter("companion_min_z", -1.5)
        self.declare_parameter("companion_max_z", 1.0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self.pub = self.create_publisher(PointCloud2, output_topic, qos_profile_sensor_data)
        self.sub = self.create_subscription(
            PointCloud2, input_topic, self.cloud_callback, qos_profile_sensor_data
        )

        self.frames = 0
        self.in_points = 0
        self.out_points = 0
        self.create_timer(2.0, self.report)

        self.get_logger().info(f"input : {input_topic}")
        self.get_logger().info(f"output: {output_topic}")
        self.get_logger().info(
            "This is a ROS-side software filter; it does not change MID-360 firmware settings."
        )

    def _box_mask(self, x, y, z, prefix):
        if not bool(self.get_parameter(f"{prefix}_box_enabled").value):
            return np.zeros(x.shape, dtype=bool)

        xmin = float(self.get_parameter(f"{prefix}_min_x").value)
        xmax = float(self.get_parameter(f"{prefix}_max_x").value)
        ymin = float(self.get_parameter(f"{prefix}_min_y").value)
        ymax = float(self.get_parameter(f"{prefix}_max_y").value)
        zmin = float(self.get_parameter(f"{prefix}_min_z").value)
        zmax = float(self.get_parameter(f"{prefix}_max_z").value)

        return (
            (x >= xmin) & (x <= xmax)
            & (y >= ymin) & (y <= ymax)
            & (z >= zmin) & (z <= zmax)
        )

    def cloud_callback(self, msg: PointCloud2):
        if msg.point_step <= 0 or not msg.data:
            return

        offsets = {f.name: int(f.offset) for f in msg.fields}
        if not all(name in offsets for name in ("x", "y", "z")):
            self.get_logger().error("PointCloud2 must contain x/y/z fields.")
            return

        n = len(msg.data) // msg.point_step
        if n <= 0:
            return

        endian = ">" if msg.is_bigendian else "<"
        try:
            x = np.ndarray(
                (n,), dtype=endian + "f4", buffer=msg.data,
                offset=offsets["x"], strides=(msg.point_step,)
            )
            y = np.ndarray(
                (n,), dtype=endian + "f4", buffer=msg.data,
                offset=offsets["y"], strides=(msg.point_step,)
            )
            z = np.ndarray(
                (n,), dtype=endian + "f4", buffer=msg.data,
                offset=offsets["z"], strides=(msg.point_step,)
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to read x/y/z: {exc}")
            return

        keep = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)

        min_range = float(self.get_parameter("min_range_m").value)
        max_range = float(self.get_parameter("max_range_m").value)

        if min_range > 0.0 or max_range > 0.0:
            r2 = x*x + y*y + z*z
            if min_range > 0.0:
                keep &= r2 >= min_range * min_range
            if max_range > 0.0:
                keep &= r2 <= max_range * max_range

        keep &= ~self._box_mask(x, y, z, "self")
        keep &= ~self._box_mask(x, y, z, "companion")

        # Preserve every original point record (all fields) for selected points.
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        records = raw[: n * msg.point_step].reshape(n, msg.point_step)
        filtered = np.ascontiguousarray(records[keep])

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = int(filtered.shape[0])
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = out.width * out.point_step
        out.is_dense = msg.is_dense
        out.data = filtered.tobytes()

        self.pub.publish(out)

        self.frames += 1
        self.in_points += n
        self.out_points += out.width

    def report(self):
        if self.frames == 0:
            return
        removed = self.in_points - self.out_points
        ratio = removed / self.in_points if self.in_points else 0.0
        self.get_logger().info(
            f"frames={self.frames} points={self.in_points}->{self.out_points} "
            f"removed={removed} ({ratio:.1%})"
        )
        self.frames = 0
        self.in_points = 0
        self.out_points = 0


def main():
    rclpy.init()
    node = Mid360MappingFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
