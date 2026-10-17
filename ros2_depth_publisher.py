#!/usr/bin/env python3
"""
DepthStreamer ROS 2 Publisher Node
-----------------------------------
Streams raw Android ToF laser depth data into ROS 2 topics:
  - /camera/depth/image_raw   (sensor_msgs/msg/Image, 16UC1 in mm)
  - /camera/depth/camera_info (sensor_msgs/msg/CameraInfo)
  - /camera/depth/points      (sensor_msgs/msg/PointCloud2)

Compatible with ROS 2 Humble, Iron, Jazzy, Rolling.
Directly plugs into:
  - RViz2
  - Nav2 (Costmaps & Obstacle Avoidance)
  - RTAB-Map (RGB-D / 3D LiDAR SLAM)
  - depthimage_to_laserscan (Virtual 2D LiDAR)
  - KISS-ICP / Point Cloud SLAM
"""

import sys
import socket
import struct
import numpy as np

# Graceful import check for environments where ROS 2 is not yet sourced
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
    import std_msgs.msg
except ImportError:
    print("\n" + "=" * 65)
    print("[ERROR] ROS 2 Python modules (rclpy / sensor_msgs) were not found!")
    print("=" * 65)
    print("To run this ROS 2 node, ensure you are inside a ROS 2 environment:")
    print("  1. Source ROS 2 (Linux / WSL2):")
    print("       source /opt/ros/<humble|iron|jazzy>/setup.bash")
    print("  2. Ensure ROS 2 sensor_msgs is installed:")
    print("       sudo apt install ros-<distro>-sensor-msgs")
    print("  3. Run again:")
    print("       python3 ros2_depth_publisher.py")
    print("=" * 65 + "\n")
    sys.exit(1)


class AndroidDepthPublisher(Node):
    def __init__(self):
        super().__init__("android_depth_publisher")

        # Parameters
        self.declare_parameter("ip", "0.0.0.0")
        self.declare_parameter("port", 5005)
        self.declare_parameter("frame_id", "camera_depth_optical_frame")
        self.declare_parameter("publish_points", True)
        self.declare_parameter("min_range_m", 0.1)
        self.declare_parameter("max_range_m", 4.0)
        self.declare_parameter("fx", 203.1)
        self.declare_parameter("fy", 203.7)
        self.declare_parameter("cx", 90.0)
        self.declare_parameter("cy", 120.0)

        self.ip = self.get_parameter("ip").get_parameter_value().string_value
        self.port = self.get_parameter("port").get_parameter_value().integer_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.publish_points = self.get_parameter("publish_points").get_parameter_value().bool_value
        self.min_range = self.get_parameter("min_range_m").get_parameter_value().double_value
        self.max_range = self.get_parameter("max_range_m").get_parameter_value().double_value

        self.fx = self.get_parameter("fx").get_parameter_value().double_value
        self.fy = self.get_parameter("fy").get_parameter_value().double_value
        self.cx = self.get_parameter("cx").get_parameter_value().double_value
        self.cy = self.get_parameter("cy").get_parameter_value().double_value

        # Active rotated resolution (portrait mode: 180w x 240h)
        self.width = 180
        self.height = 240

        # ROS 2 Publishers
        self.image_pub = self.create_publisher(Image, "/camera/depth/image_raw", 10)
        self.info_pub = self.create_publisher(CameraInfo, "/camera/depth/camera_info", 10)
        if self.publish_points:
            self.points_pub = self.create_publisher(PointCloud2, "/camera/depth/points", 10)

        # Precompute coordinate grids for fast projection
        u_grid, v_grid = np.meshgrid(np.arange(self.width), np.arange(self.height))
        self.u_flat = u_grid.flatten()
        self.v_flat = v_grid.flatten()

        # UDP setup
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.ip, self.port))
        self.sock.settimeout(0.002)

        # Buffers for raw sensor frame (240x180 landscape on phone)
        self.frame_buffer = np.zeros((180, 240), dtype=np.uint16)
        self.confidence_buffer = np.zeros((180, 240), dtype=np.uint16)

        # High-frequency poll timer
        self.timer = self.create_timer(0.005, self.read_udp)

        self.get_logger().info(f"DepthStreamer ROS 2 Node active on UDP {self.ip}:{self.port}")
        self.get_logger().info(f"Target Frame ID: {self.frame_id}")
        self.get_logger().info("Publishing: /camera/depth/image_raw, /camera/depth/camera_info, /camera/depth/points")

    def read_udp(self):
        try:
            while True:
                data, addr = self.sock.recvfrom(65535)
                if len(data) < 16:
                    continue

                w, h, y_offset, y_count = struct.unpack('<IIII', data[:16])
                expected_size = w * y_count * 2
                if len(data) - 16 != expected_size:
                    continue

                depth_array_raw = np.frombuffer(data, dtype=np.uint16, offset=16)
                distance_chunk = np.bitwise_and(depth_array_raw, 0x1FFF).reshape((y_count, w))
                conf_chunk = np.right_shift(depth_array_raw, 13).reshape((y_count, w))

                self.frame_buffer[y_offset:y_offset + y_count, :] = distance_chunk
                self.confidence_buffer[y_offset:y_offset + y_count, :] = conf_chunk

                # End of frame reached -> publish ROS 2 messages
                if y_offset + y_count >= h:
                    self.publish_frame()

        except (socket.timeout, BlockingIOError):
            pass
        except Exception as e:
            self.get_logger().warn(f"UDP Packet Error: {e}")

    def publish_frame(self):
        stamp = self.get_clock().now().to_msg()

        # Rotate 90 degrees clockwise to match portrait orientation
        depth_rotated = np.rot90(self.frame_buffer, k=-1).astype(np.uint16)
        conf_rotated = np.rot90(self.confidence_buffer, k=-1).astype(np.uint8)

        # 1. Publish CameraInfo
        info_msg = CameraInfo()
        info_msg.header.stamp = stamp
        info_msg.header.frame_id = self.frame_id
        info_msg.width = self.width
        info_msg.height = self.height
        info_msg.distortion_model = "plumb_bob"
        info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info_msg.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
        info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info_msg.p = [self.fx, 0.0, self.cx, 0.0, 0.0, self.fy, self.cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info_msg)

        # 2. Publish Depth Image (16UC1 format: millimeters)
        img_msg = Image()
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self.frame_id
        img_msg.height = self.height
        img_msg.width = self.width
        img_msg.encoding = "16UC1"
        img_msg.is_bigendian = 0
        img_msg.step = self.width * 2
        img_msg.data = depth_rotated.tobytes()
        self.image_pub.publish(img_msg)

        # 3. Publish PointCloud2
        if self.publish_points:
            self.publish_point_cloud(depth_rotated, conf_rotated, stamp)

    def publish_point_cloud(self, depth_mm, conf_map, stamp):
        # Convert mm to meters
        z_flat = depth_mm.flatten().astype(np.float32) / 1000.0
        conf_flat = conf_map.flatten().astype(np.float32)

        valid_mask = (z_flat >= self.min_range) & (z_flat <= self.max_range)
        if not np.any(valid_mask):
            return

        z_valid = z_flat[valid_mask]
        u_valid = self.u_flat[valid_mask]
        v_valid = self.v_flat[valid_mask]
        intensity_valid = conf_flat[valid_mask] / 7.0  # Normalized confidence 0.0 - 1.0

        # Optical frame: X=right, Y=down, Z=forward
        x_valid = (u_valid - self.cx) * z_valid / self.fx
        y_valid = (v_valid - self.cy) * z_valid / self.fy

        num_points = len(z_valid)

        # Pack XYZ + Intensity into structured binary array
        cloud_data = np.zeros(num_points, dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32)
        ])
        cloud_data['x'] = x_valid
        cloud_data['y'] = y_valid
        cloud_data['z'] = z_valid
        cloud_data['intensity'] = intensity_valid

        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = num_points
        msg.is_dense = True
        msg.is_bigendian = False

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        msg.point_step = 16
        msg.row_step = msg.point_step * num_points
        msg.data = cloud_data.tobytes()

        self.points_pub.publish(msg)

    def destroy_node(self):
        if hasattr(self, 'sock'):
            self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AndroidDepthPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
