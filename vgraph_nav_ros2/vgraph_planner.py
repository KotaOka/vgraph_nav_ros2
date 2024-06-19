#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (C) 2023-2024  Alapaca-zip
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import os
import sys
import time  # timeモジュールをインポート

import cv2
import mip
import numpy
import yaml
from PIL import Image, ImageDraw

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
from tf2_ros import TransformListener, Buffer

from ament_index_python.packages import get_package_share_directory

import tf2_ros
from geometry_msgs.msg import TransformStamped


class VgraphPlannerNode(Node):
    def __init__(self):
        super().__init__('vgraph_planner_node')
        
        # ROSパラメータの初期化
        self.test_folder_path = self.declare_parameter("test_folder", "test").get_parameter_value().string_value
        self.epsilon_factor = self.declare_parameter("epsilon_factor", 0.05).get_parameter_value().double_value
        self.robot_radius = self.declare_parameter("robot_radius", 0.1).get_parameter_value().double_value
        self.odom_topic = self.declare_parameter("odom_topic", "/odom").get_parameter_value().string_value
        self.scan_topic = self.declare_parameter("scan_topic", "/scan").get_parameter_value().string_value
        self.vel_linear = self.declare_parameter("vel_linear", 0.1).get_parameter_value().double_value
        self.vel_theta = self.declare_parameter("vel_theta", 0.1).get_parameter_value().double_value
        self.angle_tolerance = self.declare_parameter("angle_tolerance", 0.1).get_parameter_value().double_value
        self.goal_tolerance = self.declare_parameter("goal_tolerance", 0.1).get_parameter_value().double_value
        self.verbose = self.declare_parameter("verbose", False).get_parameter_value().bool_value
        


        self.start_point = None
        self.end_point = None

        # パブリッシャーの作成
        self.costmap_pub = self.create_publisher(OccupancyGrid, "/cost_map", 10)
        self.path_pub = self.create_publisher(Path, "/path", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # サブスクリプションの作成
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.initial_pose_callback, 10)
        self.create_subscription(PoseStamped, "/move_base_simple/goal", self.goal_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # パッケージの共有ディレクトリを取得
        package_share_directory = get_package_share_directory('vgraph_nav_ros2')
        
        # パッケージ内のデフォルトのマップファイルのパスを設定
        default_map_file_path = os.path.join(package_share_directory, 'maps', 'map.yaml')
        
        # パラメータを宣言し、その値を取得
        self.map_file_path = self.declare_parameter("map_file", default_map_file_path).get_parameter_value().string_value
        self.get_logger().info(f'Map file path: {self.map_file_path}')
        
        # マップファイルを読み込む
        self.original_image, self.resolution, self.origin = self.load_map_file(self.map_file_path)
        
        # 初期の黒いピクセルを見つけて、画像を拡大する
        black_pixels = self.find_black_pixels(self.original_image)
        self.enlarged_image, _ = self.enlarge_black_pixels(self.original_image, black_pixels, self.robot_radius)
        
        # 初期のコストマップをパブリッシュ
        self.publish_initial_cost_map(self.enlarged_image)

    def load_map_file(self, yaml_file_path):
        with open(yaml_file_path, "r") as file:
            map_yaml = yaml.safe_load(file)

        directory = os.path.dirname(yaml_file_path)
        image_file_name = map_yaml.get("image", None)

        if image_file_name is not None:
            image_path = os.path.join(directory, image_file_name)
        else:
            image_path = None

        image = Image.open(image_path)
        resolution = map_yaml.get("resolution", None)
        origin = map_yaml.get("origin", None)

        return image, resolution, origin

    def find_black_pixels(self, image):
        black_pixels = []

        for y in range(image.height):
            for x in range(image.width):
                if image.getpixel((x, y)) == 0:
                    black_pixels.append((x, y))

        return black_pixels

    def enlarge_black_pixels(self, image, black_pixels, width):
        new_image = image.copy()
        pixel_width = math.ceil(width / self.resolution)
        pixel_width = pixel_width + 1
        new_black_pixels = set(black_pixels)

        for x, y in black_pixels:
            for dx in range(-pixel_width, pixel_width + 1):
                for dy in range(-pixel_width, pixel_width + 1):
                    selected_x, selected_y = x + dx, y + dy
                    if (
                        0 <= selected_x < new_image.width
                        and 0 <= selected_y < new_image.height
                    ):
                        new_black_pixels.add((selected_x, selected_y))

        for x, y in new_black_pixels:
            new_image.putpixel((x, y), 0)

        return new_image, new_black_pixels

    def publish_initial_cost_map(self, image):
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid.header.frame_id = "map"
        occupancy_grid.info.resolution = self.resolution
        occupancy_grid.info.width = image.width
        occupancy_grid.info.height = image.height
        occupancy_grid.info.origin.position.x = self.origin[0]
        occupancy_grid.info.origin.position.y = self.origin[1]
        occupancy_grid.info.origin.position.z = 0.0
        occupancy_grid.info.origin.orientation.w = 1.0

        # コストマップのデータを設定
        data = []
        for y in range(image.height):
            for x in range(image.width):
                pixel = image.getpixel((x, y))
                if pixel == 0:  # 黒いピクセルは障害物
                    data.append(100)
                else:  # その他のピクセルは空きスペース
                    data.append(0)
        occupancy_grid.data = data

        self.costmap_pub.publish(occupancy_grid)

        # 5秒間スリープしてから次の操作を実行
        time.sleep(5)  # Python標準のtime.sleep()を使用

    # ここに他のコールバック関数やメソッドを追加


    def publish_cost_map(self, image, edges=None):
        rgb_image = image.convert("RGB")
        draw = ImageDraw.Draw(rgb_image)

        if edges is not None:
            for start, end in edges:
                draw.line([start, end], fill=(128, 128, 128), width=1)

        cost_map_msg = OccupancyGrid()
        cost_map_msg.header.frame_id = "map"
        cost_map_msg.header.stamp = self.get_clock().now().to_msg()
        cost_map_msg.info.resolution = self.resolution
        cost_map_msg.info.width = rgb_image.width
        cost_map_msg.info.height = rgb_image.height
        cost_map_msg.info.origin.position.x = self.origin[0]
        cost_map_msg.info.origin.position.y = self.origin[1]
        cost_map_msg.info.origin.orientation.w = 1.0
        cost_map_msg.data = []

        for y in reversed(range(rgb_image.height)):
            for x in range(rgb_image.width):
                original_pixel = image.getpixel((x, y))
                rgb_pixel = rgb_image.getpixel((x, y))
                if original_pixel == 0 and rgb_pixel == (128, 128, 128):
                    cost_map_msg.data.append(50)
                elif original_pixel == 0:
                    cost_map_msg.data.append(100)
                else:
                    cost_map_msg.data.append(0)

        self.costmap_pub.publish(cost_map_msg)

    def broadcast_cost_map_tf(self):
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = "cost_map_frame"
        transform_stamped.child_frame_id = "base_link"

        # /cost_map トピックからの情報を利用して、位置と姿勢を設定する
        # ここでは例として原点を基準にしていますが、適切な位置情報をセットしてください
        transform_stamped.transform.translation.x = 0.0
        transform_stamped.transform.translation.y = 0.0
        transform_stamped.transform.translation.z = 0.0
        transform_stamped.transform.rotation.x = 0.0
        transform_stamped.transform.rotation.y = 0.0
        transform_stamped.transform.rotation.z = 0.0
        transform_stamped.transform.rotation.w = 1.0

        # TFフレームをブロードキャスト
        self.tf_broadcaster.sendTransform(transform_stamped)

    
    def initial_pose_callback(self, msg):
        self.start_point = self.pose_to_pixel(
            (msg.pose.pose.position.x, msg.pose.pose.position.y)
        )
        self.get_logger().info("Set initial pose.")

    def goal_callback(self, msg):
        if self.start_point is None:
            current_position = self.get_current_position()
            if current_position is not None:
                self.start_point = self.pose_to_pixel(current_position)
                self.get_logger().info("Automatically set initial pose.")
            else:
                self.get_logger().warn("Cannot set initial pose automatically.")
                return

        self.end_point = self.pose_to_pixel(
            (msg.pose.position.x, msg.pose.position.y)
        )

        path = self.make_plan(self.start_point, self.end_point)

        if path is None:
            self.get_logger().info("No valid path found.")
            return

        self.get_logger().info("Path found.")

        final_orientation = self.quaternion_to_euler(msg.pose.orientation)
        final_pose = (msg.pose.position.x, msg.pose.position.y, final_orientation[2])

        self.publish_cost_map(self.enlarged_image, path)

        self.navigate_along_path(path, final_pose)

    def odom_callback(self, msg):
        pass

    def scan_callback(self, msg):
        pass

    def get_current_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", Time(), Duration(seconds=1.0)
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            theta = self.quaternion_to_euler(transform.transform.rotation)[2]
            return (x, y, theta)
        except Exception as e:
            self.get_logger().error(f"Error getting current position: {e}")
            return None

    def pose_to_pixel(self, pose):
        x = pose[0] - self.origin[0]
        y = pose[1] - self.origin[1]
        pixel_x = int(round(x / self.resolution))
        pixel_y = int(round((self.origin[1] + self.resolution * self.original_image.height - y) / self.resolution))
        return pixel_x, pixel_y

    def pixel_to_pose(self, pixel):
        x = pixel[0] * self.resolution + self.origin[0]
        y = (self.original_image.height - pixel[1]) * self.resolution + self.origin[1]
        return x, y

    def find_corners(self, image, points):
        corners = set()

        for x, y in points:
            x_lower = x - 1 if x > 0 else 0
            x_upper = x + 2 if x < image.width - 1 else image.width
            y_lower = y - 1 if y > 0 else 0
            y_upper = y + 2 if y < image.height - 1 else image.height

            neighbor_points = []
            for neighbor_x in range(x_lower, x_upper):
                for neighbor_y in range(y_lower, y_upper):
                    if image.getpixel((neighbor_x, neighbor_y)) == 0:
                        neighbor_points.append((neighbor_x, neighbor_y))

            if len(neighbor_points) == 3:
                corners.add((x, y))

        return corners

    def get_valid_edges(self, image, corners):
        valid_edges = set()

        for (start_x, start_y) in corners:
            for (end_x, end_y) in corners:
                if (start_x, start_y) == (end_x, end_y):
                    continue

                lower_x = min(start_x, end_x)
                upper_x = max(start_x, end_x)
                lower_y = min(start_y, end_y)
                upper_y = max(start_y, end_y)

                has_obstacle = False
                for x in range(lower_x, upper_x + 1):
                    for y in range(lower_y, upper_y + 1):
                        if image.getpixel((x, y)) == 0:
                            has_obstacle = True
                            break
                    if has_obstacle:
                        break

                if not has_obstacle:
                    valid_edges.add(((start_x, start_y), (end_x, end_y)))

        return valid_edges

    def calculate_shortest_path(self, corners, edges):
        vertices = [(int(v[0]), int(v[1])) for v in corners]
        shortest_path_edges = []

        model = mip.Model()

        vars = {}
        for v1 in vertices:
            for v2 in vertices:
                if v1 != v2 and (v1, v2) in edges:
                    dist = math.sqrt((v1[0] - v2[0]) ** 2 + (v1[1] - v2[1]) ** 2)
                    vars[(v1, v2)] = model.add_var(var_type=mip.BINARY, obj=dist)
                    vars[(v2, v1)] = vars[(v1, v2)]

        for v in vertices:
            model += mip.xsum(vars[(v, v2)] for v2 in vertices if (v, v2) in vars) == 2

        model.optimize()

        for (v1, v2), var in vars.items():
            if var.x >= 0.99:
                shortest_path_edges.append((v1, v2))

        return shortest_path_edges

    def make_plan(self, start, goal):
        if not self.is_within_map_bounds(start) or not self.is_within_map_bounds(goal):
            self.get_logger().warn("Start or goal point is out of map bounds.")
            return None

        if self.original_image.getpixel(start) == 0 or self.original_image.getpixel(goal) == 0:
            self.get_logger().warn("Start or goal point is inside an obstacle.")
            return None

        corners = self.find_corners(self.enlarged_image, [start, goal])
        valid_edges = self.get_valid_edges(self.enlarged_image, corners)

        if len(corners) == 0 or len(valid_edges) == 0:
            self.get_logger().warn("No valid corners or edges found.")
            return None

        shortest_path_edges = self.calculate_shortest_path(corners, valid_edges)
        return shortest_path_edges

    def is_within_map_bounds(self, point):
        x, y = point
        return 0 <= x < self.original_image.width and 0 <= y < self.original_image.height

    def quaternion_to_euler(self, quaternion):
        return euler_from_quaternion([
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        ])

    def navigate_along_path(self, edges, final_pose):
        for edge in edges:
            if not self.move_to_goal(edge):
                return

        if final_pose is not None:
            self.rotate_to_final_pose(final_pose)

    def move_to_goal(self, edge):
        start, end = edge
        start_pose = self.pixel_to_pose(start)
        end_pose = self.pixel_to_pose(end)

        self.get_logger().info(f"Moving to goal: {end_pose}")

        goal_reached = False
        rate = self.create_rate(10)

        while not goal_reached and rclpy.ok():
            current_position = self.get_current_position()

            if current_position is None:
                return False

            x_diff = end_pose[0] - current_position[0]
            y_diff = end_pose[1] - current_position[1]
            distance = math.sqrt(x_diff ** 2 + y_diff ** 2)

            if distance < self.goal_tolerance:
                goal_reached = True
                self.cmd_vel_pub.publish(Twist())
                break

            angle_to_goal = math.atan2(y_diff, x_diff)
            angle_diff = angle_to_goal - current_position[2]

            cmd_vel = Twist()
            if abs(angle_diff) > self.angle_tolerance:
                cmd_vel.angular.z = self.vel_theta if angle_diff > 0 else -self.vel_theta
            else:
                cmd_vel.linear.x = self.vel_linear

            self.cmd_vel_pub.publish(cmd_vel)
            rate.sleep()

        return goal_reached

    def rotate_to_goal(self, goal):
        self.get_logger().info("Rotating to goal.")

        rate = self.create_rate(10)

        while rclpy.ok():
            current_position = self.get_current_position()

            if current_position is None:
                return False

            angle_diff = goal[2] - current_position[2]

            if abs(angle_diff) < self.angle_tolerance:
                self.cmd_vel_pub.publish(Twist())
                break

            cmd_vel = Twist()
            cmd_vel.angular.z = self.vel_theta if angle_diff > 0 else -self.vel_theta
            self.cmd_vel_pub.publish(cmd_vel)
            rate.sleep()

    def rotate_to_final_pose(self, final_pose):
        self.get_logger().info("Rotating to final pose.")
        self.rotate_to_goal(final_pose)

def main(args=None):
    rclpy.init(args=args)
    node = VgraphPlannerNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

