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

'''
import math
import os
import sys

import cv2
import mip
import numpy
import rclpy
import tf
import tf2_ros
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from PIL import Image, ImageDraw
from sensor_msgs.msg import LaserScan
'''
import math
import os
import sys

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


class VgraphPlannerNode(Node):
    def __init__(self):
        self.map_file_path = self.declare_parameter("map_file", "map.yaml").get_parameter_value().string_value
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

        self.costmap_pub = self.create_publisher(OccupancyGrid, "/cost_map", 10)
        self.path_pub = self.create_publisher(Path, "/path", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.initial_pose_callback, 10
        )
        self.create_subscription(PoseStamped, "/move_base_simple/goal", self.goal_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.original_image, self.resolution, self.origin = self.load_map_file(
            self.map_file_path
        )
        black_pixels = self.find_black_pixels(self.original_image)
        self.enlarged_image, _ = self.enlarge_black_pixels(
            self.original_image, black_pixels, self.robot_radius
        )
        self.publish_initial_cost_map(self.enlarged_image)

    # def __del__(self):
        # pass

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
                        if (selected_x, selected_y) not in new_black_pixels:
                            new_image.putpixel((selected_x, selected_y), 0)
                            new_black_pixels.add((selected_x, selected_y))

        return new_image, new_black_pixels

    def publish_initial_cost_map(self, image):
        while True:
            if image is not None and self.costmap_pub.get_num_connections() > 0:
                self.publish_cost_map(image)
                break
            self.get_logger().info("Waiting for subscribers to /cost_map...")
            rospy.sleep(Duration(seconds=5.0))

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
                self.get_logger().warn(
                    "Cannot set initial pose automatically. Please set initial pose manually."
                )
                return
        
        # elif self.enlarged_image is None:
            # rospy.logwarn("Map is not loaded. Please wait for the map to be loaded.")
            # return

        self.end_point = self.pose_to_pixel((msg.pose.position.x, msg.pose.position.y))
        # rospy.loginfo("Set goal.")

        path = self.make_plan(self.start_point, self.end_point)
        
        if path is None:
            self.get_logger().info("No valid path found.")
            return

        self.get_logger().info("Path found.")

        final_orientation = self.quaternion_to_euler(msg.pose.orientation)
        final_pose = (msg.pose.position.x, msg.pose.position.y, final_orientation[2])

        self.publish_cost_map(self.enlarged_image, path)

        self.navigate_along_path(path, final_pose)

        # self.navigate_along_path(path, msg)

        # self.start_point = None
        # self.end_point = None

    def odom_callback(self, msg):
        # self.current_odom = msg
        pass

    def scan_callback(self, msg):
        # self.scan_data = []
        # self.range_min = msg.range_min

        # for angle in range(0, 31):
            # self.scan_data.append(msg.ranges[angle])

        # for angle in range(330, 360):
            # self.scan_data.append(msg.ranges[angle])
        pass

    def pose_to_pixel(self, pose):
        '''
        origin_x = self.origin[0]
        origin_y = self.origin[1]
        pixel_x = int(round((pose[0] - origin_x) / self.resolution))
        pixel_y = self.original_image.height - int(
            round((pose[1] - origin_y) / self.resolution)
        )

        return (pixel_x, pixel_y)
        '''
        x = pose[0] - self.origin[0]
        y = pose[1] - self.origin[1]
        pixel_x = int(round(x / self.resolution))
        pixel_y = int(round((self.origin[1] + self.resolution * self.original_image.height - y) / self.resolution))
        return pixel_x, pixel_y

    def pixel_to_pose(self, pixel):
        '''
        origin_x = self.origin[0]
        origin_y = self.origin[1]
        pose_x = origin_x + (pixel[0] + 0.5) * self.resolution
        pose_y = (
            origin_y + (self.original_image.height - pixel[1] - 0.5) * self.resolution
        )

        return (pose_x, pose_y)
        '''
        x = pixel[0] * self.resolution + self.origin[0]
        y = (self.original_image.height - pixel[1]) * self.resolution + self.origin[1]
        return x, y

    def make_plan(self, start, goal):
        '''
        rospy.loginfo("[Make Plan] Start planning.")
        self.animation_running = True
        timer = rospy.Timer(rospy.Duration(0.1), self.animate_loading)
        corners = []
        corners.append(start)
        corners = self.find_corners(self.enlarged_image, corners)
        corners.append(goal)

        valid_edges = self.get_valid_edges(self.enlarged_image, corners)

        shortest_path_edges = self.calculate_shortest_path(corners, valid_edges)

        if shortest_path_edges is not None:
            shortest_path_edges = self.aligne_path(shortest_path_edges, start)

            self.publish_path(shortest_path_edges)

            self.publish_cost_map(self.enlarged_image, shortest_path_edges)

            path_graph_image = self.draw_path_with_markers(
                self.enlarged_image, corners, valid_edges
            )
            optimized_path_image_enlarged = self.draw_path_with_markers(
                self.enlarged_image, corners, shortest_path_edges
            )
            optimized_path_image_original = self.draw_path_with_markers(
                self.original_image, corners, shortest_path_edges
            )

            self.original_image.save(self.test_folder_path + "/original.png")
            self.enlarged_image.save(self.test_folder_path + "/enlarged.png")
            path_graph_image.save(self.test_folder_path + "/path_graph.png")
            optimized_path_image_enlarged.save(
                self.test_folder_path + "/optimized_path_enlarged.png"
            )
            optimized_path_image_original.save(
                self.test_folder_path + "/optimized_path_original.png"
            )

            self.animation_running = False
            timer.shutdown()
            sys.stdout.write("\r" + " " * 30 + "\r")
            rospy.loginfo("[Make Plan] Finished planning successfully.")

        return shortest_path_edges
        '''
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

    '''
    def animate_loading(self, event):
        if self.animation_running:
            chars = "/—\\|"
            char = chars[int(event.current_real.to_sec() * 10) % len(chars)]
            sys.stdout.write("\r" + "Planning... " + char)
            sys.stdout.flush()
    '''

    def find_corners(self, image, corners):
        '''
        numpy_image = numpy.array(image)
        _, binary = cv2.threshold(numpy_image, 1, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            epsilon = self.epsilon_factor * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            for point in approx:
                corner = tuple(point[0])
                corners.append(corner)

        return corners
        '''
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
        '''
        valid_edges = []

        for i in range(len(corners)):
            for j in range(i + 1, len(corners)):
                if not self.check_line_crossing(image, corners[i], corners[j]):
                    valid_edges.append((corners[i], corners[j]))
                    valid_edges.append((corners[j], corners[i]))

        return valid_edges
        '''
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

    '''
    def check_line_crossing(self, image, start, end):
        start_pixels = self.get_surrounding_pixels(start)
        end_pixels = self.get_surrounding_pixels(end)
        x0, y0 = start
        x1, y1 = end
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        err = dx - dy
        not_all_black = False
        not_all_white = False
        is_horizontal_or_vertical = x0 == x1 or y0 == y1

        while True:
            if (x, y) not in start_pixels and (x, y) not in end_pixels:
                pixel = image.getpixel((x, y))

                if pixel != 0:
                    not_all_black = True
                if pixel != 254:
                    not_all_white = True

                if is_horizontal_or_vertical:
                    if x0 == x1:
                        left_pixel = image.getpixel((x - 1, y)) if x > 0 else 0
                        right_pixel = (
                            image.getpixel((x + 1, y)) if x < image.width - 1 else 0
                        )

                        if left_pixel != 254 and right_pixel != 254:
                            not_all_black = True
                    else:
                        top_pixel = image.getpixel((x, y - 1)) if y > 0 else 0
                        bottom_pixel = (
                            image.getpixel((x, y + 1)) if y < image.height - 1 else 0
                        )

                        if top_pixel != 254 and bottom_pixel != 254:
                            not_all_black = True

            if x == x1 and y == y1:
                break

            e2 = 2 * err

            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

        if not is_horizontal_or_vertical:
            not_all_black = True

        return not_all_black and not_all_white
    '''
    
    '''
    def get_surrounding_pixels(self, point):
        x, y = point
        pixels = []

        for i in range(-1, 2):
            for j in range(-1, 2):
                pixels.append((x + i, y + j))

        return pixels
    '''
    
    def calculate_shortest_path(self, corners, valid_edges):
        '''
        model = mip.Model()
        model.verbose = int(self.verbose)
        x = {(i, j): model.add_var(var_type=mip.BINARY) for i, j in valid_edges}
        start_point = corners[0]
        end_point = corners[-1]

        model += mip.xsum(x[i, j] for i, j in valid_edges if i == start_point) == 1
        model += mip.xsum(x[i, j] for i, j in valid_edges if j == start_point) == 0
        model += mip.xsum(x[i, j] for i, j in valid_edges if i == end_point) == 0
        model += mip.xsum(x[i, j] for i, j in valid_edges if j == end_point) == 1

        for k in corners[1:-1]:
            model += mip.xsum(x[i, j] for i, j in valid_edges if j == k) == mip.xsum(
                x[i, j] for i, j in valid_edges if i == k
            )

        model.objective = mip.minimize(
            mip.xsum(x[i, j] * self.euclidean_distance(i, j) for i, j in valid_edges)
        )

        model.optimize()

        if model.status == mip.OptimizationStatus.OPTIMAL:
            shortest_path_edges = [(i, j) for i, j in valid_edges if x[i, j].x >= 0.99]

            if not shortest_path_edges:
                rospy.logerr("Optimization succeeded, but no valid path was found.")
                return None

            return shortest_path_edges
        else:
            rospy.logerr("Optimization failed. No path found.")
            return None
        '''
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

    '''
    def euclidean_distance(self, point1, point2):
        return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    def aligne_path(self, edges, start):
        aligned_edges = []
        edge_map = {e[0]: e[1] for e in edges}
        current_point = start

        while current_point in edge_map:
            next_point = edge_map[current_point]
            aligned_edges.append((current_point, next_point))
            current_point = next_point

        return aligned_edges

    def publish_path(self, aligned_edges):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        start_edge = aligned_edges[0]
        start_pose = self.pixel_to_pose((start_edge[0][0], start_edge[0][1]))

        start_pose_stamped = PoseStamped()
        start_pose_stamped.header = path_msg.header
        start_pose_stamped.pose.position.x = start_pose[0]
        start_pose_stamped.pose.position.y = start_pose[1]
        start_pose_stamped.pose.orientation.w = 1.0
        path_msg.poses.append(start_pose_stamped)

        for edge in aligned_edges:
            end_pose = self.pixel_to_pose((edge[1][0], edge[1][1]))

            end_pose_stamped = PoseStamped()
            end_pose_stamped.header = path_msg.header
            end_pose_stamped.pose.position.x = end_pose[0]
            end_pose_stamped.pose.position.y = end_pose[1]
            end_pose_stamped.pose.orientation.w = 1.0
            path_msg.poses.append(end_pose_stamped)

        self.path_pub.publish(path_msg)

    def draw_path_with_markers(self, image, corners, aligned_edges):
        rgb_image = image.convert("RGB")
        draw = ImageDraw.Draw(rgb_image)

        for start, end in aligned_edges:
            draw.line([start, end], fill=(255, 0, 0), width=1)

        start_point = corners[0]
        end_point = corners[-1]
        radius = 5
        draw.ellipse(
            (
                start_point[0] - radius,
                start_point[1] - radius,
                start_point[0] + radius,
                start_point[1] + radius,
            ),
            fill=(0, 255, 0),
        )
        draw.ellipse(
            (
                end_point[0] - radius,
                end_point[1] - radius,
                end_point[0] + radius,
                end_point[1] + radius,
            ),
            fill=(0, 0, 255),
        )

        return rgb_image

    '''
    
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
        '''
        rospy.loginfo("[Navigate] Start navigation.")
        if edges is None or self.current_odom is None or self.scan_data is None:
            return

        for edge in edges:
            rospy.loginfo(f"[Navigate] Moving to waypoint: {edge[1]}.")
            if not self.move_to_goal(edge):
                rospy.logwarn("Aborting navigation.")
                return
            rospy.loginfo("[Navigate] Reached waypoint.")

        if final_pose is not None:
            if not self.rotate_to_final_pose(final_pose):
                rospy.logwarn("Aborting navigation.")
                return
            rospy.loginfo("[Navigate] Reached final pose.")
        '''
        for edge in edges:
            if not self.move_to_goal(edge):
                return

        if final_pose is not None:
            self.rotate_to_final_pose(final_pose)

    def move_to_goal(self, edge):
        '''
        current_position = self.get_current_position()
        if current_position is None:
            return False
        end_position = self.pixel_to_pose((edge[1][0], edge[1][1]))
        goal_distance = math.sqrt(
            (end_position[0] - current_position[0]) ** 2
            + (end_position[1] - current_position[1]) ** 2
        )

        while goal_distance > self.goal_tolerance:
            if not self.rotate_to_goal(edge):
                return False

            current_position = self.get_current_position()
            if current_position is None:
                return False
            goal_distance = math.sqrt(
                (end_position[0] - current_position[0]) ** 2
                + (end_position[1] - current_position[1]) ** 2
            )

            if self.check_obstacle():
                rospy.logwarn_throttle(3.0, "Obstacle detected. Stopping the robot.")
                self.send_stop()
                continue

            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = self.vel_linear
            self.cmd_vel_pub.publish(cmd_vel_msg)

            rospy.sleep(0.1)

        self.send_stop()
        return True
        '''
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

    def rotate_to_goal(self, edge):
        '''
        rotate_flag = False
        current_position = self.get_current_position()
        if current_position is None:
            return False
        end_position = self.pixel_to_pose((edge[1][0], edge[1][1]))
        goal_angle = math.atan2(
            end_position[1] - current_position[1], end_position[0] - current_position[0]
        )
        current_angle = self.get_current_angle()
        if current_angle is None:
            return False
        diff_angle = self.normalize_angle(goal_angle - current_angle)
        current_speed = self.vel_theta

        while abs(diff_angle) > self.angle_tolerance:
            rotate_flag = True
            current_angle = self.get_current_angle()
            if current_angle is None:
                return False
            diff_angle = self.normalize_angle(goal_angle - current_angle)
            rotation_time = abs(diff_angle) / current_speed
            rotation_direction = 1 if diff_angle > 0 else -1

            if self.check_obstacle():
                rospy.logwarn_throttle(3.0, "Obstacle detected. Stopping the robot.")
                self.send_stop()
                continue

            cmd_vel_msg = Twist()
            cmd_vel_msg.angular.z = rotation_direction * current_speed
            self.cmd_vel_pub.publish(cmd_vel_msg)
            rospy.sleep(rotation_time)

            current_speed *= 0.8

        if rotate_flag:
            self.send_stop()

        return True
        '''
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
        '''
        final_angle = tf.transformations.euler_from_quaternion(
            [
                final_pose.pose.orientation.x,
                final_pose.pose.orientation.y,
                final_pose.pose.orientation.z,
                final_pose.pose.orientation.w,
            ]
        )[2]
        current_angle = self.get_current_angle()
        if current_angle is None:
            return False
        diff_angle = self.normalize_angle(final_angle - current_angle)
        current_speed = self.vel_theta

        while abs(diff_angle) > self.angle_tolerance:
            current_angle = self.get_current_angle()
            if current_angle is None:
                return False
            diff_angle = self.normalize_angle(final_angle - current_angle)
            rotation_time = abs(diff_angle) / current_speed
            rotation_direction = 1 if diff_angle > 0 else -1

            cmd_vel_msg = Twist()
            cmd_vel_msg.angular.z = rotation_direction * current_speed
            self.cmd_vel_pub.publish(cmd_vel_msg)
            rospy.sleep(rotation_time)

            current_speed *= 0.8

        self.send_stop()
        return True
        '''
        self.get_logger().info("Rotating to final pose.")
        self.rotate_to_goal(final_pose)

    '''
    def send_stop(self):
        start_time = rospy.get_time()
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.angular.z = 0.0

        while True:
            self.cmd_vel_pub.publish(cmd_vel_msg)

            current_velocity = math.sqrt(
                self.current_odom.twist.twist.linear.x**2
                + self.current_odom.twist.twist.linear.y**2
                + self.current_odom.twist.twist.angular.z**2
            )

            if current_velocity < 0.01:
                break
            elif rospy.get_time() - start_time > 10:
                rospy.logwarn_throttle(
                    3.0,
                    "It took more than 10 seconds to stop. Please check if the odometry is working properly.",
                )

            rospy.sleep(0.1)

    def check_obstacle(self):
        if self.robot_radius < self.range_min:
            rospy.logwarn(
                "robot_radius is smaller than range_min. Please check the parameters."
            )
            return False

        for distance in self.scan_data:
            if self.range_min < distance < self.robot_radius:
                return True

        return False

    def get_current_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                self.current_odom.child_frame_id,
                rospy.Time(0),
                rospy.Duration(1.0),
            )

            return (
                transform.transform.translation.x,
                transform.transform.translation.y,
            )
        except:
            rospy.logwarn("Failed to get current position.")
            return None

    def get_current_angle(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                self.current_odom.child_frame_id,
                rospy.Time(0),
                rospy.Duration(1.0),
            )
            quaternion = transform.transform.rotation
            euler = tf.transformations.euler_from_quaternion(
                [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
            )

            return euler[2]
        except:
            rospy.logwarn("Failed to get current angle.")
            return None

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi

        return angle
    '''


def main(args=None):
    # rospy.init_node("vgraph_planner_node")
    # VgraphPlannerNode()
    # rospy.spin()
    rclpy.init(args=args)
    node = VgraphPlannerNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
