# Copyright 2025 NXP

# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from rclpy.action import ActionClient
from rclpy.parameter import Parameter

import math
import time
import numpy as np
import cv2
from typing import Optional, Tuple
import asyncio
import threading
from pyzbar import pyzbar

from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import CompressedImage

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import BehaviorTreeLog
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from synapse_msgs.msg import Status
from synapse_msgs.msg import WarehouseShelf

from scipy.ndimage import label, center_of_mass
from scipy.spatial.distance import euclidean
from sklearn.decomposition import PCA

import tkinter as tk
from tkinter import ttk


QOS_PROFILE_DEFAULT = 10
SERVER_WAIT_TIMEOUT_SEC = 5.0

PROGRESS_TABLE_GUI = True


class WindowProgressTable:
	def __init__(self, root, shelf_count):
		self.root = root
		self.root.title("Shelf Objects & QR Link")
		self.root.attributes("-topmost", True)

		self.row_count = 2
		self.col_count = shelf_count

		self.boxes = []
		for row in range(self.row_count):
			row_boxes = []
			for col in range(self.col_count):
				box = tk.Text(root, width=10, height=3, wrap=tk.WORD, borderwidth=1,
					      relief="solid", font=("Helvetica", 14))
				box.insert(tk.END, "NULL")
				box.grid(row=row, column=col, padx=3, pady=3, sticky="nsew")
				row_boxes.append(box)
			self.boxes.append(row_boxes)

		# Make the grid layout responsive.
		for row in range(self.row_count):
			self.root.grid_rowconfigure(row, weight=1)
		for col in range(self.col_count):
			self.root.grid_columnconfigure(col, weight=1)

	def change_box_color(self, row, col, color):
		self.boxes[row][col].config(bg=color)

	def change_box_text(self, row, col, text):
		self.boxes[row][col].delete(1.0, tk.END)
		self.boxes[row][col].insert(tk.END, text)

box_app = None
def run_gui(shelf_count):
	global box_app
	root = tk.Tk()
	box_app = WindowProgressTable(root, shelf_count)
	root.mainloop()


class WarehouseExplore(Node):
	""" Initializes warehouse explorer node with the required publishers and subscriptions.

		Returns:
			None
	"""
	def __init__(self):
		super().__init__('warehouse_explore')

		self.action_client = ActionClient(
			self,
			NavigateToPose,
			'/navigate_to_pose')

		self.subscription_pose = self.create_subscription(
			PoseWithCovarianceStamped,
			'/pose',
			self.pose_callback,
			QOS_PROFILE_DEFAULT)

		self.subscription_global_map = self.create_subscription(
			OccupancyGrid,
			'/global_costmap/costmap',
			self.global_map_callback,
			QOS_PROFILE_DEFAULT)

		self.subscription_simple_map = self.create_subscription(
			OccupancyGrid,
			'/map',
			self.simple_map_callback,
			QOS_PROFILE_DEFAULT)

		self.subscription_status = self.create_subscription(
			Status,
			'/cerebri/out/status',
			self.cerebri_status_callback,
			QOS_PROFILE_DEFAULT)

		self.subscription_behavior = self.create_subscription(
			BehaviorTreeLog,
			'/behavior_tree_log',
			self.behavior_tree_log_callback,
			QOS_PROFILE_DEFAULT)

		self.subscription_shelf_objects = self.create_subscription(
			WarehouseShelf,
			'/shelf_objects',
			self.shelf_objects_callback,
			QOS_PROFILE_DEFAULT)

		# Subscription for camera images.
		self.subscription_camera = self.create_subscription(
			CompressedImage,
			'/camera/image_raw/compressed',
			self.camera_image_callback,
			QOS_PROFILE_DEFAULT)

		self.publisher_joy = self.create_publisher(
			Joy,
			'/cerebri/in/joy',
			QOS_PROFILE_DEFAULT)

		# Publisher for output image (for debug purposes).
		self.publisher_qr_decode = self.create_publisher(
			CompressedImage,
			"/debug_images/qr_code",
			QOS_PROFILE_DEFAULT)

		self.publisher_shelf_data = self.create_publisher(
			WarehouseShelf,
			"/shelf_data",
			QOS_PROFILE_DEFAULT)

		self.declare_parameter('shelf_count', 1)
		self.declare_parameter('initial_angle', 0.0)

		self.shelf_count = \
			self.get_parameter('shelf_count').get_parameter_value().integer_value
		self.initial_angle = \
			self.get_parameter('initial_angle').get_parameter_value().double_value

		# --- Robot State ---
		self.armed = False
		self.logger = self.get_logger()

		# --- Robot Pose ---
		self.pose_curr = PoseWithCovarianceStamped()
		self.buggy_pose_x = 0.0
		self.buggy_pose_y = 0.0
		self.buggy_center = (0.0, 0.0)
		self.world_center = (0.0, 0.0)

		# --- Map Data ---
		self.simple_map_curr = None
		self.global_map_curr = None

		# --- Goal Management ---
		self.xy_goal_tolerance = 0.25
		self.yaw_goal_tolerance = 0.35
		self.goal_completed = True  # No goal is currently in-progress.
		self.goal_handle_curr = None
		self.cancelling_goal = False
		self.recovery_threshold = 10

		# --- Goal Creation ---
		self._frame_id = "map"

		# --- Exploration Parameters ---
		self.max_step_dist_world_meters = 7.0
		self.min_step_dist_world_meters = 4.0
		self.full_map_explored_count = 0

		# --- QR Code Data ---
		self.qr_code_str = "Empty"
		if PROGRESS_TABLE_GUI:
			self.table_row_count = 0
			self.table_col_count = 0

		# --- Shelf Data ---
		self.shelf_objects_curr = WarehouseShelf()

		self.NAVIGATE_TO_SHELF = 0
		self.CAPTURE_OBJECTS = 1
		self.NAVIGATE_TO_QR_SIDE = 2
		self.SCAN_QR = 3
		self.PROCESS_DATA = 4
		self.EXPLORATION_COMPLETE = 5
		self.DO_NOTHING = 6
		self.EXPLORE = 7
		self.current_shelf_centre = None
		self.current_shelf_orientation = None
		self.shelf_info = None

		self.current_state = -1
		self.current_shelf_id	= 1
		self.current_angle = self.initial_angle
		self.world_centre = (150,150)
		self.navigation_started = False
		self.current_shelf_objects = None
		self.should_detect_objects = False
		self.should_detect_qr = False
		
		# Data storage
		self.current_qr_data = None
		self.qr_code_str = ""
		
		# Detection timers
		self.detection_start_time = 0
		self.qr_scan_start_time = 0
		self.qr_reached = False
		self.pyzbar = pyzbar
		self.qr_detection_available = True
		self.goal_sent = False

		self.state_timer = self.create_timer(1.0, self.state_machine_update)

	
	def pose_callback(self, message):
		"""Callback function to handle pose updates.

		Args:
			message: ROS2 message containing the current pose of the rover.

		Returns:
			None
		"""
		self.pose_curr = message
		self.buggy_pose_x = message.pose.pose.position.x
		self.buggy_pose_y = message.pose.pose.position.y
		self.buggy_center = (self.buggy_pose_x, self.buggy_pose_y)

	def simple_map_callback(self, message):
		"""Callback function to handle simple map updates.

		Args:
			message: ROS2 message containing the simple map data.

		Returns:
			None
		"""
		self.simple_map_curr = message
		map_info = self.simple_map_curr.info
		self.world_center = self.get_world_coord_from_map_coord(
			map_info.width / 2, map_info.height / 2, map_info
		)
	
	def state_machine_update(self):
		"""Main state machine logic"""
		# if self.current_shelf_id == 3:
		# 	height, width = self.global_map_curr.info.height, self.global_map_curr.info.width
		# 	map_array = np.array(self.global_map_curr.data).reshape((height, width))
		# 	np.save('warehouse.npy', map_array)
		if self.current_state == self.EXPLORE:
			self.frontier_explore()
		elif self.current_state == self.NAVIGATE_TO_SHELF:
			# height, width = self.global_map_curr.info.height, self.global_map_curr.info.width
			# map_array = np.array(self.global_map_curr.data).reshape((height, width))
			# np.save('after_front_global.npy', map_array)
			self.get_logger().info('NAVIGATING')
			self.handle_nav_to_shelf()
		elif self.current_state == self.CAPTURE_OBJECTS:
			self.get_logger().info('CAPTURING')
			self.handle_capture_objects()
		elif self.current_state == self.SCAN_QR:
			self.should_detect_qr = True
			self.get_logger().info('SCANNING')
			self.handle_navigate_to_qr_side()
			self.handle_SCAN_QR()
		elif self.current_state == self.DO_NOTHING:
			self.get_logger().info("Waiting for next command...")
			# Do nothing, just wait for the next command
		
		
	def is_free_space(self, map_array, point):
		"""Check if a point is free space in the occupancy grid"""
		x, y = int(point[0]), int(point[1])
		if 0 <= x < map_array.shape[1] and 0 <= y < map_array.shape[0]:
			return map_array[y, x] == 0
		return False
	
	def handle_navigate_to_qr_side(self):
		"""State 3: Move to side of shelf to scan QR code"""
		if not self.goal_completed or self.qr_reached:
			self.get_logger().info("Still moving ......")
			return  # Still moving
		self.get_logger().info(f"\nMoving to side of shelf {self.current_shelf_id} for QR scan")
		map_array = np.array(self.global_map_curr.data).reshape((self.global_map_curr.info.height, self.global_map_curr.info.width))
		if self.current_shelf_centre is None or self.current_shelf_orientation is None:
			target_point, shelf_info = self.find_shelf_and_target_point(slam_map= map_array, robot_pos=self.world_centre, shelf_angle_deg=self.current_angle, search_distance=400)
			if target_point is not None:
				self.get_logger().info(f"Found target point: {target_point} shelf info: {shelf_info}")
				self.current_shelf_centre = shelf_info['center']
				self.current_shelf_orientation = shelf_info['rotation_angle']

		left,right = self.find_front_back_points(self.current_shelf_centre, self.shelf_info, 55,True)
		self.get_logger().info(f"Left point: {left}, Right point: {right}")

		# choose left or right which is safe and navigate there
		curr_angle = self.current_shelf_orientation
		curr_robot_angle = self.get_current_robot_yaw()
		curr_robot_angle = math.degrees(curr_robot_angle)

		#find angle from a direction
		direction = self.shelf_info['orientation']['primary_direction']
		angle = math.degrees(math.atan2(direction[1], direction[0]))
		if curr_robot_angle < 0:
			curr_robot_angle += 360
		# Choose the point (left or right) with the shortest distance to the robot
		dist_left = self.calc_distance(self.buggy_center, left)
		dist_right = self.calc_distance(self.buggy_center, right)

		if dist_left < dist_right:
			goal_x, goal_y = float(left[0]), float(left[1])
			yaw = angle + 180
		else:
			goal_x, goal_y = float(right[0]), float(right[1])
			yaw = angle
		goal_x, goal_y = self.get_world_coord_from_map_coord(goal_x, goal_y, self.global_map_curr.info)	
		goal = self.create_goal_from_world_coord(goal_x, goal_y, math.radians(yaw))
		if self.send_goal_from_world_pose(goal):
			self.get_logger().info(f"Goal sent to ({goal_x:.2f}, {goal_y:.2f}) with yaw {yaw:.2f} degree")
			self.qr_reached = True
		else:
			self.get_logger().error("Failed to send navigation goal!")
			self.goal_sent = False
			self.qr_reached = False
			self.current_state = self.DO_NOTHING
		# if np.abs(yaw-curr_robot_angle) > 10:
		# 	goal_x, goal_y = self.adjust_robot_orientation(yaw)
		# 	self.get_logger().info(f'\nmap cords of goal: ({goal_x:.2f}, {goal_y:.2f})')
		# 	goal = self.create_goal_from_world_coord(goal_x, goal_y, math.radians(yaw))
		# 	if self.send_goal_from_world_pose(goal):
		# 		self.get_logger().info(f"ADJUSTED goal sent to ({goal_x:.2f}, {goal_y:.2f})")
		# 	else:
		# 		self.get_logger().error("Failed to send navigation goal!")


	def handle_SCAN_QR(self):
		"""State 4: Capture QR code"""
		if not self.goal_completed or self.qr_reached == False:
			return  # Still moving to QR position
		self.get_logger().info("Starting QR code scan...")
		if not self.should_detect_qr:
			
			self.should_detect_qr = True
			self.current_qr_data = None
			self.qr_scan_start_time = time.time()
			return
		
		# Check if we have QR data or timeout
		if self.current_qr_data is not None:
			self.should_detect_qr = False
			self.get_logger().info(f"\n\nQR Code captured: {self.current_qr_data}")
			self.shelf_objects_curr.qr_decoded = self.current_qr_data
			self.get_logger().info(f'Shelf objects current: {self.shelf_objects_curr}')
			#publish shel current
			self.publisher_shelf_data.publish(self.shelf_objects_curr)
			self.get_logger().info("Published shelf data with QR code")
			self.current_shelf_objects = None  # Reset objects after QR scan
			self.qr_reached = False  # Reset QR reached flag

			# reset next goals
			self.shelf_objects_curr = WarehouseShelf()
			self.current_angle = self.parse_qr_for_next_angle(self.current_qr_data)
			self.world_centre = self.current_shelf_centre
			self.current_shelf_id +=1
			self.current_shelf_centre = None
			self.current_shelf_orientation = None
			self.logger.info(f"Next shelf ID: {self.current_shelf_id}, Next angle: {self.current_angle}°")
			self.logger.info(f"World centre: {self.world_centre}, Current shelf centre: {self.current_shelf_centre}")
			self.logger.info(f"Shelf info new: {self.shelf_info}")
			self.current_state = self.EXPLORE
			
			
		elif time.time() - self.qr_scan_start_time > 5.0:  # 15 second timeout
			self.should_detect_qr = False
			self.get_logger().warn("QR scan timeout")
			self.current_state = self.DO_NOTHING
		
	def parse_qr_for_next_angle(self, qr_data):
		"""Parse QR code to extract next shelf angle"""
		try:
			parts = qr_data.split('_')
			if len(parts) >= 2:
				next_angle = float(parts[1])
				return next_angle
		except (ValueError, IndexError):
			self.get_logger().error(f"Failed to parse QR code: {qr_data}")
		
		return None


	def get_yaw_from_quaternion(self, quaternion):
		x = quaternion.x
		y = quaternion.y
		z = quaternion.z
		w = quaternion.w
		yaw = math.atan2(
			2.0 * (w * z + x * y),
			1.0 - 2.0 * (y * y + z * z)
		)
		return yaw

	def get_current_robot_yaw(self):
		"""Get current robot orientation as yaw angle"""
		if self.pose_curr is not None:
			orientation = self.pose_curr.pose.pose.orientation
			yaw_radians = self.get_yaw_from_quaternion(orientation)
			return yaw_radians  
		return None

	def calc_distance(self, point1, point2):
		return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

	def adjust_robot_orientation(self, angle):
		curr_robot_angle = self.get_current_robot_yaw()
		if curr_robot_angle is not None:
			# move back to 10 units
			goal_x = self.buggy_pose_x - .5 * math.cos(angle)
			goal_y = self.buggy_pose_y - .5 * math.sin(angle)
			return goal_x, goal_y
		return None, None

	def handle_nav_to_shelf(self):
		"""State 0: Navigate to the shelf"""
		if not self.goal_completed:
			return
		
		self.get_logger().info(f"Navigating to shelf number {self.current_shelf_id} at angle {self.current_angle}°")
		map_array = np.array(self.global_map_curr.data).reshape((self.global_map_curr.info.height, self.global_map_curr.info.width))
		target_point, shelf_info = self.find_shelf_and_target_point(slam_map= map_array, robot_pos=self.world_centre, shelf_angle_deg=self.current_angle, search_distance=400)
		self.get_logger().info(f"Target point: {target_point}")	
		self.shelf_info = shelf_info
		# If a target point is found, navigate to it
		if target_point is not None:
			self.get_logger().info(f"Found target point: {target_point} shelf info: {shelf_info}")
			self.current_shelf_centre = shelf_info['center']
			self.current_shelf_orientation = shelf_info['rotation_angle']
			curr_robot_angle = self.get_current_robot_yaw()
			curr_robot_angle = math.degrees(curr_robot_angle)
			if curr_robot_angle < 0:
				curr_robot_angle += 360
			yaw = self.current_angle
			buggy_mapcoord = self.get_map_coord_from_world_coord(self.buggy_pose_x, self.buggy_pose_y, self.global_map_curr.info)
			front,back = self.find_front_back_points(self.current_shelf_centre, shelf_info, 50,False)
			self.get_logger().info(f"Front Robot position in map coordinates: {buggy_mapcoord}")
			self.get_logger().info(f"Front point: {front}, Back point: {back}")
			self.get_logger().info(f"Distance to front: {self.calc_distance(buggy_mapcoord, front):.2f}, Back: {self.calc_distance(buggy_mapcoord, back):.2f}")
			self.get_logger().info(f"Shelf: {self.current_angle} Robot: {curr_robot_angle} diff : {self.current_angle - curr_robot_angle}")
			direction = self.shelf_info['orientation']['secondary_direction']
			angle = math.degrees(math.atan2(direction[1], direction[0]))
			if self.calc_distance(buggy_mapcoord, front) < self.calc_distance(buggy_mapcoord, back):
				goal_x, goal_y = float(front[0]), float(front[1])
				yaw = angle
				self.get_logger().info(f'\nNavigating to Front point: ({goal_x:.2f}, {goal_y:.2f}) with yaw {yaw:.2f}°')
			else:
				goal_x, goal_y = float(back[0]), float(back[1])
				yaw = angle + 180
				self.get_logger().info(f'\nNavigating to Back point: ({goal_x:.2f}, {goal_y:.2f}) with yaw {yaw:.2f}°')
			
			if (self.calc_distance(buggy_mapcoord, front) < 5 or self.calc_distance(buggy_mapcoord, back) < 5):
				self.get_logger().info("\n\nRobot is aligned with shelf\n\n")
				self.current_state = self.CAPTURE_OBJECTS
			elif self.calc_distance(buggy_mapcoord, front) < 40 or self.calc_distance(buggy_mapcoord, back) < 40:
				
				if self.calc_distance(buggy_mapcoord, front) < self.calc_distance(buggy_mapcoord, back):
					goal_x, goal_y = float(front[0]), float(front[1])
					yaw = angle
					self.get_logger().info(f'\nNavigating to Front point: ({goal_x:.2f}, {goal_y:.2f}) with yaw {yaw:.2f}°')
				else:
					goal_x, goal_y = float(back[0]), float(back[1])
					yaw = angle + 180
					self.get_logger().info(f'\nNavigating to Back point: ({goal_x:.2f}, {goal_y:.2f}) with yaw {yaw:.2f}°')
				self.get_logger().info(f'\nmap cords of goal: ({goal_x:.2f}, {goal_y:.2f})')	
				goal_x, goal_y = self.get_world_coord_from_map_coord(goal_x, goal_y, self.global_map_curr.info)
				goal = self.create_goal_from_world_coord(goal_x, goal_y, math.radians(yaw))
				if self.send_goal_from_world_pose(goal):
					self.get_logger().info(f"IDEAL Goal sent to ({goal_x:.2f}, {goal_y:.2f})")
				else:
					self.get_logger().error("Failed to send navigation goal!")
			else:
				# If the robot is far from the shelf, navigate to the target point
				goal_x, goal_y = float(target_point[0]), float(target_point[1])
				self.get_logger().info(f'\nmap cords of goal: ({goal_x:.2f}, {goal_y:.2f})')
				goal_x, goal_y = self.get_world_coord_from_map_coord(goal_x, goal_y, self.global_map_curr.info)
				goal = self.create_goal_from_world_coord(goal_x, goal_y, math.radians(yaw))

				if self.send_goal_from_world_pose(goal):
					self.get_logger().info(f"TEMPORARY Goal sent to ({goal_x:.2f}, {goal_y:.2f})")
					self.goal_sent = False
				else:
					self.get_logger().error("Failed to send navigation goal!")
		else:
			print("Use frontier logic here")


	
	def handle_capture_objects(self):
		"""State 2: Capture objects from the shelf"""
		if not self.goal_completed:
			return  # Still navigating to shelf
		if not self.should_detect_objects:
			self.get_logger().info("Starting object detection...")
			self.should_detect_objects = True
			self.current_shelf_objects = None
			self.detection_start_time = time.time()
			return
		
		# Check if we have objects or timeout
		if self.current_shelf_objects is not None:
			self.should_detect_objects = False
			self.get_logger().info(f"\n\nObjects detected: {self.current_shelf_objects.object_name}")
			self.shelf_objects_curr.object_name = self.current_shelf_objects.object_name
			self.shelf_objects_curr.object_count = self.current_shelf_objects.object_count
			self.logger.info(f"shelf objects curr: {self.shelf_objects_curr}")
			# self.publisher_shelf_data.publish(self.shelf_objects_curr)
			# self.get_logger().info("Published shelf objects data")
			self.current_state = self.SCAN_QR
			if len(self.shelf_objects_curr.object_name) < 1:
				self.current_state = self.NAVIGATE_TO_SHELF

		elif time.time() - self.detection_start_time > 5.0:  # 5 second timeout
			self.should_detect_objects = False
			self.get_logger().warn("Object detection timeout")
			self.get_logger().info("Moving to QR scanning anyway...")

	def frontier_explore(self):
		"""State 3: Explore the frontier"""
		self.get_logger().info("Exploring frontier...")
		height, width = self.global_map_curr.info.height, self.global_map_curr.info.width
		map_array = np.array(self.global_map_curr.data).reshape((height, width))
		frontiers = self.get_frontiers_for_space_exploration(map_array)
		self.logger.info(f"\nFound {len(frontiers)} frontiers in the map.")
		_, shelf_info = self.find_shelf_and_target_point(slam_map= map_array, robot_pos=self.world_centre, shelf_angle_deg=self.current_angle, search_distance=400)
		self.shelf_info = shelf_info
		self.get_logger().info(f"\n\nworld centre: {self.world_centre}, Current shelf info: {shelf_info}")
		map_info = self.global_map_curr.info
		if frontiers:
			closest_frontier = None
			min_distance_curr = float('inf')
			self.shelf_info = shelf_info
			world_self_center = self.get_world_coord_from_map_coord(
				shelf_info['center'][0],
				shelf_info['center'][1],
				map_info
			)
			if shelf_info is not None:
				world_self_center = self.get_world_coord_from_map_coord(
					shelf_info['center'][0],
					shelf_info['center'][1],
					map_info
				)
			else:
				initial_world_pos = self.get_world_coord_from_map_coord(
					self.current_pos[0], 
					self.current_pos[1], 
					map_info
				)

				distance_to_shelf = 100 
				angle_rad = math.radians(self.initial_angle)

				shelf_direction_x = initial_world_pos[0] + distance_to_shelf * math.cos(angle_rad)
				shelf_direction_y = initial_world_pos[1] + distance_to_shelf * math.sin(angle_rad)

				world_self_center = (
					(initial_world_pos[0] + shelf_direction_x) / 2,
					(initial_world_pos[1] + shelf_direction_y) / 2
				)
			for fy, fx in frontiers:
				fx_world, fy_world = self.get_world_coord_from_map_coord(fx, fy,
											 map_info)
				distance = euclidean((fx_world, fy_world), world_self_center)
				if (distance < min_distance_curr and
				    distance <= self.max_step_dist_world_meters and
				    distance >= self.min_step_dist_world_meters):
					min_distance_curr = distance
					closest_frontier = (fy, fx)

			if closest_frontier:
				fy, fx = closest_frontier
				self.get_logger().info(f'\nFound frontier closest at: ({fx}, {fy})')
				self.get_logger().info(f'World coordinates: ({fx_world}, {fy_world})')
				goal = self.create_goal_from_map_coord(fx, fy, map_info)
				self.send_goal_from_world_pose(goal)
				print("Sending goal for space exploration.")
				self.current_state = self.NAVIGATE_TO_SHELF
				return
			else:
				self.max_step_dist_world_meters += 2.0
				new_min_step_dist = self.min_step_dist_world_meters - 1.0
				self.min_step_dist_world_meters = max(0.25, new_min_step_dist)

			self.full_map_explored_count = 0
		else:
			self.full_map_explored_count += 1
			print(f"Nothing found in frontiers; count = {self.full_map_explored_count}")

	def global_map_callback(self, message):
		"""Callback function to handle global map updates.

		Args:
			message: ROS2 message containing the global map data.

		Returns:
			None
		"""
		self.global_map_curr = message
		height, width = self.global_map_curr.info.height, self.global_map_curr.info.width
		
		if self.current_state == -1:
			self.world_centre = self.get_map_coord_from_world_coord(0,0, self.global_map_curr.info)
			self.get_logger().info(f"World center: {self.world_centre}")
			self.get_logger().info(f"Map size: {width} x {height}")
			self.current_state = self.EXPLORE
			# self.get_logger().info(f"Detected {len(detected_lines)} lines in current direction {self.current_angle}°")

		# if not self.goal_completed:
		# 	return

		
		# save mpy file
		

		# frontiers = self.get_frontiers_for_space_exploration(map_array)

		# map_info = self.global_map_curr.info
		# self.get_logger().info(f'map_info: {map_info}')
		# self.get_logger().info(f'map_size: {width} x {height}')
		# fx = 150
		# fy = 150
		# # Move to a point at initial_angle from the world center at a fixed distance
		# # Move to a point at initial_angle (interpreted as yaw in the XY plane)
		# distance = 40
		# # Convert initial_angle from degrees to radians if necessary
		# angle = math.radians(self.initial_angle)  # This is yaw (rotation about Z, in-plane)
		# goal_x = fx + distance * math.cos(angle)
		# goal_y = fy + distance * math.sin(angle)
		# goal = self.create_goal_from_map_coord(goal_x, goal_y, map_info,angle)
		# self.send_goal_from_world_pose(goal)
		# self.get_logger().info(f"Sent goal at angle {angle} from center ({fx:.2f}, {fy:.2f}) to ({goal_x:.2f}, {goal_y:.2f})")
		# return
	
		# if frontiers:
		# 	closest_frontier = None
		# 	min_distance_curr = float('inf')

		# 	for fy, fx in frontiers:
		# 		fx_world, fy_world = self.get_world_coord_from_map_coord(fx, fy,
		# 									 map_info)
		# 		distance = euclidean((fx_world, fy_world), self.buggy_center)
		# 		if (distance < min_distance_curr and
		# 		    distance <= self.max_step_dist_world_meters and
		# 		    distance >= self.min_step_dist_world_meters):
		# 			min_distance_curr = distance
		# 			closest_frontier = (fy, fx)

		# 	if closest_frontier:
		# 		fy, fx = closest_frontier
		# 		self.get_logger().info(f'Found frontier at: ({fx}, {fy})')
		# 		self.get_logger().info(f'World coordinates: ({fx_world}, {fy_world})')
		# 		goal = self.create_goal_from_map_coord(fx, fy, map_info)
		# 		self.send_goal_from_world_pose(goal)
		# 		print("Sending goal for space exploration.")
		# 		return
		# 	else:
		# 		self.max_step_dist_world_meters += 2.0
		# 		new_min_step_dist = self.min_step_dist_world_meters - 1.0
		# 		self.min_step_dist_world_meters = max(0.25, new_min_step_dist)

		# 	self.full_map_explored_count = 0
		# else:
		# 	self.full_map_explored_count += 1
		# 	print(f"Nothing found in frontiers; count = {self.full_map_explored_count}")


	def get_frontiers_for_space_exploration(self, map_array):
		"""Identifies frontiers for space exploration.

		Args:
			map_array: 2D numpy array representing the map.

		Returns:
			frontiers: List of tuples representing frontier coordinates.
		"""
		frontiers = []
		for y in range(1, map_array.shape[0] - 1):
			for x in range(1, map_array.shape[1] - 1):
				if map_array[y, x] == -1:  # Unknown space and not visited.
					neighbors_complete = [
						(y, x - 1),
						(y, x + 1),
						(y - 1, x),
						(y + 1, x),
						(y - 1, x - 1),
						(y + 1, x - 1),
						(y - 1, x + 1),
						(y + 1, x + 1)
					]

					near_obstacle = False
					for ny, nx in neighbors_complete:
						if map_array[ny, nx] > 0:  # Obstacles.
							near_obstacle = True
							break
					if near_obstacle:
						continue

					neighbors_cardinal = [
						(y, x - 1),
						(y, x + 1),
						(y - 1, x),
						(y + 1, x),
					]

					for ny, nx in neighbors_cardinal:
						if map_array[ny, nx] == 0:  # Free space.
							frontiers.append((ny, nx))
							break

		return frontiers



	def publish_debug_image(self, publisher, image):
		"""Publishes images for debugging purposes.

		Args:
			publisher: ROS2 publisher of the type sensor_msgs.msg.CompressedImage.
			image: Image given by an n-dimensional numpy array.

		Returns:
			None
		"""
		if image.size:
			message = CompressedImage()
			_, encoded_data = cv2.imencode('.jpg', image)
			message.format = "jpeg"
			message.data = encoded_data.tobytes()
			publisher.publish(message)


	def camera_image_callback(self, message):
		"""Callback function to handle incoming camera images.

		Args:
			message: ROS2 message of the type sensor_msgs.msg.CompressedImage.

		Returns:
			None
		"""
		np_arr = np.frombuffer(message.data, np.uint8)
		image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		# Process the image from front camera as needed.
		if self.should_detect_qr:
			qr_codes = self.pyzbar.decode(image)
			
			if qr_codes:
				for qr_code in qr_codes:
					qr_data = qr_code.data.decode('utf-8')
					
					# Validate QR format: 'shelfid_nextangle_uniquecode'
					if self.validate_qr_format(qr_data):
						self.current_qr_data = qr_data
						self.get_logger().info(f"info: {qr_data}")
						# Draw bounding box for visualization
						points = qr_code.polygon
						if len(points) == 4:
							pts = np.array([[point.x, point.y] for point in points], np.int32)
							cv2.polylines(image, [pts], True, (0, 255, 0), 3)
							cv2.putText(image, qr_data, (pts[0][0], pts[0][1] - 10),
									cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
						break
		
		# Always publish debug image
		self.publish_debug_image(self.publisher_qr_decode, image)

	def validate_qr_format(self, qr_data):
		"""Validate QR code format: 'shelfid_nextangle_uniquecode'"""
		parts = qr_data.split('_')
		if len(parts) == 3:
			try:
				shelf_id = int(parts[0])
				next_angle = float(parts[1])
				return True
			except ValueError:
				return False
		return False

	def cerebri_status_callback(self, message):
		"""Callback function to handle cerebri status updates.

		Args:
			message: ROS2 message containing cerebri status.

		Returns:
			None
		"""
		if message.mode == 3 and message.arming == 2:
			self.armed = True
		else:
			# Initialize and arm the CMD_VEL mode.

			# CHANGING TO MANUAL JATAYU : msg.buttons from [0, 1, 0, 0, 0, 0, 0, 1] to [1, 0, 0, 0, 0, 0, 0, 1]

			msg = Joy()
			msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]
			msg.axes = [0.0, 0.0, 0.0, 0.0]
			self.publisher_joy.publish(msg)

	def behavior_tree_log_callback(self, message):
		"""Alternative method for checking goal status.

		Args:
			message: ROS2 message containing behavior tree log.

		Returns:
			None
		"""
		for event in message.event_log:
			if (event.node_name == "FollowPath" and
				event.previous_status == "SUCCESS" and
				event.current_status == "IDLE"):
				# self.goal_completed = True
				# self.goal_handle_curr = None
				pass

	def shelf_objects_callback(self, message):
		"""Callback function to handle shelf objects updates.

		Args:
			message: ROS2 message containing shelf objects data.

		Returns:
			None
		"""
		#self.shelf_objects_curr = message
		# Process the shelf objects as needed.
		if self.should_detect_objects:
			self.current_shelf_objects = message
		
		# self.logger.info("Received shelf objects data:")
		# self.logger.info(f"Object names: {self.shelf_objects_curr.object_name}")
		# self.logger.info(f"Object counts: {self.shelf_objects_curr.object_count}")
		# self.logger.info(f"QR Decoded: {self.shelf_objects_curr.qr_decoded}")
		# How to send WarehouseShelf messages for evaluation.
		"""
		
		* Example for sending WarehouseShelf messages for evaluation.
			shelf_data_message = WarehouseShelf()

			shelf_data_message.object_name = ["car", "clock"]
			shelf_data_message.object_count = [1, 2]
			shelf_data_message.qr_decoded = "test qr string"

			self.publisher_shelf_data.publish(shelf_data_message)

		* Alternatively, you may store the QR for current shelf as self.qr_code_str.
			Then, add it as self.shelf_objects_curr.qr_decoded = self.qr_code_str
			Then, publish as self.publisher_shelf_data.publish(self.shelf_objects_curr)
			This, will publish the current detected objects with the last QR decoded.
		"""

		# Optional code for populating TABLE GUI with detected objects and QR data.
	
		if PROGRESS_TABLE_GUI and self.shelf_objects_curr is not None:
			shelf = self.shelf_objects_curr
			obj_str = ""
			for name, count in zip(shelf.object_name, shelf.object_count):
				obj_str += f"{name}: {count}\n"

			# Ensure indices are within bounds
			if box_app is not None:
				row_count = len(box_app.boxes)
				col_count = len(box_app.boxes[0]) if row_count > 0 else 0

				# Only update if indices are valid
				if self.table_row_count < row_count and self.table_col_count < col_count:
					box_app.change_box_text(self.table_row_count, self.table_col_count, obj_str)
					box_app.change_box_color(self.table_row_count, self.table_col_count, "cyan")
					self.table_row_count += 1

					if self.table_row_count < row_count:
						box_app.change_box_text(self.table_row_count, self.table_col_count, self.qr_code_str)
						box_app.change_box_color(self.table_row_count, self.table_col_count, "yellow")
					self.table_row_count = 0
					self.table_col_count += 1
				else:
					# Reset indices if out of bounds
					self.table_row_count = 0
					self.table_col_count = 0


	def rover_move_manual_mode(self, speed, turn):
		"""Operates the rover in manual mode by publishing on /cerebri/in/joy.

		Args:
			speed: The speed of the car in float. Range = [-1.0, +1.0];
				   Direction: forward for positive, reverse for negative.
			turn: Steer value of the car in float. Range = [-1.0, +1.0];
				  Direction: left turn for positive, right turn for negative.

		Returns:
			None
		"""
		msg = Joy()
		msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]
		msg.axes = [0.0, speed, 0.0, turn]
		self.publisher_joy.publish(msg)



	def cancel_goal_callback(self, future):
		"""
		Callback function executed after a cancellation request is processed.

		Args:
			future (rclpy.Future): The future is the result of the cancellation request.
		"""
		cancel_result = future.result()
		if cancel_result:
			self.logger.info("Goal cancellation successful.")
			self.cancelling_goal = False  # Mark cancellation as completed (success).
			return True
		else:
			self.logger.error("Goal cancellation failed.")
			self.cancelling_goal = False  # Mark cancellation as completed (failed).
			return False

	def cancel_current_goal(self):
		"""Requests cancellation of the currently active navigation goal."""
		if self.goal_handle_curr is not None and not self.cancelling_goal:
			self.cancelling_goal = True  # Mark cancellation in-progress.
			self.logger.info("Requesting cancellation of current goal...")
			cancel_future = self.action_client._cancel_goal_async(self.goal_handle_curr)
			cancel_future.add_done_callback(self.cancel_goal_callback)

	def goal_result_callback(self, future):
		"""
		Callback function executed when the navigation goal reaches a final result.

		Args:
			future (rclpy.Future): The future that is result of the navigation action.
		"""
		status = future.result().status
		# NOTE: Refer https://docs.ros2.org/foxy/api/action_msgs/msg/GoalStatus.html.

		if status == GoalStatus.STATUS_SUCCEEDED:
			self.logger.info("Goal completed successfully!")
		else:
			self.logger.warn(f"Goal failed with status: {status}")

		self.goal_completed = True  # Mark goal as completed.
		self.goal_handle_curr = None  # Clear goal handle.

	def goal_response_callback(self, future):
		"""
		Callback function executed after the goal is sent to the action server.

		Args:
			future (rclpy.Future): The future that is server's response to goal request.
		"""
		goal_handle = future.result()
		if not goal_handle.accepted:
			self.logger.warn('Goal rejected :(')
			self.goal_completed = True  # Mark goal as completed (rejected).
			self.goal_handle_curr = None  # Clear goal handle.
		else:
			self.logger.info('Goal accepted :)')
			self.goal_completed = False  # Mark goal as in progress.
			self.goal_handle_curr = goal_handle  # Store goal handle.

			get_result_future = goal_handle.get_result_async()
			get_result_future.add_done_callback(self.goal_result_callback)

	def goal_feedback_callback(self, msg):
		"""
		Callback function to receive feedback from the navigation action.

		Args:
			msg (nav2_msgs.action.NavigateToPose.Feedback): The feedback message.
		"""
		distance_remaining = msg.feedback.distance_remaining
		number_of_recoveries = msg.feedback.number_of_recoveries
		navigation_time = msg.feedback.navigation_time.sec
		estimated_time_remaining = msg.feedback.estimated_time_remaining.sec

		self.logger.debug(f"Recoveries: {number_of_recoveries}, "
				  f"Navigation time: {navigation_time}s, "
				  f"Distance remaining: {distance_remaining:.2f}, "
				  f"Estimated time remaining: {estimated_time_remaining}s")

		if number_of_recoveries > self.recovery_threshold and not self.cancelling_goal:
			self.logger.warn(f"Cancelling. Recoveries = {number_of_recoveries}.")
			self.cancel_current_goal()  # Unblock by discarding the current goal.

	def send_goal_from_world_pose(self, goal_pose):
		"""
		Sends a navigation goal to the Nav2 action server.

		Args:
			goal_pose (geometry_msgs.msg.PoseStamped): The goal pose in the world frame.

		Returns:
			bool: True if the goal was successfully sent, False otherwise.
		"""
		if not self.goal_completed or self.goal_handle_curr is not None:
			return False

		self.goal_completed = False  # Starting a new goal.

		goal = NavigateToPose.Goal()
		goal.pose = goal_pose

		if not self.action_client.wait_for_server(timeout_sec=SERVER_WAIT_TIMEOUT_SEC):
			self.logger.error('NavigateToPose action server not available!')
			return False

		# Send goal asynchronously (non-blocking).
		goal_future = self.action_client.send_goal_async(goal, self.goal_feedback_callback)
		goal_future.add_done_callback(self.goal_response_callback)

		return True



	def _get_map_conversion_info(self, map_info) -> Optional[Tuple[float, float]]:
		"""Helper function to get map origin and resolution."""
		if map_info:
			origin = map_info.origin
			resolution = map_info.resolution
			return resolution, origin.position.x, origin.position.y
		else:
			return None

	def get_world_coord_from_map_coord(self, map_x: int, map_y: int, map_info) \
					   -> Tuple[float, float]:
		"""Converts map coordinates to world coordinates."""
		if map_info:
			resolution, origin_x, origin_y = self._get_map_conversion_info(map_info)
			world_x = (map_x + 0.5) * resolution + origin_x
			world_y = (map_y + 0.5) * resolution + origin_y
			return (world_x, world_y)
		else:
			return (0.0, 0.0)

	def get_map_coord_from_world_coord(self, world_x: float, world_y: float, map_info) \
					   -> Tuple[int, int]:
		"""Converts world coordinates to map coordinates."""
		if map_info:
			resolution, origin_x, origin_y = self._get_map_conversion_info(map_info)
			map_x = int((world_x - origin_x) / resolution)
			map_y = int((world_y - origin_y) / resolution)
			return (map_x, map_y)
		else:
			return (0, 0)

	def _create_quaternion_from_yaw(self, yaw: float) -> Quaternion:
		"""Helper function to create a Quaternion from a yaw angle."""
		cy = math.cos(yaw * 0.5)
		sy = math.sin(yaw * 0.5)
		q = Quaternion()
		q.x = 0.0
		q.y = 0.0
		q.z = sy
		q.w = cy
		return q

	def create_yaw_from_vector(self, dest_x: float, dest_y: float,
				   source_x: float, source_y: float) -> float:
		"""Calculates the yaw angle from a source to a destination point.
			NOTE: This function is independent of the type of map used.

			Input: World coordinates for destination and source.
			Output: Angle (in radians) with respect to x-axis.
		"""
		delta_x = dest_x - source_x
		delta_y = dest_y - source_y
		yaw = math.atan2(delta_y, delta_x)

		return yaw

	def create_goal_from_world_coord(self, world_x: float, world_y: float,
					 yaw: Optional[float] = None) -> PoseStamped:
		"""Creates a goal PoseStamped from world coordinates.
			NOTE: This function is independent of the type of map used.
		"""
		goal_pose = PoseStamped()
		goal_pose.header.stamp = self.get_clock().now().to_msg()
		goal_pose.header.frame_id = self._frame_id

		goal_pose.pose.position.x = world_x
		goal_pose.pose.position.y = world_y

		if yaw is None and self.pose_curr is not None:
			source_x = self.pose_curr.pose.pose.position.x
			source_y = self.pose_curr.pose.pose.position.y
			yaw = self.create_yaw_from_vector(world_x, world_y, source_x, source_y)
		elif yaw is None:
			yaw = 0.0
		else: 
			pass

		goal_pose.pose.orientation = self._create_quaternion_from_yaw(yaw)

		pose = goal_pose.pose.position
		print(f"Goal created: ({pose.x:.2f}, {pose.y:.2f}, yaw={yaw:.2f})")
		return goal_pose

	def create_goal_from_map_coord(self, map_x: int, map_y: int, map_info,
				       yaw: Optional[float] = None) -> PoseStamped:
		"""Creates a goal PoseStamped from map coordinates."""
		world_x, world_y = self.get_world_coord_from_map_coord(map_x, map_y, map_info)

		return self.create_goal_from_world_coord(world_x, world_y, yaw)


	def find_shelf_and_target_point(self,slam_map, robot_pos, shelf_angle_deg, search_distance=100):
		"""
		Find shelf in specified direction and locate a safe point to move towards it.
		NOW INCLUDES:
		- 30-unit margin around robot position
		- 20-unit margin from map edges
		- WIDER SEARCH CORRIDOR (±3 units perpendicular to search direction)
		"""
		# Convert angle to radians
		angle_rad = np.radians(shelf_angle_deg)
		
		# Create a mask for obstacles (shelves are typically obstacles)
		obstacle_mask = (slam_map == 99) | (slam_map == 100)
		
		# Create search line in the specified direction
		robot_x, robot_y = robot_pos
		
		# MODIFICATION: Start search from 30 units away from robot
		min_search_distance = 25  # Exclude 25-unit margin around robot

		# Map boundary margins
		edge_margin = 3  # Exclude 20-unit margin from map edges
		map_height, map_width = slam_map.shape
		
		# WIDER SEARCH: Create a corridor instead of just a line
		search_width = 3  # Search ±3 units perpendicular to the main direction
		
		# Calculate perpendicular direction
		perp_angle = angle_rad + np.pi/2  # 90 degrees perpendicular
		perp_dx = np.cos(perp_angle)
		perp_dy = np.sin(perp_angle)
		
		# Generate points along the search corridor
		search_points = []
		for distance in range(min_search_distance, search_distance + 1):
			# Main search line point
			main_x = robot_x + distance * np.cos(angle_rad)
			main_y = robot_y + distance * np.sin(angle_rad)
			
			# Create points across the search width
			for offset in range(-search_width, search_width + 1):
				x = int(main_x + offset * perp_dx)
				y = int(main_y + offset * perp_dy)
				
				# Check bounds WITH edge margins
				if (edge_margin <= x < map_width - edge_margin and 
					edge_margin <= y < map_height - edge_margin):
					search_points.append((x, y))
		
		# Remove duplicates while preserving order
		search_points = list(dict.fromkeys(search_points))
		
		print(f"Generated {len(search_points)} search points in {2*search_width+1}-unit wide corridor")
		
		# Find obstacles along the search corridor
		obstacles_on_line = []
		for x, y in search_points:
			if obstacle_mask[y, x]:  # Note: y first for numpy array indexing
				obstacles_on_line.append((x, y))
		
		if not obstacles_on_line:
			print(f"No obstacles found in {2*search_width+1}-unit wide search corridor")
			print(f"Beyond {min_search_distance} units from robot and {edge_margin} units from map edges")
			return None, None
		
		print(f"Found {len(obstacles_on_line)} obstacle points in search corridor")
		
		# Additional validation: Ensure detected obstacles meet both distance criteria
		valid_obstacles = []
		for obs_x, obs_y in obstacles_on_line:
			# Check distance from robot
			distance_from_robot = np.sqrt((obs_x - robot_x)**2 + (obs_y - robot_y)**2)
			
			# Check distance from map edges
			distance_from_edges = min(
				obs_x,                          # Distance from left edge
				obs_y,                          # Distance from top edge
				map_width - 1 - obs_x,         # Distance from right edge
				map_height - 1 - obs_y         # Distance from bottom edge
			)
			
			# Keep obstacle only if it meets both criteria
			if (distance_from_robot >= min_search_distance and 
				distance_from_edges >= edge_margin):
				valid_obstacles.append((obs_x, obs_y))
		
		if not valid_obstacles:
			print(f"All detected obstacles are either:")
			print(f"  - Within {min_search_distance}-unit exclusion zone around robot, OR")
			print(f"  - Within {edge_margin}-unit margin from map edges")
			return None, None
		
		print(f"Found {len(valid_obstacles)} valid obstacles (excluding robot vicinity and map edges)")
		
		# Use valid obstacles (beyond both margins) for shelf detection
		shelf_info = self.detect_shelf_with_orientation(slam_map, valid_obstacles, robot_pos, angle_rad)
		
		# Find a safe point to move towards the shelf
		target_point = None
		if shelf_info and shelf_info['center']:
			target_point = self.find_safe_approach_point(slam_map, robot_pos, shelf_info, angle_rad)
		
		return target_point, shelf_info

	def detect_shelf_with_orientation(self,slam_map, obstacles_on_line, robot_pos, angle_rad):
		"""
		Detect shelf and determine its orientation/rotation.
		NOW INCLUDES: Edge margin filtering to exclude map boundary pixels
		"""
		if not obstacles_on_line:
			return None
		
		# Map boundary margins (same as in find_shelf_and_target_point)
		edge_margin = 5  # Exclude 5-unit margin from map edges
		map_height, map_width = slam_map.shape
		
		# Filter obstacles_on_line to exclude those near map edges
		filtered_obstacles = []
		for obs_x, obs_y in obstacles_on_line:
			# Check distance from map edges
			distance_from_edges = min(
				obs_x,                          # Distance from left edge
				obs_y,                          # Distance from top edge
				map_width - 1 - obs_x,         # Distance from right edge
				map_height - 1 - obs_y         # Distance from bottom edge
			)
			
			# Keep obstacle only if it's away from edges
			if distance_from_edges >= edge_margin:
				filtered_obstacles.append((obs_x, obs_y))
		
		if not filtered_obstacles:
			print(f"No obstacles found after excluding {edge_margin}-unit margin from map edges")
			return None
		
		print(f"Filtered obstacles: {len(obstacles_on_line)} -> {len(filtered_obstacles)} (removed edge obstacles)")
		
		# Create a region of interest around the FILTERED obstacles
		obstacle_points = np.array(filtered_obstacles)  # Use filtered list
		min_x, min_y = np.min(obstacle_points, axis=0)
		max_x, max_y = np.max(obstacle_points, axis=0)
		
		# Simple margin
		margin = 30
		roi_x1 = max(edge_margin, min_x - margin)          # Respect edge margin
		roi_y1 = max(edge_margin, min_y - margin)          # Respect edge margin
		roi_x2 = min(map_width - edge_margin, max_x + margin)   # Respect edge margin
		roi_y2 = min(map_height - edge_margin, max_y + margin)  # Respect edge margin
		
		print(f"ROI bounds (with edge margins): ({roi_x1}, {roi_y1}) to ({roi_x2}, {roi_y2})")
		
		# Extract ROI
		roi = slam_map[roi_y1:roi_y2, roi_x1:roi_x2]
		
		# Create obstacle mask for ROI
		obstacle_roi = (roi == 99) | (roi == 100)
		
		# Find connected components
		num_features, labeled = cv2.connectedComponents(obstacle_roi.astype(np.uint8))
		
		if num_features == 0:
			return None
		
		# SIMPLE: Just find the largest component
		component_sizes = []
		for i in range(1, num_features + 1):
			component_mask = (labeled == i)
			size = np.sum(component_mask)
			component_sizes.append((size, i))
		
		# Get the largest component
		largest_size, largest_label = max(component_sizes)
		largest_component = (labeled == largest_label)
		
		# Get coordinates of the shelf pixels
		y_coords, x_coords = np.where(largest_component)
		
		if len(x_coords) == 0:
			return None
		
		# Convert back to global coordinates
		global_x_coords = x_coords + roi_x1
		global_y_coords = y_coords + roi_y1
		
		# ADDITIONAL FILTERING: Remove any points that are still too close to edges
		final_x_coords = []
		final_y_coords = []
		for x, y in zip(global_x_coords, global_y_coords):
			distance_from_edges = min(x, y, map_width - 1 - x, map_height - 1 - y)
			if distance_from_edges >= edge_margin:
				final_x_coords.append(x)
				final_y_coords.append(y)
		
		if len(final_x_coords) == 0:
			print("No shelf pixels found after edge filtering")
			return None
		
		global_x_coords = np.array(final_x_coords)
		global_y_coords = np.array(final_y_coords)
		
		# Calculate shelf center
		center_x = int(np.mean(global_x_coords))
		center_y = int(np.mean(global_y_coords))
		
		print(f"Shelf center: ({center_x}, {center_y})")
		print(f"Shelf contains {len(global_x_coords)} pixels (after edge filtering)")
		
		# Find shelf orientation using PCA
		shelf_points = np.column_stack([global_x_coords, global_y_coords])
		orientation_info = self.calculate_shelf_orientation(shelf_points)
		
		# Find minimum area rectangle for better shape analysis
		contour_points = np.column_stack([global_x_coords, global_y_coords]).astype(np.int32)
		rect_info = cv2.minAreaRect(contour_points)
		
		# Extract rectangle information
		(rect_center_x, rect_center_y), (width, height), rotation_angle = rect_info
		
		# Get corner points of the rotated rectangle
		box_points = cv2.boxPoints(rect_info)
		box_points = np.int32(box_points)
		
		# Normalize rotation angle to [0, 180) degrees
		rotation_angle = rotation_angle % 180
		
		# Create shelf info dictionary
		shelf_info = {
			'center': (center_x, center_y),
			'orientation': orientation_info,
			'corners': box_points.tolist(),
			'dimensions': {
				'width': width,
				'height': height,
				'area': len(global_x_coords)
			},
			'rotation_angle': rotation_angle,
			'rect_center': (rect_center_x, rect_center_y)
		}
		
		return shelf_info

	def calculate_shelf_orientation(self,points):
		"""
		Parameters:
		points: numpy array of (x, y) coordinates
		
		Returns:
		dict with orientation information
		"""
		
		mean_point = np.mean(points, axis=0)
		centered_points = points - mean_point
		cov_matrix = np.cov(centered_points.T)
		eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)
		idx = np.argsort(eigenvalues)[::-1]
		eigenvalues = eigenvalues[idx]
		eigenvectors = eigenvectors[:, idx]

		primary_direction = eigenvectors[:, 0]
		secondary_direction = eigenvectors[:, 1]
		primary_angle = np.degrees(np.arctan2(primary_direction[1], primary_direction[0]))
		secondary_angle = np.degrees(np.arctan2(secondary_direction[1], secondary_direction[0]))
		primary_angle = primary_angle % 180
		secondary_angle = secondary_angle % 180
		aspect_ratio = eigenvalues[0] / eigenvalues[1] if eigenvalues[1] != 0 else float('inf')
		
		return {
			'primary_angle': primary_angle,
			'secondary_angle': secondary_angle,
			'primary_direction': primary_direction,
			'secondary_direction': secondary_direction,
			'aspect_ratio': aspect_ratio,
			'elongation': np.sqrt(eigenvalues[0] / eigenvalues[1]) if eigenvalues[1] != 0 else float('inf')
		}

	def find_safe_approach_point(self,slam_map, robot_pos, shelf_info, angle_rad):
		"""
		Parameters:
		slam_map: 2D numpy array with pixel values
		robot_pos: (x, y) robot position
		shelf_info: dict with shelf information including center and orientation
		angle_rad: search angle in radians
		
		Returns:
		(x, y) coordinates of best safe point closest to the 30-unit front position
		"""
		if shelf_info is None or shelf_info['center'] is None:
			return None
		
		shelf_center = shelf_info['center']
		shelf_x, shelf_y = shelf_center
		robot_x, robot_y = robot_pos
		
		# Get shelf orientation information
		orientation = shelf_info['orientation']
		primary_direction = orientation['primary_direction']
		secondary_direction = orientation['secondary_direction']
		target_distance = 50
		front_candidates = []
		
		for direction in [secondary_direction, -secondary_direction]:
			direction = direction / np.linalg.norm(direction)
			target_x = shelf_x + target_distance * direction[0]
			target_y = shelf_y + target_distance * direction[1]
			if (0 <= target_x < slam_map.shape[1] and 0 <= target_y < slam_map.shape[0]):
				front_candidates.append((target_x, target_y))
		
		if not front_candidates:
			return None
		
		# Find the best target point (closest to robot or most accessible)
		best_target = min(front_candidates, key=lambda t: np.sqrt((t[0] - robot_x)**2 + (t[1] - robot_y)**2))
		target_x, target_y = best_target
		best_approach_point = None
		best_distance_to_target = float('inf')
		
		# Search in expanding circles around the robot
		for search_radius in range(10, 301, 5):
			num_points = max(12, search_radius // 2)
			for i in range(num_points):
				angle = i * 2 * np.pi / num_points
				# Calculate candidate position around robot
				candidate_x = int(robot_x + search_radius * np.cos(angle))
				candidate_y = int(robot_y + search_radius * np.sin(angle))
				if (0 <= candidate_x < slam_map.shape[1] and 
					0 <= candidate_y < slam_map.shape[0]):
					# Check if this point is free space and safe
					if (slam_map[candidate_y, candidate_x] == 0 and 
						self.is_safe_area(slam_map, (candidate_x, candidate_y), radius=5)):
						# Calculate distance from this candidate to the target point (30 units in front of shelf)
						distance_to_target = np.sqrt(
							(candidate_x - target_x)**2 + (candidate_y - target_y)**2
						)
						if distance_to_target < best_distance_to_target:
							best_distance_to_target = distance_to_target
							best_approach_point = (candidate_x, candidate_y)
		
		return best_approach_point



	def is_safe_area(self,slam_map, point, radius=3):
		# Check if an area around a point is safe (all pixels are 0).
		x, y = point
		for dx in range(-radius, radius + 1):
			for dy in range(-radius, radius + 1):
				check_x, check_y = x + dx, y + dy
				if 0 <= check_x < slam_map.shape[1] and 0 <= check_y < slam_map.shape[0]:
					if slam_map[check_y, check_x] != 0:
						return False
		return True

	# def find_front_back_points(self,point, direction, distance):
	# 	"""
	# 	Find points in front and back of a given point in a specific direction.
		
	# 	Parameters:
	# 	point: tuple (x, y) - the reference point
	# 	direction: tuple (dx, dy) or angle in radians or degrees
	# 	distance: float - how far in front/back to place the points
		
	# 	Returns:
	# 	front_point: (x, y) - point in front
	# 	back_point: (x, y) - point behind
	# 	"""
	# 	x, y = point
		
	# 	# Handle different direction formats
	# 	if isinstance(direction, (int, float)):
	# 		# If direction is an angle (assume radians, convert if needed)
	# 		if abs(direction) > 2 * np.pi:  # Likely degrees
	# 			direction = np.radians(direction)
			
	# 		# Convert angle to direction vector
	# 		dx = np.cos(direction)
	# 		dy = np.sin(direction)
	# 	else:
	# 		# If direction is a vector (dx, dy)
	# 		dx, dy = direction
		
	# 	# Normalize the direction vector
	# 	magnitude = np.sqrt(dx**2 + dy**2)
	# 	if magnitude == 0:
	# 		return point, point  # No movement if no direction
		
	# 	dx_norm = dx / magnitude
	# 	dy_norm = dy / magnitude
	# 	front_point = (
	# 		x + distance * dx_norm,
	# 		y - distance * dy_norm
	# 	)
	# 	back_point = (
	# 		x - distance * dx_norm,
	# 		y + distance * dy_norm
	# 	)
	# 	return front_point, back_point

	def find_front_back_points(self, shelf_center, shelf_info, distance, use_primary=True):
		"""
		Find points in front and back of the shelf center using shelf's principal directions.
		
		Parameters:
		shelf_center: tuple (x, y) - the shelf center point
		shelf_info: dict - shelf information containing orientation data
		distance: float - how far in front/back to place the points from shelf center
		use_primary: bool - if True, use primary direction; if False, use secondary direction
		
		Returns:
		front_point: (x, y) - point in front along the chosen direction
		back_point: (x, y) - point behind along the chosen direction
		"""
		x, y = shelf_center
		
		# Get the orientation information from shelf_info
		if 'orientation' not in shelf_info:
			self.get_logger().warn("No orientation information in shelf_info")
			return (x, y), (x, y)
		
		orientation = shelf_info['orientation']
		
		# Choose which direction to use
		if use_primary:
			direction_vector = orientation['primary_direction']
			direction_name = "primary"
		else:
			direction_vector = -orientation['primary_direction']
			direction_name = "secondary"
		
		# Extract direction components
		dx, dy = direction_vector[0], direction_vector[1]
		
		# Normalize the direction vector (should already be normalized from PCA, but ensure it)
		magnitude = np.sqrt(dx**2 + dy**2)
		if magnitude == 0:
			self.get_logger().warn("Zero magnitude direction vector")
			return (x, y), (x, y)
		
		dx_norm = dx / magnitude
		dy_norm = dy / magnitude
		
		# Calculate front and back points along the chosen direction
		# Front point: move in the direction of the vector
		front_point = (
			int(x + distance * dx_norm),
			int(y + distance * dy_norm)
		)
		
		# Back point: move in the opposite direction
		back_point = (
			int(x - distance * dx_norm),
			int(y - distance * dy_norm)
		)

		
		self.get_logger().info(f"Using {direction_name} direction: [{dx_norm:.3f}, {dy_norm:.3f}]")
		self.get_logger().info(f"Shelf center: ({x}, {y})")
		self.get_logger().info(f"Front point ({distance} units ahead): {front_point}")
		self.get_logger().info(f"Back point ({distance} units behind): {back_point}")
		
		return front_point, back_point


def main(args=None):
	rclpy.init(args=args)

	warehouse_explore = WarehouseExplore()

	if PROGRESS_TABLE_GUI:
		gui_thread = threading.Thread(target=run_gui, args=(warehouse_explore.shelf_count,))
		gui_thread.start()

	rclpy.spin(warehouse_explore)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	warehouse_explore.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
