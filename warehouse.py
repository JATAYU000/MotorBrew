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
		self.xy_goal_tolerance = 0.5
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
		self.current_shelf_centre = None
		self.current_shelf_orientation = None

		self.current_state = -1
		self.current_shelf_id	= 1
		self.current_angle = self.initial_angle
		self.current_pos = (150,150)
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
		
		# Safety checks - don't start until everything is ready
		# self.get_logger().info(f"current state : {self.current_state}")
		# self.get_logger().info(f"goal completed : {self.goal_completed} goal sent? {self.goal_sent}")
		# State machine logic
		if self.goal_sent and self.goal_completed:
			self.logger.info("\n\n\n\Goal completed, CAPTURING data...")
			self.current_state = self.DO_NOTHING
			self.goal_sent = False  # Reset goal sent flag after handling
		if self.current_state == self.NAVIGATE_TO_SHELF:
			self.get_logger().info('NAVIGATING')
			self.handle_navigate_to_shelf()
		elif self.current_state == self.CAPTURE_OBJECTS:
			self.get_logger().info('CAPTURING')
			self.handle_capture_objects()
			 # Reset state after capturing objects
		elif self.current_state == self.SCAN_QR:
			self.handle_navigate_to_qr_side()
			self.handle_SCAN_QR()
		elif self.current_state == self.DO_NOTHING:
			self.get_logger().info("Waiting for next command...")
			# Do nothing, just wait for the next command
		
		

	def handle_navigate_to_qr_side(self):
		"""State 3: Move to side of shelf to scan QR code"""
		if not self.goal_completed or self.qr_reached:
			return  # Still moving
		
		current_x, current_y = self.buggy_center
		self.get_logger().info(f"Current position: ({current_x:.2f}, {current_y:.2f})")
		map_x, map_y = self.get_map_coord_from_world_coord(current_x, current_y, self.global_map_curr.info)
		self.get_logger().info(f"Current map position: ({map_x}, {map_y})")
		self.get_logger().info(f"Current angle: {self.current_angle}°")
		self.get_logger().info(f"Current shelf ID: {self.global_map_curr.info}")
		if map_x is not None and map_y is not None:
			# Move to side of shelf for QR scanning
			qr_scan_x = 120.0
			qr_scan_y = 250.0

			# Face toward the shelf side
			face_angle = math.radians(self.current_angle + 90)
			goal_x, goal_y = self.get_world_coord_from_map_coord(qr_scan_x, qr_scan_y, self.global_map_curr.info)
			goal = self.create_goal_from_world_coord(goal_x, goal_y, face_angle)
			
			self.send_goal_from_world_pose(goal)
			self.get_logger().info(f"Moving to QR scan position")
			self.qr_reached = True
			self.current_state = self.SCAN_QR

	def handle_SCAN_QR(self):
		"""State 4: Capture QR code"""
		if not self.goal_completed or self.qr_reached == False:
			return  # Still moving to QR position
		
		if not self.should_detect_qr:
			self.get_logger().info("Starting QR code scan...")
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
			self.current_state = self.DO_NOTHING
			
		elif time.time() - self.qr_scan_start_time > 5.0:  # 15 second timeout
			self.should_detect_qr = False
			self.get_logger().warn("QR scan timeout")
			self.current_state = self.DO_NOTHING
		
	def parse_qr_for_next_angle(self, qr_data):
		"""Parse QR code to extract next shelf angle"""
		try:
			# QR format: "shelfid_nextangle_uniquecode"
			parts = qr_data.split('_')
			if len(parts) >= 2:
				next_angle = float(parts[1])
				return next_angle
		except (ValueError, IndexError):
			self.get_logger().error(f"Failed to parse QR code: {qr_data}")
		
		return None

	def handle_process_data(self):
		"""State 5: Process collected data and publish"""
		# Create shelf data message
		shelf_data_message = WarehouseShelf()
		
		# Add object data if available
		if self.current_shelf_objects:
			shelf_data_message.object_name = self.current_shelf_objects.object_name
			shelf_data_message.object_count = self.current_shelf_objects.object_count
		
		# Add QR data if available
		if self.current_qr_data:
			shelf_data_message.qr_decoded = self.current_qr_data
			
			# Parse QR to get next angle
			next_angle = self.parse_qr_for_next_angle(self.current_qr_data)
			if next_angle is not None:
				self.current_angle = next_angle
				self.current_shelf_id += 1
				
				self.get_logger().info(f"Next shelf: {self.current_shelf_id} at angle {self.current_angle}°")
				
				# Check if more shelves to explore
				if self.current_shelf_id <= self.shelf_count:
					self.current_state = self.NAVIGATE_TO_SHELF
				else:
					self.current_state = self.EXPLORATION_COMPLETE
			else:
				self.get_logger().warn("Could not parse QR code for next angle")
				self.current_state = self.EXPLORATION_COMPLETE
		else:
			self.get_logger().warn("No QR code found, exploration complete")
			self.current_state = self.EXPLORATION_COMPLETE
		
		# Publish shelf data (this triggers curtain removal)
		self.publisher_shelf_data.publish(shelf_data_message)
		self.get_logger().info("Published shelf data")
		
		# Reset detection data
		self.current_shelf_objects = None
		self.current_qr_data = None

	def calculate_shelf_position_from_angle(self, angle_degrees):
		"""Calculate shelf position using the same logic as before but cleaner"""
		if self.global_map_curr is None:
			return None, None
		
		map_info = self.global_map_curr.info
		
		# Use map center as reference (like your original code)
		fx = 150
		fy = 150
		
		# Distance in map coordinates (like your original code)
		distance = 40
		
		# Convert angle to radians
		angle_rad = math.radians(angle_degrees)
		
		# Calculate goal in map coordinates
		goal_map_x = fx + distance * math.cos(angle_rad)
		goal_map_y = fy + distance * math.sin(angle_rad)
		
		# Convert to world coordinates
		goal_world_x, goal_world_y = self.get_world_coord_from_map_coord(
			int(goal_map_x), int(goal_map_y), map_info)
		
		self.get_logger().info(f"Calculated position: map({goal_map_x:.1f}, {goal_map_y:.1f}) -> world({goal_world_x:.2f}, {goal_world_y:.2f})")
		
		return goal_world_x, goal_world_y
	
	def handle_navigate_to_shelf(self):
		"""State 1: Navigate to the shelf"""
		if not self.goal_completed:
			return  # Still navigating
		
		self.get_logger().info(f"Navigating to shelf {self.current_shelf_id} at angle {self.current_angle}°")
		map_array = np.array(self.global_map_curr.data).reshape((self.global_map_curr.info.height, self.global_map_curr.info.width))
		target_point, shelf_info = self.find_shelf_and_target_point(slam_map= map_array, robot_pos=(150,150), shelf_angle_deg=135, search_distance=100)

		# Calculate shelf position using your existing logic
		if self.current_shelf_centre is None or self.current_shelf_orientation is None:
			self.get_logger().info(f"Target point found: {target_point}")
			self.get_logger().info(f"Shelf info: {shelf_info}")
			self.current_shelf_centre = shelf_info['center']
			self.current_shelf_orientation = shelf_info['rotation_angle']

		diff_x = int(35*np.cos(math.radians(self.current_shelf_orientation)))
		diff_y = int(35*np.sin(math.radians(self.current_shelf_orientation)))
		# log x+d,y+d , x-d,y+d .... those 4 points
		self.get_logger().info(f"Current shelf centre: {self.current_shelf_centre}")
		self.get_logger().info(f"Current shelf orientation: {self.current_shelf_orientation}°")
		self.get_logger().info(f"Diff X: {diff_x}, Diff Y: {diff_y}")
		front,back = self.find_front_back_points(self.current_shelf_centre, self.current_shelf_orientation, 35)
		self.get_logger().info(f"Front point: {front}, Back point: {back}")

		for point in [(self.current_shelf_centre[0]+diff_x, self.current_shelf_centre[1]-diff_y),(self.current_shelf_centre[0]-diff_x, self.current_shelf_centre[1]-diff_y)]:
			if self.is_safe_area(map_array,point):
				goal_x, goal_y = self.get_world_coord_from_map_coord(point[0], point[1], self.global_map_curr.info)
				yaw = math.radians(self.current_angle)
				goal = self.create_goal_from_world_coord(goal_x, goal_y, yaw)
				if self.send_goal_from_world_pose(goal):
					self.get_logger().info(f"Goal ideal sent to ({point[0]:.2f}, {point[1]:.2f})")
					self.get_logger().info(f"Goal ideal sent to ({goal_x:.2f}, {goal_y:.2f})")
					self.goal_sent = True
					self.current_state = self.CAPTURE_OBJECTS
					return
				else:
					self.get_logger().error("Failed to send navigation goal!")


		if target_point is not None:
			goal_x, goal_y = float(target_point[0]), float(target_point[1])
			goal_x, goal_y = self.get_world_coord_from_map_coord(goal_x, goal_y, self.global_map_curr.info)
			yaw = math.radians(self.current_angle)
			goal = self.create_goal_from_world_coord(goal_x, goal_y, yaw)
			
			if self.send_goal_from_world_pose(goal):
				self.get_logger().info(f"Goal not ideal sent to ({goal_x:.2f}, {goal_y:.2f})")
				self.goal_sent = True
			else:
				self.get_logger().error("Failed to send navigation goal!")
		else:
			self.get_logger().error("Could not calculate valid shelf position!")
			
		
	
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

		elif time.time() - self.detection_start_time > 5.0:  # 5 second timeout
			self.should_detect_objects = False
			self.get_logger().warn("Object detection timeout")
			self.get_logger().info("Moving to QR scanning anyway...")
			self.current_state = self.SCAN_QR  
	
	def global_map_callback(self, message):
		"""Callback function to handle global map updates.

		Args:
			message: ROS2 message containing the global map data.

		Returns:
			None
		"""
		self.global_map_curr = message
		if self.current_state == -1:
			self.current_state = self.NAVIGATE_TO_SHELF
			# self.get_logger().info(f"Detected {len(detected_lines)} lines in current direction {self.current_angle}°")

		# if not self.goal_completed:
		# 	return

		# height, width = self.global_map_curr.info.height, self.global_map_curr.info.width
		# map_array = np.array(self.global_map_curr.data).reshape((height, width))

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

	def detect_lines_in_direction(self, map_array, target_angle_degrees, angle_tolerance=15):
		"""Detect lines that align with the target angle direction"""
		
		# Convert to binary and detect edges
		binary_map = (map_array > 50).astype(np.uint8) * 255
		edges = cv2.Canny(binary_map, 50, 150)
		
		# Hough Line Transform
		lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50,
							minLineLength=30, maxLineGap=10)
		
		target_angle_rad = np.radians(target_angle_degrees)
		shelf_lines = []
		
		if lines is not None:
			for line in lines:
				x1, y1, x2, y2 = line[0]
				
				# Calculate line angle
				line_angle = np.arctan2(y2 - y1, x2 - x1)
				
				# Check if line aligns with target direction (±tolerance)
				angle_diff = abs(line_angle - target_angle_rad)
				angle_diff = min(angle_diff, abs(angle_diff - np.pi))  # Handle 180° symmetry
				
				if angle_diff <= np.radians(angle_tolerance):
					length = np.sqrt((x2-x1)**2 + (y2-y1)**2)
					shelf_lines.append({
						'start': (x1, y1),
						'end': (x2, y2),
						'angle': np.degrees(line_angle),
						'length': length,
						'map_coords': True
					})
		
		return shelf_lines

	def analyze_directional_lines(self):
		"""Analyze SLAM map for lines in the target direction"""
		
		if self.global_map_curr is None:
			return []
		
		# Convert occupancy grid to numpy array
		height, width = self.global_map_curr.info.height, self.global_map_curr.info.width
		map_array = np.array(self.global_map_curr.data).reshape((height, width))
		
		# Detect lines in current target direction
		detected_lines = self.detect_lines_in_direction(map_array, self.current_angle, angle_tolerance=20)
		
		# Convert to world coordinates and add more info
		world_lines = []
		for line in detected_lines:
			# Convert start and end points to world coordinates
			start_world = self.get_world_coord_from_map_coord(
				line['start'][0], line['start'][1], self.global_map_curr.info)
			end_world = self.get_world_coord_from_map_coord(
				line['end'][0], line['end'][1], self.global_map_curr.info)
			
			# Calculate center point
			center_map = ((line['start'][0] + line['end'][0]) / 2, 
						(line['start'][1] + line['end'][1]) / 2)
			center_world = self.get_world_coord_from_map_coord(
				center_map[0], center_map[1], self.global_map_curr.info)
			
			world_lines.append({
				'start_map': line['start'],
				'end_map': line['end'],
				'start_world': start_world,
				'end_world': end_world,
				'center_world': center_world,
				'angle': line['angle'],
				'length_pixels': line['length'],
				'length_meters': line['length'] * self.global_map_curr.info.resolution
			})
		
		# Log the results
		self.get_logger().info(f"\n=== DIRECTIONAL LINE DETECTION RESULTS ===")
		self.get_logger().info(f"Target angle: {self.current_angle:.1f}°")
		self.get_logger().info(f"Found {len(world_lines)} lines aligned with target direction")
		
		for i, line in enumerate(world_lines):
			self.get_logger().info(f"\nLine {i+1}:")
			self.get_logger().info(f"  Map coords: ({line['start_map'][0]:.0f},{line['start_map'][1]:.0f}) -> ({line['end_map'][0]:.0f},{line['end_map'][1]:.0f})")
			self.get_logger().info(f"  World coords: ({line['start_world'][0]:.2f},{line['start_world'][1]:.2f}) -> ({line['end_world'][0]:.2f},{line['end_world'][1]:.2f})")
			self.get_logger().info(f"  Center: ({line['center_world'][0]:.2f}, {line['center_world'][1]:.2f})")
			self.get_logger().info(f"  Angle: {line['angle']:.1f}°")
			self.get_logger().info(f"  Length: {line['length_meters']:.2f}m ({line['length_pixels']:.0f} pixels)")
		
		return world_lines


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
			# Calculate yaw from current position to goal position.
			source_x = self.pose_curr.pose.pose.position.x
			source_y = self.pose_curr.pose.pose.position.y
			yaw = self.create_yaw_from_vector(world_x, world_y, source_x, source_y)
		elif yaw is None:
			yaw = 0.0
		else:  # No processing needed; yaw is supplied by the user.
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
