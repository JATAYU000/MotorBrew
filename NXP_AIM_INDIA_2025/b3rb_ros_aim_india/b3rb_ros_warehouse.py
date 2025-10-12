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

		# ----------------------- Robot State ----------------------- 
		self.armed = False
		self.logger = self.get_logger()

		# ----------------------- Robot Pose ----------------------- 
		self.pose_curr = PoseWithCovarianceStamped()
		self.buggy_pose_x = 0.0
		self.buggy_pose_y = 0.0
		self.buggy_center = (0.0, 0.0)
		self.world_center = (0.0, 0.0)

		# ----------------------- Map Data ----------------------- 
		self.simple_map_curr = None
		self.global_map_curr = None

		# ----------------------- Goal Management ----------------------- 
		self.xy_goal_tolerance = 0.2
		self.yaw_goal_tolerance = 0.15
		self.goal_completed = True  # No goal is currently in-progress.
		self.goal_handle_curr = None
		self.cancelling_goal = False
		self.recovery_threshold = 2

		# ----------------------- Goal Creation ----------------------- 
		self._frame_id = "map"

		# ----------------------- Exploration Parameters ----------------------- 
		self.max_step_dist_world_meters = 7.0
		self.min_step_dist_world_meters = 4.0
		self.full_map_explored_count = 0

		# -------------------- QR Code Data --------------------
		self.qr_code_str = "Empty"

		# ----------------------- Shelf Data -----------------------
		self.shelf_objects_curr = WarehouseShelf()

		# ----------------------- Robot State -----------------------
		self.NAVIGATE_TO_SHELF = 0
		self.CAPTURE_OBJECTS = 1
		self.SCAN_QR = 2
		self.DO_NOTHING = 3
		self.EXPLORE = 4
		self.ADJUST = 5

		self.adjusting = +1 # +1 for front, -1 for back
		self.goal_type = 0 # temp or ideal
		# ----------------------- Current Information -------------------------
		self.current_shelf_centre = None
		self.current_shelf_orientation = None
		self.shelf_info = None
		self.current_pos = None
		self.current_state = -1
		self.current_shelf_id	= 1
		self.current_angle = self.initial_angle
		self.world_centre = (150,150)
		self.current_qr_data = None

		# ----------------------- State Variables -----------------------
		self.navigation_started = False
		self.current_shelf_objects = None
		self.should_detect_objects = False
		self.should_detect_qr = False
		self.qr_code_str = ""
		self.detection_start_time = 0
		self.qr_scan_start_time = 0
		self.qr_reached = False
		self.qr_array = [0]*self.shelf_count
		self.pyzbar = pyzbar
		self.qr_detection_available = True
		self.goal_sent = False

		# ----------------------- State Machine ----------------------- 
		self.state_timer = self.create_timer(1.0, self.state_machine_update)
		self.time_start = self.get_clock().now().seconds_nanoseconds()[0]

	

	
	def state_machine_update(self):
		"""Main state machine logic"""
		if self.current_state == self.EXPLORE:
			self.frontier_explore()
		elif self.current_state == self.NAVIGATE_TO_SHELF:
			self.handle_nav_to_shelf()
		elif self.current_state == self.CAPTURE_OBJECTS:
			self.handle_capture_objects()
		elif self.current_state == self.SCAN_QR:
			self.should_detect_qr = True
			self.handle_navigate_to_qr_side()
			self.handle_qr_scan()
		elif self.current_state == self.ADJUST:
			self.handle_adjusting_shelf()
		elif self.current_state == self.DO_NOTHING:
			self.logger.info(f"Doing Nothing its been {self.get_clock().now().seconds_nanoseconds()[0] - self.time_start} seconds")



	# ----------------------- EXPLORE FUNCTIONS ----------------------- 



	def handle_adjusting_shelf(self,d=10):
		if not self.goal_completed:
			return
		
		x,y = self.get_map_coord_from_world_coord(self.buggy_pose_x, self.buggy_pose_y, self.global_map_curr.info)
		# current angle of robot
		rob_angle = math.degrees(self.get_current_robot_yaw())
		if rob_angle < 0:
			rob_angle +=360
		x_goal, y_goal = x + self.adjusting * d * math.cos(math.radians(rob_angle)), y + self.adjusting * d * math.sin(math.radians(rob_angle))
		goal = self.create_goal_from_map_coord(x_goal, y_goal, self.global_map_curr.info, math.radians(rob_angle))
		self.send_goal_from_world_pose(goal)
		self.current_state = self.CAPTURE_OBJECTS
		self.adjusting *= -1 

	def get_frontiers_for_space_exploration(self, map_array):
		"""
		Identifies frontier cells in a 2D occupancy grid map for space exploration.
		A frontier is defined as a cell that is adjacent to both unknown space and free space,
		but not adjacent to obstacles. The function scans the map and returns a list of coordinates
		representing such frontiers, which are useful for exploration tasks in robotics.
			map_array (numpy.ndarray): 2D numpy array representing the occupancy grid map.
				-1: Unknown space
				 0: Free space
				>0: Obstacles
			list[tuple[int, int]]: List of (y, x) tuples representing the coordinates of frontier cells.
		returns:
			frontiers: List of (y, x) tuples representing the coordinates of frontier cells.
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
	
	def frontier_explore(self):
		"""
		Handles the robot's frontier exploration state.
		This method is responsible for exploring unknown areas (frontiers) in the map.
		It performs the following tasks:
		  - Checks if the robot is close to the current frontier goal and, if so, cancels exploration and switches to navigation towards the shelf.
		  - If the robot is still moving towards a frontier, the method returns early.
		  - If the map is mostly free space or the area around the shelf is sufficiently explored, skips further exploration and switches to navigation.
		  - Identifies frontiers (boundaries between known and unknown space) in the map.
		  - Selects the closest valid frontier to the shelf or a calculated direction, considering minimum and maximum step distances.
		  - Sends a navigation goal to the selected frontier and stores the goal for distance checking.
		  - Adjusts exploration parameters if no suitable frontier is found.
		  - Tracks the number of times no frontiers are found to determine if the map is fully explored.
		Returns:
			None
		"""
		height, width = self.global_map_curr.info.height, self.global_map_curr.info.width
		map_array = np.array(self.global_map_curr.data).reshape((height, width))
		if not self.goal_completed and self.goal_handle_curr is not None:
			# Get current robot position in map coordinates
			robot_map_pos = self.get_map_coord_from_world_coord(
				self.buggy_pose_x, self.buggy_pose_y, self.global_map_curr.info
			)
			
			
			# Calculate distance to current goal (you'll need to store the goal position)
			if hasattr(self, 'current_frontier_goal'):
				distance_to_goal = self.calc_distance(robot_map_pos, self.current_frontier_goal[0])
				# if self.current_frontier_goal is not None and self.current_frontier_goal[1] != 0:
				# 	self.logger.info(f"DIFF: {abs(distance_to_goal - self.current_frontier_goal[1]):.2f}")
				# 	if abs(distance_to_goal - self.current_frontier_goal[1]) < 0.5:
				# 		self.cancel_current_goal()
				# 		self.current_frontier_goal = None
				# if self.current_frontier_goal is not None: self.current_frontier_goal[1] = distance_to_goal
				self.logger.info(f"Distance to current frontier goal: {distance_to_goal:.2f} rob: {robot_map_pos}frontgoal: {self.current_frontier_goal[0]}")
				if self.goal_completed:
					self.current_state = self.NAVIGATE_TO_SHELF
					return
				if distance_to_goal < 20 or self.find_percentage_of_free_space(map_array) > 92:  # Within 20 units of frontier goal
					self.logger.info(f"Robot within 20 units of frontier goal (distance: {distance_to_goal:.2f})")
					self.logger.info("Cancelling frontier exploration and switching to NAVIGATE_TO_SHELF")
					map_array = np.array(self.global_map_curr.data).reshape((self.global_map_curr.info.height, self.global_map_curr.info.width))
					# Save the current map for debugging
					np.save('frontier_exploration.npy', map_array)
					# Cancel current goal
					self.cancel_current_goal()
					self.current_frontier_goal = None
					# Switch to navigation state
					self.current_state = self.NAVIGATE_TO_SHELF
					return
		
		_, shelf_info = self.find_shelf_and_target_point(slam_map=map_array, robot_pos=self.world_centre, shelf_angle_deg=self.current_angle, search_distance=400)
		self.shelf_info = shelf_info
		if shelf_info is not None:self.logger.info(f"Percentage completed around the shelf : {self.find_percentage_of_free_space_around_point(map_array, shelf_info['center'], radius=75)}")
		if self.shelf_info is not None and self.find_percentage_of_free_space_around_point(map_array, shelf_info['center'], radius=75)>70:
			self.logger.info("Map is mostly free space, skipping exploration")
			self.cancel_current_goal()
			self.current_state = self.NAVIGATE_TO_SHELF
			return
		
		if not self.goal_completed :
			return  # Still moving to current frontier
		self.logger.info("Exploring frontier...")
		frontiers = self.get_frontiers_for_space_exploration(map_array)
		self.logger.info(f"\nFound {len(frontiers)} frontiers in the map.")
		self.logger.info(f"Slam map size: {self.global_map_curr.info}")
		self.logger.info(f"world centre: {self.world_centre}, Current shelf info: {shelf_info}")
		map_info = self.global_map_curr.info
		if frontiers:
			closest_frontier = None
			min_distance_curr = float('inf')
			self.shelf_info = shelf_info
			
			if shelf_info is not None:
				world_self_center = self.get_world_coord_from_map_coord(
					shelf_info['center'][0],
					shelf_info['center'][1],
					map_info
				)
			else:
				self.logger.info(f"Initial world position: {self.current_pos}")
				
				angle_rad = math.radians(self.current_angle)
				self.logger.info(f"Initial angle: {self.current_angle}°")
				
				# Get map dimensions with margin
				map_height = self.global_map_curr.info.height
				map_width = self.global_map_curr.info.width
				edge_margin = 12
				
				# Starting position
				start_x, start_y = self.current_pos
				
				# Calculate endpoint at map boundary in the given direction
				# Step through the direction until we hit boundary
				step_size = 10
				current_x, current_y = start_x, start_y
				
				while (edge_margin < current_x < map_width - edge_margin and 
					   edge_margin < current_y < map_height - edge_margin):
					current_x += step_size * math.cos(angle_rad)
					current_y += step_size * math.sin(angle_rad)
				
				# Step back one to stay within bounds
				endpoint_x = int(current_x - step_size * math.cos(angle_rad))
				endpoint_y = int(current_y - step_size * math.sin(angle_rad))
				
				self.logger.info(f"Map endpoint found at: ({endpoint_x}, {endpoint_y})")
				
				# Convert to world coordinates
				world_self_center = self.get_world_coord_from_map_coord(
					endpoint_x, 
					endpoint_y, 
					map_info
				)
				self.logger.info(f"Calculated world self center: {world_self_center}")

			for fy, fx in frontiers:
				fx_world, fy_world = self.get_world_coord_from_map_coord(fx, fy, map_info)
				distance = euclidean((fx_world, fy_world), world_self_center)
				if (distance < min_distance_curr and
					distance <= self.max_step_dist_world_meters and
					distance >= self.min_step_dist_world_meters):
					min_distance_curr = distance
					closest_frontier = (fy, fx)

			if closest_frontier:
				fy, fx = closest_frontier
				
				# NEW: Store the frontier goal for distance checking
				self.current_frontier_goal = [(fx, fy),0]
				
				self.logger.info(f'\nFound frontier closest at: ({fx}, {fy})')
				self.logger.info(f'World coordinates: ({fx_world}, {fy_world})')
				goal = self.create_goal_from_map_coord(fx, fy, map_info)
				self.send_goal_from_world_pose(goal)
				return
			else:
				self.max_step_dist_world_meters += 2.0
				new_min_step_dist = self.min_step_dist_world_meters - 1.0
				self.min_step_dist_world_meters = max(0.25, new_min_step_dist)

			self.full_map_explored_count = 0
		else:
			self.full_map_explored_count += 1



	# ----------------------- NAVIGATION FUNCTIONS ----------------------- 



	def handle_nav_to_shelf(self):
		"""
		Handles the navigation logic for moving the robot to a specified shelf location.
		This method determines the appropriate navigation goal for the robot based on its current position,
		the location and orientation of the target shelf, and the robot's alignment with the shelf. It computes
		front and back approach points for the shelf, selects the optimal goal, and sends navigation commands.
		The method also manages state transitions based on proximity to the shelf and handles error cases where
		the shelf or target point cannot be found.
		Parameters:
			None (operates on instance attributes):
				- self.current_shelf_id (int): The ID of the shelf to navigate to.
				- self.current_angle (float): The desired approach angle to the shelf in degrees.
				- self.global_map_curr (OccupancyGrid): The current global map.
				- self.world_centre (tuple): The robot's current position in world coordinates.
				- self.buggy_pose_x (float): The robot's current x position.
				- self.buggy_pose_y (float): The robot's current y position.
				- self.goal_completed (bool): Whether the previous goal has been completed.
				- self.logger (Logger): Logger for status and error messages.
				- Other instance attributes used for navigation and state management.
		Returns:
			None
		"""
		
		if not self.goal_completed:
			if self.goal_type == 1:
				map_array = np.array(self.global_map_curr.data).reshape((self.global_map_curr.info.height, self.global_map_curr.info.width))
				target_point, shelf_info = self.find_shelf_and_target_point(slam_map= map_array, robot_pos=self.world_centre, shelf_angle_deg=self.current_angle, search_distance=400)
				front,back = self.find_front_back_points(shelf_info['center'], shelf_info, 47,False)
				buggy_mapcoord = self.get_map_coord_from_world_coord(self.buggy_pose_x, self.buggy_pose_y, self.global_map_curr.info)
				if self.calc_distance(buggy_mapcoord, front) < self.calc_distance(buggy_mapcoord, back):
					yaw = self.find_angle_point_direction(self.shelf_info['center'], front)
				else:
					yaw = self.find_angle_point_direction(self.shelf_info['center'], back)
				rob_angle = math.degrees(self.get_current_robot_yaw())
				if rob_angle <0:rob_angle+=360
				self.logger.info(f"DIFF : {np.abs(yaw-rob_angle)}")
				if (self.calc_distance(buggy_mapcoord, front) < 8.5 or self.calc_distance(buggy_mapcoord, back) < 8.5) and np.abs(yaw-rob_angle) < 13:
					self.logger.info("Target reached!!!!!!!!!!!")
					self.cancel_current_goal()
					self.goal_type = 0
			return

		self.logger.info(f"Navigating to shelf number {self.current_shelf_id} at angle {self.current_angle}°")
		map_array = np.array(self.global_map_curr.data).reshape((self.global_map_curr.info.height, self.global_map_curr.info.width))
		target_point, shelf_info = self.find_shelf_and_target_point(slam_map= map_array, robot_pos=self.world_centre, shelf_angle_deg=self.current_angle, search_distance=400)
		self.logger.info(f"Target point: {target_point}")	
		self.shelf_info = shelf_info
		# If a target point is found, navigate to it
		if target_point is not None:
			self.logger.info(f"Found target point: {target_point} shelf info: {shelf_info}")
			self.current_shelf_centre = shelf_info['center']
			self.current_shelf_orientation = shelf_info['rotation_angle']
			yaw = self.current_angle
			buggy_mapcoord = self.get_map_coord_from_world_coord(self.buggy_pose_x, self.buggy_pose_y, self.global_map_curr.info)
			front,back = self.find_front_back_points(self.current_shelf_centre, shelf_info, 47,False)
			direction = self.shelf_info['orientation']['secondary_direction']
			if self.calc_distance(buggy_mapcoord, front) < self.calc_distance(buggy_mapcoord, back):
				goal_x, goal_y = float(front[0]), float(front[1])
				yaw = self.find_angle_point_direction(self.shelf_info['center'], front, direction)
				self.logger.info(f'\nNavigating to Front point: ({goal_x:.2f}, {goal_y:.2f}) with yaw {yaw:.2f}°')
			else:
				goal_x, goal_y = float(back[0]), float(back[1])
				yaw = self.find_angle_point_direction(self.shelf_info['center'], back, direction)
				self.logger.info(f'\nNavigating to Back point: ({goal_x:.2f}, {goal_y:.2f}) with yaw {yaw:.2f}°')
		
			if (self.calc_distance(buggy_mapcoord, front) < 6 or self.calc_distance(buggy_mapcoord, back) < 6):
				self.logger.info("\n\nRobot is aligned with shelf\n\n")
				self.current_state = self.CAPTURE_OBJECTS

			elif self.calc_distance(buggy_mapcoord, front) < 40 or self.calc_distance(buggy_mapcoord, back) < 40:
				if sum(self.shelf_objects_curr.object_count) == 6:
					self.logger.info("All objects already detected, navigating to shelf for object capture.")
					self.current_state = self.CAPTURE_OBJECTS
					return
				self.logger.info(f"NUM CAPTURED OBJECTS: {sum(self.shelf_objects_curr.object_count)}")
				if self.calc_distance(buggy_mapcoord, front) < self.calc_distance(buggy_mapcoord, back):
					goal_x, goal_y = float(front[0]), float(front[1])
					yaw = self.find_angle_point_direction(self.shelf_info['center'], front, direction)
					self.logger.info(f'\nNavigating to Front point: ({goal_x:.2f}, {goal_y:.2f}) with yaw {yaw:.2f}°')
				else:
					goal_x, goal_y = float(back[0]), float(back[1])
					yaw = self.find_angle_point_direction(self.shelf_info['center'], back, direction)
					self.logger.info(f'\nNavigating to Back point: ({goal_x:.2f}, {goal_y:.2f}) with yaw {yaw:.2f}°')
				self.logger.info(f'\nmap cords of goal: ({goal_x:.2f}, {goal_y:.2f})')
				goal_x, goal_y = self.get_world_coord_from_map_coord(goal_x, goal_y, self.global_map_curr.info)
				goal = self.create_goal_from_world_coord(goal_x, goal_y, math.radians(yaw))
				if self.send_goal_from_world_pose(goal):
					self.logger.info(f"IDEAL Goal sent to ({goal_x:.2f}, {goal_y:.2f}) with yaw {yaw:.2f}°")
					self.goal_type = 0
				else:
					self.logger.error("Failed to send navigation goal!")
			else:
				# If the robot is far from the shelf, navigate to the target point
				goal_x, goal_y = float(target_point[0]), float(target_point[1])
				self.logger.info(f'\nmap cords of goal: ({goal_x:.2f}, {goal_y:.2f})')
				# update yaw if target is closer to front  or back
				if self.calc_distance(target_point, front) < self.calc_distance(target_point, back):
					yaw = self.find_angle_point_direction(self.shelf_info['center'], front, direction)
				else:
					yaw = self.find_angle_point_direction(self.shelf_info['center'], back, direction)
				goal_x, goal_y = self.get_world_coord_from_map_coord(goal_x, goal_y, self.global_map_curr.info)
				goal = self.create_goal_from_world_coord(goal_x, goal_y, math.radians(yaw))

				if self.send_goal_from_world_pose(goal):
					self.logger.info(f"TEMPORARY Goal sent to ({goal_x:.2f}, {goal_y:.2f}) with yaw {yaw:.2f}°")
					self.goal_sent = False
					self.goal_type = 1
				else:
					self.logger.error("Failed to send navigation goal!")
		else:
			self.logger.error("No target point found for shelf navigation!")
			self.logger.info(f"world centre: {self.world_centre}, current shelf centre: {self.current_shelf_centre}")
			map_array = np.array(self.global_map_curr.data).reshape((self.global_map_curr.info.height, self.global_map_curr.info.width))
			np.save('shelf_not_found.npy', map_array)
			self.current_state = self.EXPLORE



	# ----------------------- OBJECT DETECTION FUNCTIONS ----------------------- 



	def handle_capture_objects(self):
		"""
		Handles the object detection process when the robot reaches the shelf.
		This method manages the state where the robot attempts to detect objects on a shelf.
		It initiates object detection if not already started, checks for detection results or timeout,
		and transitions to the next state based on the outcome.
		Parameters:
			self: The instance of the class containing state, logger, and detection attributes.
		Returns:
			None. Updates internal state and attributes based on detection results or timeout.
		"""

		if not self.goal_completed:
			return  # Still navigating to shelf
		if not self.should_detect_objects:
			self.logger.info("Starting object detection...")
			self.should_detect_objects = True
			self.current_shelf_objects = None
			self.detection_start_time = time.time()
			return
		
		# Check if we have objects or timeout
		if self.current_shelf_objects is not None:
			self.should_detect_objects = False
			self.logger.info(f"\n\nObjects detected: {self.current_shelf_objects.object_name}")
			self.shelf_objects_curr.object_name = self.current_shelf_objects.object_name
			self.shelf_objects_curr.object_count = self.current_shelf_objects.object_count
			self.logger.info(f"shelf objects curr: {self.shelf_objects_curr}")
			self.current_state = self.SCAN_QR
			if sum(self.shelf_objects_curr.object_count) != 6:
				# Handle fix alignment again -- not implemented yet
				self.current_state = self.ADJUST
			if sum(self.shelf_objects_curr.object_count) ==7 and self.current_shelf_id == 3:
				for i in range(len(self.current_shelf_objects.object_name)):
					if self.current_shelf_objects.object_name[i] == "clock":
						self.shelf_objects_curr.object_name[i] = self.current_shelf_objects.object_name[i]
						self.shelf_objects_curr.object_count[i] = self.current_shelf_objects.object_count[i] - 1
				self.logger.info(f"OH OHHH 3, shelf objects curr: {self.shelf_objects_curr}")
		elif time.time() - self.detection_start_time > 5.0:  # 5 second timeout
			self.should_detect_objects = False
			self.logger.warn("Object detection timeout")
			self.logger.info("Moving to QR scanning anyway...")



	# ----------------------- QR CODE FUNCTIONS ----------------------- 



	def handle_navigate_to_qr_side(self):
		"""
		Handles navigation of the robot to the side of the current shelf for QR code scanning.
		This method checks if the robot has reached the correct shelf based on the QR code data.
		If the QR code matches the current shelf, it cancels the navigation goal if necessary.
		Otherwise, it computes the optimal side (left or right) of the shelf to approach for scanning,
		based on the robot's current position and the shelf's orientation. It then sends a navigation
		goal to move the robot to the selected side of the shelf.
		Parameters:
			None (uses instance attributes such as current_qr_data, current_shelf_id, global_map_curr, etc.)
		Returns:
			None
		"""

		if self.qr_array[self.current_shelf_id - 1] !=0:
			self.logger.info(f"QR code already scanned for shelf {self.current_shelf_id}, skipping navigation.")
			self.cancel_current_goal()
			self.current_qr_data = self.qr_array[self.current_shelf_id - 1]
			self.qr_reached == True
			self.goal_completed = True
			self.current_state = self.EXPLORE
			self.handle_qr_scan()
			return
		
		if (self.current_qr_data is not None and 
			self.validate_qr_format(self.current_qr_data)):
			try:
				if self.qr_array[self.current_shelf_id - 1] !=0:
					self.logger.info(f"QR code already scanned for shelf {self.current_shelf_id}, skipping navigation.")
					self.cancel_current_goal()
					self.handle_qr_scan()
					return
				# qr_shelf_id = int(self.current_qr_data.split('_')[0])
				# if qr_shelf_id == self.current_shelf_id:
				# 	self.logger.info("Cancelling QR navigation and moving on..")
				# 	if not self.goal_completed and self.goal_handle_curr is not None:
				# 		self.cancel_current_goal()
				# 		self.handle_qr_scan()
				# 		return
				
					# return
			except (ValueError, IndexError):
				self.logger.warn(f"Failed to parse QR shelf ID from: {self.current_qr_data}")
		
		if not self.goal_completed or self.qr_reached:
			return  # Still moving
			
		self.logger.info(f"\nMoving to side of shelf {self.current_shelf_id} for QR scan")
		map_array = np.array(self.global_map_curr.data).reshape((self.global_map_curr.info.height, self.global_map_curr.info.width))
		if self.current_shelf_centre is None or self.current_shelf_orientation is None:
			target_point, shelf_info = self.find_shelf_and_target_point(slam_map= map_array, robot_pos=self.world_centre, shelf_angle_deg=self.current_angle, search_distance=400)
			if target_point is not None:
				self.logger.info(f"Found target point: {target_point} shelf info: {shelf_info}")
				self.current_shelf_centre = shelf_info['center']
				self.current_shelf_orientation = shelf_info['rotation_angle']

		left,right = self.find_front_back_points(self.current_shelf_centre, self.shelf_info, 50,True)
		self.logger.info(f"Left point: {left}, Right point: {right}")
		direction = self.shelf_info['orientation']['primary_direction']
		# Choose the point (left or right) with the shortest distance to the robot
		dist_left = self.calc_distance(self.buggy_center, left)
		dist_right = self.calc_distance(self.buggy_center, right)

		if dist_left < dist_right:
			goal_x, goal_y = float(left[0]), float(left[1])
			yaw = self.find_angle_point_direction(self.shelf_info['center'], left, direction)
		else:
			goal_x, goal_y = float(right[0]), float(right[1])
			yaw = self.find_angle_point_direction(self.shelf_info['center'], right, direction)
		
		goal_x, goal_y = self.get_world_coord_from_map_coord(goal_x, goal_y, self.global_map_curr.info)	
		goal = self.create_goal_from_world_coord(goal_x, goal_y, math.radians(yaw))
		if self.send_goal_from_world_pose(goal):
			self.logger.info(f"Goal sent to ({goal_x:.2f}, {goal_y:.2f}) with yaw {yaw:.2f} degree")
			self.qr_reached = True
		else:
			self.logger.error("Failed to send navigation goal!")
			self.goal_sent = False
			self.qr_reached = False
			self.current_state = self.DO_NOTHING

	def handle_qr_scan(self):
		"""
		Handles the process of scanning a QR code at a designated shelf location.
		This method manages the state transitions and logic for initiating a QR scan, 
		handling the scan result, publishing the decoded QR data, and updating the robot's 
		exploration state. It also manages timeouts if a QR code is not detected within a 
		specified duration.
		Parameters:
			None (operates on instance variables of the class)
		Returns:
			None
		Workflow:
			- Checks if the robot has reached the QR position and if the goal is completed.
			- Initiates QR scanning if not already started.
			- If QR data is captured, processes and publishes the data, updates exploration state, 
			  and prepares for the next shelf.
			- If QR data is not captured within the timeout period, logs a warning and halts further actions.
		"""
		
		if not self.goal_completed or self.qr_reached == False:
			return  # Still moving to QR position
		
		self.logger.info("Starting QR code scan...")
		if not self.should_detect_qr:
			
			self.should_detect_qr = True
			self.current_qr_data = None
			self.qr_scan_start_time = time.time()
			return
		
		# Check if we have QR data or timeout
		if self.current_qr_data is not None:
			self.should_detect_qr = False
			self.logger.info(f"\n\nQR Code captured: {self.current_qr_data}")
			self.shelf_objects_curr.qr_decoded = self.current_qr_data
			#publish shel current
			self.publisher_shelf_data.publish(self.shelf_objects_curr)
			self.send_request_to_server(self.shelf_objects_curr)
			self.current_shelf_objects = None  # Reset objects after QR scan
			self.qr_reached = False  # Reset QR reached flag

			# reset next goals
			self.shelf_objects_curr = WarehouseShelf()
			self.current_angle = self.parse_qr_for_next_angle(self.current_qr_data)
			self.current_pos = self.world_centre
			self.world_centre = self.current_shelf_centre
			self.current_shelf_id +=1
			if self.current_shelf_id > self.shelf_count:
				self.logger.info("\n\n\nExploration complete, no more shelves to process.\n\n\n")
				self.current_state = self.DO_NOTHING
				return
			self.adjusting = 1
			self.current_shelf_centre = None
			self.current_shelf_orientation = None
			self.current_state = self.EXPLORE
			
			
		elif time.time() - self.qr_scan_start_time > 5.0:  # 15 second timeout
			self.should_detect_qr = False
			self.logger.warn("QR scan timeout")
			self.current_state = self.DO_NOTHING


	def send_request_to_server(self, shelf_data):
		import requests
		url = 'https://backend-nxp.vercel.app/upload/'
		object_names = shelf_data.object_name
		object_counts = shelf_data.object_count
		data_dict = {name: count for name, count in zip(object_names, object_counts)}

		key = "MotorBrew-" + shelf_data.qr_decoded
		data = {
			"data": data_dict,
		}
		headers = {
			"x-api-key": key
		}

		try:
			requests.post(url, json=data, headers=headers)
		except requests.exceptions.RequestException as e:
			self.logger.info(f"❌ Error: {e}")

	# ----------------------- SHELF FUNCTIONS ----------------------- 



	def find_angle_point_direction(self,center, point, direction = None):
		"""
		Calculates the angle (in degrees) from a given point to a center, normalized to the [0, 360) range.
		This function computes the direction that a point should face to look towards a specified center, 
		taking into account the provided direction. The result is logged and returned as a degree value.
		Args:
			center (tuple): A tuple (cx, cy) representing the coordinates of the center point.
			point (tuple): A tuple (px, py) representing the coordinates of the point whose direction is being calculated.
			direction: Unused parameter, reserved for future use or interface compatibility.
		Returns:
			float: The angle in degrees from the point to the center, normalized to the [0, 360) range.
		"""
		cx, cy = center
		px, py = point
		to_center_x = cx - px  
		to_center_y = cy - py  
		
		angle_to_center = np.arctan2(to_center_y, to_center_x)
		relative_angle_deg = np.degrees(angle_to_center)

		if relative_angle_deg > 180:
			relative_angle_deg -= 360
		elif relative_angle_deg < -180:
			relative_angle_deg += 360
		if relative_angle_deg < 0:
			relative_angle_deg += 360
		self.logger.info(f" ANGLE to center: {relative_angle_deg}°")
		return relative_angle_deg

	def find_shelf_and_target_point(self, slam_map, robot_pos, shelf_angle_deg, search_distance=100):
		"""
		Detects a shelf within a specified corridor in a SLAM map and determines a safe target point for the robot to approach.
		This method searches for obstacles (representing shelves) along a corridor extending from the robot's current position
		in a specified direction. It validates detected obstacles based on distance from the robot and map edges, then attempts
		to identify a shelf with expected dimensions. If a valid shelf is found, it computes a safe approach point for the robot.
		Parameters:
			slam_map (np.ndarray): 2D numpy array representing the SLAM occupancy grid map. 
									Obstacles (shelves) are expected to have values 99 or 100.
			robot_pos (tuple): (x, y) coordinates of the robot's current position in map units.
			shelf_angle_deg (float): Angle in degrees indicating the direction to search for the shelf, relative to the robot.
			search_distance (int, optional): Maximum distance (in map units) to search for the shelf. Default is 100.
		Returns:
			tuple:
				target_point (tuple or None): (x, y) coordinates of a safe point to approach the detected shelf, or None if not found.
				shelf_info (dict or None): Dictionary containing shelf properties (center, dimensions, etc.), or None if no valid shelf is detected.
		"""
		
		# Convert angle to radians
		angle_rad = np.radians(shelf_angle_deg)
		
		# Create a mask for obstacles (shelves are typically obstacles)
		obstacle_mask = (slam_map == 99) | (slam_map == 100)
		
		# Create search line in the specified direction
		robot_x, robot_y = robot_pos
		min_search_distance = 30  # Exclude 30-unit margin around robot
		edge_margin = 12  # Exclude 5-unit margin from map edges
		map_height, map_width = slam_map.shape
		search_width = 3  # Search ±3 units perpendicular to the main direction
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
		
		# Find obstacles along the search corridor
		obstacles_on_line = []
		for x, y in search_points:
			if obstacle_mask[y, x]:  # Note: y first for numpy array indexing
				obstacles_on_line.append((x, y))
		
		if not obstacles_on_line:
			return None, None
		
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
			return None, None
		# Use valid obstacles (beyond both margins) for shelf detection
		shelf_info = self.detect_shelf_with_orientation(slam_map, valid_obstacles, robot_pos, angle_rad)
		if shelf_info:
			h = shelf_info['dimensions']['height']
			w = shelf_info['dimensions']['width']
			# self.logger.info(f"Detect	ed shelf with height {h} and width {w} in the search corridor")
			if h>w: h,w=w,h
			if 26<w<=40 and 9<h<=22:
				pass
			else:
				return self.find_shelf_and_target_point(slam_map,shelf_info['center'],shelf_angle_deg,search_distance)
		else:
			return None, None
		# Find a safe point to move towards the shelf
		target_point = None
		if shelf_info and shelf_info['center']:
			target_point = self.find_safe_approach_point(slam_map, robot_pos, shelf_info, angle_rad)
		
		return target_point, shelf_info

	def detect_shelf_with_orientation(self, slam_map, obstacles_on_line, robot_pos, angle_rad):
		"""
		Detects a shelf in a SLAM map along a specified search line and estimates its orientation.
		This method processes a list of obstacle points (typically detected along a search line)
		to identify the most likely shelf structure within the map. It filters out obstacles near
		the map edges, clusters the remaining obstacles, selects the cluster (connected component)
		closest to the search line, and computes the shelf's center and orientation using PCA and
		minimum area rectangle fitting.
		Parameters:
			slam_map (np.ndarray): 2D numpy array representing the SLAM occupancy grid map.
									Obstacle cells are expected to have values 99 or 100.
			obstacles_on_line (list of tuple): List of (x, y) tuples representing obstacle coordinates
												detected along the search line.
			robot_pos (tuple): (x, y) coordinates of the robot's current position in map coordinates.
			angle_rad (float): Angle (in radians) of the search line along which obstacles were detected.
		Returns:
			dict or None: Dictionary containing shelf information if a shelf is detected, else None.
				The dictionary contains:
					- 'center': (x, y) coordinates of the shelf center (int, int)
					- 'orientation': Output of self.calculate_shelf_orientation(shelf_points)
					- 'corners': List of 4 (x, y) tuples for the corners of the minimum area rectangle
					- 'dimensions': Dict with 'width', 'height', and 'area' (number of pixels)
					- 'rotation_angle': Rotation angle (degrees) of the minimum area rectangle [0, 180)
					- 'rect_center': (x, y) center of the minimum area rectangle (float, float)
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
			return None
		
		
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

		
		# Extract ROI
		roi = slam_map[roi_y1:roi_y2, roi_x1:roi_x2]
		
		# Create obstacle mask for ROI
		obstacle_roi = (roi == 99) | (roi == 100)
		
		# Find connected components
		num_features, labeled = cv2.connectedComponents(obstacle_roi.astype(np.uint8))
		
		if num_features == 0:
			return None
		
		# NEW: Find component whose center is closest to the search line
		robot_x, robot_y = robot_pos
		search_line_dx = np.cos(angle_rad)
		search_line_dy = np.sin(angle_rad)
		
		component_info = []
		for i in range(1, num_features + 1):
			component_mask = (labeled == i)
			size = np.sum(component_mask)
			
			# Skip very small components
			if size < 10:
				continue
				
			# Get component coordinates
			y_coords, x_coords = np.where(component_mask)
			
			# Convert to global coordinates
			global_x_coords = x_coords + roi_x1
			global_y_coords = y_coords + roi_y1
			
			# Calculate component center
			center_x = np.mean(global_x_coords)
			center_y = np.mean(global_y_coords)
			
			# Calculate distance from component center to search line
			# Vector from robot to component center
			to_center_x = center_x - robot_x
			to_center_y = center_y - robot_y
			
			# Project onto search line direction to get along-line distance
			along_line_distance = to_center_x * search_line_dx + to_center_y * search_line_dy
			
			# Calculate perpendicular distance from center to search line
			# Point on search line closest to component center
			closest_point_x = robot_x + along_line_distance * search_line_dx
			closest_point_y = robot_y + along_line_distance * search_line_dy
			
			# Distance from component center to closest point on search line
			perp_distance = np.sqrt(
				(center_x - closest_point_x)**2 + (center_y - closest_point_y)**2
			)
			
			component_info.append({
				'label': i,
				'size': size,
				'center': (center_x, center_y),
				'perp_distance': perp_distance,
				'along_line_distance': along_line_distance
			})
			
		
		if not component_info:
			return None
		
		# Select component closest to search line (minimum perpendicular distance)
		best_component = min(component_info, key=lambda x: x['perp_distance'])
		selected_label = best_component['label']
		
		
		# Get the selected component
		largest_component = (labeled == selected_label)
		
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
			return None
		
		global_x_coords = np.array(final_x_coords)
		global_y_coords = np.array(final_y_coords)
		
		# Calculate shelf center
		center_x = int(np.mean(global_x_coords))
		center_y = int(np.mean(global_y_coords))
		
		
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

	def calculate_shelf_orientation(self, points):
		"""
		Calculates the orientation and geometric properties of a shelf given a set of 2D points.
		This method performs Principal Component Analysis (PCA) on the input points to determine the primary and secondary orientation angles of the shelf, as well as its aspect ratio and elongation. The primary and secondary directions correspond to the eigenvectors of the covariance matrix of the points, representing the main axes of the point distribution.
		Parameters:
			points (np.ndarray): A 2D NumPy array of shape (N, 2), where each row represents the (x, y) coordinates of a point on the shelf.
		Returns:
			dict: A dictionary containing the following keys:
				- 'primary_angle' (float): The angle (in degrees, [0, 180)) of the primary axis with respect to the x-axis.
				- 'secondary_angle' (float): The angle (in degrees, [0, 180)) of the secondary axis with respect to the x-axis.
				- 'primary_direction' (np.ndarray): The unit vector representing the primary axis direction.
				- 'secondary_direction' (np.ndarray): The unit vector representing the secondary axis direction.
				- 'aspect_ratio' (float): The ratio of the largest to the second largest eigenvalue, indicating the spread along the primary axis relative to the secondary.
				- 'elongation' (float): The square root of the aspect ratio, representing the elongation of the point distribution.
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



	# ----------------------- UTILITY FUNCTIONS ----------------------- 



	def find_front_back_points(self, shelf_center, shelf_info, distance, use_primary=True):
		"""
		Calculates the front and back points at a specified distance from the center of a shelf,
		along a given orientation direction.
		This method uses the orientation information from the shelf_info dictionary to determine
		the direction in which to compute the front and back points relative to the shelf center.
		The direction can be either the primary or secondary orientation, as specified.
		Args:
			shelf_center (tuple): The (x, y) coordinates of the shelf center.
			shelf_info (dict): Dictionary containing shelf metadata, must include an 'orientation'
				key with 'primary_direction' and 'secondary_direction' as (dx, dy) tuples.
			distance (float): The distance from the shelf center to compute the front and back points.
			use_primary (bool, optional): If True, use the primary direction; otherwise, use the secondary direction.
				Defaults to True.
		Returns:
			tuple: A tuple containing two (x, y) integer coordinate tuples:
				- front_point: The point at the specified distance in the chosen direction from the center.
				- back_point: The point at the specified distance in the opposite direction from the center.
		"""
		
		x, y = shelf_center
		
		# Get the orientation information from shelf_info
		if 'orientation' not in shelf_info:
			self.logger.warn("No orientation information in shelf_info")
			return (x, y), (x, y)
		
		orientation = shelf_info['orientation']
		
		# Choose which direction to use
		if use_primary:
			direction_vector = orientation['primary_direction']
			direction_name = "primary"
		else:
			direction_vector = orientation['secondary_direction']
			direction_name = "secondary"
		
		# Extract direction components
		dx, dy = direction_vector[0], direction_vector[1]
		
		# Normalize the direction vector (should already be normalized from PCA, but ensure it)
		magnitude = np.sqrt(dx**2 + dy**2)
		if magnitude == 0:
			self.logger.warn("Zero magnitude direction vector")
			return (x, y), (x, y)
		
		dx_norm = dx / magnitude
		dy_norm = dy / magnitude
		
		front_point = (
			int(x + distance * dx_norm),
			int(y + distance * dy_norm)
		)
		back_point = (
			int(x - distance * dx_norm),
			int(y - distance * dy_norm)
		)

		return front_point, back_point
	
	def is_free_space(self, map_array, point):
		"""
		Checks if a given point corresponds to free space in a 2D occupancy grid map.
		Args:
			map_array (np.ndarray): A 2D numpy array representing the occupancy grid map,
				where 0 indicates free space and non-zero values indicate obstacles or unknown space.
			point (tuple or list): A tuple or list of two numeric values (x, y) representing
				the coordinates to check in the map.
		Returns:
			bool: True if the specified point is within the bounds of the map and corresponds
				to free space (i.e., the value at that location is 0), False otherwise.
		"""
		
		x, y = int(point[0]), int(point[1])
		if 0 <= x < map_array.shape[1] and 0 <= y < map_array.shape[0]:
			return map_array[y, x] == 0
		return False
		
	def parse_qr_for_next_angle(self, qr_data):
		"""
		Parses the QR code data to extract the next angle value.
		The method expects the QR code data to be a string with parts separated by underscores ('_').
		It attempts to extract the second part (index 1) and convert it to a float, which represents the next angle.
		Args:
			qr_data (str): The QR code data string, expected in the format 'prefix_angle[_...]'.
		Returns:
			float or None: The extracted angle as a float if parsing is successful; otherwise, None.
		"""
		
		try:
			parts = qr_data.split('_')
			if len(parts) >= 2:
				next_angle = float(parts[1])
				return next_angle
		except (ValueError, IndexError):
			self.logger.error(f"Failed to parse QR code: {qr_data}")
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
		"""
		Retrieves the current yaw (rotation around the Z-axis) of the robot in radians.
		This method extracts the orientation from the current pose, converts the quaternion
		orientation to a yaw angle in radians, and returns it. If the current pose is not available,
		it returns None.
		Returns:
			float or None: The current yaw of the robot in radians if the pose is available,
			otherwise None.
		"""
		
		if self.pose_curr is not None:
			orientation = self.pose_curr.pose.pose.orientation
			yaw_radians = self.get_yaw_from_quaternion(orientation)
			return yaw_radians  
		return None

	def calc_distance(self, point1, point2):
		"""
		Calculates the Euclidean distance between two 2D points.
		Args:
			point1 (tuple or list): The (x, y) coordinates of the first point.
			point2 (tuple or list): The (x, y) coordinates of the second point.
		Returns:
			float: The Euclidean distance between point1 and point2.
		"""

		return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

	def adjust_robot_orientation(self, angle):
		"""
		Adjusts the robot's orientation by calculating a new goal position based on the specified angle.
		This method computes a target (x, y) position that is 0.5 units behind the robot's current position
		in the direction opposite to the given angle. It uses the robot's current yaw to determine if the
		operation can proceed.
		Parameters:
			angle (float): The target orientation angle in radians, relative to the global frame.
		Returns:
			tuple:
				goal_x (float or None): The computed x-coordinate for the robot to move to, or None if the current yaw is unavailable.
				goal_y (float or None): The computed y-coordinate for the robot to move to, or None if the current yaw is unavailable.
		"""

		curr_robot_angle = self.get_current_robot_yaw()
		if curr_robot_angle is not None:
			# move back to 10 units
			goal_x = self.buggy_pose_x - .5 * math.cos(angle)
			goal_y = self.buggy_pose_y - .5 * math.sin(angle)
			return goal_x, goal_y
		return None, None
	
	def find_percentage_of_free_space(self, map_array):
		"""
		Calculates the percentage of free space in a given occupancy grid map.
		This method assumes that free space in the map is represented by the value 0.
		It computes the ratio of free cells to the total number of cells in the map and returns it as a percentage.
		Parameters:
			map_array (np.ndarray): A NumPy array representing the occupancy grid map,
									where free space is indicated by 0.
		Returns:
			float: The percentage of free space in the map.
		"""
		
		free_space_count = np.sum(map_array == 0)
		total_cells = map_array.size
		return (free_space_count / total_cells) * 100
	
	def find_percentage_of_free_space_around_point(self, map_array, point, radius):
		"""
		Calculates the percentage of free space (cells with value 0) within a circular area around a given point in a 2D map array.
		Parameters:
			map_array (np.ndarray): 2D numpy array representing the map, where 0 indicates free space and other values indicate obstacles or unknown areas.
			point (tuple): (x, y) coordinates of the center point around which to calculate free space.
			radius (int or float): Radius of the circular area (in cells) to consider around the point.
		Returns:
			float: Percentage of free space within the specified circular area. Returns 0 if no valid cells are found within the area.
		"""
		
		x, y = int(point[0]), int(point[1])
		radius = int(radius)
		free_space_count = 0
		total_cells = 0
		
		for dy in range(-radius, radius + 1):
			for dx in range(-radius, radius + 1):
				if dx**2 + dy**2 <= radius**2:
					ny, nx = y + dy, x + dx
					if 0 <= nx < map_array.shape[1] and 0 <= ny < map_array.shape[0]:
						total_cells += 1
						if map_array[ny, nx] == 0:
							free_space_count += 1
		if total_cells > 0:
			return (free_space_count / total_cells) * 100
		return 0

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
		target_distance = 45
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
		"""
		Checks if a given point and its surrounding area within a specified radius are free (safe) on the SLAM map.
		This function iterates over a square region centered at the given point and checks if all cells within the radius are free (i.e., have a value of 0 in the SLAM map).
		Parameters:
			slam_map (np.ndarray): 2D numpy array representing the SLAM map, where 0 indicates a free cell and non-zero indicates an obstacle or unknown area.
			point (tuple): (x, y) coordinates of the center point to check for safety.
			radius (int, optional): The radius (in cells) around the point to check. Defaults to 3.
		Returns:
			bool: True if all cells within the specified radius are free (safe), False if any cell is occupied or out of bounds.
		"""
		
		x, y = point
		for dx in range(-radius, radius + 1):
			for dy in range(-radius, radius + 1):
				check_x, check_y = x + dx, y + dy
				if 0 <= check_x < slam_map.shape[1] and 0 <= check_y < slam_map.shape[0]:
					if slam_map[check_y, check_x] != 0:
						return False
		return True



# ----------------------- CALLBACK/DEBUG FUNCTIONS -----------------------



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
		self.logger.info(f"Map info: {map_info}")
		self.world_center = self.get_world_coord_from_map_coord(
			map_info.width / 2, map_info.height / 2, map_info
		)

	def global_map_callback(self, message):
		"""Callback function to handle global map updates.

		Args:
			message: ROS2 message containing the global map data.

		Returns:
			None
		"""
		self.global_map_curr = message
		
		if self.current_state == -1:
			self.world_centre = self.get_map_coord_from_world_coord(0,0, self.global_map_curr.info)
			self.current_pos = self.world_centre
			self.current_state = self.EXPLORE

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
		# if self.should_detect_qr:
		qr_codes = self.pyzbar.decode(image)
		if qr_codes:
			for qr_code in qr_codes:
				qr_data = qr_code.data.decode('utf-8')
				if self.validate_qr_format(qr_data):
					self.current_qr_data = qr_data
					# self.logger.info(f"info: {qr_data}")
					if self.qr_array[int(qr_data[0])-1] == 0:
						self.qr_array[int(qr_data[0])-1] = qr_data
						self.logger.info(f"[ QR ARRAY ]   : {self.qr_array}")	
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
		self.logger.info("CEREBRI STATUS CALLBACK")
		self.logger.info(f"MODE: {message.mode}, ARMING: {message.arming}")
		if message.mode == 3 and message.arming == 2:
			self.armed = True
		else:
			# Initialize and arm the CMD_VEL mode.
			# CHANGING TO MANUAL JATAYU : msg.buttons from [0, 1, 0, 0, 0, 0, 0, 1] to [1, 0, 0, 0, 0, 0, 0, 1]
			msg = Joy()
			msg.buttons = [0, 1, 0, 0, 0, 0, 0, 1]
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
		# Process the shelf objects as needed
		predicates = {'horse','car','banana','potted plant','clock','cup','zebra','teddy bear'}
		mapping = {'potted plant': 'plant', 'teddy bear': 'teddy','zebra': 'zebra', 'cup': 'cup', 'clock': 'clock','horse':'horse','car':'car','banana':'banana'}
		if self.should_detect_objects:
			filtered_object_names = []
			filtered_object_counts = []
			
			for obj_name, obj_count in zip(message.object_name, message.object_count):
				obj_name_lower = obj_name.lower()
				
				if obj_name_lower in predicates:
					filtered_object_names.append(mapping[obj_name])
					filtered_object_counts.append(obj_count)
			
			if filtered_object_names:
				# Create filtered message with only warehouse objects
				filtered_message = WarehouseShelf()
				filtered_message.object_name = filtered_object_names
				filtered_message.object_count = filtered_object_counts
				self.current_shelf_objects = filtered_message
		
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
		return goal_pose

	def create_goal_from_map_coord(self, map_x: int, map_y: int, map_info,
				       yaw: Optional[float] = None) -> PoseStamped:
		"""Creates a goal PoseStamped from map coordinates."""
		world_x, world_y = self.get_world_coord_from_map_coord(map_x, map_y, map_info)

		return self.create_goal_from_world_coord(world_x, world_y, yaw)


def main(args=None):
	rclpy.init(args=args)

	warehouse_explore = WarehouseExplore()

	rclpy.spin(warehouse_explore)

	warehouse_explore.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
