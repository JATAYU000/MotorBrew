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

from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import CompressedImage
from pyzbar import pyzbar

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

		# --- Robot State ---
		self.armed = False
		self.logger = self.get_logger()

		# --- Robot Pose ---
		self.pose_curr = PoseWithCovarianceStamped()
		self.buggy_pose_x = 0.0
		self.buggy_pose_y = 0.0
		self.buggy_center = (0.0, 0.0)
		self.world_center = (0.0, 0.0)
		self.buggy_map_xy = (0.0,0.0)
		self.robot_initial_angle = None

		# --- Map Data ---
		self.simple_map_curr = None
		self.global_map_curr = None

		# --- Goal Management ---
		self.xy_goal_tolerance = 0.27
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
		self.further_angle_point = None
		self.curr_frontier_goal = None

		# --- QR Code Data ---
		self.qr_code_str = None

		# --- Shelf Data ---
		self.shelf_objects_curr = WarehouseShelf()
		self.shelf_angle_deg = self.initial_angle
		self.prev_shelf_center = None
		self.current_shelf_objects = None
		self.search_point = None
		self.current_shelf_number = 1
		self._fb_dist = 42

		# --- State Machine ---
		self.current_state = -1
		self.EXPLORE = 0
		self.MOVE_TO_SHELF = 1
		self.CAPTURE_OBJECTS = 2
		self.MOVE_TO_QR = 3
		self.ADJUST_TO = 4
		self.DEBUG = 5
		self.start = time.time()

		self.send_request_to_server(rtype='reset')


	# -------------------- MOVE TO THE SHELF -----------------------

	def handle_move_to_shelf(self):
		self.front, self.back = self.find_front_back_points(self._fb_dist,False)
		direction = self.shelf_info['orientation']['secondary_direction']
		self.target_view_point = self.front if self.calc_distance(self.buggy_map_xy, self.front) < self.calc_distance(self.buggy_map_xy, self.back) else self.back
		yaw = self.find_angle_point_direction(self.shelf_info['center'], self.target_view_point, direction)
		goal_x, goal_y = self.get_world_coord_from_map_coord(float(self.target_view_point[0]), float(self.target_view_point[1]), self.global_map_curr.info)
		goal = self.create_goal_from_world_coord(goal_x, goal_y, math.radians(yaw))
		if self.send_goal_from_world_pose(goal):
			self.logger.info(f"NAV TO SHELF Goal sent to ({goal_x:.2f}, {goal_y:.2f}) with yaw {yaw:.2f}°")
			self.current_state = self.CAPTURE_OBJECTS
		else:
			self.logger.error("Failed to send navigation goal!")
		
	# -------------------- CAPTURE OBJECTS -------------------------

	def handle_capture_objects(self):
		if self.current_shelf_objects is not None:
			self.shelf_objects_curr.object_name = self.current_shelf_objects.object_name
			self.shelf_objects_curr.object_count = self.current_shelf_objects.object_count
			self.logger.info(f"Captured {self.shelf_objects_curr.object_count} objects: {self.shelf_objects_curr.object_name}")

			if sum(self.current_shelf_objects.object_count) >= 5:
				info = self.find_first_rectangle()
				if info is not None: self.shelf_info = info
				self.current_state = self.MOVE_TO_QR
			else:
				self.shelf_info = self.find_first_rectangle()
				self.logger.info("Adjusting position to capture all objects...")
				if self._fb_dist > 50: self._fb_dist -= 10
				else: self._fb_dist += 10
				self.current_state = self.MOVE_TO_SHELF
		else:
			self.logger.info("No shelf objects received yet.")

	# -------------------- QR PROCESSING --------------------

	def handle_qr_navigation(self):
		self.left, self.right = self.find_front_back_points(50,True)
		direction = self.shelf_info['orientation']['primary_direction']
		self.target_view_point = self.left if self.calc_distance(self.buggy_map_xy, self.left) < self.calc_distance(self.buggy_map_xy, self.right) else self.right
		yaw = self.find_angle_point_direction(self.shelf_info['center'], self.target_view_point, direction)
		goal_x, goal_y = self.get_world_coord_from_map_coord(float(self.target_view_point[0]), float(self.target_view_point[1]), self.global_map_curr.info)
		self.qr_yaw = yaw
		goal = self.create_goal_from_world_coord(goal_x, goal_y, math.radians(yaw))
		if self.send_goal_from_world_pose(goal):
			self.logger.info(f"NAV TO QR Goal sent to ({goal_x:.2f}, {goal_y:.2f}) with yaw {yaw:.2f}°")
			self.current_state = self.ADJUST_TO
		else:
			self.logger.error("Failed to send navigation goal!")
		
	def adjust_qr(self):
		if self.qr_code_str is None:
			angle_rn = self.get_yaw_from_quaternion(self.pose_curr.pose.pose.orientation)
			# give new goal units in front of the robot
			unit = 15
			self.buggy_map_xy = self.get_map_coord_from_world_coord(self.buggy_pose_x, self.buggy_pose_y, self.global_map_curr.info)

			goalp_x = self.buggy_map_xy[0] + unit * math.cos(self.qr_yaw)
			goalp_y = self.buggy_map_xy[1] + unit * math.sin(self.qr_yaw)
			goaln_x = self.buggy_map_xy[0] - unit * math.cos(self.qr_yaw)
			goaln_y = self.buggy_map_xy[1] - unit * math.sin(self.qr_yaw)
			dist_p = self.calc_distance((goalp_x, goalp_y), self.shelf_info['center'])
			dist_n = self.calc_distance((goaln_x, goaln_y), self.shelf_info['center'])
			if dist_p < dist_n:
				goal_x, goal_y = goalp_x, goalp_y
			else:
				goal_x, goal_y = goaln_x, goaln_y
			
			goal_x,goal_y = self.get_world_coord_from_map_coord(goal_x, goal_y, self.global_map_curr.info)
			goal = self.create_goal_from_world_coord(goal_x, goal_y, math.radians(self.qr_yaw))
			self.logger.info(f"Adjusting to QR at map coords ({goal_x:.2f}, {goal_y:.2f}) with yaw {(self.qr_yaw):.2f}°")
			if self.send_goal_from_world_pose(goal):
				self.current_state = self.DEBUG
				self.logger.info(f"ADJUST TO QR Goal sent to ({goal_x:.2f}, {goal_y:.2f}) with yaw {(self.qr_yaw):.2f}°")
		else:
			return

	def get_next_angle(self):
		try:
			parts = self.qr_code_str.split('_')
			if len(parts) >= 2:
				next_angle = float(parts[1])
				return next_angle
		except (ValueError, IndexError):
			self.logger.error(f"Failed to parse QR code: {self.qr_code_str}")
		return None

	# -------------------- FRONTIER EXPLORATION --------------------


	def frontier_explore(self):
		self.shelf_info = self.find_first_rectangle()
		self.logger.info(f"SHELF INFO: {self.shelf_info}")
		self.logger.info(f"prev shelf center: {self.prev_shelf_center}")

		if self.shelf_info is not None and self.find_free_space_around_point(self.shelf_info['center'], radius=75) > 10:
			self.logger.info(f"Map is mostly free, skipping exp: {self.find_free_space_around_point(self.shelf_info['center'], radius=75)}% free")
			self.current_state = self.MOVE_TO_SHELF
			return
		
		self.logger.info("Exploring frontier...")
		frontiers = self.get_frontiers_for_space_exploration(self.map_array)
		self.logger.info(f"Found {len(frontiers)} frontiers in the map.")
		self.logger.info(f"world center: {self.world_center}, Current shelf info: {self.shelf_info}\n")
		
		if frontiers:
			closest_frontier = None
			min_distance_curr = float('inf')
			
			if self.shelf_info is not None:
				world_self_center = self.get_world_coord_from_map_coord(
					self.shelf_info['center'][0],
					self.shelf_info['center'][1],
					self.global_map_curr.info
				)
				# map coordinates of the reference center
				center_map_point = (int(self.shelf_info['center'][0]), int(self.shelf_info['center'][1]))
			else:
				self.logger.info(f"Initial world position: {self.prev_shelf_center}")
				
				angle_rad = math.radians(self.shelf_angle_deg)
				self.logger.info(f"Initial angle: {self.shelf_angle_deg}°")
				
				map_height = self.global_map_curr.info.height
				map_width = self.global_map_curr.info.width
				edge_margin = 12
				
				start_x, start_y = self.prev_shelf_center
				
				step_size = 10
				current_x, current_y = start_x, start_y
				
				while (edge_margin < current_x < map_width - edge_margin and 
					   edge_margin < current_y < map_height - edge_margin):
					current_x += step_size * math.cos(angle_rad)
					current_y += step_size * math.sin(angle_rad)
				
				endpoint_x = int(current_x - step_size * math.cos(angle_rad))
				endpoint_y = int(current_y - step_size * math.sin(angle_rad))
				
				self.logger.info(f"Map endpoint found at: ({endpoint_x}, {endpoint_y})")
				
				world_self_center = self.get_world_coord_from_map_coord(
					endpoint_x, 
					endpoint_y, 
					self.global_map_curr.info
				)
				self.logger.info(f"Calculated world self center: {world_self_center}")
				# use endpoint as map reference point
				center_map_point = (endpoint_x, endpoint_y)

			for fy, fx in frontiers:
				fx_world, fy_world = self.get_world_coord_from_map_coord(fx, fy, self.global_map_curr.info)
				distance = euclidean((fx_world, fy_world), world_self_center)
				if (distance < min_distance_curr and
					distance <= self.max_step_dist_world_meters and
					distance >= self.min_step_dist_world_meters):
					min_distance_curr = distance
					closest_frontier = (fy, fx)

			if closest_frontier:
				fy, fx = closest_frontier
				self.current_frontier_goal = [(fx, fy),0]
				self.logger.info(f'\nFound frontier closest at: ({fx}, {fy})')
				self.logger.info(f'World coordinates: ({fx_world}, {fy_world})')

				# Ensure chosen frontier is not too close to obstacles.
				# If it is, step back along the line from center_map_point to the frontier
				# until a cell with sufficient clearance is found.
				clearance_cells = 7       # radius to check for clearance
				required_free_pct = 90.0  # percent free required
				step_back_cells = 5       # how far to move back each attempt (in map cells)
				max_back_attempts = 8

				# starting candidate
				cand_x, cand_y = int(fx), int(fy)

				# quick check: if frontier cell or its neighborhood isn't clear, step back
				def candidate_is_safe(mx, my):
					if not (0 <= my < self.map_array.shape[0] and 0 <= mx < self.map_array.shape[1]):
						return False
					# cell must be free
					if self.map_array[my, mx] != 0:
						return False
					# neighborhood must be mostly free
					pct = self.find_free_space_around_point((mx, my), radius=clearance_cells)
					return pct >= required_free_pct

				if not candidate_is_safe(cand_x, cand_y):
					cx, cy = center_map_point
					vec_x = cand_x - cx
					vec_y = cand_y - cy
					norm = math.hypot(vec_x, vec_y)
					if norm == 0:
						self.logger.warn("Frontier coincides with center_map_point, cannot step back.")
					else:
						ux, uy = vec_x / norm, vec_y / norm
						found_safe = False
						for attempt in range(1, max_back_attempts + 1):
							new_x = int(cand_x - attempt * step_back_cells * ux)
							new_y = int(cand_y - attempt * step_back_cells * uy)
							if candidate_is_safe(new_x, new_y):
								self.logger.info(f"Adjusted frontier from ({cand_x},{cand_y}) to safer point ({new_x},{new_y})")
								cand_x, cand_y = new_x, new_y
								found_safe = True
								break
						if not found_safe:
							self.logger.warn("Could not find safer point near chosen frontier; using original frontier (may be near obstacle)")

				# create and send goal using adjusted candidate
				goal_x,goal_y = self.get_world_coord_from_map_coord(float(cand_x), float(cand_y), self.global_map_curr.info)
				goal = self.create_goal_from_world_coord(goal_x, goal_y, math.radians(self.shelf_angle_deg))
				self.curr_frontier_goal = (cand_x, cand_y)
				self.send_goal_from_world_pose(goal)
				return
			else:
				self.max_step_dist_world_meters += 2.0
				new_min_step_dist = self.min_step_dist_world_meters - 1.0
				self.min_step_dist_world_meters = max(0.25, new_min_step_dist)

			self.full_map_explored_count = 0
		else:
			self.full_map_explored_count += 1
	
	# -------------------- SHELF FINDING --------------------

	def find_first_rectangle(self,rect_fill_ratio=0.50,min_pixel_area=250,max_pixel_area=1200,ignore_radius=30):
		start_point = self.prev_shelf_center
		search_angle_deg = self.shelf_angle_deg

		binary_obstacle_map = np.zeros(self.map_array.shape, dtype=np.uint8)
		binary_obstacle_map[self.map_array == 99] = 255 # Mark obstacles
		binary_obstacle_map[self.map_array == 100] = 255 # Mark obstacles
		num_labels, labels_map = cv2.connectedComponents(binary_obstacle_map)
		
		if num_labels <= 1:
			self.logger.info("No obstacles (value 99 or 100) found in the map.")
			return None

		search_angle_rad = math.radians(search_angle_deg)
		map_height, map_width = labels_map.shape
		max_range = int(math.hypot(map_height, map_width))
		
		last_hit_object_id = -1 

		self.logger.info(f"Starting ray cast from {start_point} at {search_angle_deg} deg...")
		for t in range(ignore_radius, max_range):
			x = int(start_point[0] + t * math.cos(search_angle_rad))
			y = int(start_point[1] + t * math.sin(search_angle_rad))
			
			# Check boundaries
			if not (0 <= y < map_height and 0 <= x < map_width):
				break # Ray went out of bounds
				
			current_object_id = labels_map[y, x]
			
			if current_object_id == 0:
				last_hit_object_id = -1 
				continue 

			if current_object_id == last_hit_object_id:
				continue # Don't re-validate, just move along the ray
				
			self.logger.info(f"\nRay hit new object with ID: {current_object_id} at distance {t}")
			last_hit_object_id = current_object_id 
			object_mask = np.uint8(labels_map == current_object_id)
			points = cv2.findNonZero(object_mask)
			
			if points is None:
				self.logger.info("... Error: Could not extract points. Skipping.")
				continue # Skip this object
				
			object_area = len(points)
			if object_area < min_pixel_area or object_area > max_pixel_area:
				self.logger.info(f"... Object is too small/large (Area: {object_area}). Skipping.")
				continue 

			found_rect = cv2.minAreaRect(points)
			(center, (w, h), angle) = found_rect

			if w == 0 or h == 0:
				self.logger.info("... Object is just a line. Skipping.")
				continue # *** KEY CHANGE: Continue loop, don't return None

			bounding_box_area = w * h
			rectangularity_ratio = object_area / bounding_box_area

			if rectangularity_ratio >= rect_fill_ratio:
				oriented_w = max(w, h)
				oriented_h = min(w, h)
				wh_ratio = oriented_w / oriented_h
				if not (29 <= oriented_w <= 42):
					self.logger.info(f"  -> FAILURE: Width {oriented_w:.1f} not in [31, 42]. Skipping.")
					continue

				if not (1.7 <= wh_ratio <= 2.6):
					self.logger.info(f"  -> FAILURE: W/H ratio {wh_ratio:.2f} not in [1.8, 2.6]. Skipping.")
					continue
				
				self.logger.info(f"  -> SUCCESS: Object is a valid rectangle (Ratio > {rect_fill_ratio}).")

				box_points = cv2.boxPoints(found_rect)
				box_points = np.int0(box_points)
				points_array = np.squeeze(points)
				orientation_info = self.calculate_shelf_orientation(points_array)
				
				return {
					"status": "Rectangle Found",
					"object_id": current_object_id,
					"center": center,
					"dimensions": (w, h),
					"angle": angle,
					"fill_ratio": rectangularity_ratio,
					"orientation": orientation_info,
					"box_points": box_points
				}
			else:
				self.logger.info(f"  -> FAILURE: Not a ratio. Skipping.")
				continue 

		self.logger.info("\nRay search finished without finding a valid rectangle.")
		return None
	
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

	def find_front_back_points(self, distance, use_primary=True):
			"""
			Calculates the front and back points at a specified distance from the center of a shelf,
			along a given orientation direction.
			This method uses the orientation information from the shelf_info dictionary to determine
			the direction in which to compute the front and back points relative to the shelf center.
			The direction can be either the primary or secondary orientation, as specified.
			Args:
				self.shelf_info['center'] (tuple): The (x, y) coordinates of the shelf center.
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
			
			x, y = self.shelf_info['center']
			if 'orientation' not in self.shelf_info:
				self.logger.warn("No orientation information in shelf_info")
				return (x, y), (x, y)

			orientation = self.shelf_info['orientation']
			if use_primary:
				direction_vector = orientation['primary_direction']
				direction_name = "primary"
			else:
				direction_vector = orientation['secondary_direction']
				direction_name = "secondary"
			
			dx, dy = direction_vector[0], direction_vector[1]
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
	

	# -------------------- UTILITY -------------------

	def find_free_space_around_point(self, point, radius):
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
					if 0 <= nx < self.map_array.shape[1] and 0 <= ny < self.map_array.shape[0]:
						total_cells += 1
						if self.map_array[ny, nx] == 0:
							free_space_count += 1
		if total_cells > 0:
			return (free_space_count / total_cells) * 100
		return 0
	
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
	
	def send_request_to_server(self, rtype='upload'):
		import requests
		if rtype == 'upload':
			url = 'https://backend-nxp.vercel.app/upload/'
			object_names = self.shelf_objects_curr.object_name
			object_counts = self.shelf_objects_curr.object_count
			data_dict = {name: count for name, count in zip(object_names, object_counts)}

			key = "MotorBrew-" + self.shelf_objects_curr.qr_decoded
			data = {"data": data_dict,}
			headers = {"x-api-key": key}
			try:
				requests.post(url, json=data, headers=headers)
			except requests.exceptions.RequestException as e:
				self.logger.info(f"❌ Error: {e}")

		if rtype == 'reset':
			url = 'https://backend-nxp.vercel.app/reset/'
			headers = {"x-api-key": 'MotorBrew'}
			data = {"data": {}}
			try:
				requests.post(url, json=data, headers=headers)
			except requests.exceptions.RequestException as e:
				self.logger.info(f"❌ Error: {e}")
			
	# -------------------- CALLBACKS --------------------

	def global_map_callback(self, message):
		"""Callback function to handle global map updates.

		Args:
			message: ROS2 message containing the global map data.

		Returns:
			None
		"""
		self.global_map_curr = message

		if self.qr_code_str is not None and (self.current_state == self.MOVE_TO_QR or self.current_state == self.ADJUST_TO):
			self.logger.info(f"\n\n\nQR code detected: {self.qr_code_str}, processing...")
			self.cancel_current_goal()
			self.further_angle_point = None
			self.prev_shelf_center = self.shelf_info['center']
			self.shelf_angle_deg = self.get_next_angle() + self.robot_initial_angle
			self.shelf_objects_curr.qr_decoded = self.qr_code_str
			self.publisher_shelf_data.publish(self.shelf_objects_curr)
			self.send_request_to_server(rtype='upload')
			self.current_shelf_number+=1
			if self.current_shelf_number>self.shelf_count:
				self.current_state = self.DEBUG
				self.logger.info("All shelves processed, entering DEBUG state.")
				self.qr_code_str = None
				return
			self.current_shelf_objects = None
			self.shelf_objects_curr = WarehouseShelf()
			self.qr_code_str = None
			self.current_state = self.EXPLORE

			self.logger.info(f"QR processed, resuming exploration towards angle {self.shelf_angle_deg}°")
			return
	
		if not self.goal_completed:
			return	

		
		self.map_array = np.array(self.global_map_curr.data).reshape((self.global_map_curr.info.height, self.global_map_curr.info.width))
		self.buggy_map_xy = self.get_map_coord_from_world_coord(self.buggy_pose_x, self.buggy_pose_y, self.global_map_curr.info)
		
		
		# state machine
		if self.current_state == -1:
			self.prev_shelf_center = self.get_map_coord_from_world_coord(
				self.buggy_pose_x,
				self.buggy_pose_y,
				self.global_map_curr.info)
			self.current_state = self.EXPLORE

		elif self.current_state == self.EXPLORE:
			self.frontier_explore()
			# self.send_goal_closest_free_in_circles()

		elif self.current_state == self.MOVE_TO_SHELF:
			self.handle_move_to_shelf()

		elif self.current_state == self.CAPTURE_OBJECTS:
			self.handle_capture_objects()

		elif self.current_state == self.MOVE_TO_QR:
			self.handle_qr_navigation()

		elif self.current_state == self.ADJUST_TO:
			self.logger.info("Adjusting position to QR code...")
			self.adjust_qr()

		elif self.current_state == self.DEBUG:
			self.time_taken = time.time() - self.start
			self.logger.info(f"DEBUG: Time taken for processing: {self.time_taken:.2f} seconds")
		else:
			return 

	def pose_callback(self, message):
		"""Callback function to handle pose updates.

		Args:
			message: ROS2 message containing the current pose of the rover.

		Returns:
			None
		"""
		self.pose_curr = message
		# self.logger.info(f"Current pose: {self.pose_curr.pose.pose.orientation}")

		self.buggy_pose_x = message.pose.pose.position.x
		self.buggy_pose_y = message.pose.pose.position.y
		self.buggy_center = (self.buggy_pose_x, self.buggy_pose_y)
		if self.current_state == self.EXPLORE:
			self.buggy_map_xy = self.get_map_coord_from_world_coord(self.buggy_pose_x, self.buggy_pose_y, self.global_map_curr.info)

		if self.robot_initial_angle is None:
			self.robot_initial_angle = self.get_yaw_from_quaternion(message.pose.pose.orientation)
			self.shelf_angle_deg += self.robot_initial_angle
			self.logger.info(f"Initial robot angle set to: {self.robot_initial_angle:.2f}°")

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

		# Optional line for visualizing image on foxglove.
		if self.current_state == self.MOVE_TO_QR or self.current_state == self.ADJUST_TO:
			qr_codes = pyzbar.decode(image)
			if qr_codes:
				for qr_code in qr_codes:
					qr_data = qr_code.data.decode('utf-8')
					self.qr_code_str = qr_data	
				self.logger.info(f"QR Code Detected: {self.qr_code_str}")
		
		self.publish_debug_image(self.publisher_qr_decode, image)

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
		predicates = {'horse','car','banana','potted plant','clock','cup','zebra','teddy bear'}
		mapping = {'potted plant': 'plant', 'teddy bear': 'teddy','zebra': 'zebra', 'cup': 'cup', 'clock': 'clock','horse':'horse','car':'car','banana':'banana'}
		if self.current_state == self.CAPTURE_OBJECTS:
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
		
		if self.current_state == self.EXPLORE:
			if number_of_recoveries>2:
				self.logger.info(f"Cancelling since trying to recover {number_of_recoveries} or dist {self.calc_distance(self.buggy_map_xy,self.curr_frontier_goal)}<15")
				self.logger.info(f"\n\nRecoveries: {number_of_recoveries}, "
				  f"Navigation time: {navigation_time}s, "
				  f"Distance remaining: {distance_remaining:.2f}, "
				  f"Estimated time remaining: {estimated_time_remaining}s")
				self.cancel_current_goal()
			if self.curr_frontier_goal is not None and self.calc_distance(self.buggy_map_xy,self.curr_frontier_goal)<15:
				self.logger.info(f"Cancelling since dist {self.calc_distance(self.buggy_map_xy,self.curr_frontier_goal)}<15")
				self.cancel_current_goal()
				self.curr_frontier_goal = None

		elif self.current_state == self.CAPTURE_OBJECTS and self.current_shelf_objects!=None and sum(self.current_shelf_objects.object_count) == 6:
			self.logger.info("All objects captured, cancelling goal.")
			self.cancel_current_goal()

	# -------------------- GOAL MANAGEMENT --------------------

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
		self.logger.info(f"Goal created: ({pose.x:.2f}, {pose.y:.2f}, yaw={yaw:.2f})")
		return goal_pose

	def create_goal_from_map_coord(self, map_x: int, map_y: int, map_info,
				       yaw: Optional[float] = None) -> PoseStamped:
		"""Creates a goal PoseStamped from map coordinates."""
		world_x, world_y = self.get_world_coord_from_map_coord(map_x, map_y, map_info)

		return self.create_goal_from_world_coord(world_x, world_y, yaw)

	def get_yaw_from_quaternion(self, quaternion):
		"""
		Convert quaternion to yaw angle in degrees.
		
		Args:
			quaternion: geometry_msgs.msg.Quaternion
			
		Returns:
			float: yaw angle in degrees in range [0, 360)
		"""
		x = quaternion.x
		y = quaternion.y
		z = quaternion.z
		w = quaternion.w
		
		siny_cosp = 2 * (w * z + x * y)
		cosy_cosp = 1 - 2 * (y * y + z * z)
		yaw = math.atan2(siny_cosp, cosy_cosp)
		yaw_deg = math.degrees(yaw)
		yaw_deg = yaw_deg % 360
		
		return yaw_deg

def main(args=None):
	rclpy.init(args=args)
	warehouse_explore = WarehouseExplore()

	rclpy.spin(warehouse_explore)

	warehouse_explore.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()