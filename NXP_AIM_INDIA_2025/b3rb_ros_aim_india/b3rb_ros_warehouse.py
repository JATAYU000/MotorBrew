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

		# --- QR Code Data ---
		self.qr_code_str = "Empty"
		

		# --- Shelf Data ---
		self.shelf_objects_curr = WarehouseShelf()
		self.shelf_angle_deg = self.initial_angle
		self.prev_shelf_center = None

		# --- State Machine ---
		self.current_state = -1
		self.EXPLORE = 0
		self.MOVE_TO_SHELF = 1
		self.CAPTURE_OBJECTS = 2
		self.MOVE_TO_QR = 3
		self.ADJUST_TO = 4
		self.DEBUG = 5

	# -------------------- MOVE TO THE SHELF -----------------------


	# -------------------- FRONTIER EXPLORATION --------------------

	def frontier_explore(self):
		self.shelf_info = self.find_shelf(search_distance=200)
		self.logger.info(f"{self.shelf_angle_deg}")
		self.logger.info(f"prev shelf cneter: {self.prev_shelf_center}")
		
		if self.shelf_info is not None and self.find_free_space_around_shelf_center(radius=75) > 70:
			self.logger.info(f"Map is mostly free, skipping exp: {self.find_free_space_around_shelf_center(radius=75)}% free")
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
				goal = self.create_goal_from_map_coord(fx, fy, self.global_map_curr.info)
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

	def find_shelf(self, search_distance=100):
		"""
		Detects a shelf within a specified corridor in a SLAM map and determines a safe target point for the robot to approach.
		This method searches for obstacles (representing shelves) along a corridor extending from the robot's current position
		in a specified direction. It validates detected obstacles based on distance from the robot and map edges, then attempts
		to identify a shelf with expected dimensions. If a valid shelf is found, it computes a safe approach point for the robot.
		Parameters:
			self.map_array (np.ndarray): 2D numpy array representing the SLAM occupancy grid map. 
									Obstacles (shelves) are expected to have values 99 or 100.
			self.buggy_map_xy (tuple): (x, y) coordinates of the robot's current position in map units.
			self.shelf_angle_deg (float): Angle in degrees indicating the direction to search for the shelf, relative to the robot.
			search_distance (int, optional): Maximum distance (in map units) to search for the shelf. Default is 100.
		Returns:
			tuple:
				target_point (tuple or None): (x, y) coordinates of a safe point to approach the detected shelf, or None if not found.
				shelf_info (dict or None): Dictionary containing shelf properties (center, dimensions, etc.), or None if no valid shelf is detected.
		"""
		
		# Convert angle to radians
		angle_rad = np.radians(self.shelf_angle_deg)
		
		obstacle_mask = (self.map_array == 99) | (self.map_array == 100)
		
		point_x, point_y = self.prev_shelf_center
		min_search_distance = 30  # Exclude 30-unit margin around robot
		edge_margin = 12  # Exclude 5-unit margin from map edges
		map_height, map_width = self.map_array.shape
		search_width = 3  # Search ±3 units perpendicular to the main direction
		perp_angle = angle_rad + np.pi/2  # 90 degrees perpendicular
		perp_dx = np.cos(perp_angle)
		perp_dy = np.sin(perp_angle)
		
		# Generate points along the search corridor
		search_points = []
		for distance in range(min_search_distance, search_distance + 1):
			# Main search line point
			main_x = point_x + distance * np.cos(angle_rad)
			main_y = point_y + distance * np.sin(angle_rad)
			
			# Create points across the search width
			for offset in range(-search_width, search_width + 1):
				x = int(main_x + offset * perp_dx)
				y = int(main_y + offset * perp_dy)
				
				# Check bounds WITH edge margins
				if (edge_margin <= x < map_width - edge_margin and 
					edge_margin <= y < map_height - edge_margin):
					search_points.append((x, y))
		
		search_points = list(dict.fromkeys(search_points))
		
		obstacles_on_line = []
		for x, y in search_points:
			if obstacle_mask[y, x]:  # Note: y first for numpy array indexing
				obstacles_on_line.append((x, y))
		
		if not obstacles_on_line:
			return None
		
		valid_obstacles = []
		for obs_x, obs_y in obstacles_on_line:
			distance_from_robot = np.sqrt((obs_x - point_x)**2 + (obs_y - point_y)**2)
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
			return None
		
		shelf_info = self.detect_shelf_with_orientation(valid_obstacles, angle_rad)
		if shelf_info:
			h = shelf_info['dimensions']['height']
			w = shelf_info['dimensions']['width']
			if h>w: h,w=w,h
			if 26<w<=40 and 9<h<=22:
				pass
			else:
				self.logger.info(f"height and width of box found {h} {w} so retring")
				return self.find_shelf(search_distance)
		else:
			return None
		# Find a safe point to move towards the shelf
		target_point = None
		# if shelf_info and shelf_info['center']:
		# 	target_point = self.find_safe_approach_point(self.map_array, self.buggy_map_xy, shelf_info, angle_rad)

		return shelf_info

	def detect_shelf_with_orientation(self, obstacles_on_line, angle_rad):
		"""
		Detects a shelf in a SLAM map along a specified search line and estimates its orientation.
		This method processes a list of obstacle points (typically detected along a search line)
		to identify the most likely shelf structure within the map. It filters out obstacles near
		the map edges, clusters the remaining obstacles, selects the cluster (connected component)
		closest to the search line, and computes the shelf's center and orientation using PCA and
		minimum area rectangle fitting.
		Parameters:
			self.map_array (np.ndarray): 2D numpy array representing the SLAM occupancy grid map.
									Obstacle cells are expected to have values 99 or 100.
			obstacles_on_line (list of tuple): List of (x, y) tuples representing obstacle coordinates
												detected along the search line.
			self.buggy_map_xy (tuple): (x, y) coordinates of the robot's current position in map coordinates.
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
		
		edge_margin = 5  # Exclude 5-unit margin from map edges
		map_height, map_width = self.map_array.shape
		
		filtered_obstacles = []
		for obs_x, obs_y in obstacles_on_line:
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
		
		
		obstacle_points = np.array(filtered_obstacles)  # Use filtered list
		min_x, min_y = np.min(obstacle_points, axis=0)
		max_x, max_y = np.max(obstacle_points, axis=0)
		
		margin = 30
		roi_x1 = max(edge_margin, min_x - margin)          # Respect edge margin
		roi_y1 = max(edge_margin, min_y - margin)          # Respect edge margin
		roi_x2 = min(map_width - edge_margin, max_x + margin)   # Respect edge margin
		roi_y2 = min(map_height - edge_margin, max_y + margin)  # Respect edge margin

		
		# Extract ROI
		roi = self.map_array[roi_y1:roi_y2, roi_x1:roi_x2]
		obstacle_roi = (roi == 99) | (roi == 100)
		
		num_features, labeled = cv2.connectedComponents(obstacle_roi.astype(np.uint8))
		
		if num_features == 0:
			return None
		
		robot_x, robot_y = self.buggy_map_xy
		search_line_dx = np.cos(angle_rad)
		search_line_dy = np.sin(angle_rad)
		
		component_info = []
		for i in range(1, num_features + 1):
			component_mask = (labeled == i)
			size = np.sum(component_mask)
			
			if size < 10:
				continue
				
			y_coords, x_coords = np.where(component_mask)
			
			global_x_coords = x_coords + roi_x1
			global_y_coords = y_coords + roi_y1
			
			center_x = np.mean(global_x_coords)
			center_y = np.mean(global_y_coords)
			
			to_center_x = center_x - robot_x
			to_center_y = center_y - robot_y
			
			# Project onto search line direction to get along-line distance
			along_line_distance = to_center_x * search_line_dx + to_center_y * search_line_dy
			closest_point_x = robot_x + along_line_distance * search_line_dx
			closest_point_y = robot_y + along_line_distance * search_line_dy
			
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
		
		best_component = min(component_info, key=lambda x: x['perp_distance'])
		selected_label = best_component['label']
		
		largest_component = (labeled == selected_label)
		y_coords, x_coords = np.where(largest_component)
		
		if len(x_coords) == 0:
			return None
		
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
		
		center_x = int(np.mean(global_x_coords))
		center_y = int(np.mean(global_y_coords))
		
		shelf_points = np.column_stack([global_x_coords, global_y_coords])
		orientation_info = self.calculate_shelf_orientation(shelf_points)
		contour_points = np.column_stack([global_x_coords, global_y_coords]).astype(np.int32)
		rect_info = cv2.minAreaRect(contour_points)
		(rect_center_x, rect_center_y), (width, height), rotation_angle = rect_info
		box_points = cv2.boxPoints(rect_info)
		box_points = np.int32(box_points)
		rotation_angle = rotation_angle % 180
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



	# -------------------- UTILITY -------------------


	def find_free_space_around_shelf_center(self, radius):
		"""
		Calculates the percentage of free space (cells with value 0) within a circular area around a given point in a 2D map array.
		Parameters:
			map_array (np.ndarray): 2D numpy array representing the map, where 0 indicates free space and other values indicate obstacles or unknown areas.
			point (tuple): (x, y) coordinates of the center point around which to calculate free space.
			radius (int or float): Radius of the circular area (in cells) to consider around the point.
		Returns:
			float: Percentage of free space within the specified circular area. Returns 0 if no valid cells are found within the area.
		"""
		self.logger.info(f"\n\n\n{self.shelf_info}")
		point = self.shelf_info['center']
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
	

	# -------------------- CALLBACKS --------------------


	def global_map_callback(self, message):
		"""Callback function to handle global map updates.

		Args:
			message: ROS2 message containing the global map data.

		Returns:
			None
		"""
		self.global_map_curr = message

		if not self.goal_completed:
			return		
		
		self.map_array = np.array(self.global_map_curr.data).reshape((self.global_map_curr.info.height, self.global_map_curr.info.width))
		self.buggy_map_xy = self.get_map_coord_from_world_coord(self.buggy_pose_x, self.buggy_pose_y, self.global_map_curr.info)
		
		
		# state machine
		if self.current_state == -1:
			self.prev_shelf_center = self.get_map_coord_from_world_coord(
				self.world_center[0],
				self.world_center[1],
				self.global_map_curr.info)
			self.current_state = self.EXPLORE

		elif self.current_state == self.EXPLORE:
			self.frontier_explore()
		elif self.current_state == self.MOVE_TO_SHELF:
			self.logger.info("MOVE TO SHELF")
		elif self.current_state == self.CAPTURE_OBJECTS:
			self.logger.info("CAPTURE OBJECT")
		elif self.current_state == self.MOVE_TO_QR:
			self.logger.info("MOVE TO QR")
		elif self.current_state == self.ADJUST_TO:
			self.logger.info("ADJUST TO")
		elif self.current_state == self.DEBUG:
			self.logger.info("DEBUG")
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
		# self.publish_debug_image(self.publisher_qr_decode, image)

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
		self.shelf_objects_curr = message
		# Process the shelf objects as needed.

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
			if number_of_recoveries>0:
				self.logger.info(f"Cancelling since trying to recover {number_of_recoveries}")
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

	rclpy.spin(warehouse_explore)

	warehouse_explore.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
