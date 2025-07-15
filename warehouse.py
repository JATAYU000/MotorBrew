import cv2
import numpy as np
import matplotlib.pyplot as plt

def find_shelf_and_target_point(slam_map, robot_pos, shelf_angle_deg, search_distance=100):
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
    min_search_distance = 20  # Exclude 20-unit margin around robot

    # Map boundary margins
    edge_margin = 5  # Exclude 5-unit margin from map edges
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
    shelf_info = detect_shelf_with_orientation(slam_map, valid_obstacles, robot_pos, angle_rad)
    
    # Find a safe point to move towards the shelf
    target_point = None
    if shelf_info and shelf_info['center']:
        target_point = find_safe_approach_point(slam_map, robot_pos, shelf_info, angle_rad)
    
    return target_point, shelf_info

def detect_shelf_with_orientation(slam_map, obstacles_on_line, robot_pos, angle_rad):
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
    orientation_info = calculate_shelf_orientation(shelf_points)
    
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

def calculate_shelf_orientation(points):
    """
    Calculate the orientation of the shelf using Principal Component Analysis (PCA).
    
    Parameters:
    points: numpy array of (x, y) coordinates
    
    Returns:
    dict with orientation information
    """
    # Center the points
    mean_point = np.mean(points, axis=0)
    centered_points = points - mean_point
    
    # Calculate covariance matrix
    cov_matrix = np.cov(centered_points.T)
    
    # Find eigenvalues and eigenvectors
    eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)
    
    # Sort by eigenvalue (largest first)
    idx = np.argsort(eigenvalues)[::-1]
    eigenvalues = eigenvalues[idx]
    eigenvectors = eigenvectors[:, idx]
    
    # Primary direction (longest axis)
    primary_direction = eigenvectors[:, 0]
    secondary_direction = eigenvectors[:, 1]
    
    # Calculate angle of primary direction
    primary_angle = np.degrees(np.arctan2(primary_direction[1], primary_direction[0]))
    secondary_angle = np.degrees(np.arctan2(secondary_direction[1], secondary_direction[0]))
    
    # Normalize angles to [0, 180) for shelf orientation
    primary_angle = primary_angle % 180
    secondary_angle = secondary_angle % 180
    
    # Calculate aspect ratio
    aspect_ratio = eigenvalues[0] / eigenvalues[1] if eigenvalues[1] != 0 else float('inf')
    
    return {
        'primary_angle': primary_angle,
        'secondary_angle': secondary_angle,
        'primary_direction': primary_direction,
        'secondary_direction': secondary_direction,
        'aspect_ratio': aspect_ratio,
        'elongation': np.sqrt(eigenvalues[0] / eigenvalues[1]) if eigenvalues[1] != 0 else float('inf')
    }

def find_safe_approach_point(slam_map, robot_pos, shelf_info, angle_rad):
    """
    Find the best safe point (value 0) around the robot's position that gets closest 
    to a point that's 30 units in front of the shelf.
    
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
    
    # Define the target distance (30 units in front of shelf)
    target_distance = 30
    
    # Calculate the target point 30 units in front of shelf
    # Try both perpendicular directions to find the best "front"
    front_candidates = []
    
    # Try both perpendicular directions (secondary axis directions)
    for direction in [secondary_direction, -secondary_direction]:
        # Normalize direction
        direction = direction / np.linalg.norm(direction)
        
        # Calculate the target point 30 units in front
        target_x = shelf_x + target_distance * direction[0]
        target_y = shelf_y + target_distance * direction[1]
        
        # Check if this target point is within bounds
        if (0 <= target_x < slam_map.shape[1] and 0 <= target_y < slam_map.shape[0]):
            front_candidates.append((target_x, target_y))
    
    if not front_candidates:
        return None
    
    # Find the best target point (closest to robot or most accessible)
    best_target = min(front_candidates, key=lambda t: np.sqrt((t[0] - robot_x)**2 + (t[1] - robot_y)**2))
    target_x, target_y = best_target
    
    # Search around the robot's position in expanding circles
    best_approach_point = None
    best_distance_to_target = float('inf')
    
    # Search in expanding circles around the robot
    for search_radius in [10, 15, 20, 25, 30, 35, 40, 45, 50]:
        # Try points in a circle around the robot
        num_points = max(12, search_radius // 2)  # More points for larger radius
        
        for i in range(num_points):
            angle = i * 2 * np.pi / num_points
            
            # Calculate candidate position around robot
            candidate_x = int(robot_x + search_radius * np.cos(angle))
            candidate_y = int(robot_y + search_radius * np.sin(angle))
            
            # Check if point is within map bounds
            if (0 <= candidate_x < slam_map.shape[1] and 
                0 <= candidate_y < slam_map.shape[0]):
                
                # Check if this point is free space and safe
                if (slam_map[candidate_y, candidate_x] == 0 and 
                    is_safe_area(slam_map, (candidate_x, candidate_y), radius=3)):
                    
                    # Check if there's a clear path from robot to this point
                    # if is_path_clear(slam_map, robot_pos, (candidate_x, candidate_y)):
                        
                        # Calculate distance from this candidate to the target point (30 units in front of shelf)
                    distance_to_target = np.sqrt(
                        (candidate_x - target_x)**2 + (candidate_y - target_y)**2
                    )
                    
                    # If this point is closer to the target than our current best
                    if distance_to_target < best_distance_to_target:
                        best_distance_to_target = distance_to_target
                        best_approach_point = (candidate_x, candidate_y)
    
    return best_approach_point

def is_path_clear(slam_map, start_point, end_point, step_size=2):
    """
    Check if there's a clear path between two points using line traversal.
    
    Parameters:
    slam_map: 2D numpy array
    start_point: (x, y) starting coordinates
    end_point: (x, y) ending coordinates
    step_size: step size for path checking
    
    Returns:
    bool: True if path is clear (all points are 0), False otherwise
    """
    start_x, start_y = start_point
    end_x, end_y = end_point
    
    # Calculate the path length
    distance = np.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
    
    if distance == 0:
        return True
    
    # Number of steps to check
    num_steps = int(distance // step_size)
    
    # Check points along the path
    for i in range(num_steps + 1):
        t = i / max(num_steps, 1)
        
        # Interpolate position
        x = int(start_x + t * (end_x - start_x))
        y = int(start_y + t * (end_y - start_y))
        
        # Check bounds
        if 0 <= x < slam_map.shape[1] and 0 <= y < slam_map.shape[0]:
            # Check if point is not free space
            if slam_map[y, x] != 0:
                return False
        else:
            return False
    
    return True

def is_safe_area(slam_map, point, radius=10):
    """
    Check if an area around a point is safe (all pixels are 0).
    """
    x, y = point
    
    for dx in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1):
            check_x, check_y = x + dx, y + dy
            
            # Check bounds
            if 0 <= check_x < slam_map.shape[1] and 0 <= check_y < slam_map.shape[0]:
                if slam_map[check_y, check_x] != 0:
                    return False
    
    return True

def visualize_results(slam_map, robot_pos, target_point, shelf_info, angle_deg):
    """
    Visualize the slam map with detected shelf orientation and target point.
    """
    plt.figure(figsize=(15, 12))
    
    # Create a colored version of the map
    colored_map = np.zeros((slam_map.shape[0], slam_map.shape[1], 3), dtype=np.uint8)
    
    # Color coding: 
    # White for free space (0)
    # Black for obstacles (99, 100)
    # Gray for unexplored (255)
    colored_map[slam_map == 0] = [255, 255, 255]  # White for free space
    colored_map[(slam_map == 99) | (slam_map == 100)] = [0, 0, 0]  # Black for obstacles
    colored_map[slam_map == 255] = [128, 128, 128]  # Gray for unexplored
    
    plt.imshow(colored_map, origin='upper')
    
    # Plot robot position
    plt.plot(robot_pos[0], robot_pos[1], 'bo', markersize=12, label='Robot')
    
    # Plot search direction line
    angle_rad = np.radians(angle_deg)
    line_length = 100
    end_x = robot_pos[0] + line_length * np.cos(angle_rad)
    end_y = robot_pos[1] + line_length * np.sin(angle_rad)
    plt.plot([robot_pos[0], end_x], [robot_pos[1], end_y], 'b--', alpha=0.5, linewidth=2, label='Search Direction')
    
    # Plot shelf information if found
    if shelf_info:
        shelf_center = shelf_info['center']
        
        # Plot shelf center
        plt.plot(shelf_center[0], shelf_center[1], 'ro', markersize=10, label='Shelf Center')
        
        # Plot shelf orientation (principal axes)
        orientation = shelf_info['orientation']
        primary_dir = orientation['primary_direction']
        secondary_dir = orientation['secondary_direction']
        
        # Scale the direction vectors for visualization
        scale = 30
        
        # Primary axis (longest)
        primary_end_x = shelf_center[0] + scale * primary_dir[0]
        primary_end_y = shelf_center[1] + scale * primary_dir[1]
        plt.plot([shelf_center[0] - scale * primary_dir[0], primary_end_x], 
                [shelf_center[1] - scale * primary_dir[1], primary_end_y], 
                'r-', linewidth=3, alpha=0.8, label=f'Primary Axis ({orientation["primary_angle"]:.1f}°)')
        
        # Secondary axis (shorter)
        secondary_end_x = shelf_center[0] + scale * secondary_dir[0]
        secondary_end_y = shelf_center[1] + scale * secondary_dir[1]
        plt.plot([shelf_center[0] - scale * secondary_dir[0], secondary_end_x], 
                [shelf_center[1] - scale * secondary_dir[1], secondary_end_y], 
                'orange', linewidth=2, alpha=0.8, label=f'Secondary Axis ({orientation["secondary_angle"]:.1f}°)')
        
        # Plot the minimum area rectangle (bounding box)
        corners = np.array(shelf_info['corners'])
        corners = np.vstack([corners, corners[0]])  # Close the rectangle
        plt.plot(corners[:, 0], corners[:, 1], 'purple', linewidth=2, alpha=0.7, 
                label=f'Bounding Box (θ={shelf_info["rotation_angle"]:.1f}°)')
        
        # Plot corner points
        for i, corner in enumerate(shelf_info['corners']):
            plt.plot(corner[0], corner[1], 'mo', markersize=6, alpha=0.7)
    
    # Plot target point if found
    if target_point:
        plt.plot(target_point[0], target_point[1], 'go', markersize=10, label='Target Point')
        
        # Draw line from robot to target
        plt.plot([robot_pos[0], target_point[0]], [robot_pos[1], target_point[1]], 
                'g-', linewidth=2, alpha=0.7, label='Path to Target')
    
    plt.title(f'SLAM Map with Shelf Orientation Analysis\nSearch Angle: {angle_deg}°')
    plt.xlabel('X coordinate')
    plt.ylabel('Y coordinate')
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.tight_layout()
    plt.show()
    
    # Print detailed shelf information
    if shelf_info:
        print("\n=== SHELF ORIENTATION ANALYSIS ===")
        print(f"Shelf Center: {shelf_info['center']}")
        print(f"Rotation Angle (MinAreaRect): {shelf_info['rotation_angle']:.2f}°")
        print(f"Primary Axis Angle: {shelf_info['orientation']['primary_angle']:.2f}°")
        print(f"Secondary Axis Angle: {shelf_info['orientation']['secondary_angle']:.2f}°")
        print(f"Dimensions: {shelf_info['dimensions']['width']:.1f} x {shelf_info['dimensions']['height']:.1f}")
        print(f"Aspect Ratio: {shelf_info['orientation']['aspect_ratio']:.2f}")
        print(f"Elongation: {shelf_info['orientation']['elongation']:.2f}")
        print(f"Area: {shelf_info['dimensions']['area']} pixels")
        
        # Determine shelf orientation relative to robot
        robot_to_shelf_angle = np.degrees(np.arctan2(
            shelf_info['center'][1] - robot_pos[1],
            shelf_info['center'][0] - robot_pos[0]
        )) % 360
        
        print(f"Robot to Shelf Angle: {robot_to_shelf_angle:.2f}°")
        
        # Analyze shelf orientation
        primary_angle = shelf_info['orientation']['primary_angle']
        if primary_angle < 22.5 or primary_angle > 157.5:
            orientation_desc = "Horizontal"
        elif 67.5 <= primary_angle <= 112.5:
            orientation_desc = "Vertical"
        else:
            orientation_desc = "Diagonal"
        
        print(f"Shelf Orientation: {orientation_desc}")
        print("=" * 35)

# Example usage
def main():
    # Create a sample SLAM map for testing
    slam_map = np.load('NXPaim/global.npy')
    print(type(slam_map))
    # Robot parameters
    robot_position = (150,150)
    shelf_angle = 135  # degrees
    
    # Find shelf and target point
    target_point, shelf_info = find_shelf_and_target_point(
        slam_map, robot_position, shelf_angle, search_distance=400
    )
    
    print(f"Robot position: {robot_position}")
    print(f"Search angle: {shelf_angle}°")
    
    if shelf_info:
        print(f"Shelf center found at: {shelf_info['center']}")
        print(f"Shelf rotation angle: {shelf_info['rotation_angle']:.2f}°")
    else:
        print("No shelf found in the specified direction")
    
    if target_point:
        print(f"Target point to move to: {target_point}")
        distance = np.sqrt((target_point[0] - robot_position[0])**2 + 
                          (target_point[1] - robot_position[1])**2)
        print(f"Distance to target: {distance:.2f} pixels")
    else:
        print("No safe target point found")
    
    # Visualize results
    visualize_results(slam_map, robot_position, target_point, shelf_info, shelf_angle)
    
    return target_point, shelf_info

if __name__ == "__main__":
    main()