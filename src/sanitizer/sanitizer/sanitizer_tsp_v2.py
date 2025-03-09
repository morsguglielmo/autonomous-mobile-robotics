import rclpy
from rclpy.node import Node
import cv2
import yaml
import os
import networkx as nx
from scipy.spatial import distance_matrix
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from geometry_msgs.msg import Quaternion
import math
import numpy as np
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
import random
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path

##############################################################################################################
#
# PARAMS
#
##############################################################################################################

GRID_RESOLUTION = 0.5

MAP_YAML_FILE = '/home/guglielmo/project_ws/map/map.yaml'
MAP_GRID_FILE = '/home/guglielmo/project_ws/map/map_with_grid.png'
FREE_CELLS_FILE = '/home/guglielmo/project_ws/map/free_cells.png'
SUBSET_CELLS_FILE = '/home/guglielmo/project_ws/map/subset_cells.png'
TSP_PATH_FILE = '/home/guglielmo/project_ws/map/tsp_path.png'

USER_INPUT_FILE = '/home/guglielmo/project_ws/src/sanitizer/user_input.txt'
CORNERS_FILE = '/home/guglielmo/project_ws/src/sanitizer/corners.txt'

BLACKLIST_OCCUPIED = True
BLACK_PIXEL_TOLERANCE = 0.001

WALL_THICKNESS = 0.2 # [m]
ENERGY_THRESHOLD = 0.01 # [J]s
MAX_ENERGY_CELLS_HEURISTIC = 1
SANITIZED_THRESHOLD = 1.0 # %/100

N_NODES_TO_VISIT = 10

SAVE_MAP = True


##############################################################################################################
#
# MAP READER -> (GRID CREATION, CELL SELECTION, NEIGHBOR COMPUTATION, TSP PATH PLANNING, SANITIZATION CHECK)
#
##############################################################################################################

class MapReader(Node):
    def __init__(self):
        super().__init__('map_reader_node')

        # Read map resolution and image file path from YAML file
        map_resolution, image_file = self.read_map_metadata(MAP_YAML_FILE)
        if map_resolution is None or image_file is None:
            self.get_logger().error(f"Error: Could not read map metadata from {MAP_YAML_FILE}")
            return

        self.px = None
        self.py = None

        self.current_room_cells = []
        self.cells_to_sanitize = []

        self.room = OccupancyGrid()
        #self.energy_cells_to_sanitize = []
        self.cells = []
        self.occupied_ids = []
        self.neighbors = {}
        self.map_resolution = map_resolution
        self.map_image = cv2.imread(image_file, cv2.IMREAD_COLOR)
        self.map_width = self.map_image.shape[1]
        self.map_height = self.map_image.shape[0]
        self.get_logger().info(f"Map image loaded: {image_file} ({self.map_width}x{self.map_height})")
        #self.new_occopied_cells = []

        self.energy_grid = None
        self.create_subscription(OccupancyGrid, '/energy_grid', self.energy_grid_callback, 10)
        self.pose_subscription = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.pose_callback, 10)

        self.room_publisher = self.create_publisher(OccupancyGrid, '/room', 10)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.marker_publisher = self.create_publisher(Marker, '/marker', 10)
        self.robot_marker_publisher = self.create_publisher(Marker, '/robot_marker', 10)

        self.create_timer(0.1, self.timer_callback)

        # Wait for the energy grid to be available
        while self.energy_grid is None:
            rclpy.spin_once(self)
            self.get_logger().info("INIT: Waiting for energy grid...")
        self.get_logger().info(f"Energy grid received")

        self.set_occupied_ids()
        self.wait_for_amcl_pose()

    def timer_callback(self):
        pass


    def pose_callback(self, msg):
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y

    def read_map_metadata(self, yaml_file):
        try:
            with open(yaml_file, 'r') as file:
                map_metadata = yaml.safe_load(file)
                resolution = map_metadata['resolution']
                image_file = os.path.join(os.path.dirname(yaml_file), map_metadata['image'])
                return resolution, image_file
        except Exception as e:
            self.get_logger().error(f"Error reading map metadata: {e}")
            return None, None
    
    def energy_grid_callback(self, msg):
        self.energy_grid = msg

    def wait_for_amcl_pose(self):
        self.get_logger().info("Waiting for AMCL pose...")
        while self.px is None or self.py is None:
            rclpy.spin_once(self)
            # publish a random small twist
            twist = Twist()
            twist.linear.x = random.uniform(-0.1, 0.1)
            twist.angular.z = random.uniform(-0.1, 0.1)
            self.velocity_publisher.publish(twist)
        # stop the robot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.velocity_publisher.publish(twist)

    ##############################################################################################################
    # MAP READER CELLS OPERATIONS

    def set_occupied_ids(self):
        '''
        This function sets the ids of the occupied cells (contain black pixel)
        '''
        self.occupied_ids = []
        for grid_id in range(len(self.energy_grid.data)):
            cell_x = (grid_id % self.energy_grid.info.width) * self.energy_grid.info.resolution + self.energy_grid.info.resolution / 2.0
            cell_y = (grid_id // self.energy_grid.info.width) * self.energy_grid.info.resolution + self.energy_grid.info.resolution / 2.0
            if not self.is_cell_free(cell_x, cell_y, self.energy_grid.info.resolution):
                self.occupied_ids.append(grid_id)

    def get_room_cells(self, bottom_left_x, bottom_left_y, top_right_x, top_right_y, subset):
        '''
        This function returns the cells of the current room
        '''
        # Publish room as area of different color
        room = OccupancyGrid()
        room.header.frame_id = 'map'
        room.info.resolution = self.energy_grid.info.resolution
        room.info.width = int(self.map_width * self.map_resolution / self.energy_grid.info.resolution)
        room.info.height = int(self.map_height * self.map_resolution / self.energy_grid.info.resolution)
        room.info.origin.position.x = -7.45
        room.info.origin.position.y = 5.2
        room.info.origin.position.z = 0.0
        room.info.origin.orientation.x = 1.0
        room.info.origin.orientation.y = 0.0
        room.info.origin.orientation.z = 0.0
        room.info.origin.orientation.w = 0.0
        room.data = [0] * (room.info.width * room.info.height)

        self.current_room_cells = []
        self.cells_to_sanitize = []
        for grid_id in range(len(self.energy_grid.data)):
            if grid_id in self.occupied_ids:
                continue
            cell_x = (grid_id % self.energy_grid.info.width) * self.energy_grid.info.resolution + self.energy_grid.info.resolution / 2.0
            cell_y = (grid_id // self.energy_grid.info.width) * self.energy_grid.info.resolution + self.energy_grid.info.resolution / 2.0
            if bottom_left_x <= cell_x <= top_right_x and bottom_left_y <= cell_y <= top_right_y:
                self.current_room_cells.append(grid_id)
                room.data[grid_id] = 100
        self.cells_to_sanitize = self.current_room_cells.copy()
        self.room = room



    def cell_selection(self, grid_resolution):
        '''
        This function creates a list of cells: id center_x center_y is_free
        is_free is a boolean that is True if the cell is free, False otherwise
        It is false if a cell contains a black pixel, or if it is a neighbor of a black pixel
        cells size is grid_resolution x grid_resolution
        '''
        half_grid_res = grid_resolution / 2.0
        num_cells_x = int(self.map_width * self.map_resolution / grid_resolution)
        num_cells_y = int(self.map_height * self.map_resolution / grid_resolution)
        
        for i in range(num_cells_x):
            for j in range(num_cells_y):
                center_x = i * grid_resolution + half_grid_res
                center_y = j * grid_resolution + half_grid_res
                if BLACKLIST_OCCUPIED:
                    is_free = self.is_cell_free(center_x, center_y, grid_resolution)
                else:
                    is_free = True
                self.cells.append((len(self.cells), center_x, center_y, is_free))

    def cell_selection_no_walls(self, grid_resolution):
        '''
        This function creates a list of cells: id center_x center_y is_free
        is_free is a boolean that is True if the cell is free, False otherwise
        It is false if a cell contains a black pixel, or if it is a neighbor of a black pixel
        cells size is grid_resolution x grid_resolution
        Only central part is considered, walls are not considered
        So a ring of thickness WALL_THICKNESS is neglected around the map
        '''
        half_grid_res = grid_resolution / 2.0
        num_cells_x = int((self.map_width * self.map_resolution - 2 * WALL_THICKNESS) / grid_resolution)
        num_cells_y = int((self.map_height * self.map_resolution - 2 * WALL_THICKNESS) / grid_resolution)

        for i in range(num_cells_x):
            for j in range(num_cells_y):
                center_x = i * grid_resolution + half_grid_res + WALL_THICKNESS
                center_y = j * grid_resolution + half_grid_res + WALL_THICKNESS
                if BLACKLIST_OCCUPIED:
                    is_free = self.is_cell_free(center_x, center_y, grid_resolution)
                else:
                    is_free = True
                self.cells.append((len(self.cells), center_x, center_y, is_free))

        
    def is_cell_free(self, center_x, center_y, grid_resolution):
        '''
        This function checks if a cell is free
        '''
        half_grid_res = grid_resolution / 2.0
        black_pixels = 0
        tot_pixels = int(grid_resolution / self.map_resolution) ** 2
        start_x = int((center_x - half_grid_res) / self.map_resolution)
        start_y = int((center_y - half_grid_res) / self.map_resolution)
        end_x = int((center_x + half_grid_res) / self.map_resolution)
        end_y = int((center_y + half_grid_res) / self.map_resolution)

        for x in range(start_x, end_x):
            for y in range(start_y, end_y):
                if 0 <= x < self.map_width and 0 <= y < self.map_height:
                    if self.is_black_pixel(x, y):
                        black_pixels += 1

        return black_pixels / tot_pixels <= BLACK_PIXEL_TOLERANCE

    def is_black_pixel(self, x, y):
        '''
        This function checks if a pixel is black
        '''
        return all(self.map_image[y, x] == [0, 0, 0])
    
    def is_not_white_pixel(self, x, y):
        '''
        This function checks if a pixel is not white
        '''
        return not all(self.map_image[y, x] == [255, 255, 255])

    def cell_subset(self, top_left_x, top_left_y, bottom_right_x, bottom_right_y):
        '''
        This function returns a subset of cells that are within the specified rectangle
        '''
        subset = []
        for cell in self.cells:
            if top_left_x <= cell[1] <= bottom_right_x and top_left_y <= cell[2] <= bottom_right_y:
                if cell[3]:
                    subset.append(cell)

        self.get_logger().info(f"Subset cells found: {len(subset)}")

        if SAVE_MAP:
            # Draw the subset cells on the map image (fill with light blue color)
            image_subset_cells = self.map_image.copy()
            cell_size_pixels = int(GRID_RESOLUTION / self.map_resolution)
            for cell in subset:
                start_x = int((cell[1] - GRID_RESOLUTION / 2) / self.map_resolution)
                start_y = int((cell[2] - GRID_RESOLUTION / 2) / self.map_resolution)
                end_x = start_x + cell_size_pixels
                end_y = start_y + cell_size_pixels
                start_x = max(0, start_x)
                start_y = max(0, start_y)
                end_x = min(self.map_width, end_x)
                end_y = min(self.map_height, end_y)
                image_subset_cells[start_y:end_y, start_x:end_x] = (255, 165, 0)
            # draw grid lines for visualization
            for x in range(0, self.map_width, cell_size_pixels):
                image_subset_cells = cv2.line(image_subset_cells, (x, 0), (x, self.map_height), (255, 0, 0), 1)
            for y in range(0, self.map_height, cell_size_pixels):
                image_subset_cells = cv2.line(image_subset_cells, (0, y), (self.map_width, y), (255, 0, 0), 1)
            
            # draw orange rectangle around the subset
            top_left_x_pixels = int(top_left_x / self.map_resolution)
            top_left_y_pixels = int(top_left_y / self.map_resolution)
            bottom_right_x_pixels = int(bottom_right_x / self.map_resolution)
            bottom_right_y_pixels = int(bottom_right_y / self.map_resolution)
            image_subset_cells = cv2.rectangle(image_subset_cells, (top_left_x_pixels, top_left_y_pixels), (bottom_right_x_pixels, bottom_right_y_pixels), (0, 165, 255), 2)

            # test draw a rectangle in purple
            top_left_test = (0, 0)
            bottom_right_test = (14.95, 13.2)
            top_left_test_pixels = int(top_left_test[0] / self.map_resolution), int(top_left_test[1] / self.map_resolution)
            bottom_right_test_pixels = int(bottom_right_test[0] / self.map_resolution), int(bottom_right_test[1] / self.map_resolution)
            image_subset_cells = cv2.rectangle(image_subset_cells, top_left_test_pixels, bottom_right_test_pixels, (255, 0, 255), 2)
            


            cv2.imwrite(SUBSET_CELLS_FILE, image_subset_cells)
            self.get_logger().info(f"Subset cells drawn on map image: {SUBSET_CELLS_FILE}")

        return subset


    ##############################################################################################################
    # MAP READER TRANSFORMATIONS
    
    def mymap2rviz(self, x, y):
        
        x_out = x - 7.45
        y_out = -(y - 5.2)

        return x_out, y_out

    def rviz2mymap(self, x, y):

        x_out = x + 7.45
        y_out = -y + 5.2

        return x_out, y_out
    
    def convert_to_rviz(self, waypoints):
        '''
        This function converts the waypoints from the map frame to RViz
        '''
        rviz_waypoints = []
        for waypoint in waypoints:
            rviz_waypoints.append(self.mymap2rviz(waypoint[0], waypoint[1]))
        return rviz_waypoints

    def yaw2quaternion(self, yaw):
        '''
        This function converts a yaw angle to a quaternion
        '''
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(yaw / 2)
        quaternion.w = math.cos(yaw / 2)

        return quaternion

    
    ##############################################################################################################
    # MAP READER TSP PATH PLANNING

    def solve_tsp(self, points):
        """
        Solves the TSP
        """
        # Create the distance matrix
        dist_matrix = distance_matrix(points, points)

        # Create a complete graph
        G = nx.Graph()
        for i in range(len(points)):
            for j in range(i + 1, len(points)):
                G.add_edge(i, j, weight=dist_matrix[i][j])
                
        # Solve the TSP
        tsp_path = nx.approximation.traveling_salesman_problem(G, cycle=False)

        return tsp_path
        
    def plan_path_cells_left(self):
        '''
        This function plans a path through the cells that are not sanitized
        '''
        points = []
        for grid_id in self.cells_to_sanitize:
            cell_x = (grid_id % self.energy_grid.info.width) * self.energy_grid.info.resolution + self.energy_grid.info.resolution / 2.0
            cell_y = (grid_id // self.energy_grid.info.width) * self.energy_grid.info.resolution + self.energy_grid.info.resolution / 2.0
            points.append((cell_x, cell_y))

        if len(points) == 0:
            self.get_logger().info("No points to plan path through")
            return []
        elif len(points) == 1:
            return points
        else:
            tsp_path = self.solve_tsp(points)
            waypoints = [points[i] for i in tsp_path]

            if SAVE_MAP:
                # Draw the TSP path on the map image
                image_tsp_path = self.map_image.copy()
                for i in range(len(waypoints) - 1):
                    start_x = int(waypoints[i][0] / self.map_resolution)
                    start_y = int(waypoints[i][1] / self.map_resolution)
                    end_x = int(waypoints[i + 1][0] / self.map_resolution)
                    end_y = int(waypoints[i + 1][1] / self.map_resolution)
                    image_tsp_path = cv2.line(image_tsp_path, (start_x, start_y), (end_x, end_y), (255, 0, 255), 2)
                # Drar the grid lines for visualization
                cell_size_pixels = int(GRID_RESOLUTION / self.map_resolution)
                for x in range(0, self.map_width, cell_size_pixels):
                    image_tsp_path = cv2.line(image_tsp_path, (x, 0), (x, self.map_height), (255, 0, 0), 1)
                for y in range(0, self.map_height, cell_size_pixels):
                    image_tsp_path = cv2.line(image_tsp_path, (0, y), (self.map_width, y), (255, 0, 0), 1)
                cv2.imwrite(TSP_PATH_FILE, image_tsp_path)
                self.get_logger().info(f"TSP path drawn on map image: {TSP_PATH_FILE}")
            return waypoints

    def plan_path_through_nodes_og(self, subset_cells):
        """
        Plans a path through the given subset of cells using TSP.
        """
        rclpy.spin_once(self)

        points = [(cell[1], cell[2]) for cell in subset_cells]
        tsp_path = []

        if len(points) == 0:
            self.get_logger().info("No points to plan path through")
            return []
        elif len(points) == 1:
            tsp_path = points
            return points
        else:
            tsp_path = self.solve_tsp(points)

        # Rearrange the TSP path in order to start from the closest node to the robot
        if self.px is not None and self.py is not None:
            start_x = self.px
            start_y = self.py
            start_x, start_y = self.rviz2mymap(start_x, start_y)
            min_dist = 1e9
            for i in range(len(tsp_path)):
                dist = math.sqrt((points[i][0] - start_x)**2 + (points[i][1] - start_y)**2)
                if dist < min_dist:
                    min_dist = dist
                    closest_node = i
            tsp_path_ordered = tsp_path[closest_node:] + tsp_path[:closest_node]
            self.get_logger().info(f"TSP path ordered")
        else:
            self.get_logger().warn("Robot position not available, TSP path not ordered")

        

        # Rviz coord for robot and closest point
        start_x_rviz, start_y_rviz = self.mymap2rviz(start_x, start_y)
        closest_x_rviz, closest_y_rviz = self.mymap2rviz(points[closest_node][0], points[closest_node][1])
        self.get_logger().info(f"Robot position: {start_x_rviz}, {start_y_rviz}")
        self.get_logger().info(f"Closest node position: {closest_x_rviz}, {closest_y_rviz}")
        
        # Publish marker on closest node
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = closest_x_rviz
        marker.pose.position.y = closest_y_rviz
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        self.marker = marker


        # Generate waypoints from TSP path
        waypoints = [points[i] for i in tsp_path_ordered]
        if SAVE_MAP:
            # Draw the TSP path on the map image
            image_tsp_path = self.map_image.copy()
            for i in range(len(waypoints) - 1):
                start_x = int(waypoints[i][0] / self.map_resolution)
                start_y = int(waypoints[i][1] / self.map_resolution)
                end_x = int(waypoints[i + 1][0] / self.map_resolution)
                end_y = int(waypoints[i + 1][1] / self.map_resolution)
                image_tsp_path = cv2.line(image_tsp_path, (start_x, start_y), (end_x, end_y), (255, 0, 255), 2)
            # Drar the grid lines for visualization
            cell_size_pixels = int(GRID_RESOLUTION / self.map_resolution)
            for x in range(0, self.map_width, cell_size_pixels):
                image_tsp_path = cv2.line(image_tsp_path, (x, 0), (x, self.map_height), (255, 0, 0), 1)
            for y in range(0, self.map_height, cell_size_pixels):
                image_tsp_path = cv2.line(image_tsp_path, (0, y), (self.map_width, y), (255, 0, 0), 1)
            cv2.imwrite(TSP_PATH_FILE, image_tsp_path)
            self.get_logger().info(f"TSP path drawn on map image: {TSP_PATH_FILE}")

        return waypoints

    def plan_path_through_nodes(self, subset_cells, robot_x, robot_y):
        """
        Plans a path through the given subset of cells using TSP.
        """
        rclpy.spin_once(self)

        points = [(cell[1], cell[2]) for cell in subset_cells]
        tsp_path = []

        if len(points) == 0:
            self.get_logger().info("No points to plan path through")
            return []
        elif len(points) == 1:
            tsp_path = points
            return points
        else:
            tsp_path = self.solve_tsp(points)

        start_x, start_y = None, None

        # Rearrange the TSP path in order to start from the closest node to the robot
        if robot_x is not None and robot_y is not None:
            start_x = robot_x
            start_y = robot_y
            #start_x, start_y = self.rviz2mymap(start_x, start_y)
            min_dist = 1e9
            closest_node = 0  # Initialize closest_node to ensure it has a valid value
            for i in range(len(tsp_path)):
                point_id = tsp_path[i]
                point_x, point_y = points[point_id]
                point_x, point_y = self.mymap2rviz(point_x, point_y)
                dist = math.sqrt((point_x - start_x)**2 + (point_y - start_y)**2)
                if dist < min_dist:
                    min_dist = dist
                    closest_node = i
            tsp_path_ordered = tsp_path[closest_node:] + tsp_path[:closest_node]
            self.get_logger().info(f"TSP path ordered")
        else:
            self.get_logger().warn("Robot position not available, TSP path not ordered")
            tsp_path_ordered = tsp_path  # Fallback to the original tsp_path if robot position is not available
        
        # for debugging
        # create a marker on the closest node
        closest_node_id = tsp_path_ordered[0]
        closest_node_x, closest_node_y = points[closest_node_id]
        closest_node_x, closest_node_y = self.mymap2rviz(closest_node_x, closest_node_y)
        self.get_logger().info(f"Closest node position: {closest_node_x}, {closest_node_y}")
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = closest_node_x
        marker.pose.position.y = closest_node_y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        # create a marker on the robot position
        self.get_logger().info(f"Robot position: {robot_x}, {robot_y}")
        robot_marker = Marker()
        robot_marker.header.frame_id = 'map'
        robot_marker.type = Marker.SPHERE
        robot_marker.action = Marker.ADD
        robot_marker.pose.position.x = robot_x
        robot_marker.pose.position.y = robot_y
        robot_marker.pose.position.z = 0.0
        robot_marker.pose.orientation.w = 1.0
        robot_marker.scale.x = 0.2
        robot_marker.scale.y = 0.2
        robot_marker.scale.z = 0.2
        robot_marker.color.a = 1.0
        robot_marker.color.r = 0.0
        robot_marker.color.g = 1.0
        robot_marker.color.b = 0.0

        # Publish the markers
        self.marker_publisher.publish(marker)
        self.robot_marker_publisher.publish(robot_marker)


        

        # Ensure start_x and start_y have valid values
        if start_x is None or start_y is None:
            start_x, start_y = tsp_path_ordered[0]

        # Generate waypoints from TSP path
        waypoints = [points[i] for i in tsp_path_ordered]
        if SAVE_MAP:
            # Draw the TSP path on the map image
            image_tsp_path = self.map_image.copy()
            for i in range(len(waypoints) - 1):
                start_x = int(waypoints[i][0] / self.map_resolution)
                start_y = int(waypoints[i][1] / self.map_resolution)
                end_x = int(waypoints[i + 1][0] / self.map_resolution)
                end_y = int(waypoints[i + 1][1] / self.map_resolution)
                image_tsp_path = cv2.line(image_tsp_path, (start_x, start_y), (end_x, end_y), (255, 0, 255), 2)
            # Draw the grid lines for visualization
            cell_size_pixels = int(GRID_RESOLUTION / self.map_resolution)
            for x in range(0, self.map_width, cell_size_pixels):
                image_tsp_path = cv2.line(image_tsp_path, (x, 0), (x, self.map_height), (255, 0, 0), 1)
            for y in range(0, self.map_height, cell_size_pixels):
                image_tsp_path = cv2.line(image_tsp_path, (0, y), (self.map_width, y), (255, 0, 0), 1)
            cv2.imwrite(TSP_PATH_FILE, image_tsp_path)
            self.get_logger().info(f"TSP path drawn on map image: {TSP_PATH_FILE}")

        return waypoints

    
    def waypoints2poses(self, waypoints):
        '''
        This function converts the waypoints to PoseStamped messages
        '''
        poses = []
        for waypoint in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            pose.pose.position.z = 0.0
            # set orientation such that the robot is facing the next waypoint
            if len(poses) > 0:
                dx = waypoint[0] - poses[-1].pose.position.x
                dy = waypoint[1] - poses[-1].pose.position.y
                yaw = math.atan2(dy, dx)
                pose.pose.orientation = self.yaw2quaternion(yaw)
            else:
                pose.pose.orientation.w = 1.0

            poses.append(pose)
        return poses
    
    def room_id_mapper(self, room_id):
        '''
        This function reads the room_id and returns the top left and bottom right corners of the room
        Corners coordinates are read from the corners.txt file
        e.g.
        0 -7.25 0.76 -5.0 -3.80
        1 -7.20 4.6 -5.28 1.45
        '''
        mapping = {}
        with open(CORNERS_FILE, 'r') as file:
            for line in file.readlines():
                room, top_left_x, top_left_y, bottom_right_x, bottom_right_y = line.split()
                mapping[int(room)] = (float(top_left_x), float(top_left_y), float(bottom_right_x), float(bottom_right_y))
        top_left_x, top_left_y, bottom_right_x, bottom_right_y = mapping[room_id]

        return top_left_x, top_left_y, bottom_right_x, bottom_right_y
    
    
    ##############################################################################################################
    # MAP READER SANITIZATION CHECK
        
    def is_room_sanitized(self):
        '''
        This function checks if a room is sanitized
        '''
        self.energy_grid = None
        # Wait for the energy grid to be available
        while self.energy_grid is None:
            rclpy.spin_once(self)
            self.get_logger().info("Waiting for energy grid to be available...")

        cells_to_sanitize_new = []
        for grid_id in self.cells_to_sanitize:
            if self.energy_grid.data[grid_id] < 100:
                cells_to_sanitize_new.append(grid_id)
            else:
                self.room.data[grid_id] = 0
        self.cells_to_sanitize = cells_to_sanitize_new

        sanitized_cells_cnt = len(self.current_room_cells) - len(self.cells_to_sanitize)
        room_cells_cnt = len(self.current_room_cells)
        self.room_publisher.publish(self.room)

        # Check and log the results
        if room_cells_cnt == 0:
            self.get_logger().warn("Room is empty")
            return False
        elif sanitized_cells_cnt / room_cells_cnt >= SANITIZED_THRESHOLD:
            self.get_logger().info("Room is sanitized")
            return True
        else:
            self.get_logger().info(f"Room not sanitized: {sanitized_cells_cnt}/{room_cells_cnt}")
            return False


    def remove_sanitized_nodes(self, subset):
        '''
        This function removes the sanitized nodes from the subset.
        A node is sanitized if all the cells, i.e., elements of the energy_grid.data list (partially) inside the node are above the threshold.
        '''
        # Wait for the energy grid to be available
        while self.energy_grid is None:
            rclpy.spin_once(self)
            self.get_logger().info("Waiting for energy grid to be available...")

        # Update cells_to_sanitize based on the energy grid
        new_cells_to_sanitize = [grid_id for grid_id in self.cells_to_sanitize if self.energy_grid.data[grid_id] < 100]
        
        if len(new_cells_to_sanitize) != len(self.cells_to_sanitize):
            self.get_logger().info(f"Updated cells_to_sanitize from {len(self.cells_to_sanitize)} to {len(new_cells_to_sanitize)}")
        
        self.cells_to_sanitize = new_cells_to_sanitize
        
        # Update room occupancy based on sanitized cells
        for grid_id in set(self.cells_to_sanitize):
            self.room.data[grid_id] = 0

        self.room_publisher.publish(self.room)

        threshold = (self.energy_grid.info.resolution / 2 + GRID_RESOLUTION / 2)*1.41
        new_subset = []

        for node in subset:
            node_x, node_y = node[1], node[2]
            is_sanitized = True
            for grid_id in self.cells_to_sanitize:
                cell_x = (grid_id % self.energy_grid.info.width) * self.energy_grid.info.resolution + self.energy_grid.info.resolution / 2.0
                cell_y = (grid_id // self.energy_grid.info.width) * self.energy_grid.info.resolution + self.energy_grid.info.resolution / 2.0
                dist = math.sqrt((cell_x - node_x)**2 + (cell_y - node_y)**2)
                if dist < threshold:
                    is_sanitized = False
                    break
            if not is_sanitized:
                new_subset.append(node)
        
        if len(new_subset) ==0 and len(self.cells_to_sanitize) > 0:
            self.get_logger().warn("All nodes sanitized, but cells to sanitize still present")
            if False:
                # add remaining cells as nodes
                for grid_id in self.cells_to_sanitize:
                    cell_x = (grid_id % self.energy_grid.info.width) * self.energy_grid.info.resolution + self.energy_grid.info.resolution / 2.0
                    cell_y = (grid_id // self.energy_grid.info.width) * self.energy_grid.info.resolution + self.energy_grid.info.resolution / 2.0
                    new_subset.append((len(new_subset), cell_x, cell_y, True))
            if True:
                # only add closest remaining cell as node
                min_dist = 1e9
                closest_cell = None
                for grid_id in self.cells_to_sanitize:
                    cell_x = (grid_id % self.energy_grid.info.width) * self.energy_grid.info.resolution + self.energy_grid.info.resolution / 2.0
                    cell_y = (grid_id // self.energy_grid.info.width) * self.energy_grid.info.resolution + self.energy_grid.info.resolution / 2.0
                    self.get_logger().info(f"Robot position: {self.px}, {self.py}")
                    dist = math.sqrt((cell_x - self.px)**2 + (cell_y - self.py)**2)
                    if dist < min_dist:
                        min_dist = dist
                        closest_cell = (len(new_subset), cell_x, cell_y, True)
                if closest_cell is not None:
                    new_subset.append(closest_cell)

        self.get_logger().info(f"Subset size from {len(subset)} to {len(new_subset)}")
        
        return new_subset


    
    ##############################################################################################################
    # MAP READER TESTING FUNCTIONS

    def goals2file(self, goal_poses):
        '''
        This function writes the goal poses to a file
        Format:
        seq_id frame_id x y z qx qy qz qw check_string
        e.g. 0 map 0.0 0.0 0.0 0.0 0.0 0.0 1.0 end
        '''
        with open('/home/guglielmo/project_ws/src/goal_navigator/goal_poses.txt', 'w') as file:
            for i, pose in enumerate(goal_poses):
                file.write(f"{i} map {pose.pose.position.x} {pose.pose.position.y} {pose.pose.position.z} {pose.pose.orientation.x} {pose.pose.orientation.y} {pose.pose.orientation.z} {pose.pose.orientation.w} end\n")
        self.get_logger().info("Goal poses written to goal_poses.txt")

    def draw_free_cells(self):
        '''
        This function colors the free cells in green and saves the image
        '''
        if SAVE_MAP:
            free_cell_counter = 0
            image_free_cells = self.map_image.copy()
            cell_size_pixels = int(GRID_RESOLUTION / self.map_resolution)
            
            for cell in self.cells:
                if cell[3]:  # If the cell is free
                    # Calculate the pixel coordinates of the cell
                    start_x = int((cell[1] - GRID_RESOLUTION / 2) / self.map_resolution)
                    start_y = int((cell[2] - GRID_RESOLUTION / 2) / self.map_resolution)
                    end_x = start_x + cell_size_pixels
                    end_y = start_y + cell_size_pixels

                    # Ensure the coordinates are within the image boundaries
                    start_x = max(0, start_x)
                    start_y = max(0, start_y)
                    end_x = min(self.map_width, end_x)
                    end_y = min(self.map_height, end_y)

                    # Fill the cell with green color
                    image_free_cells[start_y:end_y, start_x:end_x] = (0, 255, 0)
                    free_cell_counter += 1


            # Draw grid lines for visualization
            for x in range(0, self.map_width, cell_size_pixels):
                image_free_cells = cv2.line(image_free_cells, (x, 0), (x, self.map_height), (255, 0, 0), 1)
            for y in range(0, self.map_height, cell_size_pixels):
                image_free_cells = cv2.line(image_free_cells, (0, y), (self.map_width, y), (255, 0, 0), 1)

            self.get_logger().info(f"Free cells found: {free_cell_counter}")
            cv2.imwrite(FREE_CELLS_FILE, image_free_cells)
            self.get_logger().info(f"Free cells drawn on map image: {FREE_CELLS_FILE}")

    def draw_grid(self, grid_resolution):
        '''
        This function draws a grid on the map image, with the specified resolution, saves it
        '''
        if SAVE_MAP:
            image_grid = self.map_image.copy()
            for x in range(0, self.map_width, int(grid_resolution / self.map_resolution)):
                image_grid = cv2.line(image_grid, (x, 0), (x, self.map_height), (255, 0, 0), 1)
            for y in range(0, self.map_height, int(grid_resolution / self.map_resolution)):
                image_grid = cv2.line(image_grid, (0, y), (self.map_width, y), (255, 0, 0), 1)
            cv2.imwrite(MAP_GRID_FILE, image_grid)
            self.get_logger().info(f"Grid drawn on map image: {MAP_GRID_FILE}")

##############################################################################################################
#
# BASIC GOAL NAVIGATOR
#
##############################################################################################################

class GoalNavigatorNode(Node):
    '''
    This class reads a list of goal locations from a file and navigates to each location in sequence
    '''
    def __init__(self):
        super().__init__('goal_navigator')
        self.goal_poses = []  # List of goals to navigate to
        self.n_goals = len(self.goal_poses)
        self.goal_index = 0
        self.goal_publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.navigate_through_poses_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.path_publisher_total = self.create_publisher(Path, 'total_planned_path', 10)
        self.path_publisher_partial = self.create_publisher(Path, 'partial_planned_path', 10)
        self.last_goal_rob_x = 0.0
        self.last_goal_rob_y = 0.0

    def get_poses(self, poses):
        '''
        This function is used to get the list of goal poses
        '''
        self.goal_poses = poses
        self.n_goals = len(self.goal_poses)

    def wait_for_goal_reached(self):
        # Wait for the action server to be available
        #self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()

        # Sending the goal to the action server
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.goal_poses[self.goal_index]

        #self.get_logger().info('Sending goal to action server...')
        future = self.action_client.send_goal_async(goal_msg)

        # Wait for the goal to be accepted
        #self.get_logger().info('Waiting for goal to be accepted...')
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle:
            self.get_logger().error('Goal handle is None, something went wrong.')
            return

        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the action server.')
            return

        #self.get_logger().info('Goal accepted by action server, waiting for result...')

        # Wait for the result of the goal
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if True:
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Goal reached successfully!')
                self.last_goal_rob_x = self.goal_poses[self.goal_index].pose.position.x
                self.last_goal_rob_y = self.goal_poses[self.goal_index].pose.position.y
            else:
                self.get_logger().error(f'Goal failed with status: {result.status}')
    
    def publish_path(self, goal_poses, publisher):
        # Publish path on rviz
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = goal_poses
        publisher.publish(path)

    def navigate_through_goals(self):
        '''
        This function navigates through the list of goals using action client
        '''
        if self.n_goals == 0:
            self.get_logger().info("No goals to navigate to")
            return
        self.get_logger().info("Starting navigation to goals")

        while self.goal_index < self.n_goals:

            # If the current goal is on a straight line with next goal, skip it
            if self.goal_index < self.n_goals - 1 and self.goal_index > 0:
                current_goal = self.goal_poses[self.goal_index - 1]
                next_goal = self.goal_poses[self.goal_index]
                goal_after_next = self.goal_poses[self.goal_index + 1]

                dx1 = next_goal.pose.position.x - current_goal.pose.position.x
                dy1 = next_goal.pose.position.y - current_goal.pose.position.y
                dx2 = goal_after_next.pose.position.x - next_goal.pose.position.x
                dy2 = goal_after_next.pose.position.y - next_goal.pose.position.y

                dot_product = dx1 * dx2 + dy1 * dy2
                norm1 = math.sqrt(dx1**2 + dy1**2)
                norm2 = math.sqrt(dx2**2 + dy2**2)
                try:
                    angle = math.acos(dot_product / (norm1 * norm2))
                except ValueError:
                    # Handle the math domain error
                    angle = 0  # or any appropriate default value


                # Check if the angle is less than 15/4 degrees
                if angle < math.pi / 48:
                    self.get_logger().info("Skipping goal %d" % self.goal_index)
                    self.goal_index += 1
                    continue

            self.get_logger().info("Navigating to goal %d" % self.goal_index)
            self.wait_for_goal_reached()
            rclpy.spin_once(self)
            self.goal_index += 1


        self.get_logger().info("All goals tried")
        self.goal_index = 0

    def navigate_through_goals2(self):
        '''
        This function navigates through the list of goals using NavigateThroughPoses action
        '''
        if self.n_goals == 0:
            self.get_logger().info("No goals to navigate to")
            return
        self.get_logger().info("Starting navigation to goals")
        self.navigate_through_poses_client.wait_for_server()
        goal_msg = NavigateThroughPoses.Goal()

        # Add a small random offset to last pose to visit to ensure it is not equal to the current one (last one of previous iteration)
        last_pose = self.goal_poses[-1]
        last_pose.pose.position.x += random.uniform(-0.1, 0.1)
        last_pose.pose.position.y += random.uniform(-0.1, 0.1)
        self.goal_poses[-1] = last_pose

        goal_msg.poses = self.goal_poses
        future = self.navigate_through_poses_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if not goal_handle:
            self.get_logger().error('Goal handle is None, something went wrong.')
            return
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the action server.')
            return
        
        self.get_logger().info('Goal accepted by action server, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result:
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Goal reached successfully!')
            else:
                self.get_logger().error(f'Goal failed with status: {result.status}')
        else:
            self.get_logger().error('Failed to get result')

    def destroy_node(self):
        self.action_client.destroy()
        self.navigate_through_poses_client.destroy()
        super().destroy_node()


##############################################################################################################
#
# generic helper functions
#
##############################################################################################################

def read_user_input():
    '''
    This function reads the user input from a file
    '''
    room_ids = []
    with open(USER_INPUT_FILE, 'r') as file:
        room_ids = [int(line) for line in file.readlines()]

    if 7 in room_ids:
        # insert 8 right after 7
        room_ids.insert(room_ids.index(7)+1, 8)

    return room_ids

##############################################################################################################
#
# MAIN
#
##############################################################################################################

def main(args=None):
    rclpy.init(args=args)
    map_reader = MapReader()
    map_reader.cell_selection_no_walls(GRID_RESOLUTION)

    room_ids = read_user_input()
    map_reader.get_logger().info("Rooms to sanitize: " + str(room_ids))

    last_goal_rob_x = map_reader.px
    last_goal_rob_y = map_reader.py
    
    for i in range(len(room_ids)):
        iterations = 1

        map_reader.get_logger().info(f"Sanitizing room {room_ids[i]}")

        # Define room corners
        top_left_x, top_left_y, bottom_right_x, bottom_right_y = map_reader.room_id_mapper(room_ids[i])
        top_left_x, top_left_y = map_reader.rviz2mymap(top_left_x, top_left_y)
        bottom_right_x, bottom_right_y = map_reader.rviz2mymap(bottom_right_x, bottom_right_y)

        # Get subset of cells within the room bounds
        subset_cells = map_reader.cell_subset(top_left_x, top_left_y, bottom_right_x, bottom_right_y)
        map_reader.get_room_cells(top_left_x, top_left_y, bottom_right_x, bottom_right_y, subset_cells)

        while not map_reader.is_room_sanitized():
            waypoints = []
            flag = False
            n_cells_left = len(map_reader.cells_to_sanitize)
            
            if n_cells_left < MAX_ENERGY_CELLS_HEURISTIC:
                flag = True
                map_reader.get_logger().info("Visiting remaining cells directly")
                waypoints = map_reader.plan_path_cells_left()
            else:
                map_reader.get_logger().info("Standard TSP path planning")
                subset_cells = map_reader.remove_sanitized_nodes(subset_cells)  # Only visit unsanitized cells
                if not subset_cells:
                    map_reader.get_logger().warn("No more nodes to visit, but some cells remain unsanitized.")
                    break
                waypoints = map_reader.plan_path_through_nodes(subset_cells, last_goal_rob_x, last_goal_rob_y)
            
            goal_poses = map_reader.waypoints2poses(map_reader.convert_to_rviz(waypoints))

            # Navigate through the room covering path
            map_reader.get_logger().info(f"Room {room_ids[i]}, iteration {iterations}")
            iterations += 1
            goal_navigator = GoalNavigatorNode()
            
            n_goals = N_NODES_TO_VISIT if not flag else 1
            visu_ignored_goals = goal_poses[n_goals-1:]
            goal_navigator.publish_path(visu_ignored_goals, goal_navigator.path_publisher_total)
            goal_poses = goal_poses[:n_goals]
            goal_navigator.publish_path(goal_poses, goal_navigator.path_publisher_partial)
            goal_navigator.get_poses(goal_poses)
            goal_navigator.navigate_through_goals()
            last_goal_rob_x = goal_navigator.last_goal_rob_x
            last_goal_rob_y = goal_navigator.last_goal_rob_y
            goal_navigator.destroy_node()          

        map_reader.get_logger().info(f"Room {room_ids[i]} sanitized in {iterations-1} iterations")
    map_reader.get_logger().info("Sanitization completed")

    map_reader.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
