import rclpy
from rclpy.node import Node
import cv2
import yaml
import os
import math
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

MAP_YAML_FILE = '/home/guglielmo/project_ws/map/map.yaml'

ENERGY_GRID_RESOLUTION = 0.2 # [m]
P_I = 0.0001 # [W]
ROBOT_ENCUMBRANCE = 0.1# [m]
UV_RANGE = 5.0 # [m]
POWER_MULTIPLIER = 1.0 # for testing purposes
BLACK_PIXEL_TOLERANCE = 0.001

def quaternion_to_euler(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.degrees(math.atan2(t0, t1))
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.degrees(math.atan2(t3, t4))
    return X, Y, Z

def angle_normalize(angle):
    '''
    Normalize the angle to the range [-pi, pi]
    '''
    return math.atan2(math.sin(angle), math.cos(angle))


class PowerMapper(Node):
    def __init__(self):
        super().__init__('power_mapper')

        map_resolution, image_file = self.read_map_metadata(MAP_YAML_FILE)
        if map_resolution is None or image_file is None:
            self.get_logger().error(f"Error: Could not read map metadata from {MAP_YAML_FILE}")
            return
        
        self.map_resolution = map_resolution
        self.map_image = cv2.imread(image_file, cv2.IMREAD_COLOR)
        if self.map_image is None:
            self.get_logger().error(f"Error: Could not load map image from {image_file}")
            return
        
        self.map_width = self.map_image.shape[1]
        self.map_height = self.map_image.shape[0]
        self.get_logger().info(f"Map image loaded: {image_file} ({self.map_width}x{self.map_height})")

        self.power_grid = OccupancyGrid()
        self.power_grid.header.frame_id = 'map'
        self.power_grid.info.resolution = ENERGY_GRID_RESOLUTION
        self.power_grid.info.width = int(self.map_width * self.map_resolution / ENERGY_GRID_RESOLUTION)
        self.power_grid.info.height = int(self.map_height * self.map_resolution / ENERGY_GRID_RESOLUTION)
        self.power_grid.info.origin.position.x = -7.45
        self.power_grid.info.origin.position.y = 5.2
        self.power_grid.info.origin.position.z = 0.0
        self.power_grid.info.origin.orientation.x = 1.0
        self.power_grid.info.origin.orientation.y = 0.0
        self.power_grid.info.origin.orientation.z = 0.0
        self.power_grid.info.origin.orientation.w = 0.0
        self.power_grid.data = [0] * (self.power_grid.info.width * self.power_grid.info.height)

        self.delta_t = 0.1
        self.px = None
        self.py = None
        self.yaw = None
        self.scan = LaserScan()
        self.UV_area_vertices = []

        self.occupied_ids = []

        self.timer = self.create_timer(self.delta_t, self.timer_callback)
        self.scan_subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.pose_subscription = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.pose_callback, 10)
        self.power_grid_publisher = self.create_publisher(OccupancyGrid, 'power_grid', 10)

        self.set_occupied_ids()

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
                        return False

        return True
    
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
    
    def set_occupied_ids(self):
        '''
        This function sets the ids of the occupied cells
        '''
        self.occupied_ids = []
        for grid_id in range(len(self.power_grid.data)):
            cell_x = (grid_id % self.power_grid.info.width) * ENERGY_GRID_RESOLUTION + ENERGY_GRID_RESOLUTION / 2.0
            cell_y = (grid_id // self.power_grid.info.width) * ENERGY_GRID_RESOLUTION + ENERGY_GRID_RESOLUTION / 2.0
            if not self.is_cell_free(cell_x, cell_y, ENERGY_GRID_RESOLUTION):
                self.occupied_ids.append(grid_id)

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
    
    def scan_callback(self, msg):
        self.scan = msg

    def pose_callback(self, msg):
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = quaternion_to_euler(q)
        self.yaw = angle_normalize(math.radians(yaw))

    def timer_callback(self):
        # if pose not initialized, return
        if self.px is None or self.py is None or self.yaw is None:
            return
        
        self.update_UV_area()
        self.power_evaluation()
        return None
        

    def mymap2rviz(self, x, y):
        x_out = x - 7.45
        y_out = -(y - 5.2)
        return x_out, y_out

    def rviz2mymap(self, x, y):
        x_out = x + 7.45
        y_out = -y + 5.2
        return x_out, y_out


    def power_evaluation(self):

        for grid_id in range(len(self.power_grid.data)):
            if grid_id in self.occupied_ids:
                self.power_grid.data[grid_id] = 0
                continue
            cell_x = (grid_id % self.power_grid.info.width) * ENERGY_GRID_RESOLUTION + ENERGY_GRID_RESOLUTION / 2.0
            cell_y = (grid_id // self.power_grid.info.width) * ENERGY_GRID_RESOLUTION + ENERGY_GRID_RESOLUTION / 2.0
            cell_x_rviz, cell_y_rviz = self.mymap2rviz(cell_x, cell_y)
            if self.distance_from_robot(cell_x_rviz, cell_y_rviz) < ROBOT_ENCUMBRANCE:
                power = 0
            elif not self.inside_UV_area(cell_x, cell_y):
                power = 0
            else:
                power = 100
            self.power_grid.data[grid_id] = power
        # Publish the power grid
        self.power_grid_publisher.publish(self.power_grid)
        return None


    def distance_from_robot(self, x, y):
        return math.sqrt((x - self.px)**2 + (y - self.py)**2)
    
    def map2robot(self, x, y):
        '''
        This function converts the coordinates from the map frame to the robot frame
        '''
        robot_T = np.array([[math.cos(self.yaw), -math.sin(self.yaw), self.px],
                                [math.sin(self.yaw), math.cos(self.yaw), self.py],
                                [0, 0, 1]])
        vector_map = np.array([[x], [y], [1]])
        robot_T_inv = np.linalg.inv(robot_T)
        vector_robot = robot_T_inv @ vector_map

        return vector_robot[0][0], vector_robot[1][0]
    
    def robot2map(self, x, y):
        '''
        This function converts the coordinates from the robot frame to the map frame
        '''
        robot_T = np.array([[math.cos(self.yaw), math.sin(-self.yaw), self.px],
                                [math.sin(self.yaw), math.cos(self.yaw), self.py],
                                [0, 0, 1]])
        vector_robot = np.array([[x], [y], [1]])
        vector_map = robot_T @ vector_robot

        return vector_map[0][0], vector_map[1][0]

    
    
    def update_UV_area(self):
        '''
        This function updates the UV area vertices, which is a list of points that represent perimeter of the UV area
        Updates and publishes the UV area marker
        '''
        self.UV_area_vertices = []
        for i in range(len(self.scan.ranges)):
            if i%5 == 0:
                if math.isnan(self.scan.ranges[i]) or self.scan.ranges[i] > UV_RANGE:
                    dist = UV_RANGE
                else:
                    dist = self.scan.ranges[i]
                
                angle = self.scan.angle_min + i * self.scan.angle_increment
                x = dist * math.cos(angle)
                y = dist * math.sin(angle)
                x_map, y_map = self.robot2map(x, y)
                self.UV_area_vertices.append((x_map, y_map))

        return None
    


    def inside_UV_area(self, x, y):
        '''
        This function checks if a point (x, y) is inside the UV area
        UV area is a polygon defined by the vertices in self.UV_area_vertices
        '''
        if len(self.UV_area_vertices) == 0:
            return False
        
        x, y = self.mymap2rviz(x, y)
        n = len(self.UV_area_vertices)
        inside = False
        p1x, p1y = self.UV_area_vertices[0]
        for i in range(n+1):
            p2x, p2y = self.UV_area_vertices[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside
    

    def power_scaling(self, power_value):
        if power_value == 0.0:
            return 0
        else:
            return 100



def main(args=None):
    rclpy.init(args=args)
    power_mapper = PowerMapper()
    rclpy.spin(power_mapper)
    power_mapper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
