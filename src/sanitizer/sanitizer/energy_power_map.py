import rclpy
from rclpy.node import Node
import cv2
import yaml
import os
import math
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, Polygon, Point32
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray

MAP_YAML_FILE = '/home/guglielmo/project_ws/map/map.yaml'

ENERGY_GRID_RESOLUTION = 0.2 # [m]
P_I = 0.0001 # [W]
ENERGY_THRESHOLD = 0.01 # [J]
ROBOT_ENCUMBRANCE = 0.1 # [m]
POWER_MULTIPLIER = 10.0 # for testing purposes

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


class EnergyMapper(Node):
    def __init__(self):
        super().__init__('energy_mapper')

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

        self.energy_cells = []
        self.energy_grid = OccupancyGrid()
        self.energy_grid.header.frame_id = 'map'
        self.energy_grid.info.resolution = ENERGY_GRID_RESOLUTION
        self.energy_grid.info.width = int(self.map_width * self.map_resolution / ENERGY_GRID_RESOLUTION)
        self.energy_grid.info.height = int(self.map_height * self.map_resolution / ENERGY_GRID_RESOLUTION)
        self.energy_grid.info.origin.position.x = -7.45
        self.energy_grid.info.origin.position.y = 5.2
        self.energy_grid.info.origin.position.z = 0.0
        self.energy_grid.info.origin.orientation.x = 1.0
        self.energy_grid.info.origin.orientation.y = 0.0
        self.energy_grid.info.origin.orientation.z = 0.0
        self.energy_grid.info.origin.orientation.w = 0.0
        self.energy_grid.data = [-1] * (self.energy_grid.info.width * self.energy_grid.info.height)


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
        self.power_grid.data = [-1] * (self.power_grid.info.width * self.power_grid.info.height)



        self.delta_t = 0.01
        #self.i = 0
        self.px = 0.0
        self.py = 0.0
        self.yaw = 0.0
        self.scan = LaserScan()
        self.UV_area_vertices = []
        #self.UV_area_marker_array = MarkerArray()

        self.energy_cell_initialization(ENERGY_GRID_RESOLUTION)

        self.timer = self.create_timer(self.delta_t, self.timer_callback)
        self.scan_subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.pose_subscription = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.pose_callback, 10)
        self.energy_grid_publisher = self.create_publisher(OccupancyGrid, 'energy_grid', 10)
        self.power_grid_publisher = self.create_publisher(OccupancyGrid, 'power_grid', 10)
        #self.marker_publisher = self.create_publisher(Polygon, 'marker', 10)

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

        self.update_UV_area()
        self.power_evaluation()
        self.update_energy()
        self.update_energy_gridmap()
        self.update_power_gridmap()
        
        #self.marker_publisher.publish(self.create_marker())
        self.get_logger().info(f"Robot pose: ({self.px}, {self.py}, {self.yaw})")
        

    def mymap2rviz(self, x, y):
        x_out = x - 7.45
        y_out = -(y - 5.2)
        return x_out, y_out

    def rviz2mymap(self, x, y):
        x_out = x + 7.45
        y_out = -y + 5.2
        return x_out, y_out

    def energy_cell_initialization(self, energy_grid_resolution):
        half_grid_res = energy_grid_resolution / 2.0
        num_cells_x = int(self.map_width * self.map_resolution / energy_grid_resolution)
        num_cells_y = int(self.map_height * self.map_resolution / energy_grid_resolution)
        self.get_logger().info(f"Creating energy cells: {num_cells_x}x{num_cells_y}")
        for i in range(num_cells_x):
            for j in range(num_cells_y):
                center_x = i * energy_grid_resolution + half_grid_res
                center_y = j * energy_grid_resolution + half_grid_res
                energy_level = 0.0
                power = 0.0
                self.energy_cells.append((len(self.energy_cells), center_x, center_y, energy_level, power))
        
        self.get_logger().info(f"Energy cells created: {len(self.energy_cells)}")
        return None

    def delta_energy(self, P_I, delta_t, x, y, px, py):
        px, py = self.rviz2mymap(px, py)
        numerator = P_I * delta_t * POWER_MULTIPLIER
        denominator = (x - px)**2 + (y - py)**2
        result = numerator / denominator
        return result

    def update_energy(self):
        for cell in self.energy_cells:
            energy_level = self.delta_energy(cell[4], self.delta_t, cell[1], cell[2], self.px, self.py) + cell[3]
            self.energy_cells[cell[0]] = (cell[0], cell[1], cell[2], energy_level, cell[4])
        return None
    
    def power_evaluation(self):
        counter = 0
        for cell in self.energy_cells:
            cell_x_rviz, cell_y_rviz = self.mymap2rviz(cell[1], cell[2])
            if self.distance_from_robot(cell_x_rviz, cell_y_rviz) < ROBOT_ENCUMBRANCE:
                power = 0.0
            elif not self.inside_UV_area(cell[1], cell[2]):
                power = 0.0
            else:
                power = P_I
                counter += 1
            self.energy_cells[cell[0]] = (cell[0], cell[1], cell[2], cell[3], power)
        #self.get_logger().info(f"Number of cells in UV area: {counter}")

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
            if math.isnan(self.scan.ranges[i]):
                continue
            angle = self.scan.angle_min + i * self.scan.angle_increment
            x = self.scan.ranges[i] * math.cos(angle)
            y = self.scan.ranges[i] * math.sin(angle)
            x_map, y_map = self.robot2map(x, y)
            self.UV_area_vertices.append((x_map, y_map))

        #self.get_logger().info(f"Number of UV area vertices: {len(self.UV_area_vertices)}")
        #self.marker_publisher.publish(self.create_marker())

        return None
    
    def create_marker1(self):
        '''
        This function creates a marker for each vertex of the UV area
        creates lines between the vertices to visualize the UV area
        '''
        marker_array = MarkerArray()
        for i in range(len(self.UV_area_vertices)):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = self.UV_area_vertices[i][0]
            marker.pose.position.y = self.UV_area_vertices[i][1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)

        
        return marker_array
    
    def create_marker(self):
        '''
        This function creates a marker for visualizing the UV area as a Polygon
        '''
        poly = Polygon()
        for vertex in self.UV_area_vertices:
            point = Point32()
            point.x = vertex[0]
            point.y = vertex[1]
            point.z = 0.0
            poly.points.append(point)

        return poly

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
    
    
    def energy_scaling(self, energy_value):

        if energy_value > ENERGY_THRESHOLD:
            return 100
        else:
            return int((energy_value / ENERGY_THRESHOLD) * 100) 

    def power_scaling(self, power_value):

        if power_value == 0.0:
            return 0
        else:
            return 100

    def update_energy_gridmap(self):
        
        #self.energy_grid.data = [0] * (self.energy_grid.info.width * self.energy_grid.info.height)

        for cell in self.energy_cells:
            cell_x = cell[1]
            cell_y = cell[2]
            cell_grid_x = int((cell_x - ENERGY_GRID_RESOLUTION / 2.0)/ ENERGY_GRID_RESOLUTION)
            cell_grid_y = int((cell_y - ENERGY_GRID_RESOLUTION / 2.0)/ ENERGY_GRID_RESOLUTION)
            grid_index = cell_grid_y * self.energy_grid.info.width + cell_grid_x
            self.energy_grid.data[grid_index] = self.energy_scaling(cell[3])
        # publish energy grid
        self.energy_grid.header.stamp = self.get_clock().now().to_msg()
        self.energy_grid_publisher.publish(self.energy_grid)

    def update_power_gridmap(self):

        #self.power_grid.data = [0] * (self.power_grid.info.width * self.power_grid.info.height)

        for cell in self.energy_cells:
            cell_x = cell[1]
            cell_y = cell[2]
            cell_grid_x = int((cell_x - ENERGY_GRID_RESOLUTION / 2.0)/ ENERGY_GRID_RESOLUTION)
            cell_grid_y = int((cell_y - ENERGY_GRID_RESOLUTION / 2.0)/ ENERGY_GRID_RESOLUTION)
            grid_index = cell_grid_y * self.power_grid.info.width + cell_grid_x
            self.power_grid.data[grid_index] = self.power_scaling(cell[4])
        # publish power grid
        self.power_grid.header.stamp = self.get_clock().now().to_msg()
        self.power_grid_publisher.publish(self.power_grid)


def main(args=None):
    rclpy.init(args=args)
    energy_mapper = EnergyMapper()
    rclpy.spin(energy_mapper)
    energy_mapper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
