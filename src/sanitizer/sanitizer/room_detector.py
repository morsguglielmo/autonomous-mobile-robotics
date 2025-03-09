from rclpy.node import Node
import cv2
import yaml
import os
import numpy as np
import rclpy

MAP_YAML_FILE = '/home/guglielmo/project_ws/map/map.yaml'
CLOSED_DOORS_MAP_FILE = '/home/guglielmo/project_ws/map/map_with_closed_doors.png'

class RoomDetector(Node):
    def __init__(self):
        super().__init__('map_reader_node')

        # Read map resolution and image file path from YAML file
        map_resolution, image_file = self.read_map_metadata(MAP_YAML_FILE)
        if map_resolution is None or image_file is None:
            self.get_logger().error(f"Error: Could not read map metadata from {MAP_YAML_FILE}")
            return

        self.map_resolution = map_resolution
        self.map_image = cv2.imread(image_file, cv2.IMREAD_COLOR)
        self.map_width = self.map_image.shape[1]
        self.map_height = self.map_image.shape[0]
        self.get_logger().info(f"Map image loaded: {image_file} ({self.map_width}x{self.map_height})")

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
    
    def detect_rooms(self):
        '''
        Detects rooms in the map image by finding contours.
        '''
        # Convert map image to grayscale
        gray = cv2.cvtColor(self.map_image, cv2.COLOR_BGR2GRAY)

        # Threshold the image
        ret, thresh = cv2.threshold(gray, 127, 255, 0)

        # Find contours
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Draw contours on the map image
        cv2.drawContours(self.map_image, contours, -1, (0, 255, 0), 3)
        cv2.imshow('Map', self.map_image)

    def close_doors(self):
        '''
        Detects doors in the map, then draws a line on the map image to close the doors.
        '''

        # Convert map image to grayscale
        gray = cv2.cvtColor(self.map_image, cv2.COLOR_BGR2GRAY)

        # Threshold the image
        ret, thresh = cv2.threshold(gray, 127, 255, 0)

        # Find lines
        lines = cv2.HoughLinesP(thresh, 1, np.pi/180, 100, minLineLength=100, maxLineGap=0.5)

        # Draw lines on the map image
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(self.map_image, (x1, y1), (x2, y2), (0, 0, 255), 3)

        cv2.imshow('Map with closed doors', self.map_image)
        cv2.imwrite(CLOSED_DOORS_MAP_FILE, self.map_image)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()




def main(args=None):
    rclpy.init(args=args)

    room_detector = RoomDetector()
    room_detector.detect_rooms()
    room_detector.close_doors()

    #rclpy.spin(room_detector)

    room_detector.destroy_node()
    rclpy.shutdown()










