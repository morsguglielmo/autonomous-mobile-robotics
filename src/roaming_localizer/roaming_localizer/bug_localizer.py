import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool
import random

COVARIANCE_THRESHOLD = 0.05  # threshold for the covariance to be considered ok
FORWARD_SPEED = 0.5
ROTATION_SPEED = 0.25
SCAN_THRESHOLD = 0.8  # threshold for the laser scan to detect an obstacle
SCAN_TO_CHECK = 30  # number of scans to check for obstacles (each side)
COV_INIT = 400.0  # initial covariance for the pose

class InitialPoseNode(Node):
    '''
    This class is used to set the initial pose of the robot, so that the localization algorithm can start
    Initial covariance is set to a high arbitrary value
    '''
    def __init__(self):
        super().__init__('initial_pose_node')
        self.initial_pose_publisher_ = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

    def set_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        # Setting frame id
        msg.header.frame_id = "map"
        # Random initial pose
        msg.pose.pose.position.x = random.uniform(-7, 7)
        msg.pose.pose.position.y = random.uniform(-7.5, 5)
        msg.pose.pose.position.z = 0.0

        # High covariance for the pose
        msg.pose.covariance = [COV_INIT, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                0.0, COV_INIT, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.0, COV_INIT, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, COV_INIT, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.0, COV_INIT, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.0, COV_INIT]
        
        self.initial_pose_publisher_.publish(msg)

class BugAlgorithmNode(Node):
    '''
    This class implements a bug-like algorithm for localization.
    The robot moves forward until an obstacle is detected.
    Then it rotates right until the obstacle is not detected, and moves forward again.
    The robot stops when the covariance of the pose is below a certain threshold.
    '''
    def __init__(self):
        super().__init__('bug_algorithm_node')
        self.vel_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.covariance_subscription = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.localization_callback, 10)
        self.localization_ok_publisher_ = self.create_publisher(Bool, 'localization_ok', 10)

        self.laser_data = []
        self.n_scan = 0
        self.covariance = []

    def scan_callback(self, msg):
        self.laser_data = msg.ranges
        self.n_scan = len(self.laser_data)

    def localization_callback(self, msg):
        self.covariance = msg.pose.covariance

    def localization_ok(self):
        # Max member of the covariance matrix
        if self.covariance is not None and len(self.covariance) > 0:

            max_covariance = max(self.covariance)
            self.get_logger().info("Max covariance: " + str(max_covariance))
            bool_msg = Bool()
            if max_covariance > COVARIANCE_THRESHOLD:
                bool_msg.data = False
            else:
                bool_msg.data = True
                self.get_logger().info("Localization ok")
            self.localization_ok_publisher_.publish(bool_msg)
            return bool_msg.data
        return False

    def wander(self, init_time, cnt):
        # Wait until covariance is populated
        while self.covariance is None or len(self.covariance) == 0:
            rclpy.spin_once(self)
        
        initial_time = self.get_clock().now()
        while not self.localization_ok():
            rclpy.spin_once(self)
            current_time = self.get_clock().now()
            if (current_time - init_time).nanoseconds > 120e9:
                self.stop()
                self.get_logger().info("Localization failed")
                return False
            if (current_time - initial_time).nanoseconds > 5e9 and (current_time - initial_time).nanoseconds < 6e9:
                self.rotate_right()
            elif (current_time - initial_time).nanoseconds > 5.5e9:
                initial_time = self.get_clock().now()
            else:
                self.move_forward()
                for i in range(SCAN_TO_CHECK):
                    if cnt % 2 == 0:
                        if self.laser_data[i] < SCAN_THRESHOLD or self.laser_data[self.n_scan - i - 1] < SCAN_THRESHOLD:
                            self.rotate_right()
                            break
                    else:
                        if self.laser_data[i] < SCAN_THRESHOLD or self.laser_data[self.n_scan - i - 1] < SCAN_THRESHOLD:
                            self.rotate_left()
                            break
                        
        self.stop()
        return True

    def move_forward(self):
        msg = Twist()
        msg.linear.x = FORWARD_SPEED
        self.vel_publisher_.publish(msg)

    def rotate_left(self):
        msg = Twist()
        msg.angular.z = ROTATION_SPEED
        self.vel_publisher_.publish(msg)

    def rotate_right(self):
        msg = Twist()
        msg.angular.z = -ROTATION_SPEED
        self.vel_publisher_.publish(msg)

    def stop(self):
        msg = Twist()
        self.vel_publisher_.publish(msg)

def main(args=None):
    cnt = 0
    check = False
    while not check:
        cnt += 1
        rclpy.init(args=args)
        initial_pose_node = InitialPoseNode()
        initial_pose_node.set_initial_pose()
        node = BugAlgorithmNode()
        init_time = node.get_clock().now()
        check = node.wander(init_time, cnt)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
