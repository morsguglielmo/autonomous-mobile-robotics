import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
import tf2_ros
import tf2_geometry_msgs
from rclpy.duration import Duration
from std_msgs.msg import Bool
from time import sleep


class TextFileReader:
    '''
    Text has the format:
    seq_id frame_id x y z qx qy qz qw check_string
    e.g. 0 map 0.0 0.0 0.0 0.0 0.0 0.0 1.0 end
         1 map 1.0 1.0 0.0 0.0 0.0 0.0 1.0 end
    '''
    def __init__(self, file_name):
        self.file_name = file_name
        self.file = open(file_name, "r")
        self.lines = self.file.readlines()
        self.file.close()
        print("File reader initialized")

    def lines2goalposes(self):
        goal_poses = []
        for line in self.lines:
            words = line.split()
            if words[-1] == "end":
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = words[1]
                goal_pose.pose.position.x = float(words[2])
                goal_pose.pose.position.y = float(words[3])
                goal_pose.pose.position.z = float(words[4])
                goal_pose.pose.orientation.x = float(words[5])
                goal_pose.pose.orientation.y = float(words[6])
                goal_pose.pose.orientation.z = float(words[7])
                goal_pose.pose.orientation.w = float(words[8])
                goal_poses.append(goal_pose)
        print("Read %d goal poses" % len(goal_poses))
        return goal_poses

class GoalNavigatorNode(Node):
    '''
    This class reads a list of goal locations from a file and navigates to each location in sequence
    '''
    def __init__(self):
        super().__init__('goal_navigator')
        self.goal_poses = []  # List of goals to navigate to
        self.n_goals = len(self.goal_poses)
        self.goal_index = 0
        self.Pose = None
        self.localization_ok = False

        self.goal_publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pose_subscription = self.create_subscription(PoseStamped, 'amcl_pose', self.pose_callback, 10)
        self.localization_ok_subscription = self.create_subscription(Bool, 'localization_ok', self.localization_ok_callback, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.tfBuffer = tf2_ros.Buffer(cache_time=Duration(seconds=10))  # Specify cache time
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

    def localization_ok_callback(self, msg):
        '''
        This function is used to check if the localization is ok
        '''
        self.localization_ok = msg.data

    def timer_callback(self):
        '''
        Update self.Pose
        '''
        rclpy.spin_once(self)

    def pose_callback(self, msg):
        '''
        This function is used to get the current pose of the robot
        '''
        self.Pose = msg.pose
    
    def get_poses(self, poses):
        '''
        This function is used to get the list of goal poses
        '''
        self.goal_poses = poses
        self.n_goals = len(self.goal_poses)

    def baselink_to_map(self, pose):
        '''
        This function is used to transform the goal pose from base_link frame to map frame
        '''
        new_pose = PoseStamped()
        new_pose.header.frame_id = "map"
        new_pose.header.stamp = self.get_clock().now().to_msg()
        try:
            # Use the latest available time for the transform
            latest_time = self.tfBuffer.get_latest_common_time("map", "base_link")
            transform = self.tfBuffer.lookup_transform("map", "base_link", latest_time)
            new_pose.pose = tf2_geometry_msgs.do_transform_pose(pose, transform).pose
            return new_pose
        except Exception as e:
            self.get_logger().error(f"Could not transform pose: {e}")
            return None
        

    def wait_for_goal_reached(self):
        # Wait for the action server to be available
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()

        # Sending the goal to the action server
        goal_msg = NavigateToPose.Goal()
        if self.goal_poses[self.goal_index].header.frame_id == "map":
            goal_msg.pose = self.goal_poses[self.goal_index]
        elif self.goal_poses[self.goal_index].header.frame_id == "base_link":
            self.get_logger().info('Transforming goal pose to map frame...')
            sleep(0.5)  # Adding a small delay to ensure transform data is available
            transformed_pose = self.baselink_to_map(self.goal_poses[self.goal_index])
            if transformed_pose is not None:
                goal_msg.pose = transformed_pose
            else:
                self.get_logger().error('Could not transform goal pose')
                return

        else:
            self.get_logger().error("Frame id not supported")
            return

        self.get_logger().info('Sending goal to action server...')
        future = self.action_client.send_goal_async(goal_msg)

        # Wait for the goal to be accepted
        self.get_logger().info('Waiting for goal to be accepted...')
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle:
            self.get_logger().error('Goal handle is None, something went wrong.')
            return

        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the action server.')
            return

        self.get_logger().info('Goal accepted by action server, waiting for result...')

        # Wait for the result of the goal
        result_future = goal_handle.get_result_async()

        # Introduce a timeout to avoid getting stuck indefinitely
        timeout = 600.0  # 10 minutes
        start_time = self.get_clock().now()
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=1)
            elapsed_time = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed_time > timeout:
                self.get_logger().error('Goal result timeout exceeded.')
                return

        result = result_future.result()

        if result is None:
            self.get_logger().error('Result is None, something went wrong.')
            return

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached successfully!')
        else:
            self.get_logger().error(f'Goal failed with status: {result.status}')

    def navigate_through_goals(self):
        '''
        This function navigates through the list of goals using action client
        '''
        if self.n_goals == 0:
            self.get_logger().info("No goals to navigate to")
            return
        self.get_logger().info("Starting navigation to goals")
        while self.goal_index < self.n_goals:
            self.get_logger().info("Navigating to goal %d" % self.goal_index)
            self.wait_for_goal_reached()
            self.goal_index += 1

        self.get_logger().info("All goals tried")

def main(args=None):
    rclpy.init(args=args)
    goal_navigator = GoalNavigatorNode()
    while not goal_navigator.localization_ok:
        rclpy.spin_once(goal_navigator)
        #goal_navigator.get_logger().info("Waiting for localization to be accurate...")
    print("Starting goal navigator")
    text_file_reader = TextFileReader("/home/guglielmo/project_ws/src/goal_navigator/goals.txt")
    goal_poses = text_file_reader.lines2goalposes()
    goal_navigator.get_poses(goal_poses)
    goal_navigator.navigate_through_goals()
    goal_navigator.destroy_node()
    print("Goal navigator finished")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
