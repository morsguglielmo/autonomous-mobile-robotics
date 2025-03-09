from launch import LaunchDescription
from launch_ros.actions import Node

# lunch goal_navigator node and bug_localizer node

def generate_launch_description():
    goal_navigator_node = Node(
        package="goal_navigator",
        executable="goal_navigator",
        name="goal_navigator",
        output="screen",
        parameters=[],
    )

    bug_localizer_node = Node(
        package="roaming_localizer",
        executable="bug_localizer",
        name="bug_localizer",
        output="screen",
        parameters=[],
    )

    ld = LaunchDescription()
    ld.add_action(goal_navigator_node)
    ld.add_action(bug_localizer_node)

    return ld