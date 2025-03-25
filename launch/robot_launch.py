import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('tri_project')
    robot1_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')
    robot2_description_path = os.path.join(package_dir, 'resource', 'my_robot2.urdf')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'map_0.wbt')
    )

    robot1_driver = WebotsController(
        robot_name='robot1',
        parameters=[
            {'robot_description': robot1_description_path},
        ]
    )

    robot2_driver = WebotsController(
        robot_name='robot2',
        parameters=[
            {'robot_description': robot2_description_path},
        ]
    )

    obstacle_avoider = Node(
        package='tri_project',
        executable='obstacle_avoider',
    )

    obstacle_robot_avoider = Node(
        package='tri_project',
        executable='obstacle_robot_avoider',
    )

    return LaunchDescription([
        webots,
        robot1_driver,
        robot2_driver,
        obstacle_avoider,
        obstacle_robot_avoider,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])