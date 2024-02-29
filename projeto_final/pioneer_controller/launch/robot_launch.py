import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('pioneer_controller')
    robot_description_path = os.path.join(package_dir, 'resource', 'pioneer.urdf')

    # start Webots with 'empty.wbt' world
    # initializes a 'Webots' ROS node
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'pioneer.wbt')
    )

    # initialize a 'WebotsController' ROS node to control the robot 'Pioneer 3-DX'
    # loads URDF and attaches controller plugin to the robot
    # @note required for each robot in the simulation
    pioneer_driver = WebotsController(
        robot_name='Pioneer 3-DX',    # must match robot label in Webots
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    # return node objects to be launched
    # @note a ROS action (cf. https://docs.ros.org/en/iron/Concepts/Basic/About-Actions.html)
    #       is declared on the webots node/process to force ROS shutdown on application exit/close
    return LaunchDescription([
        webots,
        pioneer_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])