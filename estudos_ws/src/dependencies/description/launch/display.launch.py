import os
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_path

def generate_launch_description():

    """
    This function starts the system in a custom configuration.
    """

    # Generate the object that must be returned by this function
    ld = LaunchDescription()

##################################################
#  Mount the path to the files that will be used      
##################################################

    urdf_path = os.path.join(
        get_package_share_path('my_robot_description'),
        'urdf',
        'my_robot.urdf.xacro'
    )
    
    rviz_config_path = os.path.join(
        get_package_share_path('my_robot_description'),
        'rviz',
        'urdf_config.rviz'
    )

##################################################
#           Mount the parameter values           
##################################################
    
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

##################################################
#                 Mount the nodes                
##################################################

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

##################################################
#   Add the action to LaunchDescription object 
##################################################

    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(rviz2_node)

    return ld