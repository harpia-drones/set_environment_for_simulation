import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

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
        'config',
        'rviz',
        'urdf_config.rviz'
    )

    gazebo_config_path = os.path.join(
        get_package_share_path('my_robot_description'),
        'config'
        'gazebo',
        'gazebo_bridge.yaml'
    )

    gazebo_launch_file_path =  os.path.join(
        get_package_share_path('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    )
    
##################################################
#           Mount the parameter values           
##################################################

    robot_description_param = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    gazebo_config_path_param = ParameterValue(gazebo_config_path, value_type=str)

##################################################
#                 Mount the nodes                
##################################################

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {'robot_description': robot_description_param}
        ]
    )

    ros_gz_sim_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            '-topic',
            'robot_description'
        ]
    )

    ros_gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {'config_file': gazebo_config_path_param}
        ]
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            '-d',
            rviz_config_path
        ]
    )

##################################################
#           Include other launch files            
##################################################

    gazebo_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            gazebo_launch_file_path
        ),
        launch_arguments={
            'gz_args': 'empty.sdf -r'
        }.items()
    )

##################################################
#   Add the action to LaunchDescription object 
##################################################

    ld.add_action(robot_state_publisher_node)
    ld.add_action(gazebo_launch_file)
    ld.add_action(ros_gz_sim_node)
    ld.add_action(ros_gz_bridge_node)
    ld.add_action(rviz2_node)

    return ld