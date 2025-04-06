import os
from ament_index_python import get_package_share_directory
import xacro
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    
    pkg_name = "spacecraft"
    pkg_path = get_package_share_directory(pkg_name)

    world_path = os.path.join(pkg_path, 'worlds', 'space.world')
    model_path = os.path.join(pkg_path, 'description')  # points to folder containing 'spacecraft/model.sdf'

    plugin_pkg_name = "spacecraft_plugins"
    plugin_path = os.path.join(get_package_share_directory(plugin_pkg_name), "lib")




    return LaunchDescription([

        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', plugin_path),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_path}.items()
        ),

        Node(
            package='py_thruster_allocation',
            executable='thruster_mapper_node',
            name='thruster_mapper',
            output='screen'
        ),

    ])