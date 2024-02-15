import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'hexabot'
    file_subpath = 'urdf/ros_robot.urdf.xacro'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()


    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
            # launch_arguments={
            #     'args' : 'basic_world.world' #Add path to World File if so desired
            # }.items(),
    )

    # Specify initial position
    position = [0.0, 0.0, 1]

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-entity', 'my_bot',
                               '-x', str(position[0]),
                               '-y', str(position[1]),
                               '-z', str(position[2]),
                               '-topic', 'robot_description',
                                ],
                    output='screen')


    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["joint_trajectory_controller"]
    )
    
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["joint_broad"]
    )

    # Run the node
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        joint_trajectory_controller_spawner,
        joint_broad_spawner
    ])


