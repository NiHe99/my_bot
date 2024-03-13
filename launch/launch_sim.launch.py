import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():

    package_name='my_bot' 

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')
    
    
    point_cloud_to_scan = Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
                        parameters=[{'scan_time': 0.1},{'angle_min': -0.5445},{'angle_max': 0.5445},{'target_frame':'camera_link'},{'range_min':0.05},{'range_max':8.0},{'min_height':0.0},{'max_height':1.0},{'use_inf':True}],
                        remappings=[('/cloud_in','/camera/points')]
                      )

            
    r_l_params = os.path.join(get_package_share_directory('my_bot'),'config','robot_local.yaml')    
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[r_l_params, {'use_sim_time': True}],
       remappings=[('/odom','/rl/odom')]
    )




    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        point_cloud_to_scan,
        robot_localization_node,
        spawn_entity,

    ])