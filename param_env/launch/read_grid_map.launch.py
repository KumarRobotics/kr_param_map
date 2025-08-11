from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    map_frame_id = DeclareLaunchArgument('map_frame_id', default_value='map')
    map_size_x = DeclareLaunchArgument('map_size_x', default_value='20.0')
    map_size_y = DeclareLaunchArgument('map_size_y', default_value='10.0')
    map_size_z = DeclareLaunchArgument('map_size_z', default_value='5.0')
    map_x_origin = DeclareLaunchArgument('map_x_origin', default_value='-5.0')
    map_y_origin = DeclareLaunchArgument('map_y_origin', default_value='-10.0')
    map_z_origin = DeclareLaunchArgument('map_z_origin', default_value='0.0')
    cloud = DeclareLaunchArgument('cloud', default_value='/read_grid_map/global_gridmap')
    clear_path = DeclareLaunchArgument(
        'clear_path',
        default_value=PathJoinSubstitution([FindPackageShare('param_env'), 'utils', 'clear_pos', 'clear_3d.csv'])
    )
    rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([FindPackageShare('param_env'), 'launch', 'default.rviz'])
    )

    # Node for read_grid_map
    read_grid_map_node = Node(
        package='param_env',
        executable='read_grid_map',
        name='read_grid_map',
        output='screen',
        # Fix remap: remove ~ and remap 'global_gridmap' to cloud topic
        remappings=[('global_gridmap', LaunchConfiguration('cloud'))],
        parameters=[{
            'map/x_size': LaunchConfiguration('map_size_x'),
            'map/y_size': LaunchConfiguration('map_size_y'),
            'map/z_size': LaunchConfiguration('map_size_z'),
            'map/x_origin': LaunchConfiguration('map_x_origin'),
            'map/y_origin': LaunchConfiguration('map_y_origin'),
            'map/z_origin': LaunchConfiguration('map_z_origin'),
            'map/resolution': 0.1,
            'map/frame_id': LaunchConfiguration('map_frame_id'),
            'map/inflate_radius': 0.2,
            'clear_path': LaunchConfiguration('clear_path'),
            'map/auto_change': False,
            'map/publish_grid_centers': True,
            'map/mode': 1,
            'folder_path': PathJoinSubstitution([FindPackageShare('param_env'), 'utils', 'data', 'img', 'maze']),
            'use_folder': True,
            'file_path': PathJoinSubstitution([FindPackageShare('param_env'), 'utils', 'data', 'img', 'maze', 'maze8.png']),
            'img/negate': 0,
            'img/occ_th': 0.6,
        }]
    )


    # Node for rviz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )


    return LaunchDescription([
        map_frame_id,
        map_size_x,
        map_size_y,
        map_size_z,
        map_x_origin,
        map_y_origin,
        map_z_origin,
        cloud,
        rviz_config,
        clear_path,
        read_grid_map_node,
        rviz_node,
    ])
