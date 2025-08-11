from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    map_frame_id = DeclareLaunchArgument('map_frame_id', default_value='map')
    map_size_x = DeclareLaunchArgument('map_size_x', default_value='30.0')
    map_size_y = DeclareLaunchArgument('map_size_y', default_value='30.0')
    map_size_z = DeclareLaunchArgument('map_size_z', default_value='5.0')
    map_x_origin = DeclareLaunchArgument('map_x_origin', default_value='-15.0')
    map_y_origin = DeclareLaunchArgument('map_y_origin', default_value='-15.0')
    map_z_origin = DeclareLaunchArgument('map_z_origin', default_value='0.0')
    cloud = DeclareLaunchArgument('cloud', default_value='/structure_map/global_gridmap')
    clear_path = DeclareLaunchArgument(
        'clear_path',
        default_value=PathJoinSubstitution([FindPackageShare('param_env'), 'utils', 'clear_pos', 'clear_3d.csv'])
    )
    simple_2d = DeclareLaunchArgument('simple_2d', default_value='false')
    auto_change = DeclareLaunchArgument('auto_change', default_value='false')
    rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([FindPackageShare('param_env'), 'launch', 'default.rviz'])
    )

    # Node for structure_map
    structure_map_node = Node(
        package='param_env',
        executable='structure_map',
        name='structure_map',
        output='screen',
        remappings=[('global_cloud', LaunchConfiguration('cloud'))],
        parameters=[{
            'map/x_size': LaunchConfiguration('map_size_x'),
            'map/y_size': LaunchConfiguration('map_size_y'),
            'map/z_size': LaunchConfiguration('map_size_z'),
            'map/x_origin': LaunchConfiguration('map_x_origin'),
            'map/y_origin': LaunchConfiguration('map_y_origin'),
            'map/z_origin': LaunchConfiguration('map_z_origin'),
            'map/resolution': 0.1,
            'map/frame_id': LaunchConfiguration('map_frame_id'),
            'map/inflate_radius': 0.1,
            'clear_path': LaunchConfiguration('clear_path'),
            'clear_pos': True,
            'map/auto_change': LaunchConfiguration('auto_change'),
            'map/simple_2d': LaunchConfiguration('simple_2d'),
            'params/cylinder_ratio': 0.1,
            'params/circle_ratio': 0.0,
            'params/gate_ratio': 0.0,
            'params/ellip_ratio': 0.0,
            'params/poly_ratio': 0.01,
            'params/w1': 0.1,
            'params/w2': 0.5,
            'params/w3': 2.0,
            'params/add_noise': False,
            'params/seed': 1.0,
            'dataset/save_map': True,
            'dataset/samples_num': 10000,
            'dataset/start_index': 100000,
            'dataset/path': PathJoinSubstitution([FindPackageShare('param_env'), 'utils',  'dataset']),
            'dyn/v_x_h': 1.0,
            'dyn/v_y_h': 1.0,
            'dyn/v_z_h': 0.0,
            'dyn/dt': 10.0,
            'dyn/dyn_mode': False,
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
        clear_path,
        simple_2d,
        auto_change,
        rviz_config,
        structure_map_node,
        rviz_node,
    ])
