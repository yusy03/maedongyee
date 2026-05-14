import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    display_only = LaunchConfiguration('display_only')

    display_only_arg = DeclareLaunchArgument(
        'display_only',
        default_value='false',
        description='When true, show LEFT/RIGHT/STRAIGHT/STOP instead of driving the motors.'
    )
    
    camera_ros_pkg_dir = get_package_share_directory('camera_ros')
    camera_launch_file = os.path.join(
        camera_ros_pkg_dir,
        'launch',
        'camera.launch.py'
    )
    
    camera_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_file)
    )

    lane_detector_node = Node(
        package='aicar_vision',
        executable='lane_detector_node',
        name='lane_detector_node'
    )

    sign_detector_node = Node(
        package='aicar_vision',
        executable='sign_detector_node',
        name='sign_detector_node',
        output='screen'
    )

    bev_view_node = ComposableNodeContainer(
        name='bev_view_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='image_view',
                plugin='image_view::ImageViewNode',
                name='bev_image_view_node',
                remappings=[
                    ('image', '/image_bev_binary'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen'
    )

    bev_color_view_node = ComposableNodeContainer(
        name='bev_color_view_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='image_view',
                plugin='image_view::ImageViewNode',
                name='bev_color_image_view_node',
                remappings=[
                    ('image', '/image_bev_color'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen'
    )

    controller_node = Node(
        package='aicar_controller',
        executable='pid_controller_node',
        name='pid_controller_node'
    )

    motor_driver_node = Node(
        package='aicar_driver',
        executable='differential_drive_node',
        name='differential_drive_node',
        output='screen',
        parameters=[{
            'display_only': ParameterValue(display_only, value_type=bool),
        }]
    )

    return LaunchDescription([
        display_only_arg,
        camera_node_launch,
        lane_detector_node,
        sign_detector_node,
        bev_view_node,
        bev_color_view_node,
        controller_node,
        motor_driver_node,
    ])
