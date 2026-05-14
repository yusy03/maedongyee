import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    display_only = LaunchConfiguration('display_only')
    enable_motors = LaunchConfiguration('enable_motors')
    real_drive_condition = PythonExpression([
        "'", display_only, "'.lower() == 'false' and '", enable_motors, "'.lower() == 'true'"
    ])
    display_condition = PythonExpression([
        "'", display_only, "'.lower() == 'true' or '", enable_motors, "'.lower() != 'true'"
    ])

    display_only_arg = DeclareLaunchArgument(
        'display_only',
        default_value='true',
        description='When true, show LEFT/RIGHT/STRAIGHT/STOP instead of driving the motors.'
    )

    enable_motors_arg = DeclareLaunchArgument(
        'enable_motors',
        default_value='false',
        description='Safety switch. Motors only run when this is true and display_only is false.'
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
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        condition=IfCondition(real_drive_condition)
    )

    controller_display_node = Node(
        package='aicar_controller',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        remappings=[
            ('/drive', '/drive_display'),
        ],
        condition=IfCondition(display_condition)
    )

    motor_driver_node = Node(
        package='aicar_driver',
        executable='motor_controller_node',
        name='motor_controller_node',
        output='screen',
        parameters=[{
            'enable_motors': True,
        }],
        condition=IfCondition(real_drive_condition)
    )

    drive_direction_display_node = Node(
        package='aicar_driver',
        executable='drive_direction_display_node',
        name='drive_direction_display_node',
        output='screen',
        parameters=[{
            'input_type': 'ackermann',
        }],
        condition=IfCondition(display_condition)
    )

    return LaunchDescription([
        display_only_arg,
        enable_motors_arg,
        camera_node_launch,
        lane_detector_node,
        bev_view_node,
        bev_color_view_node,
        controller_node,
        controller_display_node,
        motor_driver_node,
        drive_direction_display_node,
    ])
