from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the 'port' argument
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',  # Optional default value
        description='Serial port for IMU device'
    )

    # Use LaunchConfiguration to capture the 'port' argument
    gps_node = Node(
        package='imu_driver',
        executable='IMU_ROS2driver_g',
        name='imu_driver_node',
        #output='screen',
        parameters=[],
        arguments=[LaunchConfiguration('port')]  # Pass the 'port' argument
    )

    return LaunchDescription([
        port_arg,  # Add the port argument to the launch description
        gps_node,  # Add the node to the launch description
    ])

