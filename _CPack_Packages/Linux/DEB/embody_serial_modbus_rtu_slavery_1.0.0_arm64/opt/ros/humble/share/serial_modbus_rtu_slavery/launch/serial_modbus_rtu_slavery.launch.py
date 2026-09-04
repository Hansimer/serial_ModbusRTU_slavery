import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('serial_modbus_rtu_slavery')
    default_params_file = os.path.join(
        pkg_share, 'config', 'serial_modbus_rtu_slavery_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Path to the YAML parameters file',
        ),
        Node(
            package='serial_modbus_rtu_slavery',
            executable='serial_modbus_rtu_slavery_node',
            name='serial_modbus_rtu_slavery_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
