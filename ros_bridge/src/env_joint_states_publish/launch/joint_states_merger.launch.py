from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Unified launch file for:
    - Modbus Hardware Bridge
    - Joint States Merger
    - ROS 1 Parameter Bridge (with topic remapping)
    """
    simulation = LaunchConfiguration('simulation')

    return LaunchDescription([
        DeclareLaunchArgument(
            'simulation',
            default_value='false',
            description='If true, run gantry_demo. If false, run modbus_hw_bridge.',
        ),
        
        # 1. Gantry Controller / Modbus HW Bridge
        Node(
            package='gantry_controller',
            executable='modbus_hw_bridge',
            name='modbus_hardware_bridge',
            # prefix=['gdb -ex run --args'],
            output='screen',
            condition=UnlessCondition(simulation),
            parameters=[{
                'plc_ip':           '192.168.1.1',
                'plc_port':         502,
                'plc_unit_id':      1,
                'scale_factor':     10000.0,
                'default_velocity': 100,
                'feedback_rate_hz': 20.0,
            }],
        ),

        Node (
            package='gantry_controller',
            executable='gantry_demo',
            output='screen',
            condition=IfCondition(simulation),
        ),
        Node(
            package="env_joint_states_publish",
            executable="joint_states_merger",
            name="joint_states_merger",
            output="screen",
        )
    ])