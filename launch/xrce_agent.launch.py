import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, LaunchConfigurationEquals

def generate_launch_description():

    # interface
    interface = LaunchConfiguration('interface')
    port = LaunchConfiguration('port')
    device = LaunchConfiguration('device')
    baud = LaunchConfiguration('baud')

    interface_launch_arg = DeclareLaunchArgument(
        'interface',
        default_value='serial',
        description="'udp' or 'serial'"
    )

    port_launch_arg = DeclareLaunchArgument(
        'port',
        default_value='8888'
    )

    device_launch_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/ttyUSB0',
        description="Path to serial port"
    )

    baud_launch_arg = DeclareLaunchArgument(
        'baud',
        default_value='921600',
        description="Baudrate of the serial port"
    )

    xrce_agent_process_udp = ExecuteProcess(
            condition=LaunchConfigurationEquals('interface', 'udp'),
            cmd=[[
                'MicroXRCEAgent udp4 -p ',
                port
            ]],
            shell=True
    )
    xrce_agent_process_serial = ExecuteProcess(
        condition=LaunchConfigurationEquals('interface', 'serial'),
            cmd=[[
                'sudo MicroXRCEAgent serial --dev  ',
                device,
                ' -b ',
                baud
            ]],
            shell=True
    )

    ld = LaunchDescription()

    ld.add_action(port_launch_arg)
    ld.add_action(interface_launch_arg)
    ld.add_action(device_launch_arg)
    ld.add_action(baud_launch_arg)
    ld.add_action(xrce_agent_process_udp)
    ld.add_action(xrce_agent_process_serial)

    return ld