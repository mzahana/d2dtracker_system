import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

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

    if (interface == 'udp'):
        xrce_agent_process = ExecuteProcess(
                cmd=[[
                    'MicroXRCEAgent udp4 -p ',
                    port
                ]],
                shell=True
        )
    elif (interface == 'serial'):
        xrce_agent_process = ExecuteProcess(
                cmd=[[
                    'MicroXRCEAgent serial --dev  ',
                    device,
                    ' -b ',
                    baud
                ]],
                shell=True
        )
    else:
        print("Interface {} is not supported".format(interface))
        exit()

    ld = LaunchDescription()

    ld.add_action(port_launch_arg)
    ld.add_action(interface_launch_arg)
    ld.add_action(device_launch_arg)
    ld.add_action(baud_launch_arg)
    ld.add_action(xrce_agent_process)

    return ld