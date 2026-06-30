#!/usr/bin/env python3

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Source workspace so custom packages are visible even from an unsourced shell.
_SETUP = '/home/sam/ros2_ws/install/setup.bash'
if os.path.exists(_SETUP):
    _env = subprocess.run(
        ['bash', '-c', f'source {_SETUP} && env'],
        capture_output=True, text=True
    )
    for _line in _env.stdout.splitlines():
        if '=' in _line:
            _k, _, _v = _line.partition('=')
            os.environ.setdefault(_k, _v)

_ROSBAG_TOPICS = [
    '/scan',
    '/full_cloud',
    '/scan_complete',
    '/tf',
    '/tf_static',
    '/servo_serial',
]


def generate_launch_description():

    pkg = get_package_share_directory('servo_tf_demo')

    # ---------------------------------------------------------------- args

    center_angle_arg = DeclareLaunchArgument(
        'center_angle',
        default_value='127',
        description='PWM value at which the servo points straight ahead (tune per your hardware)'
    )
    scan_offset_arg = DeclareLaunchArgument(
        'scan_offset',
        default_value='28',
        description='Degrees each side of centre the servo sweeps (e.g. 28 → ±28°)'
    )
    sweep_speed_arg = DeclareLaunchArgument(
        'sweep_speed',
        default_value='10.0',
        description='Servo sweep speed in degrees per second'
    )
    step_size_arg = DeclareLaunchArgument(
        'step_size',
        default_value='1.0',
        description='Degrees moved per timer tick — use < 1.0 for finer angular sampling'
    )
    base_x_arg = DeclareLaunchArgument(
        'base_x',
        default_value='0.0',
        description='X position of base_link relative to map (metres)'
    )
    base_y_arg = DeclareLaunchArgument(
        'base_y',
        default_value='0.0',
        description='Y position of base_link relative to map (metres)'
    )
    base_z_arg = DeclareLaunchArgument(
        'base_z',
        default_value='1.0',
        description='Z position (height) of base_link relative to map (metres)'
    )
    base_roll_arg = DeclareLaunchArgument(
        'base_roll',
        default_value='0.0',
        description='Roll of base_link relative to map (degrees)'
    )
    base_pitch_arg = DeclareLaunchArgument(
        'base_pitch',
        default_value='0.0',
        description='Pitch of base_link relative to map (degrees)'
    )
    base_yaw_arg = DeclareLaunchArgument(
        'base_yaw',
        default_value='0.0',
        description='Yaw of base_link relative to map (degrees)'
    )
    servo_x_arg = DeclareLaunchArgument(
        'servo_x',
        default_value='0.0',
        description='X offset of servo_link relative to base_link (metres)'
    )
    servo_y_arg = DeclareLaunchArgument(
        'servo_y',
        default_value='0.0',
        description='Y offset of servo_link relative to base_link (metres)'
    )
    servo_z_arg = DeclareLaunchArgument(
        'servo_z',
        default_value='-0.05',
        description='Z offset of servo_link relative to base_link (metres)'
    )
    servo_roll_arg = DeclareLaunchArgument(
        'servo_roll',
        default_value='180.0',
        description='Roll of servo_link relative to base_link (degrees)'
    )
    servo_pitch_arg = DeclareLaunchArgument(
        'servo_pitch',
        default_value='0.0',
        description='Pitch of servo_link relative to base_link (degrees)'
    )
    servo_yaw_arg = DeclareLaunchArgument(
        'servo_yaw',
        default_value='0.0',
        description='Yaw of servo_link relative to base_link (degrees)'
    )
    output_mode_arg = DeclareLaunchArgument(
        'output_mode',
        default_value='rviz',
        description='Output mode: "rviz" to visualise live in RViz2, "rosbag" to record topics to disk'
    )
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value=os.path.expanduser('~/ros2_bags/scan'),
        description='Output path for the rosbag (only used when output_mode:=rosbag)'
    )

    center_angle = LaunchConfiguration('center_angle')
    scan_offset  = LaunchConfiguration('scan_offset')
    sweep_speed  = LaunchConfiguration('sweep_speed')
    step_size    = LaunchConfiguration('step_size')
    base_x       = LaunchConfiguration('base_x')
    base_y       = LaunchConfiguration('base_y')
    base_z       = LaunchConfiguration('base_z')
    base_roll    = LaunchConfiguration('base_roll')
    base_pitch   = LaunchConfiguration('base_pitch')
    base_yaw     = LaunchConfiguration('base_yaw')
    servo_x      = LaunchConfiguration('servo_x')
    servo_y      = LaunchConfiguration('servo_y')
    servo_z      = LaunchConfiguration('servo_z')
    servo_roll   = LaunchConfiguration('servo_roll')
    servo_pitch  = LaunchConfiguration('servo_pitch')
    servo_yaw    = LaunchConfiguration('servo_yaw')

    # ---------------------------------------------------------------- nodes

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch', 'rplidar_s3_launch.py'
            )
        )
    )

    serial_bridge = Node(
        package='servo_serial_reader',
        executable='serial_publisher',
        name='serial_publisher',
        output='screen',
    )

    servo_mgr = Node(
        package='servo_tf_demo',
        executable='servo_manager',
        name='servo_manager',
        output='screen',
        parameters=[{
            'center_angle': center_angle,
            'scan_offset':  scan_offset,
            'sweep_speed':  sweep_speed,
            'step_size':    step_size,
            'base_x':       base_x,
            'base_y':       base_y,
            'base_z':       base_z,
            'base_roll':    base_roll,
            'base_pitch':   base_pitch,
            'base_yaw':     base_yaw,
            'servo_x':      servo_x,
            'servo_y':      servo_y,
            'servo_z':      servo_z,
            'servo_roll':   servo_roll,
            'servo_pitch':  servo_pitch,
            'servo_yaw':    servo_yaw,
        }],
    )

    cloud_bld = Node(
        package='pointcloud_builder',
        executable='cloud_builder',
        name='cloud_builder',
        output='screen',
    )

    save_cloud = Node(
        package='pointcloud_builder',
        executable='save_cloud',
        name='save_cloud',
        output='screen',
    )

    # --------------------------------------------------------- crash helpers

    def _crash(name, hint=''):
        suffix = f'  Hint: {hint}' if hint else ''
        return [LogInfo(msg=f'\n[CRASH] {name} has exited unexpectedly!{suffix}\n')]

    # --------------------------------------------------------- common crash handlers (always registered)

    common_handlers = [
        RegisterEventHandler(OnProcessExit(
            target_action=serial_bridge,
            on_exit=_crash(
                'serial_publisher',
                'Check USB cable and that /dev/serial/by-id/... exists.'
            )
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=servo_mgr,
            on_exit=_crash('servo_manager')
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=cloud_bld,
            on_exit=_crash('cloud_builder')
        )),
    ]

    # ------------------------------------------------- output-mode branching (resolved at launch time)

    def output_mode_actions(context, *args, **kwargs):
        mode     = LaunchConfiguration('output_mode').perform(context)
        bag_path = LaunchConfiguration('bag_path').perform(context)

        if mode == 'rosbag':
            rosbag = ExecuteProcess(
                cmd=['ros2', 'bag', 'record', '-o', bag_path, *_ROSBAG_TOPICS],
                output='screen',
            )
            return [
                TimerAction(period=4.0, actions=[
                    LogInfo(msg=f'[system_launch] t=4  Starting cloud_builder, save_cloud and rosbag recorder → {bag_path}'),
                    cloud_bld,
                    save_cloud,
                    rosbag,
                ]),
            ]

        # default: rviz
        rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg, 'rviz', 'system.rviz')],
            output='screen',
        )
        return [
            TimerAction(period=4.0, actions=[
                LogInfo(msg='[system_launch] t=4  Starting cloud_builder, RViz2 and save_cloud...'),
                cloud_bld,
                rviz2,
                save_cloud,
            ]),
            RegisterEventHandler(OnProcessExit(
                target_action=rviz2,
                on_exit=_crash('rviz2', 'RViz2 closed or crashed.')
            )),
        ]

    # ------------------------------------------------------- sequenced startup

    return LaunchDescription([
        center_angle_arg,
        scan_offset_arg,
        sweep_speed_arg,
        step_size_arg,
        base_x_arg,
        base_y_arg,
        base_z_arg,
        base_roll_arg,
        base_pitch_arg,
        base_yaw_arg,
        servo_x_arg,
        servo_y_arg,
        servo_z_arg,
        servo_roll_arg,
        servo_pitch_arg,
        servo_yaw_arg,
        output_mode_arg,
        bag_path_arg,

        # t = 0 s — lidar + serial bridge
        LogInfo(msg='[system_launch] t=0  Starting lidar and serial bridge...'),
        lidar,
        serial_bridge,

        # t = 2 s — servo_manager (needs serial port open)
        TimerAction(period=2.0, actions=[
            LogInfo(msg='[system_launch] t=2  Starting servo_manager...'),
            servo_mgr,
        ]),

        # t = 4 s — output-mode-dependent nodes (rviz or rosbag)
        OpaqueFunction(function=output_mode_actions),

        *common_handlers,
    ])
