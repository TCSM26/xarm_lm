#!/usr/bin/env python3
"""
Gaze stabilization launch for xArm6.

Brings up the standard xarm6_moveit_fake simulation (move_group, RViz with
MoveIt motion planning panel, controllers) and adds:
  - MoveIt Servo node  (LM / damped-least-squares velocity IK)
  - initial_pose       (moves arm to a non-singular ready pose at startup)
  - gaze_stabilizer    (closed-loop EE pose controller)
  - base_disturbance   (lemniscate XY + sinusoidal Z TF disturbance)

Workflow:
  1. RViz opens — arm moves to ready pose automatically.
  2. Use the MoveIt panel to Plan & Execute to your desired EE pose.
  3. Lock and start:
       ros2 topic pub --once /gaze_stabilizer/start std_msgs/msg/Bool "{data: true}"
  4. The base begins its infinity + Z motion; the arm compensates in real-time.
  5. Stop:
       ros2 topic pub --once /gaze_stabilizer/start std_msgs/msg/Bool "{data: false}"

CLI overrides (all optional):
  amplitude:=0.08   z_amplitude:=0.04
  period:=8.0       z_period:=4.0
  kp_linear:=16.0   kp_angular:=12.0
"""

import os
import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import load_yaml, generate_ros2_control_params_temp_file


def launch_setup(context, *args, **kwargs):
    amplitude   = LaunchConfiguration('amplitude').perform(context)
    z_amplitude = LaunchConfiguration('z_amplitude').perform(context)
    period      = LaunchConfiguration('period').perform(context)
    z_period    = LaunchConfiguration('z_period').perform(context)
    kp_linear   = LaunchConfiguration('kp_linear').perform(context)
    kp_angular  = LaunchConfiguration('kp_angular').perform(context)

    # ── Standard xArm6 fake simulation ─────────────────────────────────────
    moveit_fake_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('xarm_moveit_config'),
            'launch', 'xarm6_moveit_fake.launch.py',
        ])),
    )

    # ── Robot description params for the servo node ─────────────────────────
    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(
            get_package_share_directory('xarm_controller'),
            'config', 'xarm6_controllers.yaml',
        ),
        prefix='', add_gripper=False, add_bio_gripper=False,
        ros_namespace='', robot_type='xarm',
    )
    moveit_config = MoveItConfigsBuilder(
        context=context,
        controllers_name='fake_controllers',
        dof='6', robot_type='xarm',
        ros2_control_plugin='uf_robot_hardware/UFRobotFakeSystemHardware',
        ros2_control_params=ros2_control_params,
    ).to_moveit_configs()

    robot_params = {}
    robot_params.update(moveit_config.robot_description)
    robot_params.update(moveit_config.robot_description_semantic)
    robot_params.update(moveit_config.robot_description_kinematics)
    robot_params.update(moveit_config.joint_limits)

    # ── MoveIt Servo config ─────────────────────────────────────────────────
    servo_yaml = load_yaml('xarm_moveit_servo', 'config/xarm_moveit_servo_config.yaml')
    servo_yaml['move_group_name']               = 'xarm6'
    servo_yaml['command_out_topic']             = '/xarm6_traj_controller/joint_trajectory'
    servo_yaml['publish_period']                = 0.02    # 50 Hz
    servo_yaml['lower_singularity_threshold']   = 50.0    # was 17 — later onset of LM damping
    servo_yaml['hard_stop_singularity_threshold'] = 200.0 # was 30 — no emergency stops
    servo_yaml['joint_limit_margin']            = 0.05    # was 0.1 — more usable range
    servo_params = {'moveit_servo': servo_yaml}

    # ── Nodes ───────────────────────────────────────────────────────────────
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_server',
        output='screen',
        parameters=[servo_params, robot_params],
    )

    initial_pose_node = Node(
        package='xarm_lm', executable='initial_pose',
        name='initial_pose', output='screen',
    )

    gaze_node = Node(
        package='xarm_lm', executable='gaze_stabilizer',
        name='gaze_stabilizer', output='screen',
        parameters=[{
            'kp_linear':       float(kp_linear),
            'kp_angular':      float(kp_angular),
            'max_linear_vel':  0.5,
            'max_angular_vel': 2.0,
            'world_frame':     'world',
            'ee_frame':        'link_eef',
            'base_frame':      'link_base',
            'control_rate':    50.0,
        }],
    )

    disturbance_node = Node(
        package='xarm_lm', executable='base_disturbance',
        name='base_disturbance', output='screen',
        parameters=[{
            'amplitude':    float(amplitude),
            'z_amplitude':  float(z_amplitude),
            'period':       float(period),
            'z_period':     float(z_period),
            'publish_rate': 50.0,
            'world_frame':  'world',
            'base_frame':   'link_base',
        }],
    )

    return [
        moveit_fake_launch,
        servo_node,
        initial_pose_node,
        gaze_node,
        disturbance_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('amplitude',   default_value='0.08'),
        DeclareLaunchArgument('z_amplitude', default_value='0.04'),
        DeclareLaunchArgument('period',      default_value='8.0'),
        DeclareLaunchArgument('z_period',    default_value='4.0'),
        DeclareLaunchArgument('kp_linear',   default_value='16.0'),
        DeclareLaunchArgument('kp_angular',  default_value='12.0'),
        OpaqueFunction(function=launch_setup),
    ])
