# Copyright 2026
# Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _remap(prefix, name):
    """
    Build a topic remapping pair (name -> prefix/name).

    When prefix is empty, maps to the original name (equivalent to no
    remapping). When prefix is non-empty (e.g. 'sim'), all topics and
    services are prefixed consistently so the nodes stay connected.
    """
    return (name, [prefix, '/', name])


def generate_launch_description():
    # ---- 命令行参数（可用 ros2 launch 覆盖）----
    topic_prefix = LaunchConfiguration('topic_prefix')
    road_profile = LaunchConfiguration('road_profile')
    amplitude = LaunchConfiguration('amplitude')
    frequency = LaunchConfiguration('frequency')
    rate = LaunchConfiguration('rate')
    state_source = LaunchConfiguration('state_source')

    declare_args = [
        DeclareLaunchArgument(
            'topic_prefix', default_value='',
            description='所有话题/服务的前缀（重映射），空则使用默认话题名'),
        DeclareLaunchArgument(
            'road_profile', default_value='sine',
            description='路面类型：sine / square / random / speed_bump'),
        DeclareLaunchArgument(
            'amplitude', default_value='0.05',
            description='路面振幅 (m)'),
        DeclareLaunchArgument(
            'frequency', default_value='0.5',
            description='路面频率 (Hz)'),
        DeclareLaunchArgument(
            'rate', default_value='100.0',
            description='模型 / 控制 / 估计频率 (Hz)'),
        DeclareLaunchArgument(
            'state_source', default_value='service',
            description='controller 状态来源：service（调用估计服务）/ '
                        'topic（订阅真值）'),
    ]

    # ---- 节点：同时启动路面、模型、估计、控制四节点 ----
    road_input = Node(
        package='suspension_sim',
        executable='road_input_node',
        name='road_input_node',
        output='screen',
        parameters=[{
            'road_profile': road_profile,
            'amplitude': amplitude,
            'frequency': frequency,
        }],
        remappings=[
            _remap(topic_prefix, 'road_height'),
        ],
    )

    model = Node(
        package='suspension_sim',
        executable='model_node',
        name='model_node',
        output='screen',
        parameters=[{
            'rate': rate,
        }],
        remappings=[
            _remap(topic_prefix, 'road_height'),
            _remap(topic_prefix, 'control_force'),
            _remap(topic_prefix, 'suspension_state'),
        ],
    )

    estimator = Node(
        package='suspension_sim',
        executable='estimator_node',
        name='estimator_node',
        output='screen',
        parameters=[{
            'rate': rate,
        }],
        remappings=[
            _remap(topic_prefix, 'suspension_state'),
            _remap(topic_prefix, 'estimated_state'),
            _remap(topic_prefix, 'estimate_state'),
        ],
    )

    controller = Node(
        package='suspension_sim',
        executable='controller_node',
        name='controller_node',
        output='screen',
        parameters=[{
            'rate': rate,
            'state_source': state_source,
        }],
        remappings=[
            _remap(topic_prefix, 'suspension_state'),
            _remap(topic_prefix, 'control_force'),
            _remap(topic_prefix, 'estimate_state'),
        ],
    )

    return LaunchDescription(declare_args + [road_input, model, estimator, controller])
