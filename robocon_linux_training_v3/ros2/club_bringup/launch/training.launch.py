"""Source-reviewed wrapper. Requires Linux/ROS2 runtime validation.

Does not turn upstream wall timers into simulation-time timers and does not
infer sensor calibration. Never starts PGO and localizer simultaneously.
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    mode = value('mode')
    if mode not in ('lio', 'mapping', 'localization'):
        raise RuntimeError('mode must be lio, mapping or localization')
    lio_config = Path(value('lio_config')).expanduser().resolve()
    if not lio_config.is_file():
        raise RuntimeError('lio_config must be a calibrated ordinary yaml-cpp config file')
    sim_time = value('use_sim_time').lower() == 'true'
    nodes = [Node(package='fastlio2', executable='lio_node', namespace='fastlio2',
                  name='lio_node', output='screen', parameters=[
                      {'config_path': str(lio_config), 'use_sim_time': sim_time}])]
    if mode != 'lio':
        module_config = Path(value('module_config')).expanduser().resolve()
        if not module_config.is_file():
            raise RuntimeError('module_config must be the pgo/localizer ordinary YAML file')
        module = 'pgo' if mode == 'mapping' else 'localizer'
        nodes.append(Node(package=module, executable=module + '_node', namespace=module,
                          name=module + '_node', output='screen', parameters=[
                              {'config_path': str(module_config), 'use_sim_time': sim_time}]))
    if value('rviz').lower() == 'true':
        package = {'lio': 'fastlio2', 'mapping': 'pgo', 'localization': 'localizer'}[mode]
        config = Path(get_package_share_directory(package)) / 'rviz' / (package + '.rviz')
        nodes.append(Node(package='rviz2', executable='rviz2', name='club_rviz', output='screen',
                          arguments=['-d', str(config)], parameters=[{'use_sim_time': sim_time}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping'),
        DeclareLaunchArgument('lio_config', description='Absolute path to calibrated lio.yaml'),
        DeclareLaunchArgument('module_config', default_value='', description='Absolute path to pgo.yaml/localizer.yaml'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=create),
    ])
