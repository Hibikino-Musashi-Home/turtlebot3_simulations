"""Final-task worlds with a task-specific camera; existing robot files stay unchanged."""
from pathlib import Path
import os
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def stop_server(context):
    # Stop the server too: the Gazebo Ruby wrapper may exit before its child on WSL.
    try:
        subprocess.run(['gz', 'service', '-s', '/server_control',
                        '--reqtype', 'gz.msgs.ServerControl', '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '2000', '--req', 'stop: true'],
                       env=dict(context.environment), timeout=3, capture_output=True)
    except (OSError, subprocess.TimeoutExpired):
        pass
    return []


def launch_task(context):
    task = LaunchConfiguration('task').perform(context)
    share = Path(get_package_share_directory('turtlebot3_gazebo'))
    gz_share = Path(get_package_share_directory('ros_gz_sim'))
    pitch = '0.65' if task == 'line' else '0'
    start = ('-3.0', '-2.0') if task == 'line' else ('-3.0', '-3.0')

    # Reuse Burger's drivetrain, lidar, plugins and meshes, replacing only the camera.
    sdf = ET.parse(share / 'models/turtlebot3_burger_cam/model.sdf').getroot()
    model = sdf.find('model')
    model.set('name', 'final_task_robot')
    link = model.find("link[@name='camera_link']")
    link.find('pose').text = f'0.08 0 0.19 0 {pitch} 0'
    sensor = link.find('sensor')
    sensor.set('type', 'camera')
    sensor.find('gz_frame_id').text = 'camera_rgb_optical_frame'
    sensor.find('update_rate').text = '15'
    camera = sensor.find('camera')
    lens = camera.find('lens')
    if lens is not None:
        camera.remove(lens)
    camera.find('horizontal_fov').text = '1.3962634'  # 80 degrees, pinhole
    camera.find('image/width').text = '640'
    camera.find('image/height').text = '480'
    camera.find('clip/far').text = '12'
    description = ET.parse(share / 'urdf/turtlebot3_burger_cam.urdf').getroot()
    origin = description.find("joint[@name='camera_joint']/origin")
    origin.set('xyz', '0.08 0 0.18')
    origin.set('rpy', f'0 {pitch} 0')
    description.find("joint[@name='camera_rgb_joint']/origin").set('xyz', '0 0 0')
    description.find("joint[@name='camera_rgb_optical_joint']/origin").set(
        'rpy', '-1.5707963267948966 0 -1.5707963267948966')

    return [
        SetEnvironmentVariable('GZ_PARTITION', os.environ.get(
            'GZ_PARTITION', f'turtlebot3_final_{task}')),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=stop_server)])),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(share / 'models')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gz_share / 'launch/gz_sim.launch.py')),
            launch_arguments={'gz_args': f'-r -s -v2 {share}/worlds/final_{task}.world',
                              'on_exit_shutdown': 'true'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gz_share / 'launch/gz_sim.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui')),
            launch_arguments={'gz_args': '-g -v2'}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'use_sim_time': True,
                          'robot_description': ParameterValue(
                              ET.tostring(description, encoding='unicode'), value_type=str)}]),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-world', 'final_' + task, '-name', 'final_task_robot',
                        '-string', ET.tostring(sdf, encoding='unicode'),
                        '-x', start[0], '-y', start[1], '-z', '0.01']),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': str(share / 'params/final_tasks/bridge.yaml'),
                          'use_sim_time': True}], output='screen'),
        Node(package='ros_gz_image', executable='image_bridge',
             arguments=['/camera/image_raw'],
             parameters=[{'qos': 'sensor_data', 'lazy': True, 'use_sim_time': True}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('task', default_value='exploration',
                              choices=['exploration', 'line', 'places']),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=launch_task),
    ])
