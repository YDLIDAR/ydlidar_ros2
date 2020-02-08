#!/usr/bin/python3
# Copyright 2020, EAIBOT
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import matches_node_name
from launch_ros.event_handlers import OnStateTransition
from launch.actions import LogInfo
from launch.events import matches_action
from launch.event_handlers.on_shutdown import OnShutdown

import lifecycle_msgs.msg
import os


def generate_launch_description():
    share_dir = get_package_share_directory('ydlidar')
    parameter_file = LaunchConfiguration('params_file')
    node_name = 'ydlidar_node'

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'ydlidar.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')

    driver_node = LifecycleNode(package='ydlidar',
                                node_executable='ydlidar_node',
                                node_name=node_name,
                                output='screen',
                                emulate_tty=True,
                                parameters=[parameter_file],
                                node_namespace='/',
                                )
    tf_node = LifecycleNode(package='tf2_ros',
                            node_executable='static_transform_publisher',
                            node_name='static_tf_pub_laser',
                            output='screen',
                            emulate_tty=True,
                            parameters=['0', '0', '0.02','0', '0', '0', '1','base_link','laser_frame'],
                            node_namespace='/',
                            )

    return LaunchDescription([
        params_declare,
        driver_node,
        tf_node,
    ])
