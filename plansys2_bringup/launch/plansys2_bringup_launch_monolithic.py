# Copyright 2019 Intelligent Robotics Lab
#
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('plansys2_bringup')

    # Create the launch configuration variables
    model_file = LaunchConfiguration('model_file')
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    default_action_bt_xml_filename = LaunchConfiguration('default_action_bt_xml_filename')
    planner = LaunchConfiguration('planner')
    planning_mode = LaunchConfiguration('planning_mode')

    declare_model_file_cmd = DeclareLaunchArgument(
        'model_file',
        description='PDDL Model file')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'plansys2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_default_bt_file_cmd = DeclareLaunchArgument(
        'default_action_bt_xml_filename',
        default_value=os.path.join(
          get_package_share_directory('plansys2_executor'),
          'behavior_trees', 'plansys2_action_bt.xml'),
        description='BT representing a PDDL action')
    
    declare_planner_cmd = DeclareLaunchArgument(
        'planner',
        default_value='POPF',
        description='planner')

    declare_planning_mode_cmd = DeclareLaunchArgument(
        'planning_mode',
        default_value='offline',
        description='planning_mode')


    plansys2_node_cmd = Node(
        package='plansys2_bringup',
        executable='plansys2_node',
        output='screen',
        namespace=namespace,
        parameters=[
          {
            'model_file': model_file,
            'default_action_bt_xml_filename': default_action_bt_xml_filename,
            'planner': planner,
            'planning_mode': planning_mode
          },
          params_file
        ])

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_model_file_cmd)
    ld.add_action(declare_default_bt_file_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_planner_cmd)
    ld.add_action(declare_planning_mode_cmd)

    # Declare the launch options
    ld.add_action(plansys2_node_cmd)

    return ld
