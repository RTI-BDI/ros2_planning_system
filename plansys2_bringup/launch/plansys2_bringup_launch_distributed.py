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

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('plansys2_bringup')

    # Create the launch configuration variables
    model_file = LaunchConfiguration('model_file')
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    planner = LaunchConfiguration('planner')
    planning_mode = LaunchConfiguration('planning_mode')
    default_action_bt_xml_filename = LaunchConfiguration('default_action_bt_xml_filename')

    declare_model_file_cmd = DeclareLaunchArgument(
        'model_file',
        description='PDDL Model file')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')
    
    declare_planner_cmd = DeclareLaunchArgument(
        'planner',
        default_value='POPF',
        description='planner')
    
    declare_planning_mode_cmd = DeclareLaunchArgument(
        'planning_mode',
        default_value='offline',
        description='planning_mode')

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

    domain_expert_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_domain_expert'),
            'launch',
            'domain_expert_launch.py')),
        launch_arguments={
          'model_file': model_file,
          'namespace': namespace,
          'params_file': params_file
        }.items())

    problem_expert_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_problem_expert'),
            'launch',
            'problem_expert_launch.py')),
        launch_arguments={
          'model_file': model_file,
          'namespace': namespace,
          'params_file': params_file
        }.items())

    planner_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_planner'),
            'launch',
            'planner_launch.py')),
        launch_arguments={
          'namespace': namespace,
          'params_file': params_file,
          'planner': planner,
          'planning_mode': planning_mode
        }.items())

    executor_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_executor'),
            'launch',
            'executor_launch.py')),
        launch_arguments={
          'namespace': namespace,
          'params_file': params_file,
          'default_action_bt_xml_filename': default_action_bt_xml_filename
        }.items())

    lifecycle_manager_cmd = Node(
        package='plansys2_lifecycle_manager',
        executable='lifecycle_manager_node',
        name='lifecycle_manager_node',
        namespace=namespace,
        output='screen',
        parameters=[{'planning_mode': planning_mode}])

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_model_file_cmd)
    ld.add_action(declare_default_bt_file_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_planner_cmd)
    ld.add_action(declare_planning_mode_cmd)

    # Declare the launch options
    ld.add_action(domain_expert_cmd)
    ld.add_action(problem_expert_cmd)
    ld.add_action(planner_cmd)
    ld.add_action(executor_cmd)
    ld.add_action(lifecycle_manager_cmd)

    return ld
