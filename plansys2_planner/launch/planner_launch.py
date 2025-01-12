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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    planner = LaunchConfiguration('planner')
    planning_mode = LaunchConfiguration('planning_mode')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    declare_planning_mode_cmd = DeclareLaunchArgument(
        'planning_mode',
        default_value='offline',
        description='planning_mode')

    declare_planner_cmd = DeclareLaunchArgument(
        'planner',
        default_value='POPF',
        description='planner')

    # Specify the actions
    planner_cmd = Node(
        package='plansys2_planner',
        executable='planner_node',
        name='planner',
        namespace=namespace,
        output='screen',
        condition=IfCondition(PythonExpression(["'", planning_mode, "' == 'offline'"])),
        parameters=[params_file, {"planner": planner}])

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_planner_cmd)
    ld.add_action(declare_planning_mode_cmd)

    # Declare the launch options
    ld.add_action(planner_cmd)

    return ld
