# Copyright (C) 2022-2025 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, Shutdown, ExecuteProcess, GroupAction, TimerAction, RegisterEventHandler, LogInfo
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.event_handlers import OnExecutionComplete
from launch.conditions import IfCondition
from carma_ros2_utils.launch.get_current_namespace import GetCurrentNamespace

import os


'''
This file is can be used to launch the CARMA v2x_ros_driver_node.
  Though in carma-platform it may be launched directly from the base launch file.
'''

def generate_launch_description():

    # Declare the log_level launch argument
    log_level = LaunchConfiguration('log_level')
    declare_log_level_arg = DeclareLaunchArgument(
        name ='log_level', default_value='WARN',
        description="The log level to use for the v2x_ros_driver_node")

    configuration_delay = LaunchConfiguration('configuration_delay')
    declare_configuration_delay_arg = DeclareLaunchArgument(
        name ='configuration_delay', default_value='2.0',
        description="The delay in seconds before the v2x_ros_driver_node is configured")

    enable_v2x_driver_lifecycle = LaunchConfiguration('enable_v2x_driver_lifecycle')
    declare_enable_v2x_driver_lifecycle = DeclareLaunchArgument(
        name ='enable_v2x_driver_lifecycle', default_value='False',
        description="Enable the v2x_ros_driver_node lifecycle. If enabled manually transitions the node to active state. Default is false")

    # Get parameter file path
    param_file_path = os.path.join(
        get_package_share_directory('v2x_ros_driver'), 'config/params.yaml')

    # Declare the global_params_override_file launch argument
    # Parameters in this file will override any parameters loaded in their respective packages
    global_params_override_file = LaunchConfiguration('global_params_override_file')
    declare_global_params_override_file_arg = DeclareLaunchArgument(
        name = 'global_params_override_file',
        default_value = ["/opt/carma/vehicle/config/GlobalParamsOverride.yaml"],
        description = "Path to global file containing the parameters overwrite"
    )

    # Launch node(s) in a carma container to allow logging to be configured
    container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='v2x_ros_driver_container',
        namespace=GetCurrentNamespace(),
        executable='carma_component_container_mt',
        composable_node_descriptions=[

            # Launch the core node(s)
            ComposableNode(
                    package='v2x_ros_driver',
                    plugin='V2XDriverApplication::Node',
                    name='v2x_ros_driver_node',
                    extra_arguments=[
                        {'use_intra_process_comms': True},
                    ],
                    remappings=[
                        ("inbound_binary_msg", "comms/inbound_binary_msg"),
                        ("outbound_binary_msg", "comms/outbound_binary_msg"),
                    ],
                    parameters=[
                        param_file_path,
                        global_params_override_file
                    ]
            ),
        ],
        arguments=["--ros-args", "--log-level", log_level],
        on_exit= Shutdown()
    )
    ros2_cmd = FindExecutable(name="ros2")
    process_configure_v2x_ros_driver_node = ExecuteProcess(
        cmd=[
            ros2_cmd, "lifecycle", "set", "/v2x_ros_driver_node", "configure",
        ],

    )

    activate_node_group_action = GroupAction(
        # Node lifecycle activation called only if enable_v2x_driver_lifecycle is set to true
        condition=IfCondition(LaunchConfiguration('enable_v2x_driver_lifecycle')),
        actions=[
            # Set node lifecycle to configure after a delay
            TimerAction(
                period=LaunchConfiguration('configuration_delay'),
                actions=[process_configure_v2x_ros_driver_node],
            ),

            # Activate node after configuration
            RegisterEventHandler(
                OnExecutionComplete(
                    target_action=process_configure_v2x_ros_driver_node,
                    on_completion=[
                        ExecuteProcess(
                            cmd=[
                                ros2_cmd, "lifecycle", "set", "/v2x_ros_driver_node", "activate",
                            ],
                        )
                    ],
                )
            ),
        ]

    )

    return LaunchDescription([
        declare_log_level_arg,
        declare_configuration_delay_arg,
        declare_global_params_override_file_arg,
        declare_enable_v2x_driver_lifecycle,
        # Node + lifecycle
        container,
        activate_node_group_action
    ])
