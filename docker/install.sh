#!/bin/bash

#  Copyright (C) 2018-2025 LEIDOS.
#
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
#
#  http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

if [[ ! -z "$ROS2_PACKAGES" ]]; then
    echo "Sourcing previous build for incremental build start point..."
    source /opt/carma/install/setup.bash
else
    echo "Sourcing base image for full build..."
    source /opt/ros/humble/setup.bash
fi

cd ~/
if [[ ! -z "$ROS2_PACKAGES" ]]; then
    colcon build \
    --packages-above $ROS2_PACKAGES \
    --parallel-workers $(nproc) \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
else
    if [[ ! -z "$PACKAGES_UP_TO" ]]; then 
        colcon build \
        --packages-up-to "$PACKAGES_UP_TO" \
        --parallel-workers $(nproc) \
        --cmake-args -DCMAKE_BUILD_TYPE=Release
    else
        colcon build \
        --packages-up-to v2x_ros_driver driver_shutdown_ros2 \
        --parallel-workers $(nproc) \
        --cmake-args -DCMAKE_BUILD_TYPE=Release
    fi
fi
