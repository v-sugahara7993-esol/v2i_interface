#!/bin/bash

# Copyright 2023 Tier IV, Inc.
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

# USER SETTING PARAMETER ############################
# export CMD_ID="uint8"
# export CMD_STATE="uint8 (0~255)"

source `ros2 pkg prefix v2i_tester`/lib/v2i_tester/generate_json_body.bash

# main process ########################################

# create infrastructure command json body
declare -A settings_list=()
settings_list["id"]="CMD_ID"
settings_list["state"]="CMD_STATE"
command=`getJsonFromSettingsList ${settings_list}`
export INFRA_COMMAND_MESSAGE="${INFRA_COMMAND_MESSAGE} ${command}"
unset settings_list
