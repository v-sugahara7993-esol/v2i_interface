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

source `ros2 pkg prefix v2i_tester`/lib/v2i_tester/generate_json_body.bash

if [[ ! -v INFRA_COMMAND_MESSAGE ]];then
  INFRA_COMMAND_MESSAGE={}
fi

# create infrastructure command array json body
command_array=`echo ${INFRA_COMMAND_MESSAGE} | jq -s`
declare -A settings_list=()
settings_list["commands"]="command_array"
INFRA_PUB_BODY=`getJsonFromSettingsList ${settings_list}`
unset settings_list

# publish messages
ros2 topic pub /v2i/infrastructure_commands v2i_interface_msgs/msg/InfrastructureCommandArray "${INFRA_PUB_BODY}"
