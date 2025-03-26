#############################
# Map exploration Mode
# Load the map specified with "FREE_MAP_NAME" and attach the ego vehicle agent to the only vehile in that map 
# No scenario required, the ego vehicle should explore the scene 
#############################

export SIMULATOR_LOCAL_HOST="localhost"
#export SIMULATOR_LOCAL_HOST="192.168.11.5"
export SIMULATOR_PORT="2000"
export TEAM_AGENT=${OP_BRIDGE_ROOT}/op_bridge/op_ros2_agent.py
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${SCENARIO_RUNNER_ROOT}":"${OP_BRIDGE_ROOT}":${PYTHONPATH}
export AGENT_FRAME_RATE="20"
# Autonomous actor default role_name
export AGENT_ROLE_NAME="hero"

# modes are 
#   * "leaderboard" : when runner the leaderboard (route based) scenario collections 
#   * "srunner" : when scenario is loaded using scenario runner, and only agent is attached
#   * "free" : when loading empty map only , either carla town or any OpenDRIVE map 
export OP_BRIDGE_MODE="free" 

# CARLA town name or custom OpenDRIVE absolute path, when BRIDGE_MODE is free 
export FREE_MAP_NAME="C_track_1_0_7" 

# Spawn point for the autonomous agent, when BRIDGE_MODE is free 
# "x,y,z,roll,pitch,yaw"
# Empty string means random starting position
# export FREE_AGENT_POSE="175.4,195.14,0,0,0,180" 
# export FREE_AGENT_POSE="88.6,-226,0,0,0,0" 
export FREE_AGENT_POSE=""


source /opt/ros/humble/setup.bash 
source ${AUTOWARE_ROOT}/install/setup.bash
ros2 run rviz2 rviz2 -d ${OP_AGENT_ROOT}/rviz/carla_autoware.rviz -s ${OP_AGENT_ROOT}/rviz/image/autoware.png & 
python3 ${OP_BRIDGE_ROOT}/op_bridge/op_bridge_ros2.py

