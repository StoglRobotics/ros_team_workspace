#!/bin/bash
set -e
shopt -s expand_aliases

# Setup environment
source /opt/ros/"$ROS_DISTRO"/setup.bash
# Determine script location to find setup.bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
RTW_ROOT="$(dirname "$SCRIPT_DIR")"

# Configure git
git config --global user.email "test@example.com"
git config --global user.name "Test User"

# Install rtwcli
echo -e "${TERMINAL_COLOR_BLUE}Installing rtwcli...${TERMINAL_COLOR_NC}"

# Setup auto-sourcing to enable aliases mechanism in setup.bash if needed
source "$RTW_ROOT"/setup.bash
echo -e "\nyes\nyes\n" | setup-auto-sourcing
source ~/.bashrc

# Remove EXTERNALLY-MANAGED to allow pip install
rm -f /usr/lib/python*/EXTERNALLY-MANAGED

cd "$RTW_ROOT"/rtwcli  # enter the RTW-CLI folder
pip3 install -r requirements.txt --break-system-packages  # since Ubuntu 24.04 is this flag required as we are not using virtual environment
# pip3 install ./rtwcli ./rtw_cmds ./rtw_rocker_extensions
cd -  # go back to the folder where you cloned the RTW
export PATH=$PATH:$HOME/.local/bin

# Create a workspace with RTW CLI
echo -e "${TERMINAL_COLOR_BLUE}Creating workspace with rtwcli...${TERMINAL_COLOR_NC}"
rtw workspace create --ws-name test_ws --ws-folder ~/ws --ros-distro "$ROS_DISTRO"

# Use the workspace to setup environment
# rtw workspace use writes to a tmp file targeting parent PID ($$)
# We must source this file manually in a non-interactive script
rtw ws test_ws
source "/tmp/ros_team_workspace/workspace_$$.bash"

echo "----------------------------------------------------------------"
echo -e "${TERMINAL_COLOR_BLUE}Creating description package...${TERMINAL_COLOR_NC}"
rosds
# Input:
# \n : Is this your workspace? (Yes)
# 1  : Standard package
# 1  : User input maintainer
# Test User : Name
# test@example.com : Email
# 1  : User input license
# Apache-2.0 : License
# 1  : ament_cmake
# \n : Confirmation
# no : Setup repository? (No) - Must use 'no' not 'n'
echo -e "\n1\n1\nTest User\ntest@example.com\n1\nApache-2.0\n1\n\nno\n" | create-new-package my_robot_description "Description package"

echo "----------------------------------------------------------------"
echo -e "${TERMINAL_COLOR_BLUE}Setting up robot description...${TERMINAL_COLOR_NC}"
cd my_robot_description
# Input:
# 1  : xml launch files
# \n : Confirmation
echo -e "1\n\n" | setup-robot-description my_robot 1

echo "----------------------------------------------------------------"
echo -e "${TERMINAL_COLOR_BLUE}Creating control package...${TERMINAL_COLOR_NC}"

# Same input as above
rosds
echo -e "\n1\n1\nTest User\ntest@example.com\n1\nApache-2.0\n1\n\nno\n" | create-new-package my_robot_control "Control package"

echo "----------------------------------------------------------------"
echo -e "${TERMINAL_COLOR_BLUE}Setting up robot bringup...${TERMINAL_COLOR_NC}"
cd my_robot_control
# Input:
# 1  : xml launch files
# \n : Confirmation
echo -e "1\n\n" | setup-robot-bringup my_robot my_robot_description

echo "----------------------------------------------------------------"
echo -e "${TERMINAL_COLOR_BLUE}Building workspace...${TERMINAL_COLOR_NC}"
rosdep_prep
rosdepi
cb

echo "----------------------------------------------------------------"
echo -e "${TERMINAL_COLOR_BLUE}Testing launch files...${TERMINAL_COLOR_NC}"
# Refresh environment after build
rtw ws test_ws
source "/tmp/ros_team_workspace/workspace_$$.bash"

# Function to wait for /robot_description topic to be available and have data
# Sleeps for an initial period, then polls up to max_retries times every poll_interval seconds
wait_for_robot_description() {
  local initial_wait=${1:-20}
  local max_retries=${2:-10}
  local poll_interval=${3:-2}

  echo -e "${TERMINAL_COLOR_YELLOW}Waiting ${initial_wait}s for nodes to start...${TERMINAL_COLOR_NC}"
  sleep "$initial_wait"

  # Check if topic exists
  local topic_found=false
  for i in $(seq 1 "$max_retries"); do
    echo -e "${TERMINAL_COLOR_YELLOW}[$i/$max_retries] Checking for /robot_description topic...${TERMINAL_COLOR_NC}"
    if ros2 topic list | grep -q "/robot_description"; then
      echo -e "${TERMINAL_COLOR_GREEN}Topic /robot_description found.${TERMINAL_COLOR_NC}"
      topic_found=true
      break
    fi
    sleep "$poll_interval"
  done

  if [ "$topic_found" != "true" ]; then
    echo -e "${TERMINAL_COLOR_RED}Error: Topic /robot_description not found after $max_retries retries.${TERMINAL_COLOR_NC}"
    exit 1
  fi

  # Check if data is available on the topic
  echo -e "${TERMINAL_COLOR_YELLOW}Checking for data on /robot_description...${TERMINAL_COLOR_NC}"
  if ros2 topic echo /robot_description --once --timeout 10 > /dev/null; then
    echo -e "${TERMINAL_COLOR_GREEN}Data received on /robot_description.${TERMINAL_COLOR_NC}"
  else
    echo -e "${TERMINAL_COLOR_RED}Error: No data received on /robot_description.${TERMINAL_COLOR_NC}"
    exit 1
  fi
}

# Function to kill background process on exit or error
cleanup() {
  if [ -n "$PID_DESC" ]; then kill "$PID_DESC" 2>/dev/null || true; fi
  if [ -n "$PID_CTRL" ]; then kill "$PID_CTRL" 2>/dev/null || true; fi
  if [ -n "$PID_TEST_PUB" ]; then kill "$PID_TEST_PUB" 2>/dev/null || true; fi
}
trap cleanup EXIT

# Test description launch
echo -e "${TERMINAL_COLOR_BLUE}Launching my_robot_description load_description.launch.xml...${TERMINAL_COLOR_NC}"
ros2 launch my_robot_description load_description.launch.xml &
PID_DESC=$!
wait_for_robot_description 20 10 2

kill $PID_DESC 2>/dev/null || true
wait $PID_DESC 2>/dev/null || true
PID_DESC=""
sleep 2

# Regression test: sim_gazebo/use_mock must select the Gazebo hardware plugin,
# not the mock one, and must forward simulation_controllers into the URDF.
echo -e "${TERMINAL_COLOR_BLUE}Launching my_robot_description load_description.launch.xml with sim_gazebo:=true...${TERMINAL_COLOR_NC}"
ros2 launch my_robot_description load_description.launch.xml sim_gazebo:=true use_mock:=false simulation_controllers:=/tmp/dummy_controllers.yaml &
PID_DESC=$!
wait_for_robot_description 20 10 2

echo -e "${TERMINAL_COLOR_YELLOW}Checking robot_description content for the Gazebo hardware plugin...${TERMINAL_COLOR_NC}"
ROBOT_DESCRIPTION_CONTENT=$(ros2 topic echo /robot_description --once --timeout 10 --full-length 2>/dev/null)

if echo "$ROBOT_DESCRIPTION_CONTENT" | grep -q "gz_ros2_control/GazeboSimSystem"; then
    echo -e "${TERMINAL_COLOR_GREEN}Success: GazeboSimSystem plugin present.${TERMINAL_COLOR_NC}"
else
    echo -e "${TERMINAL_COLOR_RED}Error: GazeboSimSystem plugin missing from robot_description with sim_gazebo:=true.${TERMINAL_COLOR_NC}"
    exit 1
fi

if echo "$ROBOT_DESCRIPTION_CONTENT" | grep -q "mock_components/GenericSystem"; then
    echo -e "${TERMINAL_COLOR_RED}Error: GenericSystem (mock) plugin still present with sim_gazebo:=true use_mock:=false.${TERMINAL_COLOR_NC}"
    exit 1
else
    echo -e "${TERMINAL_COLOR_GREEN}Success: GenericSystem plugin correctly absent.${TERMINAL_COLOR_NC}"
fi

if echo "$ROBOT_DESCRIPTION_CONTENT" | grep -q "/tmp/dummy_controllers.yaml"; then
    echo -e "${TERMINAL_COLOR_GREEN}Success: simulation_controllers path forwarded into robot_description.${TERMINAL_COLOR_NC}"
else
    echo -e "${TERMINAL_COLOR_RED}Error: simulation_controllers path missing from robot_description.${TERMINAL_COLOR_NC}"
    exit 1
fi

kill $PID_DESC 2>/dev/null || true
wait $PID_DESC 2>/dev/null || true
PID_DESC=""
sleep 2

# Test bringup launch
echo -e "${TERMINAL_COLOR_BLUE}Launching my_robot_control start_offline.launch.xml...${TERMINAL_COLOR_NC}"
ros2 launch my_robot_control start_offline.launch.xml &
PID_CTRL=$!
wait_for_robot_description 20 10 2

echo -e "${TERMINAL_COLOR_YELLOW}Checking controllers...${TERMINAL_COLOR_NC}"
CONTROLLER_LIST=$(ros2 control list_controllers)
echo "$CONTROLLER_LIST"

ACTIVE_COUNT=$(echo "$CONTROLLER_LIST" | grep "active" | wc -l)
echo -e "${TERMINAL_COLOR_BLUE}Found $ACTIVE_COUNT active controllers.${TERMINAL_COLOR_NC}"

if [ "$ACTIVE_COUNT" -ge 2 ]; then
    echo -e "${TERMINAL_COLOR_GREEN}Success: At least 2 controllers are active.${TERMINAL_COLOR_NC}"
else
    echo -e "${TERMINAL_COLOR_RED}Error: Expected at least 2 active controllers, found $ACTIVE_COUNT.${TERMINAL_COLOR_NC}"
    exit 1
fi

echo "----------------------------------------------------------------"
echo -e "${TERMINAL_COLOR_BLUE}Testing joint_trajectory_controller...${TERMINAL_COLOR_NC}"

# Record initial joint positions
INITIAL_POSITIONS=$(ros2 topic echo /joint_states --once --timeout 10 2>/dev/null | sed -n '/^position:/,/^velocity:/p' | head -n -1)
echo -e "${TERMINAL_COLOR_BLUE}Initial joint positions:\n$INITIAL_POSITIONS${TERMINAL_COLOR_NC}"

# Launch the JTC test publisher
echo -e "${TERMINAL_COLOR_BLUE}Launching test_joint_trajectory_controller publisher...${TERMINAL_COLOR_NC}"
ros2 launch my_robot_control test_joint_trajectory_controller.launch.xml &
PID_TEST_PUB=$!
echo -e "${TERMINAL_COLOR_YELLOW}Waiting for robot to move (15 seconds)...${TERMINAL_COLOR_NC}"
sleep 15

# Check if joints have moved
CURRENT_POSITIONS=$(ros2 topic echo /joint_states --once --timeout 10 2>/dev/null | sed -n '/^position:/,/^velocity:/p' | head -n -1)
echo -e "${TERMINAL_COLOR_BLUE}Current joint positions:\n$CURRENT_POSITIONS${TERMINAL_COLOR_NC}"

if [ "$INITIAL_POSITIONS" != "$CURRENT_POSITIONS" ]; then
    echo -e "${TERMINAL_COLOR_GREEN}Success: Robot moved with joint_trajectory_controller.${TERMINAL_COLOR_NC}"
else
    echo -e "${TERMINAL_COLOR_RED}Error: Robot did not move with joint_trajectory_controller.${TERMINAL_COLOR_NC}"
    exit 1
fi

# Stop JTC test publisher
kill $PID_TEST_PUB
PID_TEST_PUB=""
sleep 2

echo "----------------------------------------------------------------"
echo -e "${TERMINAL_COLOR_BLUE}Switching controllers: deactivate JTC, activate forward_position_controller...${TERMINAL_COLOR_NC}"

ros2 control switch_controllers --deactivate joint_trajectory_controller --activate forward_position_controller

echo -e "${TERMINAL_COLOR_YELLOW}Checking controller states after switch...${TERMINAL_COLOR_NC}"
ros2 control list_controllers

echo "----------------------------------------------------------------"
echo -e "${TERMINAL_COLOR_BLUE}Testing forward_position_controller...${TERMINAL_COLOR_NC}"

# Record initial joint positions
INITIAL_POSITIONS=$(ros2 topic echo /joint_states --once --timeout 10 2>/dev/null | sed -n '/^position:/,/^velocity:/p' | head -n -1)
echo -e "${TERMINAL_COLOR_BLUE}Initial joint positions:\n$INITIAL_POSITIONS${TERMINAL_COLOR_NC}"

# Launch the FPC test publisher
echo -e "${TERMINAL_COLOR_BLUE}Launching test_forward_position_controller publisher...${TERMINAL_COLOR_NC}"
ros2 launch my_robot_control test_forward_position_controller.launch.xml &
PID_TEST_PUB=$!

# Monitor positions every 5 seconds for 5 iterations
FPC_MOVED=false
for i in $(seq 1 5); do
    sleep 5
    CURRENT_POSITIONS=$(ros2 topic echo /joint_states --once --timeout 10 2>/dev/null | sed -n '/^position:/,/^velocity:/p' | head -n -1)
    echo -e "${TERMINAL_COLOR_YELLOW}[$i/5] Joint positions:\n$CURRENT_POSITIONS${TERMINAL_COLOR_NC}"
    if [ "$INITIAL_POSITIONS" != "$CURRENT_POSITIONS" ]; then
        echo -e "${TERMINAL_COLOR_GREEN}Success: Robot moved with forward_position_controller (detected at check $i).${TERMINAL_COLOR_NC}"
        FPC_MOVED=true
        break
    fi
done

if [ "$FPC_MOVED" != "true" ]; then
    echo -e "${TERMINAL_COLOR_RED}Error: Robot did not move with forward_position_controller after 5 checks.${TERMINAL_COLOR_NC}"
    exit 1
fi

# Stop FPC test publisher
kill $PID_TEST_PUB
PID_TEST_PUB=""
sleep 2

# Stop bringup
kill $PID_CTRL
PID_CTRL=""

echo -e "${TERMINAL_COLOR_GREEN}All tests passed!${TERMINAL_COLOR_NC}"
