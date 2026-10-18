#!/usr/bin/env bash
# 시뮬(unitree_mujoco)용 환경. 루프백(lo)으로 DDS 통신한다.
# usage: source env/setup_sim.sh

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash
[ -f "$WS_ROOT/ros2_ws/install/setup.bash" ] && source "$WS_ROOT/ros2_ws/install/setup.bash"
[ -f "$WS_ROOT/.venv/bin/activate" ] && source "$WS_ROOT/.venv/bin/activate"

export R1_NET_IF=lo
export ROS_DOMAIN_ID=1   # unitree_mujoco config 의 domain_id 와 일치시킬 것
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>
  <NetworkInterface name=\"${R1_NET_IF}\" priority=\"default\" multicast=\"default\"/>
</Interfaces></General></Domain></CycloneDDS>"

echo "[sim] if=$R1_NET_IF domain=$ROS_DOMAIN_ID rmw=$RMW_IMPLEMENTATION"
