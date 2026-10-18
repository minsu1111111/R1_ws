#!/usr/bin/env bash
# 실기(R1)용 환경. 로봇과 연결된 유선 NIC 이름을 인자로 준다.
# usage: source env/setup_real.sh enp3s0

if [ -z "${1:-}" ]; then
  echo "usage: source env/setup_real.sh <NIC 이름>  (ip a 로 확인, 로봇 대역 192.168.123.x)"
  return 1 2>/dev/null || exit 1
fi

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash
[ -f "$WS_ROOT/ros2_ws/install/setup.bash" ] && source "$WS_ROOT/ros2_ws/install/setup.bash"
[ -f "$WS_ROOT/.venv/bin/activate" ] && source "$WS_ROOT/.venv/bin/activate"

export R1_NET_IF="$1"
export ROS_DOMAIN_ID=0   # 실기 기본 domain
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>
  <NetworkInterface name=\"${R1_NET_IF}\" priority=\"default\" multicast=\"default\"/>
</Interfaces></General></Domain></CycloneDDS>"

echo "[real] if=$R1_NET_IF domain=$ROS_DOMAIN_ID rmw=$RMW_IMPLEMENTATION"
