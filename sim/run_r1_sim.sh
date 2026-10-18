#!/usr/bin/env bash
# R1 C++ 시뮬 실행.
# 1) sim/r1_sim_config.yaml 을 시뮬레이터가 읽는 위치로 복사한다 (설정은 이 레포에서 관리).
# 2) ROS2 를 source 한 셸에서는 ROS2 의 libddsc(iceoryx 빌드)가 먼저 잡혀 시뮬이 abort 하므로
#    unitree_sdk2 의 CycloneDDS 라이브러리를 LD_LIBRARY_PATH 맨 앞에 둔다. (docs/troubleshooting.md)
# usage: bash sim/run_r1_sim.sh [scene.xml]
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_DIR="$WS_ROOT/third_party/unitree_mujoco/simulate"
SCENE="${1:-scene.xml}"

cp "$WS_ROOT/sim/r1_sim_config.yaml" "$SIM_DIR/config.yaml"

export LD_LIBRARY_PATH="$WS_ROOT/third_party/unitree_sdk2/install/lib:${LD_LIBRARY_PATH:-}"
export DISPLAY="${DISPLAY:-:0}"

cd "$SIM_DIR/build"
exec ./unitree_mujoco -r r1 -s "$SCENE"
