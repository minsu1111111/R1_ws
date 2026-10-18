#!/usr/bin/env bash
# 로컬 데이터셋을 서버로 올리고, 서버 체크포인트를 로컬로 받는다.
# usage: R1_SERVER=user@host R1_SERVER_WS=~/aisl_robot_ws bash env/sync_to_server.sh [push|pull|both]
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
: "${R1_SERVER:?R1_SERVER=user@host 를 설정하세요}"
R1_SERVER_WS="${R1_SERVER_WS:-~/aisl_robot_ws}"
MODE="${1:-both}"

if [[ "$MODE" == push || "$MODE" == both ]]; then
  rsync -avz --progress "$WS_ROOT/data/datasets/" "$R1_SERVER:$R1_SERVER_WS/data/datasets/"
fi
if [[ "$MODE" == pull || "$MODE" == both ]]; then
  rsync -avz --progress "$R1_SERVER:$R1_SERVER_WS/data/checkpoints/" "$WS_ROOT/data/checkpoints/"
fi
