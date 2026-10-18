#!/usr/bin/env bash
# 외부 저장소를 third_party/ 에 받아 아래 고정 커밋으로 맞추고, ROS2 메시지 패키지를 ros2_ws/src 에 링크한다.
# 이미 받은 저장소도 고정 커밋으로 체크아웃한다 (로컬 수정이 있으면 건너뛰고 경고).
# 커밋을 올릴 때는 동작을 다시 확인하고 docs/setup.md 의 "외부 저장소 버전" 표도 같이 고친다.
# usage: bash env/clone_third_party.sh
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TP="$WS_ROOT/third_party"

# 이름 | URL | 고정 커밋 (2026-10-18 확인)
REPOS=(
  "unitree_mujoco|https://github.com/unitreerobotics/unitree_mujoco.git|1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d"
  "unitree_sdk2|https://github.com/unitreerobotics/unitree_sdk2.git|d1abf001220d3e5f59e00a0c4aaa595e00089a4a"
  "unitree_sdk2_python|https://github.com/unitreerobotics/unitree_sdk2_python.git|814556d15970dd2ecf1c9984e845ca02ab07e206"
  "unitree_ros2|https://github.com/unitreerobotics/unitree_ros2.git|668d1ec5a05d1c38d3306bdca7d59f2ba3581a88"
  "xr_teleoperate|https://github.com/unitreerobotics/xr_teleoperate.git|817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b"
  "unitree_IL_lerobot|https://github.com/unitreerobotics/unitree_IL_lerobot.git|41c2805742de879ddab2d8d6beaeaf215f876395"
  "unitree_rl_mjlab|https://github.com/unitreerobotics/unitree_rl_mjlab.git|1425b15f73bd4095f0df53709d7c389c3eb9e790"
  "lerobot|https://github.com/huggingface/lerobot.git|a5b29d430105f5235eb05bbf2db5a0d747a869d6"  # v0.4.1
  # 로봇 모델 저장소가 커서 R1 폴더만 받는다 (sparse checkout)
  "unitree_ros|https://github.com/unitreerobotics/unitree_ros.git|5994d4faef0a9cadd3287f8de0199a67eeb2a259|robots/r1_description robots/r1_a7_description"
)

for entry in "${REPOS[@]}"; do
  IFS='|' read -r name url commit sparse <<< "$entry"
  dir="$TP/$name"

  if [ ! -d "$dir/.git" ]; then
    if [ -n "${sparse:-}" ]; then
      git clone --filter=blob:none --sparse "$url" "$dir"
      # shellcheck disable=SC2086
      git -C "$dir" sparse-checkout set $sparse
    else
      git clone "$url" "$dir"
    fi
  fi

  if [ "$(git -C "$dir" rev-parse HEAD)" = "$commit" ]; then
    echo "[ok]   $name @ ${commit:0:8}"
    continue
  fi

  git -C "$dir" cat-file -e "$commit^{commit}" 2>/dev/null || git -C "$dir" fetch origin
  if git -C "$dir" checkout -q --detach "$commit"; then
    echo "[pin]  $name -> ${commit:0:8}"
  else
    echo "[warn] $name: 로컬 수정이 있어 ${commit:0:8} 로 체크아웃하지 못함 (git -C $dir status 확인)"
  fi
done

MSG_SRC="$TP/unitree_ros2/cyclonedds_ws/src/unitree"
for pkg in unitree_hg unitree_go unitree_api; do
  ln -sfn "$MSG_SRC/$pkg" "$WS_ROOT/ros2_ws/src/$pkg"
done

echo "완료: $TP"
