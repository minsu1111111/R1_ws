#!/usr/bin/env bash
# ACT 학습 (LeRobot 0.4.1 lerobot-train). 로컬(GTX 1060)과 서버에서 같은 스크립트를 쓴다.
#
#   source env/setup_sim.sh            # 로컬 (venv). 서버는 conda il 환경
#   bash training/il/train_act.sh                                   # 기본: r1_push_box_auto, 20k steps
#   DATASET=r1_push_box STEPS=50000 BATCH=16 bash training/il/train_act.sh
#   INIT_FROM=data/checkpoints/<RUN>/checkpoints/040000/pretrained_model DATASET=... STEPS=10000 bash training/il/train_act.sh
#     → 파인튜닝: 기존 가중치에서 이어서 학습 (구조·chunk_size 는 기존 것, 정규화 통계는 새 데이터셋으로 갱신)
#
# 결과: data/checkpoints/<RUN>/checkpoints/<step>/pretrained_model  (last → 마지막)
# 설정 근거: docs/training.md
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DATASET="${DATASET:-r1_push_box_auto}"
STEPS="${STEPS:-20000}"
BATCH="${BATCH:-8}"
SAVE_FREQ="${SAVE_FREQ:-5000}"
CHUNK="${CHUNK:-30}"            # 30 fps 에서 1 s
N_ACTION="${N_ACTION:-30}"      # 예측한 묶음을 끝까지 실행. 10 이면 출발 구간에서 제자리 반복 (docs/training.md 6절)
WORKERS="${WORKERS:-4}"
RUN="${RUN:-act_${DATASET}_$(date +%Y%m%d_%H%M)}"
INIT_FROM="${INIT_FROM:-}"

if [ -n "${INIT_FROM}" ]; then
  POLICY_ARGS=(--policy.path="${INIT_FROM}")
else
  POLICY_ARGS=(--policy.type=act --policy.chunk_size="${CHUNK}")
fi

lerobot-train \
  --dataset.repo_id="local/${DATASET}" \
  --dataset.root="${WS_ROOT}/data/datasets/${DATASET}" \
  "${POLICY_ARGS[@]}" \
  --policy.n_action_steps="${N_ACTION}" \
  --policy.device=cuda \
  --policy.push_to_hub=false \
  --batch_size="${BATCH}" \
  --steps="${STEPS}" \
  --save_freq="${SAVE_FREQ}" \
  --log_freq=100 \
  --num_workers="${WORKERS}" \
  --output_dir="${WS_ROOT}/data/checkpoints/${RUN}" \
  --job_name="${RUN}" \
  --wandb.enable=false
