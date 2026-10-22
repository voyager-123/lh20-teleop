#!/usr/bin/env bash
# Only the existing identity-query function touches CAN; it sends no motion frames.
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
base="${LH20_CONDA_BASE:-$(conda info --base)}"
source "$base/etc/profile.d/conda.sh"
conda activate lh20_ros2
echo '(lh20_ros2) 读取 can0 左手身份，写入本机 .local.env；不发送运动目标。'
python "$root/scripts/register_hand.py"
