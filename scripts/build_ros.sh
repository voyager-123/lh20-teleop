#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
base="${LH20_CONDA_BASE:-$(conda info --base)}"
source "$base/etc/profile.d/conda.sh"
conda activate lh20_ros2
cd "$root"
echo '(lh20_ros2) 构建本仓库驱动；不连接硬件。'
colcon build --symlink-install --packages-select linker_hand_ros2_sdk
