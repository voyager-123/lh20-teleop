#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
base="${LH20_CONDA_BASE:-$(conda info --base)}"
source "$base/etc/profile.d/conda.sh"
cd "$root"
echo '(lh20_ros2) 离线映射、反馈、卡滞、故障和数据校验测试。'
conda run -n lh20_ros2 python -m unittest discover -s teleop/tests -v
echo '(lh20_vision) 真实 CAD 网格恢复回归；不连接设备或摄像头。'
conda run -n lh20_vision python -E -s -m unittest discover -s teleop/vision -p test_factory_recovery.py -v
conda run -n lh20_vision python -E -s -m unittest discover -s teleop/vision -p test_thumb_profile.py -v
conda run -n lh20_vision python -E -s -m unittest discover -s teleop/vision -p test_low_latency_vector.py -v
conda run -n lh20_vision python -E -s -m unittest discover -s teleop/vision -p test_thumb_responsive.py -v
conda run -n lh20_vision python -E -s -m unittest discover -s teleop/vision -p test_dexpilot.py -v
./lh20-left vision --self-test
./lh20-left vision dexpilot --self-test
