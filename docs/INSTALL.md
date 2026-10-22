# 安装与环境复现

已验证平台是 Ubuntu 24.04 x86_64，SocketCAN，L20-10 左手固件 1.0.6。ROS 通过 Conda / RoboStack 安装；本工程不是系统 `/opt/ros` 的 Python 3.12 组合。

## 1. 系统工具

```bash
sudo apt-get update
sudo apt-get install -y git build-essential can-utils v4l-utils
```

Conda 可使用 Miniconda 或 Miniforge。先让 `conda` 命令可用；此处不代替安装器安装 Conda。若安装位置特殊：

```bash
export LH20_CONDA_BASE=/absolute/path/to/conda
```

## 2. 还原两个环境

```bash
conda env create -f environments/lh20_ros2.yml
conda env create -f environments/lh20_vision.yml
```

环境已经存在时，不要重复创建，也不要直接覆盖实验环境。两个 YAML 是当日 `conda env export --no-builds` 的完整导出，去除了本机 prefix；不是跨平台锁文件，未在一台全新电脑完成安装验收。部分依赖比较多，是原环境迁移保留的结果。完整视觉 pip 清单在 `teleop/requirements-vision.lock.txt`，关键约束在 `teleop/constraints-vision.txt`。

| 环境 | 关键组件 |
| --- | --- |
| `lh20_ros2` | Python 3.11.6，ROS2 Jazzy，rclpy 7.1.3，numpy 2.4.6，python-can 4.6.1，PyQt5 5.15.11 |
| `lh20_vision` | Python 3.10.22，MediaPipe 0.10.21，numpy 1.26.4，OpenCV contrib 4.10.0.84，dex-retargeting 0.4.6，pin 2.7.0，hpp-fcl 2.4.4，nlopt 2.7.1，torch 2.5.1+cpu |

`torch==2.5.1+cpu` 来自 PyTorch CPU wheel 仓库；若 pip 无法解析这一项，可先为当前 shell 指定其额外索引，再创建视觉环境：

```bash
export PIP_EXTRA_INDEX_URL=https://download.pytorch.org/whl/cpu
conda env create -f environments/lh20_vision.yml
unset PIP_EXTRA_INDEX_URL
```

不要把两个环境的 numpy、Python 或 ROS 的 PYTHONPATH 混用。脚本启动视觉解释器时会清理 ROS 的 Python / 动态库路径。原 SDK 的 requirements.txt 包含很多无关示例依赖，不需要再整体覆盖本工程的环境。

## 3. 取得固定算法源码并构建

```bash
git submodule update --init teleop/dex-retargeting
git -C teleop/dex-retargeting rev-parse HEAD
./scripts/build_ros.sh
./lh20-left check
./scripts/test_offline.sh
```

dex-retargeting 必须是 `8632b2cab32e1b51ce379940c414a0f78332ff6b`，启动时会核对；不要跟随上游分支随意升级。模型和必要网格已经随本仓库提供。

## 4. 摄像头与设备

```bash
v4l2-ctl --list-devices
./lh20-left vision --preview
```

摄像头预览与模型自检不要求连接灵巧手。若设备权限不足，按本机设备权限策略处理 `/dev/video*`；不要用 sudo 运行整套 Conda / 视觉程序。

连接 USB-CAN 和左手，确认 `can0` 出现，然后执行 README 的 1 Mbps 配置。`./scripts/register_hand.sh` 将读到的左手身份保存到 Git 忽略的 `.local.env`，启动遥操时必须匹配。登记不意味着已经验证关节方向或角度精度。

特殊的视觉环境安装位置可通过 `LH20_VISION_PYTHON=/absolute/path/to/python` 覆盖。配置可以写进本机 `.local.env`，禁止上传该文件。
