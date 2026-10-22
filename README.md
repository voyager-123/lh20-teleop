# LH20 左手视觉遥操

Ubuntu 24.04 下的 **L20-10 / G20 左手**遥操初稿。普通 RGB 摄像头识别人手，通过 MediaPipe、dex-retargeting、ROS2 Jazzy 和 SocketCAN 驱动灵巧手，控制 16 个独立关节。

**当前版本：0.1.0-draft（2026-10-22）。** 实机使用者已确认基本动作可用，拇指采用 **`swapped`**；尚无抓取成功率、延迟分布或角度精度的量化结果。厂家角度表不等于逐台设备测量标定。

## 已配置过环境的电脑怎么用

现有实验电脑继续在原工程运行即可，公开副本不替换原工程：

```bash
conda activate lh20_vision
cd ~/linker_hand_ros2_sdk
./lh20-left vision --scope hand --thumb-axes swapped
```

窗口识别到物理左手后自动跟随；**Q 退出**。会自动启动或复用 ROS 驱动，不需要另外开 ROS 终端。退出停止新目标，不自动回原位。复用的已有 SDK 保持运行。

## 从此仓库开始

```bash
git clone --recurse-submodules https://github.com/voyager-123/lh20-teleop.git
cd lh20-teleop
```

先完成 [安装与环境复现](docs/INSTALL.md)，再启动 CAN。仅在没有控制会话运行时配置接口：

```bash
sudo ip link set dev can0 down
sudo ip link set dev can0 type can bitrate 1000000
sudo ip link set dev can0 up
ip -details link show can0
```

首次在新副本使用时登记本机设备身份；这个步骤只查询身份，不发送运动目标：

```bash
conda activate lh20_ros2
./scripts/register_hand.sh
```

运行遥操：

```bash
conda activate lh20_vision
./lh20-left vision
```

此公开初稿的 `lh20-left vision` **默认 `--scope hand --thumb-axes swapped`**，可显式写出以便记录实验。底层原始 `lh20` 入口保留历史默认值，日常请用 `lh20-left`。

## 常用命令

| 目的 | 命令 | 环境 |
| --- | --- | --- |
| 环境与本机 ROS 自检，无硬件指令 | `./lh20-left check` | `(lh20_ros2)` |
| 模型、优化器、关节约束离线自检 | `./lh20-left vision --self-test` | `(lh20_vision)` |
| 摄像头识别与重定向预览，无电机输出 | `./lh20-left vision --preview` | `(lh20_vision)` |
| 左手整手自动随动 | `./lh20-left vision` | `(lh20_vision)`，自动管理 `(lh20_ros2)` 子进程 |
| 换摄像头 | `./lh20-left vision --camera 1` | `(lh20_vision)` |
| 单独运行 SDK | `./lh20-left sdk` | `(lh20_ros2)`，Ctrl+C 停止 |
| SDK 已运行时读取状态 | `./lh20-left status` | `(lh20_ros2)` |
| 全部发布版离线检查 | `./scripts/test_offline.sh` | 脚本分别选择两个环境 |

包装脚本只为所启动的进程激活环境，不会改变父终端提示符。手动 `conda activate` 可以让终端前面显示对应环境名。

## 工程结构

```text
docs/                      安装、原理、排错、验证和来源
environments/              两个 Conda 环境的无本机 prefix 导出
scripts/                   构建、身份登记、离线测试
src/linkerhand-ros2-sdk/    厂家 ROS2 驱动的已修改源码及原许可证
teleop/bridge/             自动会话、ROS 桥、映射、反馈与卡滞处理
teleop/vision/             摄像头、重定向、CAD 检查
teleop/config/             厂家左手转换表、向量重定向配置
teleop/models/             必要的左手 URDF / STL 网格和许可证
teleop/reference/          厂家转换函数的纯计算导出、来源与许可证
teleop/tests/              无硬件回归测试
teleop/dex-retargeting/    固定版本的上游 Git 子模块
```

运行时的 `.local.env`、`teleop/data/`、`build/`、`install/`、`log/` 都不进 Git。没有打包个人文档、现场录制或 Conda 安装目录。

- [安装与环境复现](docs/INSTALL.md)
- [控制流程、拇指配置和运行边界](docs/DESIGN.md)
- [常见问题与恢复](docs/TROUBLESHOOTING.md)
- [初稿验证记录](docs/VALIDATION.md)
- [要学的仓库和论文](docs/REFERENCES.md)
- [第三方来源及修改](THIRD_PARTY_NOTICES.md)

项目新增代码采用 Apache-2.0；第三方文件分别保留其原始许可证。
