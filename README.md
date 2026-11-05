# LH20 左右手视觉遥操

Ubuntu 24.04 下的 **L20-10 / G20 左右手**遥操工程快照。普通 RGB 摄像头识别人手，通过 MediaPipe、dex-retargeting、ROS2 Jazzy 和 SocketCAN 驱动灵巧手，控制 16 个独立关节。

**当前快照：2026-11-05，开发中。** 左手已获使用者基本动作确认；右手新增独立模型、映射和控制链路，拇指现默认 `swapped`，但尚未完成方向和延迟验收。最近一次右手会话因 `Factory CAD worker response timeout` 停止，这是仍需修复的问题。厂家角度表不等于逐台设备测量标定，离线 CAD 耗时不等于实机端到端延迟。

## 已配置过环境的电脑怎么用

现有实验电脑继续在原工程运行即可，公开副本不替换原工程：

```bash
cd ~/linker_hand_ros2_sdk
./lh20-left vision
# 接右手时使用下面这一行，左右手不要同时占用同一相机：
./lh20-right vision
```

窗口识别到对应侧物理人手后自动跟随；**Q 退出**。会自动启动或复用 ROS 驱动，不需要另外开 ROS 终端。退出停止新目标，不自动回原位。复用的已有 SDK 保持运行。

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

左右手固定入口默认均为整手、Vector、agile、segments、swapped；无需附加参数。底层原始 `lh20` 入口保留历史默认值，日常使用 `lh20-left` / `lh20-right`。右手首次启动只读核验 CAN 0x27 的身份并写入本机登记，不使用左手登记文件。完整说明见 [右手遥操](docs/RIGHT_TELEOP.md) 和根目录命令速查 PDF。上传 GitHub、离线测试与模型自检不需要灵巧手上电。

## 常用命令

DexPilot 方法组件对照入口：`./lh20-left vision dexpilot`。Vector 入口为 `./lh20-left vision vector`；原 `./lh20-left vision` 保留为兼容入口。
日常遥操使用 Vector。2026-10-25 使用者确认 Vector 尚可，但当前 LH20 的 DexPilot 适配有明显小指、拇指姿态偏差及跟随滞后，未通过实机验收，保留用于离线方法对照。
Vector 新版在原有骨段目标上增加拇指 IP→指尖方向（MediaPipe 3→4），沿用原厂家映射；实时流使用只保留最新帧的相机读取、随速度调整的滤波，并在绘图与日志写入前发送目标。`./lh20-left vision vector` 命令不变。整手自动会话记录 `teleop/data/vision_vector_*.jsonl`，包含关键点、目标和分段耗时，便于继续核对实机效果；不保存相机图像。
2026-11-02 第一版更新（现 `directions` 模式）：Vector 在原求解后单独细化拇指四个关节，以三段单位方向减少骨长差异和短骨段被弱化的问题；从上一帧拇指姿态连续求解，弱连续性约束避免不必要的根部翻转。视觉端记录目标接触，交给控制桥按实际反馈、限速增量和完整路径处理，局部接触可只暂停相关手指。厂家限位、swapped 映射、速度与卡滞保护保留。85 项离线测试及四段共 8546 帧回放完成；详见 [设计说明](docs/DESIGN.md#2026-11-02-独立拇指方向细化)。
同日根部响应更新：默认 `--thumb-root-mode agile` 用掌部坐标系中的拇指张开角、离掌角和 MCP 弯曲角，为 yaw、roll、pitch 提供显式引导，继续结合骨段方向优化；可用 `--thumb-root-mode directions` 恢复上一版。新增参数是控制映射，不是实机关节标定。五段共 10179 帧回放显示，最新记录的 roll/pitch 低于 5° 比例由 62.9%/50.8% 降至 28.0%/0.7%，同时存在指骨方向拟合和目标接触率的取舍；不能视为实机精度改善，详见 [根部响应说明](docs/DESIGN.md#2026-11-02-根部灵敏度与几何引导)。
DexPilot 入口调用固定版本源码中的 `DexPilotOptimizer`，沿用现有左手映射、限速和 CAD 检查。新增指尖参考点来自厂家末节网格，并未完成实物标定；模型自检通过不能代替实机验收。可加 `--preview` 仅看模型，或加 `--self-test` 做无硬件自检。自动会话保存 `teleop/data/vision_dexpilot_*.jsonl`，方便按同一输入做离线对照。

| 目的 | 命令 | 环境 |
| --- | --- | --- |
| 环境与本机 ROS 自检，无硬件指令 | `./lh20-left check` | `(lh20_ros2)` |
| 模型、优化器、关节约束离线自检 | `./lh20-left vision --self-test` | `(lh20_vision)` |
| 摄像头识别与重定向预览，无电机输出 | `./lh20-left vision --preview` | `(lh20_vision)` |
| 右手整手自动随动（开发中） | `./lh20-right vision` | 自动管理相应环境；已知 CAD 超时待修复 |
| 右手仅视觉预览 | `./lh20-right vision --preview` | 无电机输出 |
| 右手模型自检 | `./lh20-right vision --self-test` | 无相机或设备访问 |
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
teleop/config/             厂家左右手转换表、向量重定向配置
teleop/models/             必要的左右手 URDF / STL 网格和许可证
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
