# 常见问题

| 现象 | 检查 / 操作 |
| --- | --- |
| CAN 接口尚未启动 | `ip -details link show can0`；停止控制会话后按 README 配置为 UP、1 Mbps |
| `RTNETLINK answers: Device or resource busy` | 接口已 UP 时不能直接修改 bitrate；先停止控制，再 down → 配置 → up |
| 找不到 can0 | 检查 USB-CAN 连接、`lsusb`、`ip link`；不是激活 Conda 就能出现 can0 |
| 要求登记左手身份 | 运行 `./scripts/register_hand.sh`；身份保存在本机 `.local.env`，不要提交 |
| 另一台电脑 SDK / 右手节点干扰 | 通过本工程入口启动；会设置本机 ROS 发现范围。已有旧节点需在其所属终端停止 |
| SDK 或控制发布者重复 | 在原 `(lh20_ros2)` SDK / GUI / teleop 终端 Ctrl+C，只保留一个自动遥操会话 |
| 等待物理左手 | 让左手完整入镜、掌部和手指关键点清晰；检查相机编号。识别后自动跟随，无 READY 操作 |
| 跟随速度明显偏慢 | 检查光照、遮挡、CPU 占用；日志中的 `vision_age_ms`、`cad_ms` 可区分视觉和 CAD 耗时。大幅手势也可能受模型速度与反馈领先预算限制 |
| 某个关节停住但别的能动 | 属于单关节卡滞隔离。往反方向做手势释放；日志保留 held / released 事件，不需要重启整套程序 |
| `CAD_WAITING_FOR_SEPARATING_MOTION` | 当前手势会增加模型接触，改做分离 / 张开的手势；程序保持接收目标 |
| 64 / 128 仍导致退出 | 确认使用本次版本的左手入口；只忽略“精确等于 64 或 128”的 raw 值，其他故障和通信失效仍会停止 |
| 拇指方向不匹配 | 已验证这只手用 `--thumb-axes swapped`；新设备需逐轴核对，不能只凭外观判断角度关系 |
| Python / numpy / rclpy 导入冲突 | 不要将 ROS 的 PYTHONPATH 复制到视觉环境；使用包装脚本并核对两个环境的 Python 版本 |
| 已经打开 vision，还需 ROS 终端吗 | 不需要。自动会话会启动或复用 SDK；单独读状态时才可另外开 `(lh20_ros2)` 终端 |
| 退出后没有回位 | 设计如此；不自动发送回位动作 |

运行日志位于 `teleop/data/`。公开提问前，先去除设备身份、绝对路径和无关个人内容；不要直接上传整个 data 目录。
