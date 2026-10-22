# 第三方来源与本地修改

项目新增代码按根目录 Apache-2.0 许可证提供。第三方文件沿用各自许可证，不能仅按本项目许可证推断所有第三方授权。

| 组件 | 来源 / 版本 | 原始许可证 |
| --- | --- | --- |
| `src/linkerhand-ros2-sdk` | [linker-bot/linkerhand-ros2-sdk](https://github.com/linker-bot/linkerhand-ros2-sdk)，迁移工作副本报告版本 3.2.5；原始 Git commit 未随迁移保存 | Apache-2.0，目录内 LICENSE |
| `teleop/models/l20_10_left` | [linker-bot/linkerhand-urdf](https://github.com/linker-bot/linkerhand-urdf)，archive/lhg20/left 快照；源 URDF SHA-256 见 manifest.json，原始 commit 未保存 | Apache-2.0，目录内 LICENSE |
| `teleop/dex-retargeting` | [dexsuite/dex-retargeting](https://github.com/dexsuite/dex-retargeting)，`8632b2cab32e1b51ce379940c414a0f78332ff6b`，作为 Git 子模块 | MIT，子模块内 LICENSE |
| `teleop/reference/linkerhand_cpp_2_2_0` | [linker-bot/linkerhand-cpp-sdk](https://github.com/linker-bot/linkerhand-cpp-sdk)，`948da97ce591dc85cebcfc761463dec31ffd30a9`，SDK 2.2.0；纯函数转换样本与来源摘录 | MIT，目录内 LICENSE |
| `teleop/reference/linkerbot_python_l20` | 厂家 Python L20 角度 / 故障参考摘录；具体快照信息按目录文件 | MIT，目录内 LICENSE |

## 与上游相比的重要修改

- 厂家 ROS 节点的 L20-10 路径：启动时不自动下发回位 / 清故障 / 速度或扭矩设置；提供独立单关节控制入口；控制订阅使用深度 1，避免旧目标排队。以随仓库源码为准。
- 左手模型：修正 `middle_mcp_pithch` 拼写为 `middle_mcp_pitch`；中指 DIP 的零 effort / velocity 改为与其他耦合远端一致的模型值 1；将网格位置改成相对路径。未宣称这些速度是实测值。
- 新增：视觉到 ROS 的本机桥、厂家 raw / rad 转换检查、拇指 `swapped` 适配、反馈延迟与单关节卡滞处理、CAD 接触恢复和会话生命周期管理。
- 为公开迁移处理：设备身份改为 Git 忽略的本机配置；解释器通过 Conda 根目录或环境变量定位；不包含现场日志和个人文档。

`docs/snapshot.json` 记录原工作副本和公开副本的模型 / 配置摘要，以及网格文件摘要。模型路径和设备身份分离导致公开配置摘要改变，厂家关节锚点数值保持不变。
