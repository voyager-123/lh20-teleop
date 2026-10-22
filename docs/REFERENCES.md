# 学习材料与工程对应关系

这些是上游和相关研究链接；“相关论文”不表示本工程完整复现了论文系统。

| 建议顺序 | 资料 | 本工程对应 / 学习重点 |
| --- | --- | --- |
| 1 | [LinkerHand ROS2 SDK](https://github.com/linker-bot/linkerhand-ros2-sdk) | ROS 话题、20 槽位协议、设备反馈和厂家驱动 |
| 2 | [MediaPipe Hands](https://chuoling.github.io/mediapipe/solutions/hands.html) | 21 点、左右手判别、世界坐标、遮挡与坐标约定；当前使用 legacy solutions 接口 |
| 3 | [dex-retargeting](https://github.com/dexsuite/dex-retargeting) | 人手向量 → 机器人关节优化、关节限位、滤波和 mimic；本工程实际使用的重定向库 |
| 4 | [LinkerHand URDF](https://github.com/linker-bot/linkerhand-urdf) | LHG20 左手运动链、关节轴、STL 网格与几何碰撞 |
| 5 | [LinkerHand C++ SDK](https://github.com/linker-bot/linkerhand-cpp-sdk) | L20-10 / G20 左手 raw ↔ rad 约定；本工程离线导出转换表作为证据，不依赖运行其二进制控制硬件 |
| 6 | [AnyTeleop 论文](https://arxiv.org/abs/2307.04577)、[项目页](https://yzqin.github.io/anyteleop/) | 视觉遥操与运动重定向的系统组织；本工程只使用相关开源库中的部分组件 |
| 后续对比 | [DexPilot 论文](https://arxiv.org/abs/1910.03135) | 学习面向捏合 / 接触的重定向目标；尚未集成该论文完整算法 |
| 后续研究 | [SPIDER](https://github.com/facebookresearch/spider) | 后续方向参考；尚未集成，不是当前运行依赖 |

工程阅读顺序：`preview_l20.py` → `l20_10_left_vector.yaml` → `factory_mapping.py` → `live_hand.py` → `tracking_governor.py` → `factory_hand_audit.py` / `collision_recovery.py`。先弄清坐标、关节名和 raw 槽位，再研究优化目标。
