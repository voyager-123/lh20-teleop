# LH20 第一天：空载与接触响应验证

今天只回答一个问题：**相同位置命令下，正常执行滞后与物体阻挡，在实测位置里是否有可重复的区别？**

使用左手、食指一个关节。今天不需要机械臂、双手、视觉遥操或训练策略。数据成立后再扩展抓取。

## 今天安排：约 1.5—2 小时

| 阶段 | 用时 | 操作与产出 |
|---|---:|---|
| 接入与固定参考 | 15—20 分钟 | 启动驱动、登记设备、保存同一参考姿态 |
| 小幅方向验证 | 10 分钟 | 空载 -5 计数，再返回；确认方向和日志 |
| 慢速接触试采 | 10—15 分钟 | 固定物体在路径后半段接触，采一次，查看曲线 |
| 正式小样本 | 30—45 分钟 | 空载/接触 × 5/10/20 count/s × 各 5 次，共 30 次 |
| 当场分析与补采 | 15—20 分钟 | 查看重复性、误差、反馈到达间隔及中止原因 |

如果时间有限，先各采 2 次，共 12 次。不要为了凑数量反复采明显失效的数据。

## 当前验证状态

已通过离线采集测试、原有 40 项回归及 4 项真实 CAD 网格测试。2026-10-23 实验室已完成 -5/-15 计数空载及 -5 计数接触试采；位置读回变化与返回已记录。接触由用户确认发生，但首条接触试采没有准确接触时刻。已放宽空载行程至60计数，接触仍至多20计数；较大空载行程需另外验证。已有遥操控制入口没有修改。模拟数据带 SYNTHETIC 标记，存于项目上一级 output/study_demo_20261023，不能写成实机结果。

## 1. 两个终端都进入这个目录

```bash
cd '/home/sjtu/文档/ChatGPT/dexterous hand/lh20-teleop'
```

`./lh20-study` 自行选择现有 lh20_ros2 环境，不需要另装依赖。

先退出其他遥操、GUI、jog 控制会话。采集程序只连接已启动的 SDK，不自动启动第二个驱动。CAN 沿用现有 can0 / 1 Mbps 配置；已启动时不要重新配置接口。

如果 can0 尚未启动，按现有项目安装文档配置后再继续。新工具不修改 CAN 比特率，不提高厂家速度/力度设置，不清故障。

如果本公开副本还没有 `.local.env`，先运行一次登记（只查询身份）：

```bash
./scripts/register_hand.sh
```

如果已有登记则跳过，不要覆盖 `.local.env`。

## 2. 终端 A：启动或保留唯一的左手 SDK

如果已有当前工程的左手 SDK 在运行，保留那个终端即可；否则：

```bash
./lh20-left sdk
```

接下来所有实验命令在终端 B 运行。两者必须使用相同 ROS_DOMAIN_ID。

## 3. 终端 B：保存参考姿态

先用原有已验证方式把手放到食指较伸展、周围手指不挡路的姿态，然后退出原控制端，只保留 SDK。

```bash
./lh20-study anchor --output teleop/data/study/day1_anchor.json
```

这一步只订阅，不发布动作。文件已存在会拒绝覆盖；如确实要更换起点，用新文件名，并给之后的一组试验另命名，不能混成同一组比较。

后续试验均引用同一 anchor；起点偏差超过 2 计数会被拒绝，避免每次重新取当前位置造成起点漂移。

## 4. 先做一次空载 -5 计数方向验证

先离线预览（不初始化 ROS/CAN，不发动作）：

```bash
./lh20-study probe --anchor teleop/data/study/day1_anchor.json \
  --condition free --trial-id direction_check \
  --slot 1 --delta -5 --speed 5 --return-to-start
```

预览通过后，加 `--execute` 才进入实机试采：

```bash
./lh20-study probe --anchor teleop/data/study/day1_anchor.json \
  --condition free --trial-id direction_check \
  --slot 1 --delta -5 --speed 5 --return-to-start --execute
```

程序等待 Enter，方便你准备录像和观察。正常完成后按显式选项缓慢返回。确认 slot 1 确为食指根部、负方向为预期闭合；发现方向不符就结束，不运行后续闭合计划。

这里的 5 是**原始协议计数**，不是 5 度。`--speed` 是外部位置目标斜坡的请求速度，也不是厂家电机速度设置。CAD 和调度开销会影响实际命令节奏，日志记录真实发布时间。

### 已完成小幅验证、需要更清楚的空载动作时

移开物体，允许使用最多60计数的空载行程；接触条件仍限制在20计数以内。60计数对应厂家模型的食指根部约18度，尚非逐台实测角度。以下动作仍逐计数前进，反馈差上限、CAD和固定起点检查保持生效：

```bash
./lh20-study probe --anchor teleop/data/study/day1_anchor.json \
  --condition free --trial-id free_visible_check \
  --slot 1 --delta -60 --speed 10 --post 1 --return-to-start --execute
```

这条约6秒闭合、1秒保持、12秒返回。它是较大幅度的单独验证，不与-15计数正式组混合比较；分析程序按不同请求幅度分开绘图。

## 5. 慢速接触试采

使用一个固定、不会被轻易推动的小物体，编号 `block_A`。布置到食指小幅闭合路径的后半段才接触，接触后剩余目标增量应很小。可先用较软的表面试探；不要用自己的手指作挡块。

第一次接触只用 -5 计数：

```bash
./lh20-study probe --anchor teleop/data/study/day1_anchor.json \
  --condition contact --object-id block_A --trial-id contact_pilot \
  --slot 1 --delta -5 --speed 5 --return-to-start --execute
```

确认这种幅度和响应可接受后，才准备正式计划的 -15 计数。物体位置要对应正式行程重新确认。若 -15 不适合当前姿态，请调整计划 CSV 的 `delta_raw`，并对空载与接触行统一使用同一幅度。

运行中按键：

| 按键 | 含义 |
|---|---|
| Enter | 开始当前试验 |
| `c` | 观察到接触，记录按键并停止继续增加闭合目标，短暂保持后按已请求的选项返回 |
| `r` | 记录观察到脱离，不直接改变动作 |
| `v` | 打印 VIDEO SYNC 并记录时间，供录像粗对齐 |
| `m` | 记录一般事件 |
| `q` / Esc / Ctrl+C | 中止新目标，保留数据；异常退出不自动返回 |

**Ctrl+C 不是硬件急停。** 停止新命令后，设备可能仍执行最后一个目标。保留现场原有的停止/复位方式。

默认命令—反馈差上限为 8 计数。超限、反馈超时、未知故障、控制器冲突或 CAD 拒绝都会停止试验并保存原因。这是采集保护，会截断数据，**不是已经实现的接触检测器**。不要为了避免停止随意提高上限或力度。

手工 `c` 有反应延迟；`--condition contact` 只是布置条件。两者都不能直接作为精确接触时间真值。

## 6. 运行正式计划

随工具附带的 `teleop/research/day1_plan.csv` 已列出 30 次；空载/接触交错，部分重复反转速度顺序，减少固定顺序与运行状态的混淆。需要 12 次版本可另生成：

```bash
./lh20-study plan --output teleop/data/study/day1_plan_12.csv --repeats 2
```

30 次计划先预览：

```bash
./lh20-study run-plan teleop/research/day1_plan.csv \
  --anchor teleop/data/study/day1_anchor.json --return-to-start
```

再采集：

```bash
./lh20-study run-plan teleop/research/day1_plan.csv \
  --anchor teleop/data/study/day1_anchor.json --return-to-start --execute
```

每次会打印条件、物体、速度与目标，等 Enter。空载时移开物体；接触时恢复相同夹具和位置。正常返回后再布置下一次。

任何中止都会结束批次，不自动重试或恢复动作。查明原因、恢复参考姿态后，把尚未完成的行复制到一个新计划文件再运行。会话文件夹不覆盖，重复 trial_id 的不同会话也会分别保存。

建议手机连续录像，画面同时包括手指、物体及电脑同步标记；记录对应视频文件名。`v` 仅用于粗对齐，今天不据此宣称毫秒级接触延迟。

## 7. 当场分析

分析全部实机试采（自动创建新的分析目录，不覆盖原始文件）：

```bash
./lh20-study analyze teleop/data/study
```

也可只分析终端打印的某个会话目录：

```bash
./lh20-study analyze 'teleop/data/study/替换为实际会话文件夹名'
```

生成：

- `report.md`：质量提示和各试验结果入口。
- `summary.csv` / `summary.json`：条件、状态、消息频率、误差、人工标记及数据路径。
- `overview.png`：完整试验的描述性误差对比，模拟/实机分开。
- 每个 `trial_XXX/trajectory.png`：目标、位置反馈、误差及消息到达间隔。
- 每个 `trial_XXX/feedback.csv`、`paired.csv`、`can_frames.csv`：后续建模可用的数据。

重点看三件事：

1. **同速度空载能否重复？** 正常误差是否随速度增加，是否存在方向切换差异？
2. **接触前后是否改变？** 物体阻挡后，实际位置是否停滞，而目标继续发生变化？
3. **差异是否比噪声大？** 若空载和接触响应高度重叠，先检查采样及读回，不急于训练网络。

成功采集不等于研究假设成立。今天不计算分类准确率，也不把位置误差换算为牛顿。

## 数据中不同时间的含义

`events.jsonl` 是原始事件日志，每条带单调时钟 ns、会话相对秒和墙钟 ns。metadata 保存软件摘要、参考姿态、设备信息与设置。

- `command_sent`：采集器调用 ROS 发布的时间；厂家内部实际参考未知。
- `command_observed`：被动模式收到其他控制器命令的时间。
- `command_echo`：自身命令的 ROS 回环，分析不会拿它替代实际发布时间。
- `state`：收到位置消息的时间；另保留 SDK 发布头时间与到达时的发布年龄。
- `can_frame`：被动收帧，保存内核墙钟、方向和原始数据；本地回环不代表硬件确认。
- `marker`：人工事件，不冒充设备接触真值。
- `acquisition_limit` / `error`：截断原因，不能删除后只报成功试验。

SDK 会缓存位置并重新发布。**相同位置不等于数据陈旧，变化的 ROS 时间戳也不等于新的硬件采样。** 原始 CAN 可以帮助进一步核查，但收帧时刻仍不是设备采样时刻。驱动 info 的 current/torque 字段照原样保存，尚未验证物理含义，不当作力反馈。

接收的旧 ROS 发布消息不会用于执行新目标。统计中的表观对齐只针对完整空载数据，混合了执行器响应和反馈过程，不是端到端视觉延迟。

## 已有控制器的纯记录模式

如今天先想记录已有遥操或 jog，可在其旁边运行：

```bash
./lh20-study record --condition unknown --trial-id existing_control --duration 60
```

这个入口不创建动作发布器。**不要同时运行 `record` 和本工具的 `probe/run-plan`**：后两者已经自行记录，并严格检查订阅者数量。

## 事后标注接触区间

看过视频以后，追加一段明确接触区间（时间相对于该会话开始，以曲线横轴为准）：

```bash
./lh20-study annotate 'teleop/data/study/替换为实际会话文件夹名' \
  --start 3.2 --end 3.6 --state contact --source video \
  --note '接触已明确；边界附近暂不标注'
```

这里只追加标注文件，不改原始日志；示例时间需要替换。拿不准的区间标 `uncertain`。重新分析会显示区间。

## 常见停止原因

- **SDK找不到/冲突：** 只保留一个左手 SDK，关闭视觉跟随、GUI 和其他控制发布端。
- **参考姿态不符：** 先恢复原起点，不要每次把当前位置重新当起点。
- **设备身份未登记：** 用现有 register_hand 脚本登记；登记后重新保存 anchor。
- **CAD拒绝：** 检查实际姿态和周围手指位置；不跳过 CAD。
- **差值上限触发：** 保留这次截断数据，检查是否接触、反馈滞后或动作幅度不合适，不自动硬推重试。
- **CAN记录无法打开：** 确认 can0。确需只记ROS时显式加 `--no-can-log`，metadata 会记录这一缺项。
- **旧工程找不到新入口：** 使用本目录的 `lh20-study`；它依赖同一副本的映射、CAD 与 SDK，不是单文件脚本。

## 离线演示与检查

```bash
./lh20-study demo --output ../output/study_demo_new
python -m unittest discover -s teleop/research -p 'test_study.py' -v
```

第二条需在 lh20_ros2 环境运行。demo、analyze、plan 和 annotate 不初始化 ROS/CAN；probe 不加 --execute 时只做离线模型检查。
