# xArm6 草莓定点到达仿真（WSL2 + PyBullet）

课设《设施草莓采摘仿生机械臂控制系统设计》的定位精度仿真验证部分。唯一执行基线为同目录《实验指导书_xarm6草莓定点到达.md》。

## 范围声明

- 纯仿真；不制实物、不建模末端执行器（夹爪按机械图纸设计，未纳入本轮运动学验证）；
- 不含夹爪开合、采摘接触、状态机、视觉/CLIP、力控；
- 只验证 6 轴 xArm6 的末端 link（link6）依次到达 5 个写死的空间目标点；
- 方法为 PyBullet + 内置 `calculateInverseKinematics` 位置-only IK（不约束姿态）；
- 仿真动力学为无重力设定（`setGravity(0,0,0)`）+ 轨迹段速度前馈，验证轨迹规划与位置伺服的指令跟踪与定位收敛；重力负载前馈补偿属实物化后续工作。

## 演示素材（答辩可用）

> 以下动图与图片均由 **2026-11-04 同一次完整运行**（`results/reach_demo.mp4`，601 帧 / 960×720 / 20.03 s）派生，未重新仿真、未改脚本；画面已**抽样判读 6 帧**（第 0/120/240/360/480/600 帧），非全量逐帧校验。答辩引用数字一律以 `results/reach_results.csv` 与《实验指导书》v4 §8.3 为准。

![xArm6 依次到达 5 个草莓目标点（循环动图，约 20 s）](results/reach_demo.gif)

**关键帧拼图**（6 帧，t = 0 / 4 / 8 / 12 / 16 / 20 s，PPT 一页讲清全过程）：

![关键帧拼图](results/reach_keyframes.png)

| 素材 | 内容 | 规格 | 建议用途 |
|------|------|------|----------|
| `results/reach_demo.gif` | 循环动图，机械臂依次到达 5 点 | 400×300 / 120 帧 / 6 fps / 182 KB | README 自动播放、现场快速展示 |
| `results/reach_demo.mp4` | 完整运行录像 | 960×720 / 30 fps / 20.03 s / 716 KB | 答辩现场播放、备用录屏 |
| `results/reach_keyframes.png` | 6 帧拼图（含帧号与时间戳） | 1440×780 / 184 KB | PPT 过程页 |
| `results/scene.png` | 终态截图（5 点全到达） | 960×720 / 26 KB | 结果页配图 |
| `results/reach_results.csv` | 逐点误差与收敛数据 | 5 行 / 744 B | 答辩数字唯一来源 |

**画面语义（答辩务必说清，避免误读）**：红球 = 待到达目标点、黄球 = 当前正在到达的目标、绿球 = 已通过判定的目标；绿色薄板为栽培架示意，蓝白棋盘为仿真地面。这些球是**位置标记，不是草莓外观或果实模型**；画面中无夹爪 / 末端执行器，也不含采摘接触。本轮验证的是 link6 原点依次到位（无重力设定下的轨迹跟踪与定位收敛），不是抓取成功率或果实识别。

**建议口径**：现场播 MP4（或看动图）+ 关键帧拼图讲「梯形轨迹规划 + 位置伺服」两层结构；数字只引用 `err_track` 0.29–0.88 mm < 5 mm、5/5 收敛、601 帧 / 82.5 s；主动声明无重力设定与范围边界（指导书 §0 / §12 / §14）。

## 当前状态（截至 2026-11-04 21:05）

### 已实测

- WSL2 Ubuntu，默认用户 `slphy_kai`，项目路径 `/home/slphy_kai/xarm_lab`；
- conda `robot` 环境：Python 3.11.18、pybullet 3.2.7（manylinux cp311 wheel）、numpy 2.4.6、imageio 2.38.1、imageio-ffmpeg 0.8.1、pillow 13.0.0；
- `import pybullet / numpy / imageio / imageio_ffmpeg / pybullet_data` 全部通过（`IMPORTS_OK 202010061 2.4.6 2.38.1`）；
- 工程文件就位：`xarm6_reach.urdf`、`test_load.py`、`xarm6_reach_demo.py`、`meshes/xarm6/visual/` 下 7 个 STL；
- `test_load.py` 加载自检（2026-11-04 18:20:17）：`== ALL PASS ==`，6 关节限位逐项 PASS、末端 link 名 link6、零位 FK `(0.2070, -0.0000, 0.1120)` m（|Δ| 0.00 mm）、帧缓冲 307200 bytes；
- 快速验证 / 负向验证（18:21）：5/5 点通过、负向模式退出码 2；
- 完整运行（2026-11-04 18:27:06 → 18:28:29）：5/5 点到达，`err_ik` / `err_track` 0.29–0.88 mm，601 帧 @ 960×720，渲染耗时 82.5 s，全程耗时 82.5 s；`results/` 下 CSV / MP4 / PNG 齐备，MP4 实测可完整解码 601 帧（30 fps / 20.03 s）；
- 画面抽样判读（21:04）：MP4 第 0/120/240/360/480/600 帧画面清晰、非黑屏，红 / 黄 / 绿球语义与脚本一致，机械臂姿态变化可见；派生 GIF 与关键帧拼图已校验（GIF 120 帧 / 循环 / 首中末帧非纯色）。

### 未实测（不得当作结论引用）

- 降级渲染分支（`--res 640x480 --every 16`）的实际耗时；
- `--threshold` 换阈值复跑；
- 全新机器按指导书从零环境复跑（零探索验证 Task 6）；
- MP4 画面内容的**全量逐帧**人工判读（已完成 6 帧抽样，非全量，不作逐帧正确性结论）。

以上实测数字见 `results/reach_results.csv` 与《实验指导书》§8.3 / §14；未执行项目一律记作「未执行」。

## 一键复跑

```bash
conda activate robot
cd ~/xarm_lab
python test_load.py                              # 模型加载自检，须 ALL PASS
python xarm6_reach_demo.py --no-video            # 快速验证：只出 CSV + 截图
python xarm6_reach_demo.py --check-out-of-range  # 负向验证：预期退出码 2
python xarm6_reach_demo.py                      # 完整运行：CSV + MP4 + 截图
```

固定解释器路径（免激活 conda）：`/home/slphy_kai/miniconda3/envs/robot/bin/python`。脚本依赖相对路径的 URDF / meshes / results，**必须在 `~/xarm_lab` 目录内执行**。

阈值默认 5 mm（`--threshold` 可配）；渲染降级用 `--res 640x480 --every 16`。完整参数含义与数值来源见实验指导书附录 B。

## 目录

| 路径 | 说明 |
|------|------|
| `xarm6_reach.urdf` | 手写 6 轴 URDF，数值来源见指导书附录 B |
| `test_load.py` | 加载自检（关节数 / 限位 / 零位 FK / 渲染通路） |
| `xarm6_reach_demo.py` | 主实验：IK + 梯形轨迹 + 位置伺服 + 动画与 CSV |
| `meshes/xarm6/visual/` | xarm_ros 官方 7 个 STL |
| `results/` | 实验产物与演示素材（CSV / MP4 / GIF / 关键帧拼图 / 终态截图） |
| `实验指导书_xarm6草莓定点到达.md` | 唯一执行基线 |
| `课设计划_v3.md` / `运行日志_20261104.md` | 计划与实跑日志快照 |
