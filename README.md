# xArm6 草莓定点到达仿真（WSL2 + PyBullet）

课设《设施草莓采摘仿生机械臂控制系统设计》的定位精度仿真验证部分。唯一执行基线为同目录《实验指导书_xarm6草莓定点到达.md》。

## 范围声明

- 纯仿真；不制实物、不建模末端执行器（夹爪按机械图纸设计，未纳入本轮运动学验证）；
- 不含夹爪开合、采摘接触、状态机、视觉/CLIP、力控；
- 只验证 6 轴 xArm6 的末端 link（link6）依次到达 5 个写死的空间目标点；
- 方法为 PyBullet + 内置 `calculateInverseKinematics` 位置-only IK（不约束姿态）；
- 仿真动力学为无重力设定（`setGravity(0,0,0)`）+ 轨迹段速度前馈，验证轨迹规划与位置伺服的指令跟踪与定位收敛；重力负载前馈补偿属实物化后续工作。

## 当前状态（截至 2026-11-04 18:18）

### 已实测

- WSL2 Ubuntu，默认用户 `slphy_kai`，项目路径 `/home/slphy_kai/xarm_lab`；
- conda `robot` 环境：Python 3.11.18、pybullet 3.2.7（manylinux cp311 wheel）、numpy 2.4.6、imageio 2.38.1、imageio-ffmpeg 0.8.1、pillow 13.0.0；
- `import pybullet / numpy / imageio / imageio_ffmpeg / pybullet_data` 全部通过（`IMPORTS_OK 202010061 2.4.6 2.38.1`）；
- 工程文件就位：`xarm6_reach.urdf`、`test_load.py`、`xarm6_reach_demo.py`、`meshes/xarm6/visual/` 下 7 个 STL；`results/` 为空目录。

### 未实测（不得当作结论引用）

- `test_load.py` 加载自检结果；
- `xarm6_reach_demo.py` 的 `--no-video` 快速验证、`--check-out-of-range` 负向验证；
- 完整运行产物：`results/reach_results.csv`、`results/reach_demo.mp4`、`results/scene.png`；
- 5 个目标点的 err_ik / err_track / err_com、限位判定、收敛步与渲染/总耗时。

以上项目在实跑后按实验指导书 §8.3 / §13 回填，未跑之前一律记作「未执行」。

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
| `results/` | 实验产物（CSV / MP4 / PNG） |
| `实验指导书_xarm6草莓定点到达.md` | 唯一执行基线 |
| `课设计划_v3.md` / `运行日志_20261104.md` | 计划与实跑日志快照 |
