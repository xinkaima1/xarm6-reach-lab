# 实验指导书：xArm6 草莓采摘定点到达仿真（WSL2 + PyBullet）

> **版本**：**v7**（2026-11-08，触手三阶段动画：卷曲 → 反向展开 → 缠绕包覆）：把「一张一合」升级为按 rho=a*exp(b*theta) 向后卷曲成对数螺旋、到达指定坐标时反向展开、随后沿果实球面缠绕包覆夹住，逐颗循环、放果后回卷待机；§7 内嵌脚本三处同步。§8.2 / §8.3 / §13 按 15:24-15:26 全量重跑回填（976 帧 / 133.3 s / render_time_s 133.319），新增 §14.5 与附录 B 三阶段参数来源。叙事口径、纯视觉边界与数据口径不变。
> **v6**（2026-11-06 夜，L1 仿生触手视觉合拢示意 + 自制答辩材料包）：在 v5 演示层中加入 3 根 x 6 段纯视觉触手（示意几何、非中科大实物参数），到点合拢包覆果实、放果后张开；叙事口径保持「图纸与说明书讲仿生机构结构，仿真以通用 6 轴平台验证控制流程」，触手不进 URDF 运动学链、不进误差判据。§8.2 / §8.3 / §13 按 23:21-23:23 全量重跑回填（976 帧 / 133.6 s / render_time_s 133.581），新增 §14.4 与附录 B 示意参数来源，并交付自制答辩材料包。
> **v5**（2026-11-06，演示层抓放循环 + 全量重跑回填）：应课设「采摘草莓」叙事要求，§0 范围正式纳入「到达草莓点 → 末端下降放果 → 果实视觉入篮 → 前往下一点」的演示层循环；§7 内嵌脚本与工程脚本同步更新；§8.2 / §8.3 / §13 按 2026-11-06 全量重跑真实数字回填（976 帧 / 134.2 s），新增 §14.3 变更记录与附录 B 新参数来源。实跑时间 2026-11-06 17:59–18:34，环境 / 产物 / commit 见 §14.3。
> **v4**（2026-11-04，**首次实跑回填**）：按 WSL2 实测数据回填 §8.2 / §8.3 / §13，新增 §14 实跑变更记录；同步修复 §7 内嵌脚本的 Writer 收尾顺序与产物打印口径、CSV 补 `render_time_s` 列。实跑时间 2026-11-04 18:18–18:28，环境 / 产物 / commit 见 §14.1。
> **v3**（2026-11-04，第三轮修订：伺服动力学改**无重力设定** + positionGain 1.0 + 轨迹段速度前馈（修复重力下 kp=0.1 稳态误差不可达的伺服瘫塌阻断）；`--no-video` 真跳过常规帧渲染；增益 kwargs 改复数形式（positionGains/velocityGains）；§2.3 累计旋转修正；v3 计划文档答辩话术源对齐 PyBullet 口径）
> **v2**（2026-11-04）：按外部审查 8 点质疑修订（手写 URDF / IK 全参数可复现 / 误差口径写死 / 分关节力矩 / wheel 正常路径 / 阈值可配与未收敛交付分支 / 清除双引擎叙事）；中期追加主流程伺服化、点位回退 plan 原值、两重判定口径、整工程归档
> **执行环境**：Windows + WSL2 Ubuntu，项目驻 `~/xarm_lab`
> **阅读约定**：所有命令可直接复制粘贴执行。**v5 为当前唯一执行基线**，§8.2 / §8.3 / §13 的数字均来自 2026-11-06 全量重跑，数据源为 `results/reach_results.csv` 与《运行日志_20261104.md》11-06 章节；v4 的 11-04 数据保留于 §14.1 与历史 commit，不再作为当前结论。仍标注「未执行」的项目（降级渲染分支、换阈值复跑、全新机器从零环境复跑、MP4 全量逐帧判读）不得当作结论引用。

---

## 0. 范围声明（红线，不随排期改变）

本实验为课设《设施草莓采摘仿生机械臂控制系统设计》的**定位精度仿真验证部分**，范围锁定如下：

1. **纯仿真**：不制实物、**不建模末端执行器**（夹爪按机械图纸设计，不纳入本轮运动学验证）；
2. **不含**夹爪开合、采摘接触力学、状态机、视觉/CLIP、力控；
3. 精度验证对象只有一件事：**6 轴 xArm6 的末端 link 依次到达 5 个写死的空间目标点**；
4. 方法：PyBullet + 内置 `calculateInverseKinematics` **位置-only IK**（不约束姿态）；
5. 引擎口径全项目统一为 **PyBullet**，文档、说明书、答辩话术不出现第二引擎叙事；
6. 仿真动力学为**无重力设定**（`setGravity(0,0,0)`）+ 轨迹段速度前馈：验证的是轨迹规划与位置伺服的**指令跟踪与定位收敛**；重力负载前馈补偿（逆动力学）属实物化后续工作，不在本轮范围。本条为答辩与说明书的统一口径，主动声明而非被问出来；
7. **演示层抓放循环（v5 正式纳入范围）**：每个草莓点到位判定完成后，追加一段「移到篮子放果点 → 末端下降（示意低头放果）→ 果实视觉搬入篮中并留存 → 前往下一点」的轨迹，形成 抓-放-抓 的采摘观感。该层是**纯视觉示意**：篮子与果实均为静态视觉体，果实位置由脚本直接搬运，**无夹爪、无接触/约束、无夹持力、不约束末端姿态**；放果点不是第 6 个验收目标，不写入 CSV、不参与 §1.3 的两重静态判定与伺服跟踪判定，也不代表抓取成功率或果实识别能力。演示层另含 **v7 仿生触手三阶段视觉示意**（3 根 x 6 段纯视觉体，几何为示意值、非中科大实物参数）：接近阶段按 rho=a*exp(b*theta) **向后卷曲成对数螺旋**并收在末端后方不遮挡果实，到达指定坐标时**反向展开**，随后沿果实球面**缠绕包覆夹住**，逐颗循环、放果后回卷待机；它不是 URDF 末端执行器、不进运动学链、无接触 / 夹持力、不写 CSV。叙事口径为**图纸与说明书讲仿生机构结构，仿真以通用 6 轴平台验证控制流程**。答辩必须主动说明这一边界。

---

## 1. 实验目标与验收口径

### 1.1 目标

机械臂末端 link（**link6**）依次到达 5 个预先写死的草莓空间目标点。控制结构为两层：**关节空间梯形轨迹规划**（生成 240 Hz 目标轨迹）+ **关节位置伺服**（`POSITION_CONTROL` 带分关节力矩上限与速度前馈跟踪轨迹；动力学含 URDF 惯量与关节阻尼，**无重力设定**，见 §0 第 6 条）。逐点记录两个误差：IK 数值残差（静态）与伺服跟踪误差（无重力设定下控制回路的末端定位精度）。动画另含 §0 第 7 条的**演示层抓放循环**，用于呈现采摘流程，但不参与上述任何测量与判定。

### 1.2 误差定义（原理、脚本、结果表、答辩四处统一，不得混用）

- **末端 link 名 = `link6`**（xarm_ros 官方 xacro 中 `link_eef` 整段被注释，名义末端即 link6）；
- **到位误差 = 目标点到 link6 原点（URDF link frame 原点）的欧氏距离**；
- 代码实现：`getLinkState(robot, 5, computeForwardKinematics=1)` 返回元组的 **[4] `worldLinkFramePosition`**（主口径，与 IK 的末端参考系一致）；**[0] `linkWorldPosition` 是质心（COM）位置，仅作备查列记录**，不得作为主判据（两者相差约 1.3 cm，混用会制造虚假误差）。

### 1.3 可达性判定（两重静态判定 + 伺服跟踪判据）

一个目标点"到达"当且仅当以下全部成立：

| # | 判定 | 实现 |
|---|------|------|
| 1 | IK 解的 FK 残差 ≤ 阈值 | `resetJointState` 设置 IK 解后 `getLinkState(computeForwardKinematics=1)` 实测末端误差（`--threshold`，默认 0.005 m 可配） |
| 2 | 关节解在限位内 | 6 个关节角全部落在 URDF 限位区间（容差 1e-6） |
| 3 | 伺服跟踪判据 | 位置伺服执行完轨迹后，末端误差连续 60 步（0.25 s @240 Hz）达标；伺服步数上限只是超时兜底，不是收敛判据 |

说明：第 1、2 项是**静态数值判定**（求解层面），第 3 项是**控制回路判定**（执行层面）。不设所谓"独立复核"项——PyBullet 的 `calculateInverseKinematics` 不返回残差，FK 残差只能自设关节角后测量，同一件事测两次不构成独立验证。

水平半径（≤ 0.70 m）与 z 范围（[0.10, 0.75]）**仅作 IK 前预筛**（快速失败），不构成可达性证明。`--check-out-of-range` 负向模式只验证"脚本会拒绝越界点"，其输出明示该语义，不冒充可达性证据。

### 1.4 验收产物

1. `results/reach_results.csv`：双误差列（`err_ik` 静态残差 / `err_track` 伺服跟踪误差）+ COM 备查列 + 限位判定 + 伺服收敛步数 + 渲染耗时（`render_time_s` 为全局渲染耗时，各行同值；**v7 实跑 = 133.319 s**）；仍为 5 行、20 列，演示层不写入；
2. `results/reach_demo.mp4`：机械臂依次到达 5 个草莓点、并在每点后完成「抓 → 放入篮子 → 前往下一点」演示段的动画（**v5 实跑 976 帧 / 32.53 s**）；
3. `results/scene.png`：终态场景截图（篮中 5 颗果实可见）；
4. `results/reach_demo.gif` / `results/reach_keyframes.png`：由同一次运行派生的答辩素材（循环动图 / 关键帧拼图）；
5. 控制台汇总表（含演示层抓放段计数与边界声明）；
6. 未收敛/超阈值的点**如实标注并保留在结果表中**（这本身是合法的实验结论），按 §9.2 规则化调整后重跑并登记。

---

## 2. 原理速览（答辩口径级）

### 2.1 位置-only 逆运动学

6 轴机械臂末端位置由 6 个关节角决定；只约束末端 **位置**（3 个自由度）而不约束姿态（另 3 个自由度）时，方程数 3 < 未知数 6，为**欠定问题**，解不唯一。PyBullet 的 `calculateInverseKinematics` 在显式传入 `lowerLimits / upperLimits / jointRanges / restPoses` 时采用 **nullspace（零空间）法**：在满足位置约束的解流形中，选取最接近 `restPoses`（参考姿态）的解。因此：

- **解由限位 + restPoses + 初值共同决定**——三者在脚本中全部写死，保证他人复跑得到同一组关节角（可复现性）；
- 每次求解前 `resetJointState` 至固定 HOME 初值，消除初值依赖。

### 2.2 URDF 运动学来源（关键工程事实）

xarm_ros 官方仓库**不含现成 .urdf 文件**（全仓 `*.urdf` 数 = 0，只有 .xacro；展开依赖 ROS 工具链的 `$(find ...)` 与 `load_yaml`，无 ROS 环境无法生成）。因此本实验**手写 `xarm6_reach.urdf`**，所有数值抄自仓库 config 文件（逐项来源见附录 B）：

- 关节原点/姿态 ← `config/kinematics/default/xarm6_default_kinematics.yaml`
- 连杆惯量 ← `config/link_inertial/xarm6_type6_HT_BR2.yaml`（link_base 取 xacro 内联值）
- 关节限位 ← `xarm6.urdf.xacro` 宏默认值
- 力矩上限 ← `config/xarm6/joint_limits.yaml`（分关节，非一刀切）
- 外观 mesh ← 仓库 7 个 STL（**弃用 .dae**：TINY 渲染器兼容性风险）

### 2.3 零位 FK 理论值（用于加载自检）

所有关节角 = 0 时，对 6 个关节 origin 做逐级链式坐标变换。各级旋转复合（`Rx(θ)` 为绕 x 轴旋转）：

| 复合到该关节的累计旋转 | 推导 |
|---|---|
| R₁ = I | joint1 rpy = 0 |
| R₁₂ = Rx(-90°) | joint2 rpy = (-1.5708, 0, 0) |
| R₁₂₃ = Rx(-90°) | joint3 rpy = 0，复合不变 |
| R₁₂₃₄ = Rx(-90°)·Rx(-90°) = Rx(-180°) | joint4 rpy = (-1.5708, 0, 0) |
| R₁₂₃₄₅ = Rx(-180°)·Rx(+90°) = Rx(-90°) | joint5 rpy = (+1.5708, 0, 0)，**+90° 与前面的 -180° 复合后净剩 -90°** |
| R₁₂₃₄₅₆ = Rx(-90°)·Rx(-90°) = Rx(-180°) | joint6 rpy = (-1.5708, 0, 0)，再叠一个 -90°；但 v₃ 在 joint6 origin 处、按父 link（link5）的累计旋转 R₁₂₃₄₅ = Rx(-90°) 变换，joint6 自身旋转只影响末端姿态、不影响 link6 原点位置 |

三个平移向量按各自父 link 的累计旋转变换到世界系（`Rx(-90°)·(x,y,z) = (x, z, -y)`）：

```
v₁ = (0.0535,-0.2845, 0)（joint3 origin，link2 系）→ Rx(-90°) → (0.0535, 0, 0.2845)
v₂ = (0.0775, 0.3425, 0)（joint4 origin，link3 系）→ Rx(-90°) → (0.0775, 0,-0.3425)
v₃ = (0.076,  0.097,  0)（joint6 origin，link5 系）→ Rx(-90°) → (0.076,  0,-0.097)

p = (0,0,0.267) + v₁ + v₂ + v₃ = (0.207, 0, 0.112)  单位 m
```

注：URDF 中 rpy=-1.5708 是 -90° 的截断值（cos ≈ 3.7×10⁻⁶），实算会带 ~10⁻⁶ m 量级的 y 向偏差，属正常现象。`test_load.py` 以 (0.207, 0, 0.112) 断言（容差 ±5 mm）——这是**自洽性自检**（验证推导与 URDF 文件互洽、加载链路无损坏），"与官方参数一致"由附录 B 的逐项人工核对保证，脚本本身不读官方 config。

### 2.4 两层控制结构：轨迹规划 + 关节位置伺服

本实验的控制结构对应课设题目"控制系统设计"的最小完整回路：

1. **轨迹规划层**：对每个目标点求 IK 得 q*，从当前关节角到 q* 生成**归一化梯形速度插值**目标轨迹（加速 25% / 匀速 50% / 减速 25%，480 步 @240 Hz = 2 s）；
2. **关节伺服层**：`setJointMotorControlArray(POSITION_CONTROL)` 按分关节力矩上限（附录 B）逐仿真步跟踪目标轨迹，轨迹段同时给 `targetVelocities` 速度前馈（消除纯位置 PD 的跟踪滞后）；`stepSimulation()` 推进动力学（含 URDF 惯量与关节阻尼；**无重力设定**，见 §0 第 6 条——重力下纯位置伺服存在稳态下垂误差，需逆动力学前馈补偿，属实物化后续工作）；
3. **反馈测量**：伺服过程中/结束后用 `getLinkState(computeForwardKinematics=1)` 测末端实际位置——**伺服跟踪误差 `err_track` 才是"控制系统定位精度"**，IK 残差 `err_ik` 只是求解器的数值质量。

伺服收敛判据：到位后保持段误差**连续 60 步（0.25 s）≤ 阈值**；伺服总步数上限（超时兜底）为 3000 步（240 Hz 下 12.5 s），超时≠收敛。增益 positionGain=1.0 / velocityGain=1.0（无重力设定下的验证基线，依据见附录 B；调整指引见附录 C）。

---

## 3. 环境搭建

### 步骤一：进入 WSL2 并确认

在 PowerShell 执行：

```powershell
wsl -d Ubuntu
```

进入 Ubuntu 后确认（预期输出示例跟在 `#` 后）：

```bash
cat /etc/os-release | head -2   # Ubuntu 26.04 ...
whoami                          # 你的用户名（下文 ~/ 即其家目录）
```

### 步骤二：安装 Miniconda（已装可跳过）

```bash
# 检测是否已装：输出路径则跳过本步
command -v conda || echo "NOT_INSTALLED"
```

未安装时执行：

```bash
wget https://mirrors.tuna.tsinghua.edu.cn/anaconda/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh
bash ~/miniconda.sh -b -p ~/miniconda3
~/miniconda3/bin/conda init bash
exec bash
command -v conda   # 预期输出：/home/<你>/miniconda3/bin/conda 或 condabin 路径
```

### 步骤三：创建 robot 环境（Python 3.11）并装依赖

> **为什么锁 3.11**：pybullet 预编译 wheel 实测仅覆盖 cp36–cp311（3.2.6 / 3.2.7 均有 `cp311-manylinux_2_17_x86_64` wheel，无 cp312+）。用 3.12/3.13/3.14 会意外触发源码编译。

```bash
conda create -n robot python=3.11 -y
conda activate robot
python -V    # 预期输出：Python 3.11.x
```

安装依赖（**正常路径**：wheel 直接命中，无需任何编译器，全程 <1 分钟）：

```bash
pip install -i https://pypi.tuna.tsinghua.edu.cn/simple pybullet==3.2.7 numpy imageio imageio-ffmpeg
```

验证：

```bash
python -c "import pybullet as p; print('pybullet OK, API', p.getAPIVersion())"
python -c "import numpy; print('numpy', numpy.__version__)"
python -c "import imageio; print('imageio', imageio.__version__)"
python -c "import imageio_ffmpeg; print(imageio_ffmpeg.get_ffmpeg_exe())"
```

预期：四条均无报错；最后一条打印 ffmpeg 可执行文件路径。**实测记录（2026-11-04）**：pybullet = **3.2.7**（cp311 manylinux wheel，103.2 MB，下载 6 分 31 秒 @ ~247 kB/s，build time Jan 29 2025）；numpy 2.4.6 / imageio 2.38.1 / imageio-ffmpeg 0.8.1 / pillow 13.0.0；`IMPORTS_OK 202010061 2.4.6 2.38.1`。详见《运行日志_20261104.md》Node 1。

**异常路径（仅当 wheel 拉取失败才走，正常情况跳过）**：源码编译需要完整编译链，耗时 5–10 分钟属正常现象，不是卡死：

```bash
sudo apt update
sudo apt install -y build-essential cmake python3-dev
pip install -i https://pypi.tuna.tsinghua.edu.cn/simple --no-binary pybullet pybullet==3.2.7
```

若编译失败，按排错表 #3 处理。

---

## 4. 项目目录与资产准备

### 步骤四：建目录、拷 STL

> **为什么项目驻 `~/xarm_lab` 而不在 Windows 盘**：工作区 Windows 路径含中文（`/mnt/d/桌面/...`），PyBullet 的 C++ URDF/mesh 加载器对非 ASCII 路径支持不稳。产物最终拷回 Windows（§10）。

```bash
mkdir -p ~/xarm_lab/meshes/xarm6/visual ~/xarm_lab/results
SRC="/mnt/d/桌面/CRAIC/success/202607091551/temp_20261104_01/assets/xarm_ros_extract/xarm_ros-master/xarm_description/meshes/xarm6/visual"
for f in link_base link1 link2 link3 link4 link5 link6; do cp "$SRC/$f.stl" ~/xarm_lab/meshes/xarm6/visual/; done
ls -l ~/xarm_lab/meshes/xarm6/visual/
```

预期输出：7 个 `.stl` 文件（link_base、link1–link6），各数百 KB。
注意：`link_base_white.dae` **有意不拷贝**（TINY 渲染器对 .dae 兼容性差，本实验统一用 .stl）。

最终目录结构（后两步补齐文件）：

```
~/xarm_lab/
├── xarm6_reach.urdf          # 步骤五创建
├── test_load.py              # 步骤六创建
├── xarm6_reach_demo.py       # 步骤七创建
├── meshes/xarm6/visual/*.stl # 7 个
└── results/                  # 运行产物
```

---

## 5. 手写 xarm6_reach.urdf

### 步骤五：创建 URDF 文件

```bash
cd ~/xarm_lab
cat > xarm6_reach.urdf << 'URDF_EOF'
<?xml version="1.0"?>
<robot name="xarm6_reach">
  <material name="white"><color rgba="0.92 0.92 0.92 1.0"/></material>
  <material name="dark"><color rgba="0.20 0.20 0.22 1.0"/></material>

  <!-- ============ links ============ -->
  <link name="link_base">
    <inertial>
      <origin xyz="0.0 0.0 0.09103" rpy="0 0 0"/>
      <mass value="2.7"/>
      <inertia ixx="0.00494875" ixy="-3.5e-06" ixz="1.25e-05" iyy="0.00494174" iyz="1.67e-06" izz="0.002219"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="meshes/xarm6/visual/link_base.stl"/></geometry>
      <material name="dark"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="meshes/xarm6/visual/link_base.stl"/></geometry>
    </collision>
  </link>

  <link name="link1">
    <inertial>
      <origin xyz="0.00022 0.02951 -0.0124" rpy="0 0 0"/>
      <mass value="2.3814"/>
      <inertia ixx="0.0058562" ixy="-1.79e-05" ixz="3.55e-06" iyy="0.0050316" iyz="-0.000888336" izz="0.003536652"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="meshes/xarm6/visual/link1.stl"/></geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="meshes/xarm6/visual/link1.stl"/></geometry>
    </collision>
  </link>

  <link name="link2">
    <inertial>
      <origin xyz="0.03881 -0.22783 0.03496" rpy="0 0 0"/>
      <mass value="2.2675"/>
      <inertia ixx="0.028315776" ixy="0.005" ixz="0.00066546" iyy="0.0058" iyz="-0.0045741" izz="0.0273447"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="meshes/xarm6/visual/link2.stl"/></geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="meshes/xarm6/visual/link2.stl"/></geometry>
    </collision>
  </link>

  <link name="link3">
    <inertial>
      <origin xyz="0.07041 0.11631 0.0107" rpy="0 0 0"/>
      <mass value="1.875"/>
      <inertia ixx="0.0063483" ixy="-0.0015397" ixz="0.00096858" iyy="0.00379758" iyz="0.00186567" izz="0.00595768"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="meshes/xarm6/visual/link3.stl"/></geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="meshes/xarm6/visual/link3.stl"/></geometry>
    </collision>
  </link>

  <link name="link4">
    <inertial>
      <origin xyz="-0.00018 0.01798 -0.02291" rpy="0 0 0"/>
      <mass value="1.3192"/>
      <inertia ixx="0.004896" ixy="-6.925e-06" ixz="-1.418e-05" iyy="0.00445694" iyz="-0.00023186" izz="0.00134332"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="meshes/xarm6/visual/link4.stl"/></geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="meshes/xarm6/visual/link4.stl"/></geometry>
    </collision>
  </link>

  <link name="link5">
    <inertial>
      <origin xyz="0.0651 0.03096 0.00315" rpy="0 0 0"/>
      <mass value="1.33854"/>
      <inertia ixx="0.00146378" ixy="-0.000450624" ixz="0.000284306" iyy="0.00184192" iyz="0.000130866" izz="0.002333524"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="meshes/xarm6/visual/link5.stl"/></geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="meshes/xarm6/visual/link5.stl"/></geometry>
    </collision>
  </link>

  <!-- 名义末端：本实验误差测量点（link6 原点） -->
  <link name="link6">
    <inertial>
      <origin xyz="0.0 -0.00677 -0.01098" rpy="0 0 0"/>
      <mass value="0.17"/>
      <inertia ixx="9.3e-05" ixy="0.0" ixz="0.0" iyy="5.87e-05" iyz="-3.6e-06" izz="0.000132"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="meshes/xarm6/visual/link6.stl"/></geometry>
      <material name="dark"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="meshes/xarm6/visual/link6.stl"/></geometry>
    </collision>
  </link>

  <!-- ============ joints（axis 均为局部 z 轴） ============ -->
  <joint name="joint1" type="revolute">
    <parent link="link_base"/>
    <child link="link1"/>
    <origin xyz="0 0 0.267" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-6.283185307179586" upper="6.283185307179586" effort="163.2" velocity="3.14"/>
    <dynamics damping="1.0" friction="1.0"/>
  </joint>

  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2.059" upper="2.0944" effort="163.2" velocity="3.14"/>
    <dynamics damping="1.0" friction="1.0"/>
  </joint>

  <joint name="joint3" type="revolute">
    <parent link="link2"/>
    <child link="link3"/>
    <origin xyz="0.0535 -0.2845 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.927" upper="0.19198" effort="81.6" velocity="3.14"/>
    <dynamics damping="1.0" friction="1.0"/>
  </joint>

  <joint name="joint4" type="revolute">
    <parent link="link3"/>
    <child link="link4"/>
    <origin xyz="0.0775 0.3425 0" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-6.283185307179586" upper="6.283185307179586" effort="81.6" velocity="3.14"/>
    <dynamics damping="1.0" friction="1.0"/>
  </joint>

  <joint name="joint5" type="revolute">
    <parent link="link4"/>
    <child link="link5"/>
    <origin xyz="0 0 0" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.69297" upper="3.141592653589793" effort="30.6" velocity="3.14"/>
    <dynamics damping="1.0" friction="1.0"/>
  </joint>

  <joint name="joint6" type="revolute">
    <parent link="link5"/>
    <child link="link6"/>
    <origin xyz="0.076 0.097 0" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-6.283185307179586" upper="6.283185307179586" effort="30.6" velocity="3.14"/>
    <dynamics damping="1.0" friction="1.0"/>
  </joint>
</robot>
URDF_EOF
echo "written: $(wc -l < xarm6_reach.urdf) lines"
```

预期输出：`written: ~200 lines`（行数以实际为准）。

mesh 采用**相对路径**（相对 URDF 文件所在目录），PyBullet 按此规则解析，无需改写绝对路径。

---

## 6. 模型加载自检（test_load.py）

### 步骤六：创建并运行自检脚本

```bash
cd ~/xarm_lab
cat > test_load.py << 'PY_EOF'
#!/usr/bin/env python3
"""test_load.py - xArm6 手写 URDF 加载自检。全部 PASS 才继续步骤七。"""
import math
import sys

import pybullet as p

EXPECT_LIMITS = [
    (-2.0 * math.pi, 2.0 * math.pi),
    (-2.059, 2.0944),
    (-3.927, 0.19198),
    (-2.0 * math.pi, 2.0 * math.pi),
    (-1.69297, math.pi),
    (-2.0 * math.pi, 2.0 * math.pi),
]
# 零位 link6 原点（指导书 2.3 节链式变换推导）
EXPECT_HOME_EE = (0.207, 0.0, 0.112)
TOL = 0.005


def main() -> int:
    ok = True
    p.connect(p.DIRECT)
    robot = p.loadURDF("xarm6_reach.urdf", useFixedBase=True)
    print(f"[1] loadURDF OK, bodyUniqueId = {robot}")

    n = p.getNumJoints(robot)
    print(f"[2] getNumJoints = {n}（期望 6）")
    ok &= n == 6

    print("[3] 关节限位读入检查（来源：xarm6.urdf.xacro 宏默认值）")
    for i in range(n):
        info = p.getJointInfo(robot, i)
        lo, hi = info[8], info[9]
        elo, ehi = EXPECT_LIMITS[i]
        good = abs(lo - elo) < 1e-6 and abs(hi - ehi) < 1e-6
        ok &= good
        print(f"    joint{i+1} ({info[1].decode():8s}) "
              f"limits=[{lo:+.6f},{hi:+.6f}] "
              f"expect=[{elo:+.6f},{ehi:+.6f}] {'PASS' if good else 'FAIL'}")

    print("[4] 末端 link 确认")
    link_names = [p.getJointInfo(robot, i)[12].decode() for i in range(n)]
    print(f"    link 序列: {link_names}")
    end_is_link6 = link_names[-1] == "link6"
    ok &= end_is_link6
    print(f"    末端 link 名 = {link_names[-1]}（期望 link6，index = {n-1}）"
          f" {'PASS' if end_is_link6 else 'FAIL'}")

    print("[5] 零位 FK 断言（理论值 (0.207, 0, 0.112) m，容差 5 mm）")
    for i in range(n):
        p.resetJointState(robot, i, 0.0)
    ls = p.getLinkState(robot, n - 1, computeForwardKinematics=1)
    ee_frame = ls[4]   # worldLinkFramePosition：URDF link 原点（主口径）
    ee_com = ls[0]     # linkWorldPosition：COM（备查口径）
    err = math.dist(ee_frame, EXPECT_HOME_EE)
    ok &= err <= TOL
    print(f"    link6 原点 = ({ee_frame[0]:.4f}, {ee_frame[1]:.4f}, {ee_frame[2]:.4f})，"
          f"|Δ| = {err * 1000:.2f} mm {'PASS' if err <= TOL else 'FAIL'}")
    print(f"    link6 COM  = ({ee_com[0]:.4f}, {ee_com[1]:.4f}, {ee_com[2]:.4f})（备查，与原点相差属正常）")

    print("[6] 渲染一帧（ER_TINY_RENDERER，验证 STL 显示）")
    view = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=(0.25, 0.0, 0.25), distance=1.6,
        yaw=50, pitch=-25, roll=0, upAxisIndex=2)
    proj = p.computeProjectionMatrixFOV(fov=50, aspect=4 / 3, nearVal=0.01, farVal=10.0)
    img = p.getCameraImage(320, 240, viewMatrix=view, projectionMatrix=proj,
                           renderer=p.ER_TINY_RENDERER)
    print(f"    帧缓冲 {img[2].nbytes} bytes（= 320×240×4，非零即渲染通路正常）")
    ok &= img[2].nbytes > 0

    p.disconnect()
    print("== ALL PASS ==" if ok else "== FAILED（见上方 FAIL 项）==")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
PY_EOF
python test_load.py
```

预期输出（数值以实跑为准）：

```
[1] loadURDF OK, bodyUniqueId = 0
[2] getNumJoints = 6（期望 6）
[3] 关节限位读入检查（来源：xarm6.urdf.xacro 宏默认值）
    joint1 (joint1 ) limits=[-6.283185,+6.283185] expect=[-6.283185,+6.283185] PASS
    ...（6 行全 PASS）
[4] 末端 link 确认
    link 序列: ['link1', 'link2', 'link3', 'link4', 'link5', 'link6']
    末端 link 名 = link6（期望 link6，index = 5） PASS
[5] 零位 FK 断言（理论值 (0.207, 0, 0.112) m，容差 5 mm）
    link6 原点 = (0.2070, -0.0000, 0.1120)，|Δ| = 0.00 mm PASS
    link6 COM  = (...)（备查，与原点相差属正常）
[6] 渲染一帧（ER_TINY_RENDERER，验证 STL 显示）
    帧缓冲 307200 bytes（= 320×240×4，非零即渲染通路正常）
== ALL PASS ==
```

任一 FAIL：按排错表 #4/#5 处理，**不得进入步骤七**。

---

## 7. 主实验脚本（xarm6_reach_demo.py）

### 步骤七：创建主脚本

```bash
cd ~/xarm_lab
cat > xarm6_reach_demo.py << 'PY_EOF'
#!/usr/bin/env python3
"""
xarm6_reach_demo.py - xArm6 草莓定点到达仿真（位置-only IK + 梯形轨迹 + 位置伺服执行）。

产物：results/reach_results.csv、results/reach_demo.mp4、results/scene.png、控制台汇总表
退出码：0 = 5 点全部通过（两重静态判定 + 伺服跟踪）；1 = 存在未达标点（结果如实保留，按指导书 §9.2 处理）；
        2 = 负向验证模式（--check-out-of-range，仅验证预筛拒绝逻辑）
"""
import argparse
import csv
import math
import os
import time

import imageio
import numpy as np
import pybullet as p
import pybullet_data

# ---------------- 固定参数（数值来源见指导书附录 B，禁止随手改） ----------------
HOME = [0.0, -0.5, -1.0, 0.0, 0.5, 0.0]            # IK 初值 = nullspace 参考姿态 = 动画起始
LIMITS = [(-2 * math.pi, 2 * math.pi), (-2.059, 2.0944), (-3.927, 0.19198),
          (-2 * math.pi, 2 * math.pi), (-1.69297, math.pi), (-2 * math.pi, 2 * math.pi)]
FORCES = [163.2, 163.2, 81.6, 81.6, 30.6, 30.6]    # N·m，joint_limits.yaml max_effort（含 2.55 过载系数）
TARGETS = [                                          # 与 plan/spec 同源（已独立矩阵实算预核可达）
    ("P1", (0.45, 0.00, 0.40)),
    ("P2", (0.48, 0.20, 0.35)),
    ("P3", (0.48, -0.20, 0.35)),
    ("P4", (0.35, 0.12, 0.50)),
    ("P5", (0.35, -0.12, 0.50)),
]
RADIUS_MAX, Z_MIN, Z_MAX = 0.70, 0.10, 0.75        # IK 前预筛（快速失败，非可达性证明）
IK_MAX_ITER, IK_TOL = 100, 1e-5                    # IK 迭代上限与残差阈值
SIM_HZ = 240                                        # 仿真步频（setTimeStep 真实生效）
ANIM_STEPS, HOLD_STEPS = 480, 480                  # 轨迹段 2 s + 到位保持段 2 s（含收敛观察窗）
STEADY_N = 60                                       # 收敛判据：连续 60 步（0.25 s）达标
SERVO_MAX_STEPS = 3000                              # 每点伺服总步数上限（超时兜底 12.5 s，非收敛判据）
POS_GAIN, VEL_GAIN = 1.0, 1.0                      # 伺服增益（无重力设定下的验证基线，依据见附录 B/C）
BAD_POINT_H = (1.20, 0.00, 0.30)                   # 负向验证：水平越界（1.2 m > 0.70 m）
BAD_POINT_Z = (0.40, 0.00, 0.90)                   # 负向验证：z 越界（0.90 m > 0.75 m）

# ---------------- 演示层抓放循环（视觉示意，不参与误差判定；来源见指导书附录 B） ----------------
BASKET_CENTER = (0.25, -0.30, 0.10)                # 篮子中心（m），基座右前方近地面
BASKET_DROP = (0.25, -0.30, 0.20)                  # 放果点：末端下降示意「低头放果」（位置-only IK，不约束姿态）
BASKET_HALF = (0.09, 0.09, 0.055)                  # 篮子内腔半尺寸（m）
PLACE_STEPS, PLACE_HOLD = 480, 120                  # 放果轨迹 2 s + 入篮保持 0.5 s（@240 Hz）
FRUIT_SLOTS = [                                     # 5 颗果实在篮内的留存位置（m）
    (0.21, -0.34, 0.135), (0.29, -0.34, 0.135), (0.25, -0.30, 0.135),
    (0.21, -0.26, 0.135), (0.29, -0.26, 0.135),
]
FRUIT_HELD_Z = 0.03                                # 搬运时果实相对 link6 原点的下方偏移（m）

# ---- 演示层仿生触手：卷曲 -> 反向展开 -> 缠绕包覆（L1 纯视觉，不参与误差判定与物理） ----
# 参数按 SpiRobs 官方「数根触手抓鸡蛋」演示的比例取一组示意几何值；
# **非中科大实物参数、非论文测量值**；rho = a*exp(b*theta) 仅用于示意卷曲形态，来源见附录 B。
TENTACLE_COUNT = 3                                 # 触手根数（示意）
TENTACLE_SEGS = 6                                  # 每根离散段数（示意）
TENTACLE_LEN = 0.13                                # 展开态触手长度 (m，示意)
TENTACLE_RADIUS = 0.008                            # 段球半径 (m，示意)
TENTACLE_ROOT_R = 0.022                            # 根部离轴半径 (m，示意)
TENTACLE_OPEN_R = 0.065                            # 展开态中段离轴半径 (m，示意)
TENTACLE_COLOR = [0.90, 0.62, 0.20, 1.0]           # 触手颜色（示意）
# 卷曲态：对数螺旋 rho = SPIRAL_A*exp(SPIRAL_B*theta)，盘在末端后方 (+z)，不遮挡果实
SPIRAL_A = 0.010                                   # 螺旋起始半径 (m，示意)
SPIRAL_B = 0.22                                    # 螺旋增长系数 (示意)
SPIRAL_THETA_MAX = 2.4 * math.pi                   # 卷曲总转角 (rad，示意，约 1.2 圈)
COIL_BACK_Z = 0.050                                # 卷曲态向末端后方抬升 (m，示意)
# 缠绕态：沿果实球面（球心在 ee 下方 FRUIT_HELD_Z）自球顶绕到球底
WRAP_R = 0.033                                     # 包覆球面半径 (m，略大于果实半径 0.025)
WRAP_PHI0, WRAP_PHI1 = 0.18 * math.pi, 0.96 * math.pi   # 球顶到球底极角范围 (示意)
WRAP_TWIST = 0.55 * math.pi                        # 缠绕时绕球轴的附加转角 (示意)
# 阶段时间线：到位段后段展开，保持段前段缠绕，入篮段后段释放回卷
EXTEND_START = 0.75                                # 到位轨迹段中开始反向展开的归一化时刻
WRAP_END = 0.30                                    # 保持段中完成缠绕包覆的归一化时刻
RELEASE_START = 0.70                               # 入篮保持段中开始释放回卷的归一化时刻


def precheck(point, name):
    """预筛：水平半径与高度。越界即退出码 2（在 IK 前快速失败）。"""
    rho = math.hypot(point[0], point[1])
    if rho > RADIUS_MAX:
        print(f"[预筛拒绝] {name} 水平距离 {rho:.3f} m > {RADIUS_MAX} m，IK 前中止。")
        print("            （--check-out-of-range 模式仅验证拒绝逻辑，不构成可达性结论）")
        raise SystemExit(2)
    if not (Z_MIN <= point[2] <= Z_MAX):
        print(f"[预筛拒绝] {name} z={point[2]:.3f} m 不在 [{Z_MIN},{Z_MAX}]，IK 前中止。")
        raise SystemExit(2)


def solve_ik(robot, target):
    """全参数位置-only IK：固定 HOME 初值 → 显式限位/jointRanges/restPoses → 可复现解。"""
    for i in range(6):
        p.resetJointState(robot, i, HOME[i])
    q = p.calculateInverseKinematics(
        robot, 5, target,
        lowerLimits=[lo for lo, _ in LIMITS],
        upperLimits=[hi for _, hi in LIMITS],
        jointRanges=[hi - lo for lo, hi in LIMITS],
        restPoses=HOME,
        maxNumIterations=IK_MAX_ITER,
        residualThreshold=IK_TOL,
    )
    return list(q[:6])


def check_limits(q):
    bad = [f"joint{i+1}={q[i]:.4f} 不在 [{LIMITS[i][0]:+.4f},{LIMITS[i][1]:+.4f}]"
           for i in range(6) if not (LIMITS[i][0] - 1e-6 <= q[i] <= LIMITS[i][1] + 1e-6)]
    return len(bad) == 0, bad


def fk_ee(robot):
    """FK 复核：返回 (link6 原点, link6 COM)。主口径为原点（URDF frame，与 IK 参考系一致）。"""
    ls = p.getLinkState(robot, 5, computeForwardKinematics=1)
    return ls[4], ls[0]


def trap_profile(n):
    """归一化梯形位移序列 s(t)∈[0,1]：加速 25% / 匀速 50% / 减速 25%。峰值速度 1/(1-a)。"""
    a, out = 0.25, []
    vmax = 1.0 / (1.0 - a)
    for k in range(1, n + 1):
        t = k / n
        if t < a:                    # 加速段
            s = vmax * t * t / (2.0 * a)
        elif t <= 1.0 - a:           # 匀速段
            s = vmax * a / 2.0 + vmax * (t - a)
        else:                        # 减速段
            u = 1.0 - t
            s = 1.0 - vmax * u * u / (2.0 * a)
        out.append(min(max(s, 0.0), 1.0))
    return out


def main():
    ap = argparse.ArgumentParser(description="xArm6 草莓定点到达仿真")
    ap.add_argument("--threshold", type=float, default=0.005, help="到位误差阈值(m)，默认 5 mm，可配")
    ap.add_argument("--res", default="960x720", help="渲染分辨率 WxH（降级用 640x480）")
    ap.add_argument("--every", type=int, default=8, help="每 N 仿真步渲染一帧（降级用 16）")
    ap.add_argument("--no-video", action="store_true", help="跳过视频导出（只出 CSV/截图，快速验证）")
    ap.add_argument("--check-out-of-range", action="store_true",
                    help="负向验证：注入越界点，确认脚本在 IK 前拒绝（退出码 2）")
    args = ap.parse_args()

    if args.check_out_of_range:
        for pt, nm in ((BAD_POINT_H, "P_BAD_H"), (BAD_POINT_Z, "P_BAD_Z")):
            rho = math.hypot(pt[0], pt[1])
            if rho > RADIUS_MAX:
                print(f"[预筛拒绝] {nm} 水平距离 {rho:.3f} m > {RADIUS_MAX} m，IK 前中止。")
            elif not (Z_MIN <= pt[2] <= Z_MAX):
                print(f"[预筛拒绝] {nm} z={pt[2]:.3f} m 不在 [{Z_MIN},{Z_MAX}]，IK 前中止。")
        print("（--check-out-of-range 仅验证拒绝逻辑，不构成可达性结论）")
        raise SystemExit(2)

    w, h = (int(x) for x in args.res.lower().split("x"))
    os.makedirs("results", exist_ok=True)

    p.connect(p.DIRECT)
    p.setTimeStep(1.0 / SIM_HZ)                       # 240 Hz 仿真步长
    p.setGravity(0, 0, 0)                             # 无重力设定（§0 第 6 条）：重力前馈补偿属后续工作
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")                          # 地面
    # 栽培架示意（绿色薄板，仅视觉）
    board = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.5, 0.35, 0.005],
                                rgbaColor=[0.25, 0.55, 0.25, 1.0])
    p.createMultiBody(baseMass=0.0, baseVisualShapeIndex=board,
                      basePosition=[0.42, 0.0, 0.0005])
    robot = p.loadURDF("xarm6_reach.urdf", useFixedBase=True)

    # 5 个草莓目标点（红球标记）
    ball_ids = {}
    ball_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.025,
                                     rgbaColor=[0.85, 0.10, 0.10, 1.0])
    for name, t in TARGETS:
        ball_ids[name] = p.createMultiBody(baseMass=0.0, baseVisualShapeIndex=ball_shape,
                                           basePosition=list(t))

    # 篮子（纯视觉静态道具：底板 + 4 壁，无碰撞、无质量、不参与物理）
    def add_box(half, pos, color):
        shape = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=color)
        return p.createMultiBody(baseMass=0.0, baseVisualShapeIndex=shape,
                                 basePosition=list(pos))

    cx, cy, cz = BASKET_CENTER
    hx, hy, bh = BASKET_HALF
    wall, basket_rgba = 0.006, [0.55, 0.38, 0.20, 0.9]
    add_box((hx, hy, wall / 2), (cx, cy, cz), basket_rgba)
    add_box((wall / 2, hy, bh / 2), (cx - hx, cy, cz + bh / 2), basket_rgba)
    add_box((wall / 2, hy, bh / 2), (cx + hx, cy, cz + bh / 2), basket_rgba)
    add_box((hx, wall / 2, bh / 2), (cx, cy - hy, cz + bh / 2), basket_rgba)
    add_box((hx, wall / 2, bh / 2), (cx, cy + hy, cz + bh / 2), basket_rgba)

    # 演示层仿生触手：3 根 x 6 段纯视觉体（无质量、无碰撞、不参与物理与误差测量）
    tentacle_shape = p.createVisualShape(p.GEOM_SPHERE, radius=TENTACLE_RADIUS,
                                         rgbaColor=TENTACLE_COLOR)
    tentacle_ids = []
    for _ in range(TENTACLE_COUNT * TENTACLE_SEGS):
        tentacle_ids.append(p.createMultiBody(baseMass=0.0,
                                              baseVisualShapeIndex=tentacle_shape,
                                              basePosition=[0.0, 0.0, -1.0]))
    tentacle_t = 0.0                                 # 0=卷曲待机; 0~0.5 反向展开; 0.5~1 缠绕包覆; 1=夹住

    def update_tentacles(t):
        """三阶段形态：向后卷曲(对数螺旋) -> 反向展开 -> 球面缠绕包覆果实。

        t in [0,1]：0 = 卷曲待机；0~0.5 由卷曲插值到展开；0.5~1 由展开插值到缠绕。
        全程使用世界坐标（相对 link6 原点 ee），不随末端姿态旋转，确保与果实的
        视觉搬运位置 [ee - FRUIT_HELD_Z] 对齐。纯视觉，不参与误差判定。
        """
        def smooth(x):
            x = min(1.0, max(0.0, x))
            return x * x * (3.0 - 2.0 * x)

        ee = p.getLinkState(robot, 5, computeForwardKinematics=1)[4]
        ca = [math.cos(2.0 * math.pi * i / TENTACLE_COUNT) for i in range(TENTACLE_COUNT)]
        sa = [math.sin(2.0 * math.pi * i / TENTACLE_COUNT) for i in range(TENTACLE_COUNT)]
        a_ext = smooth(2.0 * t)                     # 卷曲 -> 展开
        a_wrap = smooth(2.0 * t - 1.0)              # 展开 -> 缠绕
        for j in range(TENTACLE_SEGS):
            u = (j + 1) / TENTACLE_SEGS
            # (1) 卷曲态：rho = a*exp(b*theta)，盘在末端后方 (+z)，不遮挡果实
            theta = SPIRAL_THETA_MAX * u
            rho = SPIRAL_A * math.exp(SPIRAL_B * theta)
            z_coil = COIL_BACK_Z * (1.0 - u)
            # (2) 展开态：向果实方向反向伸展的近直线
            r_ext = TENTACLE_ROOT_R + (TENTACLE_OPEN_R - TENTACLE_ROOT_R) * u
            z_ext = -0.004 - TENTACLE_LEN * u
            # (3) 缠绕态：沿果实球面自球顶绕到球底
            phi = WRAP_PHI0 + (WRAP_PHI1 - WRAP_PHI0) * u
            rr_wrap = WRAP_R * math.sin(phi)
            z_wrap = -FRUIT_HELD_Z + WRAP_R * math.cos(phi)
            for i in range(TENTACLE_COUNT):
                ang_c = theta + 2.0 * math.pi * i / TENTACLE_COUNT
                x_coil, y_coil = rho * math.cos(ang_c), rho * math.sin(ang_c)
                x_ext, y_ext = r_ext * ca[i], r_ext * sa[i]
                ang_w = 2.0 * math.pi * i / TENTACLE_COUNT + WRAP_TWIST * u
                x_wrap, y_wrap = rr_wrap * math.cos(ang_w), rr_wrap * math.sin(ang_w)
                x1 = x_coil + (x_ext - x_coil) * a_ext
                y1 = y_coil + (y_ext - y_coil) * a_ext
                z1 = z_coil + (z_ext - z_coil) * a_ext
                x = x1 + (x_wrap - x1) * a_wrap
                y = y1 + (y_wrap - y1) * a_wrap
                z = z1 + (z_wrap - z1) * a_wrap
                wp = [ee[0] + x, ee[1] + y, ee[2] + z]
                p.resetBasePositionAndOrientation(tentacle_ids[i * TENTACLE_SEGS + j],
                                                  wp, [0.0, 0.0, 0.0, 1.0])

    view = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=(0.25, 0.0, 0.25), distance=1.6,
        yaw=50, pitch=-25, roll=0, upAxisIndex=2)
    proj = p.computeProjectionMatrixFOV(fov=50, aspect=w / h, nearVal=0.01, farVal=10.0)

    writer = None
    n_frames = 0
    t_render = 0.0
    if not args.no_video:
        writer = imageio.get_writer("results/reach_demo.mp4", fps=30,
                                    codec="libx264", quality=8, macro_block_size=1)

    def shoot(force=False):
        """渲染一帧并写盘。计时覆盖 getCameraImage（CPU 软渲染大头）+ 写盘。
        --no-video（writer=None）时跳过常规帧，仅 force=True（终态截图）渲染一帧。"""
        nonlocal n_frames, t_render
        if writer is None and not force:
            return None
        update_tentacles(tentacle_t)                  # 渲染前更新触手三阶段形态
        t0 = time.perf_counter()
        img = p.getCameraImage(w, h, viewMatrix=view, projectionMatrix=proj,
                               renderer=p.ER_TINY_RENDERER)
        rgba = np.reshape(np.asarray(img[2]), (h, w, 4))
        frame = rgba[:, :, :3].astype(np.uint8)
        if writer is not None:
            writer.append_data(frame)
        t_render += time.perf_counter() - t0
        n_frames += 1
        return frame

    def place_into_basket(name, q_start, slot_idx):
        """演示层：果实随末端搬到篮子、末端下降放果、果实入篮留存。

        不写 rows、不参与误差判定、不改阈值与限位；--no-video 下伺服与视觉搬运仍完整执行。
        """
        nonlocal tentacle_t
        q_drop = solve_ik(robot, BASKET_DROP)
        for j in range(6):
            p.resetJointState(robot, j, q_drop[j])
        drop_err = math.dist(fk_ee(robot)[0], BASKET_DROP)
        drop_in_lim, _ = check_limits(q_drop)
        for j in range(6):                             # 放果段起始 = 当前草莓点伺服后的姿态
            p.resetJointState(robot, j, q_start[j])
        dt = 1.0 / SIM_HZ
        q_prev = list(q_start)
        for k, s in enumerate(trap_profile(PLACE_STEPS)):
            q_cmd = [q_start[j] + s * (q_drop[j] - q_start[j]) for j in range(6)]
            v_cmd = [0.0] * 6 if k == 0 else [(q_cmd[j] - q_prev[j]) / dt for j in range(6)]
            p.setJointMotorControlArray(robot, range(6), p.POSITION_CONTROL,
                                        targetPositions=q_cmd, targetVelocities=v_cmd,
                                        forces=FORCES, positionGains=[POS_GAIN] * 6,
                                        velocityGains=[VEL_GAIN] * 6)
            p.stepSimulation()
            ee = p.getLinkState(robot, 5, computeForwardKinematics=1)[4]
            p.resetBasePositionAndOrientation(ball_ids[name],
                                              [ee[0], ee[1], ee[2] - FRUIT_HELD_Z],
                                              [0.0, 0.0, 0.0, 1.0])
            q_prev = list(q_cmd)
            if writer is not None and k % args.every == 0:
                tentacle_t = 1.0                     # 搬运中保持缠绕包覆夹住
                shoot()
        for k in range(PLACE_HOLD):                    # 入篮保持段：末端停在放果点，果实随末端下降
            p.setJointMotorControlArray(robot, range(6), p.POSITION_CONTROL,
                                        targetPositions=q_drop, targetVelocities=[0.0] * 6,
                                        forces=FORCES, positionGains=[POS_GAIN] * 6,
                                        velocityGains=[VEL_GAIN] * 6)
            p.stepSimulation()
            ee = p.getLinkState(robot, 5, computeForwardKinematics=1)[4]
            p.resetBasePositionAndOrientation(ball_ids[name],
                                              [ee[0], ee[1], ee[2] - FRUIT_HELD_Z],
                                              [0.0, 0.0, 0.0, 1.0])
            if writer is not None and k % args.every == 0:
                tentacle_t = 1.0 - min(1.0, max(0.0, (k / PLACE_HOLD - RELEASE_START) / (1.0 - RELEASE_START)))
                shoot()
        p.resetBasePositionAndOrientation(ball_ids[name], list(FRUIT_SLOTS[slot_idx]),
                                          [0.0, 0.0, 0.0, 1.0])   # 果实入篮并留存
        p.changeVisualShape(ball_ids[name], -1, rgbaColor=[0.75, 0.12, 0.12, 1.0])
        tentacle_t = 0.0                                # 放果后回到卷曲待机，准备下一颗
        q_end = [p.getJointState(robot, j)[0] for j in range(6)]
        return q_end, drop_err, drop_in_lim

    rows, q_cur, place_checks = [], list(HOME), []
    for i in range(6):                                 # 伺服起始 = HOME
        p.resetJointState(robot, i, HOME[i])
    t_total0 = time.perf_counter()

    print(f"{'点':4s} {'目标(m)':22s} {'IK残差(mm)':>11s} {'伺服误差(mm)':>12s} "
          f"{'COM(mm)':>9s} {'限位':4s} {'收敛步':>6s} {'结论':6s}")
    for name, target in TARGETS:
        precheck(target, name)

        # ---- 静态判定：IK 求解 + FK 残差 + 限位 ----
        q = solve_ik(robot, target)
        for j in range(6):                             # 设置 IK 解测静态残差
            p.resetJointState(robot, j, q[j])
        ee_frame, ee_com = fk_ee(robot)
        err_ik = math.dist(ee_frame, target)
        err_com = math.dist(ee_com, target)
        in_lim, bad = check_limits(q)

        # ---- 伺服执行：回当前姿态 → 梯形轨迹跟踪 → 收敛观察 → 超时兜底 ----
        for j in range(6):
            p.resetJointState(robot, j, q_cur[j])
        p.changeVisualShape(ball_ids[name], -1, rgbaColor=[0.95, 0.80, 0.10, 1.0])  # 当前点高亮
        prof = trap_profile(ANIM_STEPS)                # 轨迹规划层：梯形目标轨迹
        dt = 1.0 / SIM_HZ
        q_prev = list(q_cur)
        for k, s in enumerate(prof):
            q_cmd = [q_cur[j] + s * (q[j] - q_cur[j]) for j in range(6)]
            v_cmd = [0.0] * 6 if k == 0 else [(q_cmd[j] - q_prev[j]) / dt for j in range(6)]
            p.setJointMotorControlArray(               # 关节伺服层：位置伺服 + 速度前馈
                robot, range(6), p.POSITION_CONTROL, targetPositions=q_cmd,
                targetVelocities=v_cmd, forces=FORCES,
                positionGains=[POS_GAIN] * 6, velocityGains=[VEL_GAIN] * 6)
            p.stepSimulation()
            q_prev = list(q_cmd)
            if writer is not None and k % args.every == 0:
                tentacle_t = 0.5 * min(1.0, max(0.0, (k / ANIM_STEPS - EXTEND_START) / (1.0 - EXTEND_START)))
                shoot()
        steady, converge_step = 0, None                # 收敛观察窗（保持段 2 s，渲染）
        for k in range(HOLD_STEPS):
            p.setJointMotorControlArray(
                robot, range(6), p.POSITION_CONTROL, targetPositions=q,
                targetVelocities=[0.0] * 6, forces=FORCES,
                positionGains=[POS_GAIN] * 6, velocityGains=[VEL_GAIN] * 6)
            p.stepSimulation()
            ee = p.getLinkState(robot, 5, computeForwardKinematics=1)[4]
            steady = steady + 1 if math.dist(ee, target) <= args.threshold else 0
            if steady >= STEADY_N and converge_step is None:
                converge_step = k
            if writer is not None and k % args.every == 0:
                tentacle_t = 0.5 + 0.5 * min(1.0, k / (WRAP_END * HOLD_STEPS))
                shoot()
        budget = SERVO_MAX_STEPS - ANIM_STEPS - HOLD_STEPS   # 超时兜底段（不渲染）
        for k in range(budget if converge_step is None else 0):
            p.setJointMotorControlArray(
                robot, range(6), p.POSITION_CONTROL, targetPositions=q,
                targetVelocities=[0.0] * 6, forces=FORCES,
                positionGains=[POS_GAIN] * 6, velocityGains=[VEL_GAIN] * 6)
            p.stepSimulation()
            ee = p.getLinkState(robot, 5, computeForwardKinematics=1)[4]
            steady = steady + 1 if math.dist(ee, target) <= args.threshold else 0
            if steady >= STEADY_N:
                converge_step = ANIM_STEPS + HOLD_STEPS + k
                break
        ee_track = p.getLinkState(robot, 5, computeForwardKinematics=1)[4]
        err_track = math.dist(ee_track, target)
        q_cur = [p.getJointState(robot, j)[0] for j in range(6)]   # 伺服后实际关节角

        reached = (in_lim and err_ik <= args.threshold
                   and err_track <= args.threshold and converge_step is not None)
        note = ""
        if not reached:
            parts = []
            if err_ik > args.threshold:
                parts.append(f"IK残差超阈值({err_ik * 1000:.1f}mm)")
            parts += bad
            if converge_step is None:
                parts.append("伺服未收敛")
            elif err_track > args.threshold:
                parts.append(f"伺服误差超阈值({err_track * 1000:.1f}mm)")
            note = "; ".join(parts)
        rows.append(dict(point_id=name, **{
            "target_x": target[0], "target_y": target[1], "target_z": target[2],
            **{f"q{k+1}": round(q[k], 6) for k in range(6)},
            "err_ik_m": round(err_ik, 6), "err_track_m": round(err_track, 6),
            "err_com_m": round(err_com, 6), "in_limits": int(in_lim),
            "converged": int(converge_step is not None),
            "converge_step": converge_step if converge_step is not None else "timeout",
            "reached": int(reached), "threshold_m": args.threshold, "note": note}))
        print(f"{name:4s} ({target[0]:.2f},{target[1]:+.2f},{target[2]:.2f})   "
              f"{err_ik * 1000:11.2f} {err_track * 1000:12.2f} {err_com * 1000:9.2f} "
              f"{'OK' if in_lim else '超限':4s} "
              f"{str(converge_step) if converge_step is not None else '超时':>6s} "
              f"{'到达' if reached else '未达标':6s}")
        p.changeVisualShape(ball_ids[name], -1, rgbaColor=[0.10, 0.70, 0.15, 1.0])  # 已到达→绿色

        # ---- 演示层：抓 → 移到篮子 → 末端下降放果 → 前往下一点（不参与误差判定） ----
        q_cur, drop_err, drop_ok = place_into_basket(name, q_cur, len(place_checks))
        place_checks.append((name, drop_err * 1000, drop_ok))

    t_total = time.perf_counter() - t_total0
    imageio.imwrite("results/scene.png", shoot(force=True))   # 终态场景截图（writer 仍打开，帧同步入视频）
    if writer is not None:
        writer.close()

    # ---- CSV ----
    fields = ["point_id", "target_x", "target_y", "target_z",
              "q1", "q2", "q3", "q4", "q5", "q6",
              "err_ik_m", "err_track_m", "err_com_m", "in_limits",
              "converged", "converge_step", "reached", "threshold_m",
              "render_time_s", "note"]
    for row in rows:
        row["render_time_s"] = round(t_render, 3)   # 全局渲染耗时（各行同值，供结果表引用）
    with open("results/reach_results.csv", "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=fields)
        wr.writeheader()
        wr.writerows(rows)

    n_ok = sum(r["reached"] for r in rows)
    print("-" * 78)
    print(f"结果：{n_ok}/{len(rows)} 点通过（两重静态判定 + 伺服跟踪；阈值 {args.threshold * 1000:.0f} mm，"
          f"可经 --threshold 配置）")
    print(f"渲染：{n_frames} 帧 @ {w}x{h}，渲染耗时 {t_render:.1f} s；全程耗时 {t_total:.1f} s")
    if place_checks:
        print(f"演示层：{len(place_checks)} 段抓放（放果点 {BASKET_DROP} m，末端下降示意「低头放果」），"
              f"放果点 IK 残差 {min(e for _, e, _ in place_checks):.2f}–"
              f"{max(e for _, e, _ in place_checks):.2f} mm，"
              f"限位 {sum(ok for _, _, ok in place_checks)}/{len(place_checks)} OK；"
              f"该段为视觉示意，不参与误差判定与 CSV")
    print(f"演示层仿生触手：{TENTACLE_COUNT} 根 x {TENTACLE_SEGS} 段三阶段视觉示意，"
          f"卷曲(rho=a*exp(b*theta)) -> 到位反向展开 -> 球面缠绕包覆果实，"
          f"逐颗循环、放果后回卷；几何为示意参数（非中科大实物数据），"
          f"无接触 / 夹持力，不参与误差判定与 CSV")
    if args.no_video:
        print("产物：results/reach_results.csv / results/scene.png（--no-video：未生成 MP4）")
    else:
        print("产物：results/reach_results.csv / results/reach_demo.mp4 / results/scene.png")
    print("（未达标点已如实标注 note，处理流程见指导书 §9.2，禁止放宽阈值或改限位）")
    p.disconnect()
    raise SystemExit(0 if n_ok == len(rows) else 1)


if __name__ == "__main__":
    main()
PY_EOF
python xarm6_reach_demo.py
```

> 脚本要点（与指导书 §1.2/§1.3/§2.1 一一对应）：IK 全参数显式传入、每次求解前 `resetJointState` 固定初值、误差主口径 `getLinkState(...)[4]`、COM 口径备查、限位独立判定、未达标如实记录。

### 步骤七附加：负向验证（证明预筛逻辑存在且工作）

```bash
python xarm6_reach_demo.py --check-out-of-range; echo "exit=$?"
```

预期输出：

```
[预筛拒绝] P_BAD_H 水平距离 1.200 m > 0.70 m，IK 前中止。
[预筛拒绝] P_BAD_Z z=0.900 m 不在 [0.1,0.75]，IK 前中止。
（--check-out-of-range 仅验证拒绝逻辑，不构成可达性结论）
exit=2
```

---

## 8. 运行与结果采集

### 8.1 正常运行（步骤七已含）与快速模式

```bash
python xarm6_reach_demo.py                 # 完整运行（CSV + MP4 + 截图）
python xarm6_reach_demo.py --no-video      # 快速验证（只出 CSV/截图，秒级）
python xarm6_reach_demo.py --threshold 0.003   # 换阈值复跑（阈值可配）
```

### 8.2 渲染预算与降级分支

| 模式 | 帧数估算 | 实测耗时（2026-11-06，v5） |
|------|----------|------------------------|
| 默认 960×720 / 每 8 步 | 每点到位段约 120 帧 + 演示层抓放段约 75 帧，共 ~975 帧 | **976 帧，渲染 133.3 s，全程 133.6 s** |
| 降级 640×480 / 每 16 步 | 约 ~490 帧 | **未执行**（默认模式 133.3 s < 5 分钟预算，未触发降级分支） |
| `--no-video` | 1 帧（截图） | **0.1 s**（渲染 1 帧 0.1 s / 全程 0.1 s；伺服、抓放段与果实搬运仍完整执行） |

ER_TINY_RENDERER 为 CPU 软件渲染，不依赖 WSLg/GPU，但高分辨率下耗时不可忽略。**渲染耗时由脚本自动打印并应在结果表登记**。若默认模式渲染超过 5 分钟，属正常范围；超过 15 分钟改用降级分支重跑。v7 实测 133.3 s 落在预算内（v4 为 601 帧 / 82.5 s，帧数增加来自 5 段抓放演示），故降级分支未运行；该分支的耗时不作任何结论引用。

### 8.3 结果记录表（2026-11-06 全量重跑回填；数据源：控制台汇总表 + `results/reach_results.csv`）

| 点 | 目标 (m) | IK 残差 (mm) | 伺服误差 (mm) | COM (mm) | 限位 | 收敛步 | 结论 | 备注 |
|----|----------|--------------|---------------|----------|------|--------|------|------|
| P1 | (0.45, 0.00, 0.40) | 0.29 | 0.29 | 13.13 | OK | 59 | 到达 | COM 末位舍入见下 |
| P2 | (0.48, 0.20, 0.35) | 0.88 | 0.88 | 13.31 | OK | 59 | 到达 | |
| P3 | (0.48, -0.20, 0.35) | 0.88 | 0.88 | 13.58 | OK | 59 | 到达 | |
| P4 | (0.35, 0.12, 0.50) | 0.48 | 0.48 | 12.65 | OK | 59 | 到达 | |
| P5 | (0.35, -0.12, 0.50) | 0.48 | 0.48 | 12.99 | OK | 59 | 到达 | |

汇总信息：阈值 = **5 mm**（`--threshold 0.005`，未放宽）；渲染耗时 = **133.3 s**（976 帧 @ 960×720）；总耗时 = **133.6 s**（墙钟 15:24:37 → 15:26:51，134 s）；pybullet 版本 = **3.2.7**（build time Jan 29 2025）。CSV 的 `render_time_s` = 133.319（全局渲染耗时，各行同值）。演示层 5 段抓放 + 触手三阶段（卷曲 / 展开 / 缠绕包覆）视觉示意：放果点 IK 残差 0.97 mm、限位 5/5 OK，该段不写入 CSV。

**COM 备查列的末位口径**：表内 COM 数值取控制台汇总表打印值；P1 控制台为 **13.13 mm**，而 CSV 的 `err_com_m=0.013135`（×1000 后按 2 位舍入为 **13.14 mm**），0.01 mm 差异来自 CSV 六位小数（m）存储。COM 仅备查、不参与任何判定；主口径 `err_ik` / `err_track` 及限位、收敛步在控制台与 CSV 两处逐点一致。

补充实测：`--no-video` 快速模式逐点数值与完整运行完全一致；`--check-out-of-range` 退出码 2；MP4 可完整解码 **976 帧 / 30 fps / 32.53 s**；画面**抽样判读 6 帧**（第 0/119/194/389/779/975 帧）：篮子与果实入篮清晰可见，终态篮中 5 颗，非黑屏（**全量逐帧判读仍未执行**）；降级渲染分支、`--threshold` 换阈值复跑、全新机器从零环境复跑均**未执行**。

**实测结论**：5 点全部满足两重静态判定 + 伺服跟踪判据，逐点伺服跟踪误差 **0.29–0.88 mm < 5 mm**（实测值见表），`converged=1`、`in_limits=1`、收敛步全为 59；未发生任何目标点坐标或伺服增益调整（§9.2 决策树未触发）。**与 v4（11-04）逐点完全一致**：抓放演示段改变了各点伺服起始位姿，但保持段 480 步足以收敛到同一 IK 解，故 `err_ik` / `err_track` / `err_com` / q1–q6 / 收敛步均无变化，唯一变化是帧数与渲染耗时（601 → 976 帧，82.501 → 134.205 → 133.581 → 133.319 s）。本结论只覆盖**无重力设定**下的轨迹跟踪与定位收敛，不代表带重力实物化性能，也不代表抓取成功率；演示层抓放循环与 v7 触手三阶段仅为流程可视化（§0 第 7 条）；v7 与 v6 / v5 的逐点误差、q1-q6、收敛步逐点完全一致，触手是纯视觉层，未改控制链路。

**结论书写规则**：仅当 5 点实测全部"到达"后，方可在结论区写"逐点误差 < 5 mm（实测值见表）"；存在未达标点时，结论如实写"Px 未收敛/超阈值，已按 §9.2 调整/记录"。**禁止先写结论再凑数。**

---

## 9. 可达性预检与目标点调整分支

### 9.1 正常情况

5 点全部通过（两重静态判定 + 伺服跟踪，§1.3）→ 直接进入 §10 归档。

### 9.2 未收敛 / 超阈值 / 超限位的规则化处理（不是探索，是固定决策树）

**第 1 步：判断失败类型**（看 CSV 的 note 列、converged/converge_step 与 q 值）

| 现象 | 诊断 | 调整动作（每次只动一档，改后重跑） |
|------|------|------------------------------------|
| err_ik > 阈值 且 水平距离 > 0.60 m | 点位过远，接近臂展极限 | 水平距离缩小 0.03 m |
| err_ik > 阈值 且 z > 0.55 m | 点位过高 | z 下调 0.03 m |
| err_ik > 阈值 且 q3 < -3.5（贴 joint3 下限 -3.927） | 姿态贴限位 | 水平距离放大 0.03 m 或 z 下调 |
| err_ik > 阈值 且以上三条都不满足 | IK 迭代不足 | `IK_MAX_ITER` 100→200 重跑一次（仍失败回到上三条） |
| converged = 0（converge_step = timeout） | 伺服增益不足或收敛窗偏短（先确认 §0 第 6 条无重力设定未被改动） | `POS_GAIN` 1.0→2.0 重跑一次（振荡则 `VEL_GAIN` 1.0→2.0 加阻尼，见附录 C） |
| in_limits = False | IK 解贴限位（姿态别扭） | 按上表同规则调整坐标后重跑 |

**第 2 步：登记变更**（下表写在指导书/报告内，坐标调整必须留痕）

| 点号 | 旧坐标 (m) | 新坐标 (m) | 失败原因（残差/超限关节及数值） | 重跑结果 |
|------|-----------|-----------|-------------------------------|----------|
| 无 | — | — | 5 点一次通过两重静态判定 + 伺服跟踪，未触发本决策树 | 无需调整重跑；`--no-video` 与完整运行两次结果逐点一致 |

> 注：实跑中发生的**脚本缺陷修复**（Writer 收尾顺序、产物打印口径、CSV 列）不属于目标点 / 增益调整，登记于 §14，不占用本表。

**禁止事项**：放宽 `--threshold` 应付验收、改 URDF 限位、删点、改 HOME 后沿用旧数据、只报调整后的点而隐瞒调整前的失败记录。

---

## 10. 工程归档（拷回 Windows）

```bash
# 目标目录不存在时：整目录拷入，xarm_lab_run 即工程根目录
cp -r ~/xarm_lab /mnt/d/桌面/CRAIC/success/202607091551/temp_20261104_01/xarm_lab_run

# 目标目录已存在时（本轮实跑即如此）：用 /. 把内容平铺进去，避免嵌套出 xarm_lab_run/xarm_lab/
cp -r ~/xarm_lab/. /mnt/d/桌面/CRAIC/success/202607091551/temp_20261104_01/xarm_lab_run/
```

> **v4 实跑注记（2026-11-04 18:39）**：归档前先把 `xarm_lab_run\` 内已完成使命的暂存脚本（`node*.sh`）与空 `wheels\` **移出**到工作区 `archive_node_scripts_20261104\`（未删除，保留备份），再按上述第二条命令的思路逐项拷入工程文件（额外排除 `.git` 与 `__pycache__`）。归档结果：7 个 STL、3 个 `results/` 产物、URDF / 2 个 py / README / 3 份文档，逐文件 `cmp` 与 WSL 工程一致；详见《运行日志_20261104.md》Node 7。

**整工程归档，不只是结果目录**：`xarm6_reach.urdf`、`test_load.py`、`xarm6_reach_demo.py`、`meshes/`、`results/` 全部进入 Windows 侧 `d:\桌面\CRAIC\success\202607091551\temp_20261104_01\xarm_lab_run\`。课设报告与说明书引用的数字一律以 `xarm_lab_run\results\reach_results.csv` 为准；"可运行仿真"交付即此目录——在任意装有 Python 3.11 + pybullet 的 Linux 环境，把该目录内容放回 `~/xarm_lab` 即可按 §6–§8 复跑。

---

## 11. 常见问题排错表

| # | 现象 | 原因 | 处理 |
|---|------|------|------|
| 1 | `pip install pybullet` 开始编译源码（大量 g++ 输出） | 当前 Python 不是 3.11（wheel 实测仅 cp36–cp311） | `conda activate robot` 后确认 `python -V` = 3.11.x 再装 |
| 2 | wheel 拉取超时/SSL 报错 | 网络或镜像波动 | 重试；确认用 `-i https://pypi.tuna.tsinghua.edu.cn/simple`；公司网络需代理时换手机热点 |
| 3 | 源码编译失败：`error: command 'gcc' failed` / `Python.h: No such file` | 走了异常路径但缺编译链 | `sudo apt install -y build-essential cmake python3-dev` 后重装；编译 5–10 分钟属正常，非卡死 |
| 4 | `error: Cannot load URDF file` | 不在 `~/xarm_lab` 目录运行（URDF 为相对路径） | `cd ~/xarm_lab` 后重跑；项目不得放在 `/mnt/d/...`（中文路径，PyBullet C++ 加载不稳） |
| 5 | 报 mesh 相关错误 / 模型不显示 | `meshes/xarm6/visual/` 下 STL 不足 7 个 | 回到步骤四重新拷贝；`.dae` 不拷贝是**正确行为** |
| 6 | test_load [5] FK 断言 FAIL | URDF 数值被改动过 | 按 §5 原文重建 URDF；数值来源见附录 B |
| 7 | 某点 err_ik 大、结论"未达标" | 点位对该臂的工作空间/限位不友好 | 按 §9.2 决策树调整并登记，**不得**放宽阈值或改限位 |
| 8 | IK 解超限位（in_limits=0） | 同上，姿态贴限位 | 同 §9.2 |
| 8b | converged=0（converge_step=timeout） | 伺服增益不足或收敛窗偏短（无重力设定下增益基线 1.0） | `POS_GAIN` 1.0→2.0（振荡则 `VEL_GAIN` 1.0→2.0），重跑并按 §9.2 登记，指引见附录 C |
| 9 | 渲染过慢（>15 分钟） | CPU 软渲染高分辨率 | `--res 640x480 --every 16` 降级重跑；或先 `--no-video` 验证数据 |
| 10 | MP4 黑屏/打不开 | 编码器问题（罕见） | 确认 `imageio-ffmpeg` 安装（步骤三验证命令）；重跑；仍失败改 `--no-video` 用 scene.png + CSV 交付 |
| 11 | err_com 与 err_ik / err_track 差 10 mm+ | 口径混淆 | 正常现象：[0] 是 COM、[4] 是 link 原点，**主口径是 [4]**，报告数字取 err_ik / err_track |
| 12 | 动画里臂"瞬移" | 用了 `--no-video` 或帧率理解偏差 | 视频按 30 fps 导出，每 8 仿真步一帧；如需慢放改用 `--every 4` |

---

## 12. 答辩追问速答（口径与本文档一致）

1. **为什么没有夹爪/末端执行器？控制在哪？**
   末端执行机构按机械图纸设计（A0 图有）；本轮控制结构为两层——**关节空间梯形轨迹规划 + 关节位置伺服**（POSITION_CONTROL，分关节力矩上限 + 轨迹段速度前馈；动力学含惯量与关节阻尼，**无重力设定**——验证指令跟踪与定位收敛，重力负载前馈补偿属实物化后续工作），验证臂本体定位链路。夹爪开合与接触力学属下一阶段。
2. **IK 是怎么解的？为什么解是确定的？**
   位置-only 对 6 轴臂是欠定问题（3 约束 / 6 未知）；PyBullet 用 nullspace 法在满足位置约束的解流形中取最接近参考姿态 HOME 的解。限位、参考姿态、初值三者在脚本中全部写死，因此任何人复跑得到同一组关节角（可复现性）。
3. **误差是怎么测的？测的是哪个点？**
   末端 link 名为 link6（官方 xacro 中 link_eef 被注释，名义末端即 link6）；误差 = 目标点到 link6 原点（URDF link frame 原点，`getLinkState[4]`）的欧氏距离，与 IK 的末端参考系一致；质心口径（[0]）同表备查。
4. **5 mm 阈值怎么定的？**
   阈值是可配参数（--threshold）；实测**伺服跟踪误差**（无重力设定下控制回路的末端定位精度）见表 §8.3。草莓果径约 20–40 mm，毫米级末端定位误差对采摘目标而言裕度充分。
5. **力矩上限是哪来的？**
   分关节取自 xarm_ros 官方 `joint_limits.yaml`：163.2/163.2/81.6/81.6/30.6/30.6 N·m（含 2.55 倍过载系数；额定值 = 过载值 / 2.55）。该数值直接用于主流程位置伺服的 `forces` 参数与 URDF effort 标签。
6. **目标点为什么选这 5 个？**
   与方案设计文档同源的 P1–P5 原始坐标；预筛（半径 ≤ 0.70 m、z ∈ [0.10, 0.75]）通过，并经独立矩阵实算预核可达（IK 残差 0、q3 处于负值区远离上限），最终以两重静态判定 + 伺服跟踪实测锁定；若发生过调整，登记于 §9.2 变更表。
7. **动画里臂是怎么运动的？**
   关节空间梯形轨迹（加速 25% / 匀速 50% / 减速 25%）作为**目标轨迹**，由**位置伺服**（POSITION_CONTROL，分关节力矩上限 + 速度前馈）在 240 Hz 动力学仿真（URDF 惯量/阻尼，无重力设定）中跟踪执行，30 fps 导出；伺服增益调整指引见附录 C。
8. **为什么用 PyBullet？**
   内置带限位的逆运动学求解器，适合快速验证定位链路；模型与参数全部来自 xArm 官方开源仓库（来源对照见附录 B）。
9. **动画里「摘草莓、放进篮子」是真的抓取吗？**
   **不是**，这是 §0 第 7 条的演示层抓放循环：果实球是位置标记，脚本直接把它的视觉位置搬到篮中；末端下降到放果点表达「低头」，因位置-only IK 不约束姿态，末端姿态并未真实改变。没有夹爪、没有接触检测、没有夹持力或接触力学，也不做果实识别。5 个草莓点的到位误差仍按 §1.3 判定，演示段不产生误差数据、不写入 CSV；动画证明的是「流程可达 + 定位精度」，不是抓取成功率。v7 的 3 根仿生触手按「向后卷曲成对数螺旋 → 到位反向展开 → 球面缠绕包覆」三阶段逐颗循环（示意几何、非实物参数），仍是演示层纯视觉示意，不是 URDF 末端执行器，不构成真实夹持；仿生触手的机构结构在 A0 图纸与说明书中讲，仿真只验证通用 6 轴平台的控制流程。

---

## 13. 实跑回填清单（v7 状态，2026-11-08）

- [x] pybullet 实际安装版本与安装耗时（步骤三）：3.2.7 / cp311 wheel 103.2 MB / 下载 6 分 31 秒（见 §3 步骤三、运行日志 Node 1）
- [x] 5 点 err_ik / err_track / err_com / 限位判定 / 收敛步 / 结论（§8.3 表）：已按 11-06 全量重跑的 `results/reach_results.csv` 回填，逐点与 v4 一致
- [x] 渲染耗时：默认模式 **976 帧 / 133.3 s**（§8.2）；`render_time_s` = 133.319；`--no-video` 0.1 s；降级模式**未执行**（未触发）
- [x] v7 仿生触手三阶段：3 根 x 6 段，卷曲(rho=a*exp(b*theta)) / 反向展开 / 球面缠绕包覆，逐颗循环、放果后回卷；抽样判读 6 帧（0/52/78/194/700/975）三阶段形态可辨；示意参数、不进运动学链 / 误差判定 / CSV
- [x] 总耗时：**134.3 s**（墙钟 17:59:38 → 18:01:53）
- [x] 演示层抓放：5 段，放果点 IK 残差 0.97 mm、限位 5/5 OK，篮子与果实入篮已抽样判读可见（§14.3）
- [x] 目标点变更登记（§9.2 表）：无变更，5 点一次通过
- [x] 结论区文字（按 §8.3 结论书写规则）：已写「逐点误差 0.29–0.88 mm < 5 mm」，附无重力设定与演示层边界限定
- [x] 答辩素材：MP4 / GIF / 关键帧拼图 / 终态截图均由 11-06 同一次运行派生
- [ ] 零探索验证：全新机器按本文档从零环境完整复跑（**未执行**，本轮为既有 WSL 环境实跑）
- [ ] 降级渲染分支 / `--threshold` 换阈值复跑 / MP4 画面**全量逐帧**人工判读（已抽样 6 帧，非全量，不作逐帧正确性结论）

---

## 14. 实跑变更记录（v4 基线 + v5 / v6 / v7 扩展）

### 14.1 实跑基线

| 项 | 值 |
|---|---|
| 实跑时间 | 2026-11-04 18:18–18:28（+08:00）；完整运行 18:27:06 → 18:28:29 |
| 环境 | WSL2 Ubuntu 26.04.4 LTS，默认用户 `slphy_kai`，项目 `/home/slphy_kai/xarm_lab`；conda `robot`，Python 3.11.18，pybullet 3.2.7（cp311 manylinux wheel），numpy 2.4.6，imageio 2.38.1，imageio-ffmpeg 0.8.1，pillow 13.0.0 |
| 命令 | `python test_load.py`；`python xarm6_reach_demo.py --no-video`；`python xarm6_reach_demo.py --check-out-of-range`；`python xarm6_reach_demo.py` |
| 对应 commit | 环境 `fd5c92b` / 自检 `94a232f` / 快速验证 `309e282` / 完整运行与脚本修复 `045065d`；v4 回填提交见《运行日志_20261104.md》终验总表 |
| 产物 | `results/reach_results.csv`（744 B，含 `render_time_s`）、`results/reach_demo.mp4`（732358 B，601 帧）、`results/scene.png`（26484 B） |
| 未执行 | 降级渲染分支、换阈值复跑、全新机器从零环境复跑、MP4 画面逐帧人工判读 |

### 14.2 实跑发现与变更（旧值 → 新值 → 原因 → 重跑结果）

| # | 类别 | 旧值 / 现象 | 新值 / 处理 | 原因 | 重跑结果 |
|---|------|-------------|-------------|------|----------|
| 1 | 加载自检显示口径 | `test_load.py` 打印 `len(img[2])` = 240 且标为 bytes | 改为 `img[2].nbytes` = 307200，文案补「= 320×240×4」 | `img[2]` 是 shape (240,320,4) 的 ndarray，`len()` 取第一维行数；判据 `>0` 语义不变 | 18:20:17 重跑 `== ALL PASS ==`，6 限位 / link6 / FK 0.00 mm / 307200 bytes 全 PASS |
| 2 | 完整运行崩溃（六步流程处理） | Node4 首跑 `FULL_EXIT=1`：`writer.close()` 先于 `shoot(force=True)`，向已关闭 Writer 追加帧 → `RuntimeError: I/O operation on closed Writer.` | 把 `writer.close()` 移到终态截图之后，截图帧正常写入 MP4 再关闭 | `shoot()` 仅判 `writer is None`、未判 closed；`--no-video` 模式 writer 恒为 None 故不触发 | 18:27:06 重跑 `FULL_EXIT=0`，601 帧 / 82.5 s，CSV / MP4 / PNG 齐备，MP4 可完整解码 601 帧 |
| 3 | 产物打印口径 | 汇总行固定打印 `reach_demo.mp4`，`--no-video` 下该文件并不生成 | 按 `args.no_video` 分模式打印，明确标注「--no-video：未生成 MP4」 | 避免声明不存在的产物 | `--no-video` 回归输出正确；完整运行打印三件产物路径 |
| 4 | CSV 与 §1.4 口径 | CSV 无渲染耗时列，而 §1.4 称其含渲染耗时 | CSV 增 `render_time_s` 列（全局值 82.501，各行同值） | 使文档表述、spec 与实现一致；渲染耗时仍由控制台汇总表打印 | 重跑 CSV 744 B，5 行均含 82.501 |

**未改动项（红线核对）**：`--threshold` 保持 0.005 m（未放宽）；URDF 关节限位、HOME / restPoses、TARGETS（P1–P5）、FORCES（163.2/163.2/81.6/81.6/30.6/30.6）、POS_GAIN / VEL_GAIN（1.0 / 1.0）、`setGravity(0,0,0)`、STEADY_N = 60、SERVO_MAX_STEPS = 3000、IK_MAX_ITER = 100 全部未改；未删点、未隐藏失败记录；§9.2 决策树未触发，无坐标 / 增益变更登记。

**同步范围**：#1–#4 的代码改动均在 WSL 工程、Windows 暂存区母本、本文档 §6 / §7 内嵌脚本三处同步（逐字比对一致），`py_compile` 通过；处理过程见《运行日志_20261104.md》Node 2 / Node 4。**失败记录保留**：Node4 首跑的 traceback 与 `FULL_EXIT=1` 全文记于运行日志，未从任何文档中删除。

### 14.3 v5 演示层抓放循环与全量重跑（2026-11-06）

| 项 | 值 |
|---|---|
| 实跑时间 | 完整运行 17:59:38 → 18:01:53（+08:00）；素材派生 18:34 |
| 环境 | 同 §14.1（WSL2 Ubuntu 26.04.4 LTS / conda `robot` / Python 3.11.18 / pybullet 3.2.7） |
| 命令 | `python xarm6_reach_demo.py --no-video`；`--check-out-of-range`；`python xarm6_reach_demo.py` |
| 产物 | CSV 749 B（5 行 19 字段）、MP4 1.4 MB（976 帧 / 32.53 s）、scene.png 28 KB、GIF 195 帧 / 337 KB、关键帧拼图 193 KB |

> 注：v5 当时文档写作「19 字段」，脚本与 CSV 实际为 **20 列**；v6 已统一校准为「5 行 20 列」，此处保留历史原文不改。
| 旧值 → 新值 | 601 帧 / 82.5 s → **976 帧 / 134.2 s**（渲染）；`render_time_s` 82.501 → **134.205**；5 点误差、q1–q6、收敛步**逐点完全一致** |
| 未执行 | 降级渲染分支、换阈值复跑、全新机器零探索复跑、MP4 全量逐帧判读（已抽样 6 帧） |

**变更登记（旧值 → 新值 → 原因 → 重跑结果）**

| # | 类别 | 旧值 | 新值 / 处理 | 原因 | 重跑结果 |
|---|------|------|-------------|------|----------|
| 1 | 范围红线 | §0 只写「依次到达 5 点」，不含采摘流程 | §0 新增第 7 条：正式纳入演示层抓放循环，主动声明无夹爪 / 接触 / 力 / 姿态约束 | 课设题目为采摘机械臂控制系统设计，动画需体现逐颗采摘 | 文档与代码同步；5 点判据、阈值、限位、TARGETS 全部未改 |
| 2 | 脚本 | 每点结束直接进入下一点 | 每点 `rows.append` 后插入独立 `place_into_basket()`：放果点 IK + 同套梯形轨迹 / 位置伺服 + 果实视觉搬运入篮 | 演示层隔离：不写 rows、不改 fields、不参与判定 | `--no-video` 与完整运行均 5/5 通过、CSV 仍 5 行；放果点 IK 残差 0.97 mm、限位 5/5 OK |
| 3 | 场景 | 地面 + 栽培架 + 5 个球 | 新增棕色开口篮（5 个静态视觉体）+ 5 个篮内果实槽位 | 让「放入篮子」在画面中可见 | 抽样判读 6 帧：frame 194 / 389 / 779 篮中分别 1 / 2 / 4 颗，终态 5 颗全在篮中 |
| 4 | 起始位姿 | 下一点伺服起始 = 上一草莓点姿态 | 下一点伺服起始 = 放果段结束姿态 | 流程连续性 | 原预判「err_track / converge_step 可能变化」；**实测逐点数值与 v4 完全一致**（含 converge_step=59），如实记为「一致」，未臆造差异 |
| 5 | 素材 | 601 帧 MP4 / 120 帧 GIF / 旧拼图 | 976 帧 MP4 / 195 帧 GIF / 覆盖抓放时刻的新拼图 | 素材须与改后运行同源 | 画面抽样判读通过，GIF 首 / 中 / 末帧非纯色 |

**未改动项（红线核对）**：`--threshold` 仍 0.005 m；URDF 关节限位、HOME / restPoses、TARGETS（P1–P5）、FORCES、POS_GAIN / VEL_GAIN、`setGravity(0,0,0)`、STEADY_N、SERVO_MAX_STEPS、IK_MAX_ITER 全部未改；未删点、未隐藏失败记录；§9.2 决策树未触发，无坐标 / 增益变更登记；`writer.close()` 收尾顺序保持不变。

**三处同步**：WSL 工程 / Windows 归档 `xarm_lab_run` / 本文档 §6 §7 内嵌脚本逐字一致（含本次修正的 `test_load.py` 归档母本），`py_compile` 通过。

### 14.4 v6：L1 仿生触手视觉合拢示意与答辩材料包（2026-11-06 夜）

| 项 | 值 |
|---|---|
| 实跑时间 | 2026-11-06 23:21:09 → 23:23:24（+08:00），完整运行 |
| 命令 | `python xarm6_reach_demo.py --no-video`（回归）→ `--check-out-of-range`（退出码 2）→ `python xarm6_reach_demo.py` |
| 产物 | CSV 749 B（5 行 20 列）、MP4 1.7 MB（976 帧 / 32.53 s）、scene.png 29 KB、GIF 195 帧 / 375 KB、关键帧拼图 213 KB、自制答辩材料包 zip 3.3 MB |
| 旧值 → 新值 | `render_time_s` 134.205 → **133.581**；渲染 134.2 s → **133.6 s**；全程 134.3 s → **133.8 s**；帧数仍 976；**5 点误差、q1-q6、收敛步与 v5 逐点完全一致** |
| 未执行 | 降级渲染分支、换阈值复跑、全新机器零探索复跑、MP4 全量逐帧判读（已抽样 6 帧） |

**变更登记（旧值 → 新值 → 原因 → 重跑结果）**

| # | 类别 | 旧值 | 新值 / 处理 | 原因 | 重跑结果 |
|---|------|------|-------------|------|----------|
| 1 | 演示层视觉 | 只有果实视觉搬运，无末端机构 | 新增 3 根 x 6 段纯视觉触手：到点合拢包覆果实、放果后张开 | 课设「采摘草莓」叙事需要末端夹取观感；用户选定 **L1 视觉示意** | 5/5 通过、CSV 仍 5 行 20 列、误差与 v5 完全一致；触手汇总正常打印 |
| 2 | 画面判读修正 | 触手按 link6 局部姿态摆放，果实按世界坐标摆放，两者不对齐（frame 100 触手在末端右侧、球在左侧） | 触手改用**世界坐标**环绕 `ee` 正下方果实 | 抽样判读发现「没夹住」的观感问题 | 重跑后 frame 100 触手位于果实正下方并随末端收拢，frame 700 抓取、终态篮中 5 颗可见 |
| 3 | 范围与叙事 | v5 §0 第 7 条只写抓放循环 | 第 7 条补 L1 触手边界；明确**图纸与说明书讲仿生机构结构，仿真以通用 6 轴平台验证控制流程** | 用户选定答辩叙事路线 A | 主结论「精度验证不建模末端执行器」保持不变；触手不进 URDF / 运动学链 / 误差判据 |
| 4 | 答辩材料 | 无自制图纸包 | 交付 6 张课设自制图纸（控制框图 / Tool IO 接线 / 控制器接口 / 电源 / 绳驱 / CAD 三视图）+ 仿真素材 + 来源与边界说明 + 官方链接清单，压缩为 `答辩材料/答辩材料包_20261106.zip` | 用户答辩要用；官方未公开控制器 PCB / 单片机电路图 | 全部图纸显著标注「本课题设计示意，非官方实物电路图」；GitHub 只传自制包，官方原件留本地 |

**未改动项（红线核对）**：`--threshold` 仍 0.005 m；URDF 关节限位、HOME / restPoses、TARGETS（P1-P5）、FORCES、POS_GAIN / VEL_GAIN、`setGravity(0,0,0)`、STEADY_N、SERVO_MAX_STEPS、IK_MAX_ITER、`getLinkState(robot,5)[4]` 主口径、CSV 字段与 5 行结构全部未改；未删点、未隐藏失败记录（首次触手未对齐的判读与重跑均留痕）；§9.2 决策树未触发。

**三处同步**：WSL 工程 / Windows 归档 `xarm_lab_run` / 本文档 §6 §7 内嵌脚本逐字一致，`py_compile` 通过；结果与素材同步归档后逐文件 `cmp` 全绿。

### 14.5 v7：触手三阶段动画（卷曲 → 反向展开 → 缠绕包覆，2026-11-08）

| 项 | 值 |
|---|---|
| 实跑时间 | 2026-11-08 15:24:37 → 15:26:51（+08:00），完整运行 |
| 命令 | `python xarm6_reach_demo.py --no-video`（回归）→ `--check-out-of-range`（退出码 2）→ `python xarm6_reach_demo.py` |
| 产物 | CSV 749 B（5 行 20 列）、MP4 1.7 MB（976 帧 / 32.53 s）、scene.png 28 KB、GIF 195 帧 / 363 KB、关键帧拼图 264 KB |
| 旧值 → 新值 | `render_time_s` 133.581 → **133.319**；渲染 133.6 s → **133.3 s**；全程 133.8 s → **133.6 s**；帧数仍 976；**5 点误差、q1-q6、收敛步与 v6 逐点完全一致** |
| 未执行 | 降级渲染分支、换阈值复跑、全新机器零探索复跑、MP4 全量逐帧判读（已抽样 6 帧） |

**变更登记（旧值 → 新值 → 原因 → 重跑结果）**

| # | 类别 | 旧值 | 新值 / 处理 | 原因 | 重跑结果 |
|---|------|------|-------------|------|----------|
| 1 | 形态函数 | `update_tentacles(close)` 只有径向插值 `r_open → r_close`，触手始终下垂，视觉上「一张一合」 | 改为 `update_tentacles(t)` 三阶段：`t=0` 按 rho=a*exp(b*theta) 向后卷曲成对数螺旋并收在末端后方；`0~0.5` 反向展开到向下伸展态；`0.5~1` 沿果实球面（球心 `ee` 下方 `FRUIT_HELD_Z`）自球顶绕到球底包覆夹住；`t=1→0` 释放回卷 | 用户指出应体现 SpiRobs 「uncurling and wrapping」机构原理，而非简单开合 | 帧 0 卷曲成螺旋团、帧 52 展开成下垂珠串、帧 78 绕成环包住果实，三阶段形态可辨 |
| 2 | 时间线 | `tentacle_close` 6 处赋值：到位段后 40% 0→1，保持/搬运段 1，入篮段 1→0 | 改为 `tentacle_t`：到位段前 75% 卷曲待机、后 25% 展开（0→0.5），保持段前 30% 缠绕（0.5→1），保持/放果/入篮前段保持 1，入篮后 30% 释放回卷（1→0），放果后回 0；新增 `EXTEND_START` / `WRAP_END` / `RELEASE_START` 三个节拍常量 | 让卷曲、展开、包覆各有独立可判读的帧区间，逐颗果实重复 | 每点仍 195 帧、总 976 帧；关键帧 0/52/78/194/700/975 覆盖三阶段与终态 |
| 3 | 参数来源 | 附录 B 记 `TENTACLE_GRIP_R` 等 7 项 | 移除不再使用的 `TENTACLE_GRIP_R`，新增螺旋（`SPIRAL_A/B/THETA_MAX`、`COIL_BACK_Z`）、缠绕（`WRAP_R/PHI0/PHI1/TWIST`）与节拍常量，全部显著标注「本实验示意值」 | 几何与真实机构参数无关，避免误导 | 附录 B 参数行已重写；汇总表改为三阶段描述 |
| 4 | 判读素材 | 关键帧 0/100/194/389/700/975 | 重抽 0/52/78/194/700/975，中文标签标明卷曲 / 展开 / 缠绕 / 释放 / 终态 | 便于答辩一眼看出三阶段 | 拼图 1440x784，抽样判读通过，非全量逐帧 |

**未改动项（红线核对）**：`--threshold` 仍 0.005 m；URDF 关节限位、HOME / restPoses、TARGETS（P1-P5）、FORCES、POS_GAIN / VEL_GAIN、`setGravity(0,0,0)`、STEADY_N、SERVO_MAX_STEPS、IK_MAX_ITER、`getLinkState(robot,5)[4]` 主口径、CSV 字段与 5 行结构、`ANIM_STEPS` / `HOLD_STEPS` / `PLACE_STEPS` / `PLACE_HOLD` / `--every` 帧预算、`writer.close()` 收尾顺序全部未改；未删点、未隐藏失败记录；§9.2 决策树未触发。

**口径边界**：三阶段仍是**纯视觉演示层**，卷曲 / 展开 / 包覆由脚本直接摆放 18 个静态视觉段体，无绳索、电机、张力、接触或夹持力，不进 URDF 运动学链与误差判据；叙事仍为「图纸与说明书讲仿生机构结构，仿真以通用 6 轴平台验证控制流程」。

**三处同步**：WSL 工程 / Windows 归档 `xarm_lab_run` / 本文档 §6 §7 内嵌脚本逐字一致（三份指导书副本同步），`py_compile` 通过；结果与素材同步归档后逐文件 `cmp` 全绿。

---

## 附录 B：参数来源对照表（每个数字可复核）

| 参数 | 值 | 来源 |
|------|-----|------|
| 关节原点/姿态（6 组 xyz+rpy） | j1 (0,0,0.267,rpy 0)；j2 (0,0,0,rpy -1.5708,0,0)；j3 (0.0535,-0.2845,0,rpy 0)；j4 (0.0775,0.3425,0,rpy -1.5708,0,0)；j5 (0,0,0,rpy 1.5708,0,0)；j6 (0.076,0.097,0,rpy -1.5708,0,0) | `xarm_description/config/kinematics/default/xarm6_default_kinematics.yaml` |
| 关节限位 | j1 ±6.2832；j2 [-2.059, 2.0944]；j3 [-3.927, 0.19198]；j4 ±6.2832；j5 [-1.69297, 3.1416]；j6 ±6.2832（rad） | `xarm_description/urdf/xarm6/xarm6.urdf.xacro` 宏默认值 |
| 力矩上限 | 163.2 / 163.2 / 81.6 / 81.6 / 30.6 / 30.6 N·m（含 2.55 过载系数） | `xarm_description/config/xarm6/joint_limits.yaml` |
| 关节速度上限 | 3.14 rad/s | 同上 xacro `<limit velocity>` |
| 连杆惯量（mass/origin/惯性张量） | link1–6 全套 | `xarm_description/config/link_inertial/xarm6_type6_HT_BR2.yaml` |
| link_base 惯量 | mass 2.7 等 | `xarm6.urdf.xacro` 内联值 |
| 零位 FK 末端理论值 | (0.207, 0, 0.112) m | 本指导书 §2.3 链式变换推导 |
| HOME / restPoses | [0, -0.5, -1.0, 0, 0.5, 0] rad | 本实验定义（限位中部、远离奇异；IK 初值=参考姿态=伺服起始三合一）；**v4 实跑锁定**（`--no-video` 与完整运行 CSV 的 q1–q6 逐点一致，见 §14.1） |
| 目标点 5 个 | 见 §7 脚本 TARGETS（P1–P5 与 plan/spec 同源原值） | 本实验定义；独立矩阵实算预核可达（残差 0，q3 负值区、远离上限）；**v4 实跑锁定**：5 点全部通过两重静态判定 + 伺服跟踪，未发生坐标调整（§9.2 表为「无」） |
| IK 迭代/残差 | maxNumIterations=100，residualThreshold=1e-5 | PyBullet API 参数，工程常规量级 |
| 伺服参数 | STEADY_N=60（连续 0.25 s 达标判收敛）、SERVO_MAX_STEPS=3000（超时兜底 12.5 s）、POS_GAIN=1.0 / VEL_GAIN=1.0 + 轨迹段速度前馈 targetVelocities（无重力设定下的验证基线：无扰动时 PD 稳态误差→0，kp=1.0 收敛时间常数约 1–3 s，12.5 s 兜底预算充足）、梯形轨迹 480 步 @240 Hz | 本实验定义。依据：PyBullet POSITION_CONTROL 力矩 = kp·Δq + kd·(v_cmd−v)，带重力时稳态需 Δq=τ_g/kp（joint2 重力矩 ~12.7 N·m，kp=0.1 时需 127 rad，物理不可达），纯位置伺服不引入重力前馈时必须关重力 |
| 预筛半径/高度 | 0.70 m / [0.10, 0.75] m | 本实验定义（依据 xArm6 名义臂展 ~0.7 m） |
| pybullet wheel 可用性 | 3.2.6/3.2.7 含 cp311-manylinux wheel，无 cp312+ | PyPI（tuna 镜像）2026-11-04 实测 |
| v7 触手三阶段示意参数（见 §7 常量区） | `TENTACLE_COUNT=3` / `TENTACLE_SEGS=6` / `TENTACLE_LEN=0.13` m / `TENTACLE_RADIUS=0.008` m / `TENTACLE_ROOT_R=0.022` m / `TENTACLE_OPEN_R=0.065` m；卷曲 `SPIRAL_A=0.010` m、`SPIRAL_B=0.22`、`SPIRAL_THETA_MAX=2.4π`、`COIL_BACK_Z=0.050` m；缠绕 `WRAP_R=0.033` m、`WRAP_PHI0/PHI1=0.18π/0.96π`、`WRAP_TWIST=0.55π`；节拍 `EXTEND_START=0.75`、`WRAP_END=0.30`、`RELEASE_START=0.70` | **本实验演示层示意值**：rho=a*exp(b*theta) 仅用于卷曲形态，比例按 SpiRobs 官方「数根触手抓鸡蛋」演示取值，**非中科大实物参数、非论文测量值**；纯视觉、不进 URDF / 运动学链、不参与 §1.3 判定、不写入 CSV |
| 演示层抓放参数（v5 新增，见 §7 常量区） | `BASKET_CENTER=(0.25,-0.30,0.10)` m、`BASKET_DROP=(0.25,-0.30,0.20)` m、`BASKET_HALF=(0.09,0.09,0.055)` m、`PLACE_STEPS=480 / PLACE_HOLD=120`、`FRUIT_SLOTS` 5 个篮内槽位、`FRUIT_HELD_Z=0.03` m | 本实验定义（演示层）；放果点经 PyBullet 位置-only IK 预核可达（残差 0.97 mm、6 关节在限位内），**不参与 §1.3 判定、不写入 CSV** |

---

## 附录 C：伺服增益调整指引（仅主流程异常时使用）

主流程位置伺服的验证基线：**无重力设定**（§0 第 6 条）+ positionGain=1.0 / velocityGain=1.0 + 轨迹段速度前馈 + 分关节力矩上限（附录 B）。基线依据：PyBullet POSITION_CONTROL 力矩 = kp·Δq + kd·(v_cmd − v)，无重力、无其他扰动时稳态误差趋于 0；kp=1.0 下收敛时间常数约 1–3 s，3000 步（12.5 s）兜底预算充足。**该基线在实测锁定前不视为"已验证"**——若 §8.3 实测出现下表现象，按表调整（一次只改一个变量）：

| 现象 | 动作 |
|------|------|
| 到位后在目标附近振荡/超调 | `VEL_GAIN` 1.0 → 2.0（加阻尼一档） |
| err_track 迟迟不达标、converge_step = timeout | 先确认 `setGravity(0,0,0)` 未被改动；再 `POS_GAIN` 1.0 → 2.0（升一档） |
| 升增益后仍不收敛 | 观察窗 `STEADY_N` 60 → 120（延长至 0.5 s）重跑 |

约束：每次增益调整后必须**全部 5 点重跑**（不同增益下的数据不得混用）；调整过程与理由登记 §9.2 变更表，禁止只报好数据。**重力场景说明**：若实物化阶段需带重力验证，纯位置伺服须引入逆动力学前馈（`calculateInverseDynamics` 算重力矩 τ_ff，改 TORQUE_CONTROL 实现 τ = τ_ff + kp·Δq − kd·qdot）或大幅提高增益并重新整定——属后续工作，不在本实验范围。

