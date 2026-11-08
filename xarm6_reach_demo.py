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

# ---- 演示层仿生触手（L1 视觉合拢示意，不参与误差判定与物理） ----
# 参数按 SpiRobs 官方「数根触手抓鸡蛋」演示的比例取一组示意几何值；
# **非中科大实物参数、非论文测量值**，仅用于动画表达「夹住红球」，来源见指导书附录 B。
TENTACLE_COUNT = 3                                 # 触手根数（示意）
TENTACLE_SEGS = 6                                  # 每根离散段数（对数螺旋离散示意）
TENTACLE_LEN = 0.13                                # 触手展开长度 (m，示意)
TENTACLE_RADIUS = 0.008                            # 段球半径 (m，示意)
TENTACLE_ROOT_R = 0.022                            # 根部离轴半径 (m，示意)
TENTACLE_OPEN_R = 0.065                            # 张开时中段离轴半径 (m，示意)
TENTACLE_GRIP_R = 0.034                            # 合拢时包覆半径 (m，略大于果实半径 0.025)
TENTACLE_COLOR = [0.90, 0.62, 0.20, 1.0]           # 触手颜色（示意）


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
    tentacle_close = 0.0                             # 0 = 张开，1 = 合拢夹住果实

    def update_tentacles(close):
        """按 link6 世界位置摆放触手段：张开 -> 弧形 -> 合拢包覆果实。

        使用世界坐标（不随末端姿态旋转），确保触手始终环绕 ee 正下方的果实，
        与演示层果实的视觉搬运位置一致。纯视觉，不参与误差判定。
        """
        ee = p.getLinkState(robot, 5, computeForwardKinematics=1)[4]
        for i in range(TENTACLE_COUNT):
            ang = 2.0 * math.pi * i / TENTACLE_COUNT
            ca, sa = math.cos(ang), math.sin(ang)
            for j in range(TENTACLE_SEGS):
                u = (j + 1) / TENTACLE_SEGS
                z = -0.004 - TENTACLE_LEN * u
                r_open = TENTACLE_ROOT_R + (TENTACLE_OPEN_R - TENTACLE_ROOT_R) * math.sin(math.pi * u * 0.85)
                r_close = TENTACLE_GRIP_R * (0.72 + 0.28 * math.sin(math.pi * u))
                r = r_open + (r_close - r_open) * close
                wp = [ee[0] + r * ca, ee[1] + r * sa, ee[2] + z]
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
        update_tentacles(tentacle_close)              # 渲染前更新触手视觉形态
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
        nonlocal tentacle_close
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
                tentacle_close = 1.0
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
                tentacle_close = 1.0 - min(1.0, max(0.0, (k / PLACE_HOLD - 0.7) / 0.3))
                shoot()
        p.resetBasePositionAndOrientation(ball_ids[name], list(FRUIT_SLOTS[slot_idx]),
                                          [0.0, 0.0, 0.0, 1.0])   # 果实入篮并留存
        p.changeVisualShape(ball_ids[name], -1, rgbaColor=[0.75, 0.12, 0.12, 1.0])
        tentacle_close = 0.0                             # 放果后张开，准备下一颗
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
                tentacle_close = min(1.0, max(0.0, (k / ANIM_STEPS - 0.6) / 0.4))
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
                tentacle_close = 1.0
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
    print(f"演示层仿生触手：{TENTACLE_COUNT} 根 x {TENTACLE_SEGS} 段视觉合拢示意，"
          f"到点后包覆果实、放果后张开；几何为示意参数（非中科大实物数据），"
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
