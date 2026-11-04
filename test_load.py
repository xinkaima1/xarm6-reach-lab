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
    print(f"    帧缓冲 {len(img[2])} bytes（非零即渲染通路正常）")
    ok &= len(img[2]) > 0

    p.disconnect()
    print("== ALL PASS ==" if ok else "== FAILED（见上方 FAIL 项）==")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
