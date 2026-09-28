#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 01 · 仅仿真：在 PyBullet 里驱动 LiteGrip 夹爪（无需真机、无需 CAN）

只用 litegrip_pybullet，不碰真机 SDK。演示一条完整的使用路径：

  1. 加载模型          自带 URDF（或 --urdf 指定），读开度/关节值
  2. 速度受限的开合     一次全行程约 1 s（85 mm/s，真机的额定速度）
  3. 走到中间位         按归一化开度命令，settle() 等它走完
  4. 夹住一个方块       按 --object-mm 放个方块，按 --force 收爪
                        —— settle() 返回 False 表示「被顶住了」，这是正常的
  5. 往下拽            给方块加外力，看摩擦力够不够把工件夹住
                        （--pull 应当夹得住，--slip 应当滑下去）
  6. 交互               有窗口时实时显示状态，Esc/Q 退出

本样例**不驱动真机**，随便跑。

运行：
  python3 examples/01_sim_only.py                       # 开窗口跑整套演示
  python3 examples/01_sim_only.py --headless            # 无窗口（跑得快，exit 0）
  python3 examples/01_sim_only.py --object-mm 60        # 换一个 60 mm 的方块
  python3 examples/01_sim_only.py --force 20            # 20 N 夹持力
  python3 examples/01_sim_only.py --speed 0.02          # 慢速收爪（约 2 s 全行程）
  python3 examples/01_sim_only.py --slip 40             # 用 40 N 下拽，看它滑
"""
import argparse
import sys

from _common import add_common_args, status_line  # noqa: I001  (必须先于 litegrip_pybullet)

import pybullet as p

from litegrip_pybullet import (
    APERTURE_OPEN_MM,
    DEFAULT_VELOCITY_M_S,
    MAX_GRIP_FORCE_N,
    QUIT_KEYS,
    STROKE_M,
    GripperSim,
    pressed,
)

#: 工件质量 [kg]。
BOX_MASS_KG = 0.05

#: 工件半高 [m]（沿 z）。
BOX_HALF_Z_M = 0.015


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="样例 01 · 仅仿真：用 PyBullet 驱动 LiteGrip 夹爪（不接真机）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    add_common_args(ap)
    ap.add_argument("--object-mm", type=float, default=40.0,
                    help="抓取方块的宽度 [mm]（默认 40）")
    ap.add_argument("--force", type=float, default=10.0,
                    help=f"夹持力上限 [N]（默认 10，上限 {MAX_GRIP_FORCE_N:g}）")
    ap.add_argument("--speed", type=float, default=DEFAULT_VELOCITY_M_S,
                    help=f"手指速度 [m/s]（默认 {DEFAULT_VELOCITY_M_S:.5f}，"
                         f"即真机的 85 mm/s）")
    ap.add_argument("--hold", type=float, default=1.0,
                    help="夹住后保持观察的时间 [s]（默认 1.0）")
    ap.add_argument("--pull", type=float, default=5.0,
                    help="第一次下拽的力 [N]，应当夹得住（默认 5）")
    ap.add_argument("--slip", type=float, default=15.0,
                    help="第二次下拽的力 [N]，应当会滑（默认 15）")
    return ap.parse_args()


def banner(text: str) -> None:
    print(f"\n── {text} " + "─" * max(0, 58 - len(text)))


def show(sim: GripperSim, label: str = "仿真", *, force: bool = True) -> None:
    print(status_line(
        label,
        fraction=sim.fraction(),
        aperture_mm=sim.aperture_mm(),
        force_n=sim.finger_force_n() if force else None,
    ))


def demo_travel(sim: GripperSim, args: argparse.Namespace) -> None:
    """全行程开合——顺便量一下速度受不受限。"""
    banner("1/4 全行程开合（速度受限）")
    for target, name in ((0.0, "闭合"), (1.0, "张开")):
        sim.command_fraction(target, force_n=args.force, velocity_m_s=args.speed)
        spent, reached = sim.settle()
        flag = "到位" if reached else "没到位（被顶住了）"
        print(f"   → {name}：用掉 {spent:.3f} s 仿真时间 · {flag}")
        show(sim)
    print(f"   单指行程 {STROKE_M * 1000:.2f} mm · 单指速度 {args.speed * 1000:.2f} mm/s"
          f" → 期望单程 ≈ {STROKE_M / args.speed:.2f} s")
    print(f"   两个手指对冲，所以开口变化的速度是这个的两倍："
          f"{args.speed * 2000:.1f} mm/s（真机规格 85 mm/s）")


def demo_midpoint(sim: GripperSim, args: argparse.Namespace) -> None:
    """按归一化开度走到中间位——这是仿真和真机共用的「同一种语言」。"""
    banner("2/4 走到中间位（归一化开度）")
    for fraction in (0.5, 0.25, 0.75):
        sim.command_fraction(fraction, force_n=args.force, velocity_m_s=args.speed)
        spent, reached = sim.settle()
        got = sim.fraction()
        print(f"   命令 {fraction * 100:5.1f}% → 实测 {got * 100:5.1f}% · "
              f"开口 {sim.aperture_mm():5.2f} mm · {spent:.3f} s · "
              f"{'到位' if reached else '没到位'}")
    sim.command_fraction(1.0)
    sim.settle()


def hold_part(sim: GripperSim, body: int, held: bool) -> None:
    """把工件「扶住」/「放开」——质量设 0 时 PyBullet 把它当静态体。

    手指合拢要花 1 s，这 1 s 里工件会自由落体，先掉到夹爪底座上。底座一托住，
    后面的摩擦测试就没意义了（工件是「坐」在底座上，不是被手指夹住的）。所以
    夹之前先扶住，夹好了再放开——这才是真的只靠摩擦挂在两指之间。
    """
    p.changeDynamics(body, -1, mass=0.0 if held else BOX_MASS_KG,
                     physicsClientId=sim.client_id)


def grasp(sim: GripperSim, box: int, args: argparse.Namespace) -> tuple[float, bool]:
    """合拢手指夹住工件，返回 ``(耗时, 是否走到位)``。

    结束时已经放开对工件的「扶持」，并多走几帧——``changeDynamics`` 会把上一步
    算出来的接触点清掉，不跑几步的话 ``contacts()`` 读到的是空的。
    """
    hold_part(sim, box, True)
    sim.command_fraction(0.0, force_n=args.force, velocity_m_s=args.speed)
    # 被工件顶住时 settle 只能等到超时，所以超时按「走完全行程」给，
    # 别用默认的 3 s——那会让窗口白等 2 s。
    spent, reached = sim.settle(timeout_s=STROKE_M / args.speed + 0.3)
    hold_part(sim, box, False)
    sim.run_for(0.05)
    return spent, reached


def demo_grasp(sim: GripperSim, args: argparse.Namespace) -> int:
    """夹一个方块：接触点数、实际夹持力、settle 的语义。"""
    banner(f"3/4 夹住一个 {args.object_mm:g} mm 的方块（{args.force:g} N）")
    center = sim.grasp_center()
    half = args.object_mm / 2000.0  # mm → 半宽 [m]
    box = sim.add_box([half, BOX_HALF_Z_M, BOX_HALF_Z_M], center, mass=BOX_MASS_KG)
    base_top = sim.link_aabb(-1)[1][2]
    print(f"   方块放在两指中间 {[round(c, 4) for c in center]}，"
          f"半宽 {half * 1000:.1f} mm")
    print(f"   方块下沿 {center[2] - BOX_HALF_Z_M:.4f} m，夹爪底座顶面 {base_top:.4f} m"
          f"——离底座 {(center[2] - BOX_HALF_Z_M - base_top) * 1000:.1f} mm，"
          f"只有手指碰得到它")

    spent, reached = grasp(sim, box, args)
    points = sim.contacts(box)
    touched = sorted({pt[3] for pt in points})
    print(f"   收爪 {spent:.3f} s · settle 到位={reached}"
          f"（夹住东西时应当是 False——手指被工件顶住了）")
    print(f"   开口 {sim.aperture_mm():.2f} mm（方块 {args.object_mm:g} mm，"
          f"差值是接触外壳的余量）")
    print(f"   电机推力 {sim.finger_force_n():.2f} N（上限 {args.force:g} N）· "
          f"接触点 {len(points)} 个，落在 link {touched}"
          f"（都是手指，没有底座）")
    if not points:
        print("   ⚠️ 没有接触点：方块可能没夹住，试试调大 --object-mm 或 --force")
    return box


def pull(sim: GripperSim, box: int, force_n: float, seconds: float) -> float:
    """给工件加一个向下的力，返回它沿 z 的位移 [mm]（负数=下滑）。"""
    z0 = p.getBasePositionAndOrientation(box, physicsClientId=sim.client_id)[0][2]
    for _ in range(max(1, int(round(seconds / sim.time_step)))):
        p.applyExternalForce(box, -1, [0.0, 0.0, -force_n], [0.0, 0.0, 0.0],
                             p.WORLD_FRAME, physicsClientId=sim.client_id)
        if not sim.step():
            break
    z1 = p.getBasePositionAndOrientation(box, physicsClientId=sim.client_id)[0][2]
    return (z1 - z0) * 1000.0


def demo_pull(sim: GripperSim, box: int, args: argparse.Namespace) -> None:
    """往下拽工件：先小力（夹得住），再大力（滑下去）。"""
    banner("4/4 往下拽，找摩擦力的极限")
    for force_n, expect in ((args.pull, "应当夹得住"), (args.slip, "应当会滑")):
        moved = pull(sim, box, force_n, args.hold)
        slipped = moved < -2.0
        print(f"   拽 {force_n:5g} N · 保持 {args.hold:g} s → 位移 {moved:+7.2f} mm · "
              f"接触点 {len(sim.contacts(box))} 个 · "
              f"{'滑了' if slipped else '没动'}（{expect}）")
    print(f"   力的方向和指面平行，全靠摩擦扛：能扛住的力大致是 "
          f"2 × {args.force:g} N × 摩擦系数。")
    print(f"   想夹得更牢：调大 --force，或者把指面换成带齿/软垫的（提高摩擦系数）")


def demo_release(sim: GripperSim, box: int, args: argparse.Namespace) -> None:
    """松爪：把工件举回两指中间重新夹住，再张开看它掉下去。

    为什么不就地松：上一步 15 N 已经把工件拽到夹爪底座上了，底座托着它，这时候
    张嘴它几乎不动（实测 0.3 mm），什么也说明不了。所以先摆回空中重夹一次——
    这才是「松爪 → 工件掉下去」该有的样子。
    """
    banner("收尾：松爪")
    center = sim.grasp_center()
    base_top = sim.link_aabb(-1)[1][2]
    z_before = p.getBasePositionAndOrientation(
        box, physicsClientId=sim.client_id)[0][2]
    if z_before - BOX_HALF_Z_M <= base_top + 2e-3:
        print(f"   （上一步滑完，方块已经落在夹爪底座上了：下沿 "
              f"{z_before - BOX_HALF_Z_M:.4f} m ≈ 底座顶面 {base_top:.4f} m）")
        print(f"    摆回两指中间重新夹一次，再张开——")
        p.resetBasePositionAndOrientation(box, center, [0.0, 0.0, 0.0, 1.0],
                                          physicsClientId=sim.client_id)
        p.resetBaseVelocity(box, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                            physicsClientId=sim.client_id)
        grasp(sim, box, args)

    z_held = p.getBasePositionAndOrientation(
        box, physicsClientId=sim.client_id)[0][2]
    print(f"   松爪前：方块下沿 {z_held - BOX_HALF_Z_M:.4f} m，"
          f"离夹爪底座顶面 {(z_held - BOX_HALF_Z_M - base_top) * 1000:.1f} mm")
    sim.command_fraction(1.0, force_n=args.force, velocity_m_s=args.speed)
    sim.settle()
    sim.run_for(0.5)  # 给方块一点下落时间
    z_dropped = p.getBasePositionAndOrientation(
        box, physicsClientId=sim.client_id)[0][2]
    print(f"   手指张开 → 方块下落 {(z_held - z_dropped) * 1000:.2f} mm，"
          f"落在夹爪底座上（接触点 {len(sim.contacts(box))} 个）")
    print(f"   夹住时靠的是两指的摩擦力，张开就没有了——这就是夹爪和「托住」的区别")


def interactive(sim: GripperSim) -> None:
    """有窗口时：实时刷状态，Esc/Q 或关窗退出。"""
    if not sim.gui:
        return
    banner("交互：Esc / Q 或直接关窗口退出")
    while sim.connected():
        if pressed(sim.keyboard_events(), QUIT_KEYS):
            print("   收到退出键")
            break
        sim.status_text(
            f"开度 {sim.fraction() * 100:5.1f}%  "
            f"开口 {sim.aperture_mm():5.2f} mm  "
            f"力 {sim.finger_force_n():5.2f} N"
        )
        if not sim.step():
            break


def main() -> int:
    args = parse_args()
    args.force = max(0.0, min(MAX_GRIP_FORCE_N, args.force))

    print("样例 01 · 仅仿真（不接真机，不会动真机）")
    sim = GripperSim(urdf_path=args.urdf, gui=not args.headless,
                     max_force_n=args.force, velocity_m_s=args.speed)
    try:
        print(f"   URDF      {sim.urdf_path}")
        print(f"   手指关节  {sim.finger_joints}")
        print(f"   初始状态  {sim.fraction() * 100:.1f}% · "
              f"开口 {sim.aperture_mm():.2f} mm")
        sim.focus_camera()

        demo_travel(sim, args)
        demo_midpoint(sim, args)
        box = demo_grasp(sim, args)
        demo_pull(sim, box, args)
        demo_release(sim, box, args)
        show(sim)

        interactive(sim)
    finally:
        sim.disconnect()
    print("\n完成。真机版本见 examples/02_sim_to_real.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
