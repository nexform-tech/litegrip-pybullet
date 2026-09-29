#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 02 · 仿真运动 — 速度受限的全行程开合，以及按归一化开度定位

不用真机、不碰 CAN：命令 → 手指按额定速度走 → settle() 等它到位，全在 PyBullet 里。
想看夹爪怎么夹住东西，继续看 examples/03_grasp.py。

演示:
  sim.command_fraction(fraction, force_n=, velocity_m_s=)  命令一个开度
  sim.settle()                    等手指真的到目标，返回 (用时, 是否到位)
  sim.fraction() / sim.aperture_mm()   读回实测开度
  sim.status_text() / sim.step()  窗口里刷状态、推进仿真

运行:
  python3 examples/02_move_sim.py                       # 开窗口跑两段演示
  python3 examples/02_move_sim.py --headless            # 无窗口（跑得快，exit 0）
  python3 examples/02_move_sim.py --speed 0.02          # 慢速收爪（约 2 s 全行程）
  python3 examples/02_move_sim.py --force 20            # 20 N 夹持力上限
"""
import argparse
import sys

from _common import add_common_args, status_line  # noqa: I001  (必须先于 litegrip_pybullet)

from litegrip_pybullet import (
    DEFAULT_VELOCITY_M_S,
    MAX_GRIP_FORCE_N,
    QUIT_KEYS,
    STROKE_M,
    GripperSim,
    pressed,
)


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="样例 02 · 仿真运动：速度受限的开合与按开度定位（不接真机）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    add_common_args(ap)
    ap.add_argument("--force", type=float, default=10.0,
                    help=f"夹持力上限 [N]（默认 10，上限 {MAX_GRIP_FORCE_N:g}）")
    ap.add_argument("--speed", type=float, default=DEFAULT_VELOCITY_M_S,
                    help=f"手指速度 [m/s]（默认 {DEFAULT_VELOCITY_M_S:.5f}，"
                         f"即真机的 85 mm/s）")
    return ap.parse_args()


def show(sim: GripperSim, label: str = "仿真", *, force: bool = True) -> None:
    print(status_line(
        label,
        fraction=sim.fraction(),
        aperture_mm=sim.aperture_mm(),
        force_n=sim.finger_force_n() if force else None,
    ))


def demo_travel(sim: GripperSim, args: argparse.Namespace) -> None:
    """全行程开合——顺便量一下速度受不受限。"""
    print("\n[1] 全行程开合（速度受限）")
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
    print("\n[2] 走到中间位（归一化开度）")
    for fraction in (0.5, 0.25, 0.75):
        sim.command_fraction(fraction, force_n=args.force, velocity_m_s=args.speed)
        spent, reached = sim.settle()
        got = sim.fraction()
        print(f"   命令 {fraction * 100:5.1f}% → 实测 {got * 100:5.1f}% · "
              f"开口 {sim.aperture_mm():5.2f} mm · {spent:.3f} s · "
              f"{'到位' if reached else '没到位'}")
    sim.command_fraction(1.0)
    sim.settle()


def interactive(sim: GripperSim) -> None:
    """有窗口时：实时刷状态，Esc/Q 或关窗退出。"""
    if not sim.gui:
        return
    print("\n[3] 实时状态（Esc / Q 退出）")
    while sim.connected():
        if pressed(sim.keyboard_events(), QUIT_KEYS):
            print("\n   收到退出键")
            break
        line = status_line(
            "仿真",
            fraction=sim.fraction(),
            aperture_mm=sim.aperture_mm(),
            force_n=sim.finger_force_n(),
        )
        print(line, end="\r")
        sim.status_text(
            f"开度 {sim.fraction() * 100:5.1f}%  "
            f"开口 {sim.aperture_mm():5.2f} mm  "
            f"力 {sim.finger_force_n():5.2f} N"
        )
        if not sim.step():
            break
    print()


def main() -> int:
    args = parse_args()
    args.force = max(0.0, min(MAX_GRIP_FORCE_N, args.force))

    print("样例 02 · 仿真运动（不接真机，不会动真机）")
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
        show(sim)

        interactive(sim)
    finally:
        sim.disconnect()
    print("\n完成。想夹住东西，继续看 examples/03_grasp.py"
          "；真机版本见 examples/05_dual_control.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
