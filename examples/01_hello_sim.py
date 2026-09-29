#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 01 · 独立仿真 — 创建仿真夹爪并读取状态（只读，不运动）

只加载模型、读它现在在哪，一个运动指令都不发。适合第一次跑、或者只想确认
PyBullet 和自带 URDF 能起来的时候。

演示:
  GripperSim(urdf_path=..., gui=True)   加载自带 URDF，打开可视化窗口
  sim.urdf_path / sim.finger_joints     模型身份：用的哪份描述、哪两个移动副手指
  sim.joint_values()                    URDF 关节值 [m]（0 = 张开）
  sim.fraction()                        归一化开度（0 闭合 … 1 张开）
  sim.aperture_mm()                     钳口间隙 [mm]（由网格几何算出）
  fraction_to_sdk_mm(fraction)          等效的 SDK 刻度 [mm]（真机 goto 用的那个）
  sim.status_text() / sim.step()        窗口里刷状态、推进仿真（读按键要靠它）
  sim.keyboard_events() + pressed()     Esc / Q 退出
  sim.disconnect()                      关闭仿真

运行:
  python3 examples/01_hello_sim.py                  # 开窗口看实时状态
  python3 examples/01_hello_sim.py --headless       # 无窗口，打印完直接退出
"""
import argparse
import sys

from _common import add_common_args, status_line  # noqa: I001  (必须先于 litegrip_pybullet)

from litegrip_pybullet import (
    APERTURE_CLOSED_MM,
    APERTURE_OPEN_MM,
    DEFAULT_VELOCITY_M_S,
    MAX_GRIP_FORCE_N,
    N_FINGERS,
    QUIT_KEYS,
    STROKE_M,
    GripperSim,
    fraction_to_sdk_mm,
    pressed,
)


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="样例 01 · 独立仿真：创建仿真夹爪并读取状态"
                    "（只读，不运动，不接真机）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    add_common_args(ap)
    return ap.parse_args()


def show_model(sim: GripperSim) -> None:
    """模型常量：全都是只读的，一个字都没命令给电机。"""
    print("\n[1] 模型常量")
    print(f"   URDF      {sim.urdf_path}")
    print(f"   手指关节  {sim.finger_joints}")
    print(f"   单指行程  {STROKE_M * 1000:.2f} mm（0 = 张开，走满行程 = 闭合）")
    print(f"   钳口间隙  {APERTURE_CLOSED_MM:.2f} mm（闭合）… "
          f"{APERTURE_OPEN_MM:.2f} mm（张开）")
    print(f"   手指数量  {N_FINGERS}")
    print(f"   力上限    {sim.max_force_n:g} N（本库的上限是 {MAX_GRIP_FORCE_N:g} N）")
    print(f"   默认速度  {DEFAULT_VELOCITY_M_S * 1000:.2f} mm/s（单指速度；"
          f"两指对冲，开口变化是它的两倍）")
    print(f"   物理步长  {sim.time_step * 1000:g} ms")


def show_opening(sim: GripperSim) -> None:
    """同一个开度有三种写法——这是本仓库最容易混起来的地方。"""
    joints = ", ".join(f"{v:.6f}" for v in sim.joint_values())
    print("\n[2] 当前开度（三种写法）")
    print(f"   归一化开度  fraction()            = {sim.fraction() * 100:5.1f}%"
          f"     ← 两边唯一共用的量")
    print(f"   关节值      joint_values()        = ({joints}) m")
    print(f"   钳口间隙    aperture_mm()         = {sim.aperture_mm():6.2f} mm"
          f"（两个指面的实际距离）")
    print(f"   SDK 毫米    fraction_to_sdk_mm()  = "
          f"{fraction_to_sdk_mm(sim.fraction()):6.2f} mm"
          f"（真机 goto(mm) 的刻度，跟钳口间隙不是一回事）")


def interactive(sim: GripperSim) -> None:
    """有窗口时：实时刷状态，Esc/Q 或关窗退出。只读——不发任何指令。"""
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

    print("样例 01 · 独立仿真（只读，不运动；不接真机，不会动真机）")
    sim = GripperSim(urdf_path=args.urdf, gui=not args.headless)
    try:
        sim.focus_camera()
        show_model(sim)
        show_opening(sim)
        interactive(sim)
    finally:
        sim.disconnect()
    print("\n完成。下一步看 examples/02_move_sim.py，让手指动起来")
    return 0


if __name__ == "__main__":
    sys.exit(main())
