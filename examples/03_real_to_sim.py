#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 03 · 真机控制仿真：把真机的位置实时镜像到 PyBullet 里

方向是 **真机 → 仿真**：真机是「主」，仿真只是显示器。每一帧读一次真机的
位置，然后把仿真手指瞬移过去（``reset_fraction``，纯运动学，不跑动力学）——
所以画面显示的就是真机现在的样子，没有跟随延迟、也不会自己漂。

两种用法：

  * **用手推着看**（推荐，最能看出镜像效果）：加 ``--zero-gravity``，真机的
    电机失力，可以用手推动手指；仿真窗口会跟着你的手走。运行中按 **Z** 也能
    随时切换失力/使能。
  * **看别人的程序驱动**：不加 ``--zero-gravity`` 时真机自己保持位置；如果
    有另一个程序（或你的上位机）在给真机发指令，仿真同样会跟着显示。

按键：
  Z         真机失力（可用手推）/ 恢复使能
  Esc / Q   退出（退出前会恢复使能，让真机自己保持住位置）

⚠️ ``--zero-gravity`` 时真机是**软**的：手指可以被推动，也会因为重力或外力
自己滑动。托住夹爪再看，别让它在行程中间突然松掉。

运行：
  python3 examples/03_real_to_sim.py --zero-gravity     # 用手推，仿真跟着动
  python3 examples/03_real_to_sim.py                    # 只镜像，不碰真机
  python3 examples/03_real_to_sim.py --headless         # 无窗口，只看终端读数
  python3 examples/03_real_to_sim.py --duration 10      # 看 10 s 后自动退出
"""
import argparse
import sys
import time

from _common import (  # noqa: I001  (必须先于 litegrip_pybullet)
    add_common_args,
    add_hardware_args,
    open_real_gripper,
    rad_to_fraction,
    status_line,
)

from litegrip_pybullet import (
    QUIT_KEYS,
    GripperSim,
    fraction_to_aperture_mm,
    pressed,
)

#: 发 MIT 帧 / 刷新镜像的频率 [Hz]，和 SDK 自己的流式循环一致。
FRAME_HZ = 200.0
FRAME_DT = 1.0 / FRAME_HZ

#: 终端读数的最小刷新间隔 [s]。
PRINT_DT = 0.5

#: 切换失力/使能的按键。
ZERO_GRAVITY_KEY = ord("z")


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="样例 03 · 真机控制仿真：把真机位置镜像到 PyBullet",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    add_common_args(ap)
    add_hardware_args(ap)
    ap.add_argument("--zero-gravity", action="store_true",
                    help="启动就让真机失力（可用手推动手指，仿真跟着走）")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="跑多少秒后自动退出（默认 0 = 一直跑到 Esc/Q 或关窗口）")
    return ap.parse_args()


def mirror(sim: GripperSim, fraction: float) -> None:
    """把仿真手指瞬移到真机的开度上。

    用 ``reset_fraction``（运动学瞬移）而不是 ``command_fraction``（跑动力学
    追过去）：镜像要的是「真机现在在哪」，不是「仿真打算去哪」。瞬移不会有
    跟随滞后，也不会因为仿真里的接触、惯性而偏离真机。
    """
    sim.reset_fraction(fraction)


def set_zero_gravity(gripper, on: bool, was_on: bool) -> bool:
    """开/关真机失力模式，返回新的状态（没变就原样返回）。"""
    if on == was_on:
        return was_on
    if on:
        gripper.enter_zero_gravity()  # 发一帧 kp=0/kd=0 打底
        print("   [真机] 失力 —— 可以推动手指了（再按 Z 恢复）")
    else:
        gripper.exit_zero_gravity()   # 锁在当前位，回到正常闭环
        print("   [真机] 恢复使能（锁在当前位置）")
    return on


def main() -> int:
    args = parse_args()

    print("样例 03 · 真机控制仿真")
    gripper = open_real_gripper(args)

    # 重力设 0：镜像是纯显示，不需要动力学；免得手指在仿真里自己往下出溜。
    sim = GripperSim(urdf_path=args.urdf, gui=not args.headless, gravity=(0, 0, 0))
    sim.focus_camera()

    print(f"   仿真：{sim.urdf_path}")
    print("   Z = 真机失力/恢复 · Esc/Q = 退出"
          + (f" · {args.duration:g} s 后自动退出" if args.duration > 0 else ""))

    zero_gravity = False
    if args.zero_gravity:
        zero_gravity = set_zero_gravity(gripper, True, zero_gravity)
    else:
        print("   （真机保持使能。想用手推着看镜像，加 --zero-gravity "
              "或运行中按 Z）")

    started = time.monotonic()
    last_frame = 0.0
    last_print = 0.0
    frames = 0

    try:
        while sim.connected():
            events = sim.keyboard_events()
            if pressed(events, QUIT_KEYS):
                print("\n收到退出键")
                break
            if pressed(events, (ZERO_GRAVITY_KEY,)):
                zero_gravity = set_zero_gravity(gripper, not zero_gravity,
                                                zero_gravity)

            now = time.monotonic()
            if args.duration > 0 and now - started >= args.duration:
                print(f"\n跑满 {args.duration:g} s，退出")
                break

            # 失力模式要持续发 kp=0/kd=0 的帧维持；正常模式只需要 poll。
            # 两种都只在这一个线程里收发，不会有第二个线程抢 CAN 帧。
            if zero_gravity and now - last_frame >= FRAME_DT:
                last_frame = now
                gripper.send_mit_frame(q=0.0, kp=0.0, kd=0.0)
                frames += 1
            state = gripper.get_state(wait=False)

            real_fraction = rad_to_fraction(gripper, state.position_rad)
            mirror(sim, real_fraction)

            sim.status_text(
                f"真机 {real_fraction * 100:5.1f}%   "
                f"开口 {fraction_to_aperture_mm(real_fraction):5.2f} mm   "
                f"力 {state.force_n:5.2f} N   "
                + ("失力中（可手推）" if zero_gravity else "使能中")
            )

            if now - last_print >= PRINT_DT:
                last_print = now
                print("  " + status_line(
                    "真机", fraction=real_fraction,
                    aperture_mm=fraction_to_aperture_mm(real_fraction),
                    sdk_mm=state.position_mm, force_n=state.force_n,
                    moving=bool(state.is_moving),
                ))

            if not sim.step():
                break
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C")
    finally:
        if zero_gravity:
            gripper.exit_zero_gravity()  # 别把真机留在「软」的状态下
            print("[真机] 已恢复使能")
        gripper.disconnect()
        sim.disconnect()
        print("[真机] 已断开")

    print(f"完成（{frames} 帧零重力指令）。"
          f"反向的（仿真 → 真机）见 examples/02_sim_to_real.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
