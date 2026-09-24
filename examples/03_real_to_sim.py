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
  * **看别人的程序驱动**：加 ``--passive``，本样例一帧都不发，只读；由那个
    程序去喂真机。

⚠️ 为什么必须持续发帧（默认模式）：电机的 ``TIMEOUT`` 寄存器（RID 9，本机实测
8000）= CAN 通信超时保护，**连续这么久收不到帧就锁进通信丢失故障**——红灯闪、
位置照读、指令一律不执行。所以「只是看」也得喂帧，见 :data:`FRAME_HZ`。默认
模式发的是「锁在实测位置」的保持帧（零前馈、目标就是它现在的位置），不命令任何
运动，但会让手指有刚度、推它它会顶回来。要看别人的程序驱动就用 ``--passive``，
否则两边发的帧会互相打架。

按键：
  Z         真机失力（可用手推）/ 恢复使能
  Esc / Q   退出（退出前会恢复使能，让真机自己保持住位置）

⚠️ ``--zero-gravity`` 时真机是**软**的：手指可以被推动，也会因为重力或外力
自己滑动。托住夹爪再看，别让它在行程中间突然松掉。

运行：
  python3 examples/03_real_to_sim.py --zero-gravity     # 用手推，仿真跟着动
  python3 examples/03_real_to_sim.py                    # 只镜像（发锁位帧保活）
  python3 examples/03_real_to_sim.py --passive          # 一帧不发，等别人喂
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
#:
#: ⚠️ 这个频率不只是「运动时才用」：电机的 ``TIMEOUT`` 寄存器（DM 寄存器表
#: RID 9，这台机器实测 8000）= CAN 通信超时保护，**连续这么久收不到帧就锁进
#: 通信丢失故障**——红灯闪、位置照读、指令一律不执行。本样例即使只是「看」，
#: 也必须按这个频率持续发帧；只 poll 不喂帧，看几秒就把真机看哑了。
FRAME_HZ = 200.0
FRAME_DT = 1.0 / FRAME_HZ

#: 上面那段说的通信超时保护时长 [s]：这台机器的 ``TIMEOUT`` 寄存器读出 8000。
#: 只用来把话说具体（``--status`` 会读真值），逻辑上不依赖它。
KEEPALIVE_TIMEOUT_S = 8.0

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
    ap.add_argument("--passive", action="store_true",
                    help="一帧都不发，只读——已经有别的程序在驱动真机时用；"
                         "单跑的话别加（没人喂帧，真机会锁通信超时故障）")
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
    if args.passive:
        print("   （--passive：一帧都不发，只读。真机得由别的程序喂帧，"
              f"否则 {KEEPALIVE_TIMEOUT_S:g} s 后会锁通信超时故障）")
        if args.zero_gravity:
            print("   （--zero-gravity 在 --passive 下无效：失力也需要发帧）")
    elif args.zero_gravity:
        zero_gravity = set_zero_gravity(gripper, True, zero_gravity)
    else:
        print(f"   （真机保持使能，本样例每 {FRAME_DT * 1000:.0f} ms 发一条"
              "「锁在实测位置」的保持帧——不命令运动，只是防止电机"
              "因收不到帧而锁通信超时故障。想用手推着看镜像，"
              "加 --zero-gravity 或运行中按 Z）")

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
            if pressed(events, (ZERO_GRAVITY_KEY,)) and not args.passive:
                zero_gravity = set_zero_gravity(gripper, not zero_gravity,
                                                zero_gravity)

            now = time.monotonic()
            if args.duration > 0 and now - started >= args.duration:
                print(f"\n跑满 {args.duration:g} s，退出")
                break

            state = gripper.get_state(wait=False)

            # 两种模式都必须**持续发帧**，理由见 :data:`FRAME_HZ`：电机的 CAN
            # 通信超时保护一到就锁通信丢失故障。只在这一个线程里收发，不会有
            # 第二个线程抢 CAN 帧。
            if not args.passive and now - last_frame >= FRAME_DT:
                last_frame = now
                if zero_gravity:
                    # 失力：kp=0/kd=0，手指可以被手推动
                    gripper.send_mit_frame(q=0.0, kp=0.0, kd=0.0)
                else:
                    # 正常模式：锁在**实测位置**（零前馈、目标就是它现在的位置）
                    # ——不命令任何运动，只是让手指有刚度、把超时计数器喂上。
                    gripper.send_mit_frame(q=state.position_rad,
                                           kp=gripper.config.kp,
                                           kd=gripper.config.kd)
                frames += 1

            real_fraction = rad_to_fraction(gripper, state.position_rad)
            mirror(sim, real_fraction)

            sim.status_text(
                f"真机 {real_fraction * 100:5.1f}%   "
                f"开口 {fraction_to_aperture_mm(real_fraction):5.2f} mm   "
                f"力 {state.force_n:5.2f} N   "
                + ("失力中（可手推）" if zero_gravity else
                   ("只读（不发帧）" if args.passive else "锁位中"))
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

    what = "一帧都没发" if args.passive else f"{frames} 帧保活/零重力指令"
    print(f"完成（{what}）。"
          f"反向的（仿真 → 真机）见 examples/02_sim_to_real.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
