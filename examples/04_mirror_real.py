#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 04 · 镜像模式 — 仿真跟随真机的开度同步显示

方向是 **真机 → 仿真**：真机是主，仿真只是显示器。每一帧读一次真机的位置，就把仿真
手指瞬移过去，所以画面显示的就是真机现在的样子，没有跟随延迟、也不会自己漂。

用法:
  不给选项        默认模式：持续发「锁在实测位置」的保持帧，手指有刚度、推它会顶回来
  --zero-gravity  真机的电机失力，可以用手推着走（推荐，最能看出镜像效果）——这时
                  手指也会因重力或外力自己滑动，托住夹爪再看
  --passive       只连接、只读，不使能也不发帧，由别的程序去驱动真机
  Z               运行中随时在失力 / 使能之间切换
  Esc / Q         退出（退出前会失能：手指会松、夹着的东西会掉）

前提: 真机接在 CAN 总线（默认 can0，用 --channel 换）· 装好 litegrip SDK · 有一份
      这台夹爪的标定（不给 --calib 就用 SDK 出厂那份）。装 SDK、选标定、为什么
      「只是看」也得发帧见 examples/README.zh-CN.md。

运行:
  python3 examples/04_mirror_real.py --zero-gravity --calib /path/to/这台夹爪的标定.json
  python3 examples/04_mirror_real.py                    # 只镜像（发锁位帧保活）
  python3 examples/04_mirror_real.py --passive          # 不使能、一帧不发，等别人喂
  python3 examples/04_mirror_real.py --headless         # 无窗口，只看终端读数
  python3 examples/04_mirror_real.py --duration 10      # 看 10 s 后自动退出
"""
import argparse
import sys
import time

from _common import (  # noqa: I001  (必须先于 litegrip_pybullet)
    FRESH_WAIT_S,
    add_common_args,
    add_hardware_args,
    fresh_state,
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

# 发 MIT 帧 / 刷新镜像的频率 [Hz]，和 SDK 自己的流式循环一致。
#
# 这个频率不只是「运动时才用」：**使能态**的电机静默约 MEASURED_COMM_LOSS_S 就锁
# 进通信丢失故障——红灯闪、位置照读、指令一律不执行。本样例即使只是「看」，也必须
# 按这个频率持续发帧；只 poll 不喂帧，看一秒就把真机看哑了。
FRAME_HZ = 200.0
FRAME_DT = 1.0 / FRAME_HZ

# 实测的通信超时闩锁时间 [s]：使能态的电机静默这么久就报 0xD（SDK 在真机上量到
# 的）。只用来把话说具体，逻辑上不依赖它——电机的 ``TIMEOUT`` 寄存器读到过 8000、
# 也读到过 0，和这个实测值都对不上（SDK 标注「待查」）。
MEASURED_COMM_LOSS_S = 0.9

# 终端读数的最小刷新间隔 [s]。
PRINT_DT = 0.5

# 切换失力/使能的按键。
ZERO_GRAVITY_KEY = ord("z")


def parse_args():
    ap = argparse.ArgumentParser(
        description="样例 04 · 镜像模式：把真机的位置实时镜像到 PyBullet 里",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    add_common_args(ap)
    add_hardware_args(ap)
    ap.add_argument("--zero-gravity", action="store_true",
                    help="启动就让真机失力（可用手推动手指，仿真跟着走）")
    ap.add_argument("--passive", action="store_true",
                    help="只连接、只读：**不使能**、一帧都不发——已经有别的程序在"
                         "驱动真机时用；单跑的话别加（没人喂帧，真机会锁通信超时"
                         "故障）")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="跑多少秒后自动退出（默认 0 = 一直跑到 Esc/Q 或关窗口）")
    return ap.parse_args()


def mirror(sim, fraction):
    """把仿真手指瞬移到真机的开度上。

    用 ``reset_fraction``（运动学瞬移）而不是 ``command_fraction``（跑动力学追
    过去）：镜像要的是「真机现在在哪」，不是「仿真打算去哪」。瞬移不会有跟随滞后，
    也不会因为仿真里的接触、惯性而偏离真机。
    """
    sim.reset_fraction(fraction)


def set_zero_gravity(gripper, on, was_on):
    """开/关真机失力模式，返回新的状态（没变就原样返回）。"""
    if on == was_on:
        return was_on
    if on:
        gripper.enter_zero_gravity()  # 发一帧 kp=0/kd=0 打底
        print("   [真机] 失力——可以推动手指了（再按 Z 恢复）")
    else:
        gripper.exit_zero_gravity()   # 锁在当前位，回到正常闭环
        print("   [真机] 恢复使能（锁在当前位置）")
    return on


def main():
    args = parse_args()

    print("样例 04 · 镜像模式（真机 → 仿真）")
    # --passive 是「只看别人的」：**不使能**、一帧都不发。以前这里也是使能了再一帧
    # 不发，于是名不副实——使能态的电机静默约 0.9 s 就自己锁 0xD 通信丢失故障。
    # 不使能的电机不需要帧，也就没有这个故障可闩，而且不用给手指任何刚度，
    # 「只读」才是真的只读。
    gripper = open_real_gripper(args, enable=not args.passive)

    # 重力设 0：镜像是纯显示，不需要动力学；免得手指在仿真里自己往下出溜。
    sim = GripperSim(urdf_path=args.urdf, gui=not args.headless, gravity=(0, 0, 0))
    sim.focus_camera()

    print(f"   [仿真] {sim.urdf_path}")
    print("   Z = 真机失力/恢复 · Esc/Q = 退出"
          + (f" · {args.duration:g} s 后自动退出" if args.duration > 0 else ""))

    zero_gravity = False
    if args.passive:
        print("   （--passive：不使能、一帧都不发，只读。真机得由别的程序喂帧；"
              "这里不抢总线）")
        if args.zero_gravity:
            print("   （--zero-gravity 在 --passive 下无效：失力也要发帧）")
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
    hold_rad = None                 # 锁位帧的目标：读到实测位置的那一刻定一次
    starved = 0                     # 读不到状态帧、于是没发成锁位帧的次数

    try:
        while sim.connected():
            events = sim.keyboard_events()
            if pressed(events, QUIT_KEYS):
                print("\n   收到退出键")
                break
            if pressed(events, (ZERO_GRAVITY_KEY,)) and not args.passive:
                was_zero_gravity = zero_gravity
                zero_gravity = set_zero_gravity(gripper, not zero_gravity,
                                                zero_gravity)
                if was_zero_gravity and not zero_gravity:
                    # 刚从失力恢复：手指可能已经被推到别处了，锁位目标必须重新取
                    # **现在**的位置。还用失力之前那个目标的话，本样例就是在命令
                    # 电机走回原处——一次没人要求的运动（SDK 的 exit_zero_gravity
                    # 自己也是锁在当前位置）。
                    state_now = fresh_state(gripper)
                    hold_rad = state_now.position_rad if state_now else None

            now = time.monotonic()
            if args.duration > 0 and now - started >= args.duration:
                print(f"\n跑满 {args.duration:g} s，退出")
                break

            # 读真机位置：**必须**是新鲜读数，所以走 fresh_state（等到一帧新的状态
            # 帧）而不是 get_state(wait=False) 的缓存。本样例展示的就是「真机现在
            # 在哪」，而缓存里可能是冻结的旧值——读不到时更是 MotorState 的初值
            # 0.0，照它渲染画面等于撒谎。读不到就把画面停在最后一次读数上，并在
            # 窗口和终端里都说明。
            state = fresh_state(gripper)
            stale = state is None
            if stale:
                state = gripper.get_state(wait=False)     # 只为把画面停住

            if not stale and hold_rad is None:
                # 锁位帧的目标只定这一次，之后一直重发同一个值。每拍都拿当次读数
                # 现造目标的话，一次读数冻结就会变成一条指向伪值的新指令——那是
                # 阶跃，见 examples/05 里 hold_frame 的说明。
                hold_rad = state.position_rad
                if not args.passive:
                    print(f"   [真机] 锁在实测位置 {hold_rad:+.4f} rad"
                          "（零前馈，不命令运动）")

            # 两种模式都必须**持续发帧**，理由见 FRAME_HZ：使能态的电机静默约
            # MEASURED_COMM_LOSS_S 就锁通信丢失故障。只在这一个线程里收发，不会
            # 有第二个线程抢 CAN 帧。
            if not args.passive and now - last_frame >= FRAME_DT:
                last_frame = now
                if zero_gravity:
                    # 失力：kp=0/kd=0，手指可以被手推动（kp=0 时 q 给什么都不出力）
                    gripper.send_mit_frame(q=0.0, kp=0.0, kd=0.0)
                    frames += 1
                elif hold_rad is not None:
                    # 正常模式：锁在**实测位置**（零前馈、目标就是它现在的位置）
                    # ——不命令任何运动，只是让手指有刚度、把超时计数器喂上。
                    gripper.send_mit_frame(q=hold_rad, kp=gripper.config.kp,
                                           kd=gripper.config.kd)
                    frames += 1
                else:
                    # 还没读到过位置：不造锁位帧——目标只能是实测位置，拿缓存的伪值
                    # 当目标是发一条阶跃指令出去，比少发一帧危险得多。
                    # 代价说清楚：DM 电机只为收到的指令帧回一帧，不会自己持续发帧，
                    # 所以这一段「等第一帧」没有自己的帧流可依靠，只吃 ``enable()``
                    # 刚流完留在缓冲里的那几帧。缓冲被读空之后这一段就再也等不到了，
                    # 本样例也**不会**为了催帧去发任何东西——那要么是运动指令（不行），
                    # 要么是把手指放松的零刚度帧（夹着工件会掉）。
                    starved += 1
                    if starved % 200 == 1:
                        print(f"   [真机] 还在等状态帧（第 {starved} 次）："
                              "读到实测位置才开始发锁位帧——"
                              "目标必须是实测位置，不能拿缓存的伪值造。")

            real_fraction = rad_to_fraction(gripper, state.position_rad)
            mirror(sim, real_fraction)

            sim.status_text(
                f"真机 {real_fraction * 100:5.1f}%   "
                f"开口 {fraction_to_aperture_mm(real_fraction):5.2f} mm   "
                f"力 {state.force_n:5.2f} N   "
                + ("未读到状态帧（画面停在最后读数）" if stale else
                   ("失力中（可手推）" if zero_gravity else
                    ("只读（不发帧）" if args.passive else "锁位中")))
            )

            if now - last_print >= PRINT_DT:
                last_print = now
                if stale:
                    print(f"  [真机] 读不到状态帧（等了 {FRESH_WAIT_S * 1000:.0f}"
                          " ms）：画面停在最后一次读数上，位置和力都不可信。")
                else:
                    print("  " + status_line(
                        "真机", fraction=real_fraction,
                        aperture_mm=fraction_to_aperture_mm(real_fraction),
                        sdk_mm=state.position_mm, force_n=state.force_n,
                        moving=bool(state.is_moving),
                    ))

            if not sim.step():
                break
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        if not args.passive:
            # 退出前**失能**（0xFD），而不是只发一帧 exit_zero_gravity() 就走：
            # 那一帧之后没人再喂，使能态的电机静默约 0.9 s 就锁 0xD。失能则不需要
            # 任何帧，也就没有故障可闩。
            # 代价和 --zero-gravity 退出时一样：手指变软、可能因自重滑动。
            gripper.disable()
            print("[真机] 已失能（0xFD）：手指会松、可能因自重滑动；"
                  "不会在电机上留下通信超时故障")
        # disconnect() 只在使能过的时候才会补一帧 0xFD，所以 --passive 这条路到
        # 这里为止确实一帧都没发出去。
        gripper.disconnect()
        sim.disconnect()
        print("[真机] 已关闭"
              + ("（--passive：未使能、未发送任何帧）" if args.passive else ""))

    what = "一帧都没发" if args.passive else f"{frames} 帧保活/零重力指令"
    print(f"完成（{what}）。"
          f"反向的（仿真 → 真机）见 examples/05_dual_control.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
