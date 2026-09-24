#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 02 · 仿真控制真机：在 PyBullet 窗口里调目标，按 Enter 才下发

方向是 **仿真 → 真机**：仿真这边是「主」，你在这里决定开度和夹持力，真机是
「从」，收到指令才动。

流程（全在 PyBullet 窗口里操作）：

  1. 拖滑条「目标开度」——仿真手指**实时跟着滑条走**，这是预览，真机还没动。
     预览本身也受 85 mm/s 的速度限制，所以「看到什么速度，真机就是什么速度」。
  2. 拖滑条「夹持力」——下发时作为前馈力矩给电机（夹住工件时的推力）。
  3. 按 **Enter / 空格** 才真的下发：真机以 --duration 流式走过去（200 Hz 发帧）。
     仿真显示的是**命令值**，真机的实测值在旁边对照——两者的差就是真机的
     跟随误差，顶住工件时这个差会一直留着，正好用来判断「夹到了没有」。
  4. **Esc / Q** 退出，退出前会让真机停住（停止发帧，电机保持当前位置）。

⚠️ 会驱动真机！第一次跑务必先 dry-run：

    python3 examples/02_sim_to_real.py --dry-run    # 只开窗口，绝不碰 CAN

真机跑：

    python3 examples/02_sim_to_real.py                       # can0, 10 N, 1 s
    python3 examples/02_sim_to_real.py --force 20 --duration 2
    python3 examples/02_sim_to_real.py --channel can1        # 换 CAN 口

为什么要自己发帧：SDK 的 move_to()/goto_rad() 内部是 control_mit_stream()，
它自己 sleep 5 ms 循环、不让出控制权，PyBullet 窗口会卡住、也读不到按键。
所以这里用 SDK 公开的 send_mit_frame() + poll() 自己组循环——这正是它们被
公开出来的用途（自定义控制循环，自己管时序）。
"""
import argparse
import sys
import time

from _common import (  # noqa: I001  (必须先于 litegrip_pybullet)
    SAFETY_BANNER,
    add_common_args,
    add_hardware_args,
    fraction_to_target_rad,
    import_litegrip,
    open_real_gripper,
    rad_to_fraction,
    status_line,
)

import pybullet as p

from litegrip_pybullet import (
    CONFIRM_KEYS,
    MAX_GRIP_FORCE_N,
    QUIT_KEYS,
    GripperSim,
    fraction_to_aperture_mm,
    pressed,
)

#: MIT 帧的发送频率 [Hz]。达妙电机要持续收帧才保持力矩；SDK 自己用的也是
#: 200 Hz（5 ms 一帧），这里对齐。
FRAME_HZ = 200.0
FRAME_DT = 1.0 / FRAME_HZ

#: 终端状态刷新的最小间隔 [s]（窗口里是每帧刷，终端刷太快没法看）。
PRINT_DT = 0.5

#: dry-run 时没有真机，用一个名义值（只会打出来，不会真的发帧）。
NOMINAL_N_TO_NM = 0.1


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="样例 02 · 仿真控制真机：滑条设目标，按 Enter 下发",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    add_common_args(ap)
    add_hardware_args(ap)
    ap.add_argument("--dry-run", action="store_true",
                    help="不连真机、不下发任何指令（只开窗口看流程）")
    ap.add_argument("--force", type=float, default=10.0,
                    help=f"夹持力前馈 [N]（默认 10，上限 {MAX_GRIP_FORCE_N:g}）")
    ap.add_argument("--duration", type=float, default=1.0,
                    help="一条指令流式走多久 [s]（默认 1.0）")
    return ap.parse_args()


class StreamMove:
    """一条流式指令：在 duration 内以 200 Hz 重复发同一帧 MIT 指令。

    这就是 ``control_mit_stream`` 干的事，搬进主循环是为了把「等待」拆开，
    好让每帧还能渲染窗口、读键盘、刷状态。DM 电机要求持续收帧才保持力矩，
    所以不能只发一帧然后 sleep 到结束。
    """

    def __init__(self, target_rad: float, tau_nm: float, duration_s: float,
                 kp: float, kd: float) -> None:
        self.target_rad = target_rad
        self.tau_nm = tau_nm
        self.kp = kp
        self.kd = kd
        self.deadline = time.monotonic() + max(0.0, float(duration_s))
        self.frames = 0

    def due(self, now: float, last_sent: float) -> bool:
        """该发下一帧了吗（按 :data:`FRAME_HZ` 限速）。"""
        return now - last_sent >= FRAME_DT

    def expired(self, now: float) -> bool:
        """这条指令的时间走完了吗。"""
        return now >= self.deadline

    def send(self, gripper) -> bool:
        """发一帧，返回是否发出去了。"""
        if gripper.send_mit_frame(q=self.target_rad, kp=self.kp, kd=self.kd,
                                  tau=self.tau_nm):
            self.frames += 1
            return True
        return False


def make_sliders(gripper, default_force_n: float) -> tuple[int, int]:
    """建两个滑条：目标开度 % / 夹持力 N。返回 ``(开度 id, 力 id)``。

    开度滑条从真机当前开度起步，这样一上来不会因为滑条值≠真机位置而「跳」。
    """
    start_pct = 100.0
    if gripper is not None:
        state = gripper.get_state(wait=True)
        start_pct = rad_to_fraction(gripper, state.position_rad) * 100.0
    target_id = p.addUserDebugParameter("目标开度 %", 0.0, 100.0, start_pct)
    force_id = p.addUserDebugParameter("夹持力 N", 0.0, MAX_GRIP_FORCE_N,
                                       min(default_force_n, MAX_GRIP_FORCE_N))
    return target_id, force_id


def read_real(gripper) -> tuple[float, float, bool] | None:
    """读一帧真机状态，返回 ``(开度, 夹持力 N, 是否在动)``，无真机返回 None。

    ``get_state(wait=False)`` 只做一次非阻塞 poll 再读缓存，不阻塞主循环。
    本样例的 CAN 收发全在主循环这一个线程里，没有第二个线程来抢帧。
    """
    if gripper is None:
        return None
    state = gripper.get_state(wait=False)
    return (
        rad_to_fraction(gripper, state.position_rad),
        state.force_n,
        bool(state.is_moving),
    )


def real_line(fraction: float, force_n: float, moving: bool) -> str:
    """真机的一行状态。真机只报 SDK 刻度，物理开口用仿真那套模型反推。"""
    return status_line(
        "真机", fraction=fraction, aperture_mm=fraction_to_aperture_mm(fraction),
        force_n=force_n, moving=moving,
    )


def main() -> int:
    args = parse_args()
    if args.headless:
        raise SystemExit(
            "❌ 样例 02 靠窗口里的滑条来设目标，没有窗口就没法操作。\n"
            "   想看无窗口的纯仿真请用 examples/01_sim_only.py；\n"
            "   想看真机 → 仿真（不需要操作）请用 examples/03_real_to_sim.py。"
        )

    force_n = max(0.0, min(MAX_GRIP_FORCE_N, args.force))
    if args.dry_run:
        print("样例 02 · 仿真控制真机（--dry-run：不碰真机，只走流程）")
        gripper = None
        n_to_nm = NOMINAL_N_TO_NM
    else:
        print(SAFETY_BANNER)
        print("\n样例 02 · 仿真控制真机")
        gripper = open_real_gripper(args)
        n_to_nm = import_litegrip().UnitConversion.N_TO_NM

    # 一定是开窗的：上面已经拦掉了 --headless，滑条只有 GUI 连接才有
    # （DIRECT 连接里 addUserDebugParameter 会返回 -1）。
    sim = GripperSim(urdf_path=args.urdf, gui=True, max_force_n=force_n)
    sim.focus_camera()
    target_id, force_id = make_sliders(gripper, force_n)
    if target_id < 0 or force_id < 0:
        sim.disconnect()
        raise SystemExit(
            "❌ 建不出滑条（PyBullet 的 addUserDebugParameter 只在 GUI 连接下可用）。\n"
            "   检查是否真的有可用显示，或改用 examples/01_sim_only.py。"
        )

    print(f"   仿真：{sim.urdf_path}")
    print(f"   拖滑条改目标 → 按 Enter/空格 下发（{args.duration:g} s，"
          f"{FRAME_HZ:g} Hz 发帧）→ Esc/Q 退出")
    if gripper is None:
        print("   （dry-run：按 Enter 只打印，不会真的下发）")

    move: StreamMove | None = None
    last_sent = 0.0
    last_print = 0.0
    sent_count = 0

    try:
        while sim.connected():
            events = sim.keyboard_events()
            if pressed(events, QUIT_KEYS):
                print("\n收到退出键")
                break

            now = time.monotonic()
            target_fraction = p.readUserDebugParameter(target_id) / 100.0
            grip_n = p.readUserDebugParameter(force_id)

            # 仿真手指跟着滑条走 —— 这就是「预览」，而且它自己就受 85 mm/s
            # 的速度限制，所以看到的速度就是真机将要走的速度。
            sim.command_fraction(target_fraction)

            if pressed(events, CONFIRM_KEYS) and move is None:
                sent_count += 1
                if gripper is None:
                    print(f"\n[dry-run #{sent_count}] 会下发："
                          f"开度 {target_fraction * 100:.1f}% · "
                          f"夹持力 {grip_n:.1f} N · {args.duration:g} s")
                else:
                    state = gripper.get_state(wait=False)
                    target_rad = fraction_to_target_rad(gripper, target_fraction)
                    # 前馈力矩是恒定推的，不分方向：张开时加力会顶住电机不让
                    # 它张开，所以只在**收拢**方向加——收拢时加力才是「夹紧」。
                    closing = target_rad > state.position_rad
                    tau_nm = grip_n * n_to_nm if closing else 0.0
                    move = StreamMove(target_rad, tau_nm, args.duration,
                                      gripper.config.kp, gripper.config.kd)
                    last_sent = 0.0
                    print(f"\n[下发 #{sent_count}] 开度 {target_fraction * 100:.1f}%"
                          f" → {target_rad:+.4f} rad · "
                          f"{'收拢' if closing else '张开'} · "
                          f"前馈 {tau_nm:.3f} Nm · {args.duration:g} s")

            if move is not None:
                if move.due(now, last_sent):
                    last_sent = now
                    move.send(gripper)
                if move.expired(now):
                    print(f"   [#{sent_count}] 发完 {move.frames} 帧")
                    move = None

            real = read_real(gripper)
            if real is None:
                sim.status_text(f"命令 {target_fraction * 100:5.1f}%  （dry-run）")
            else:
                real_fraction, real_force_n, real_moving = real
                sim.status_text(
                    f"命令 {target_fraction * 100:5.1f}%   "
                    f"真机 {real_fraction * 100:5.1f}%   "
                    f"力 {real_force_n:5.2f} N   "
                    + ("流式中" if move else ("运动中" if real_moving else "停住"))
                )
                if now - last_print >= PRINT_DT:
                    last_print = now
                    print("  " + real_line(real_fraction, real_force_n, real_moving))

            if not sim.step():
                break
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C")
    finally:
        if gripper is not None:
            # 停发 MIT 帧即可：达妙电机丢了指令会保持当前位置，不会乱动
            gripper.stop()
            gripper.disconnect()
            print("[真机] 已停止发帧并断开")
        sim.disconnect()

    if sent_count == 0:
        print("\n一条指令都没下发过：滑条调好后要按 Enter 才下发。")
    print("完成。反向的（真机 → 仿真）见 examples/03_real_to_sim.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
