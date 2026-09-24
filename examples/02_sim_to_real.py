#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 02 · 仿真控制真机：在 PyBullet 窗口里调目标，按 Enter 才下发

方向是 **仿真 → 真机**：仿真这边是「主」，你在这里决定开度和夹持力，真机是
「从」，收到指令才动。

流程（全在 PyBullet 窗口里操作）：

  1. 拖滑条「目标开度」——仿真手指**实时跟着滑条走**，这是预览，真机还没动。
     预览本身也受 85 mm/s 的速度限制，所以「看到什么速度，真机就是什么速度」。
  2. 拖滑条「夹持力」——下发时作为前馈力矩给电机（夹住工件时的推力）。
  3. 按 **Enter / 空格** 才真的下发：真机**斜坡**走过去，到位后再加力顶住
     （200 Hz 发帧）。仿真显示的是**命令值**，真机的实测值在旁边对照——两者
     的差就是真机的跟随误差，顶住工件时这个差会一直留着，正好用来判断
     「夹到了没有」。
  4. **Esc / Q** 退出，退出前会让真机停住（停止发帧，电机保持当前位置）。

⚠️ 会驱动真机！第一次跑务必先 dry-run：

    python3 examples/02_sim_to_real.py --dry-run    # 只开窗口，绝不碰 CAN

真机跑：

    python3 examples/02_sim_to_real.py                # can0, 10 N, 自动速度
    python3 examples/02_sim_to_real.py --force 20 --duration 2
    python3 examples/02_sim_to_real.py --channel can1        # 换 CAN 口

**真机「能读不能控」怎么办**（位置读得到、发指令不动、驱动板红灯闪）：

    python3 examples/02_sim_to_real.py --status             # 只连、只读，不发一帧
    python3 examples/02_sim_to_real.py --status --clear-fault   # 清掉锁死的故障

红灯闪 + 位置照读 + 指令无效，是电机进了**锁死**的故障态（欠压/过流/过温）。
最常见的原因是控制端把「目标」当阶跃发出去：MIT 的 kp 是位置刚度，一整段
行程的阶跃会让电机在第一帧就要求上百牛米（额定才 10 Nm 左右），电流拉满即
报保护。本样例现在发的是**斜坡**（见 :class:`StreamMove`），和 SDK 自己的
``goto_rad`` 一样，不会再踩这个坑。

为什么要自己发帧：SDK 的 move_to()/goto_rad() 内部是 control_mit_stream()，
它自己 sleep 5 ms 循环、不让出控制权，PyBullet 窗口会卡住、也读不到按键。
所以这里用 SDK 公开的 send_mit_frame() + poll() 自己组循环——这正是它们被
公开出来的用途（自定义控制循环，自己管时序）。斜坡的插值方式照抄 SDK 的
``_move_at_speed_rad``，两边走出来的速度是同一条曲线。
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
    ap.add_argument("--duration", type=float, default=None,
                    help=f"斜坡走多久 [s]（默认按额定手指速度 "
                         f"{RATED_SPEED_MM_S:g} mm/s 自动算；给得比这更快会被"
                         f"放慢，不会照做）")
    ap.add_argument("--status", action="store_true",
                    help="只连接、只读状态并诊断故障，**不打开窗口、不下发任何运动"
                         "指令**；真机「能读不能控」时先用它看看")
    ap.add_argument("--clear-fault", action="store_true",
                    help="配合 --status：清掉锁死的故障并重新使能（只发零力矩帧，"
                         "但清除流程含 disable→enable，电机会短暂失力）")
    return ap.parse_args()


class StreamMove:
    """一条流式指令：**斜坡 + 保持**两段，200 Hz 发帧。

    ⚠️ 为什么必须斜坡，不能直接跳到目标：

    MIT 的位置增益是 ``kp``（这儿 100 Nm/rad）。一次的阶跃如果有一整段行程那么
    大（1.845 rad），电机在第一帧就被要求输出 ``100 × 1.845 ≈ 185 Nm``——而
    DM4310 额定力矩才 ~10 Nm。电流瞬间拉满，电机进欠压/过流保护并**锁死**，红
    灯闪烁，之后就不再执行任何指令了（位置照常回报，所以看起来是「能读、不能
    控」）。SDK 自己的 ``goto_rad`` / ``move_at_speed`` 也都是线性斜坡，正是为了
    避免这一下；这里跟它保持一致。

    两段的差别：

      斜坡   线性插值到目标，``dq`` 给速度前馈，``tau=0``（不加力）
      保持   停在目标，``tau`` 加夹持力前馈——这就是 SDK ``set_force()`` 的做法

    力只在**保持**段才加：一边走一边顶，会在工件没夹住之前就把力顶上去。
    """

    def __init__(self, start_rad: float, target_rad: float, duration_s: float,
                 hold_s: float, kp: float, kd: float, tau_nm: float) -> None:
        self.start_rad = start_rad
        self.target_rad = target_rad
        self.duration_s = max(1e-3, float(duration_s))
        self.hold_s = max(0.0, float(hold_s))
        self.kp = kp
        self.kd = kd
        self.tau_nm = tau_nm
        self.started = time.monotonic()
        self.frames = 0
        self.dropped = 0              # send_mit_frame 返回 False 的帧数
        self.hold_announced = False   # 「到位，开始加力」只打一次

    @property
    def direction(self) -> float:
        return 1.0 if self.target_rad >= self.start_rad else -1.0

    @property
    def speed_rad_s(self) -> float:
        return abs(self.target_rad - self.start_rad) / self.duration_s

    def at(self, now: float) -> tuple[float, float, float]:
        """返回这一帧的 ``(q, dq, tau)``——斜坡段和保持段在此分叉。"""
        elapsed = now - self.started
        if elapsed < self.duration_s:
            frac = elapsed / self.duration_s
            q = self.start_rad + (self.target_rad - self.start_rad) * frac
            return q, self.direction * self.speed_rad_s, 0.0
        return self.target_rad, 0.0, self.tau_nm

    def in_hold(self, now: float) -> bool:
        return now - self.started >= self.duration_s

    def due(self, now: float, last_sent: float) -> bool:
        """该发下一帧了吗（按 :data:`FRAME_HZ` 限速）。"""
        return now - last_sent >= FRAME_DT

    def expired(self, now: float) -> bool:
        """这条指令的时间走完了吗。"""
        return now >= self.started + self.duration_s + self.hold_s

    def send(self, gripper, now: float) -> bool:
        """发一帧，返回是否发出去了。

        ``send_mit_frame`` 在「没连接」或「没使能」时返回 ``False``。这个返回
        值以前被丢掉了，于是帧全都没发出去也照样打印「发完 N 帧」——排查
        「能读不能控」时这会把人带偏，所以它会自己数着。
        """
        q, dq, tau = self.at(now)
        if gripper.send_mit_frame(q=q, kp=self.kp, kd=self.kd, dq=dq, tau=tau):
            self.frames += 1
            return True
        self.dropped += 1
        return False


#: 手指额定速度 [SDK 刻度 mm/s]。斜坡至少要慢到这个速度，和 SDK 的
#: ``move_at_speed`` 同一把尺子，也让真机走的速度和窗口里预览的速度一致。
RATED_SPEED_MM_S = 85.0

#: 保持段的默认时长 [s]：到位后加力顶住的时间。
DEFAULT_HOLD_S = 0.5


def plan_duration(gripper, distance_rad: float, requested: float | None) -> float:
    """算斜坡时长：慢到手指速度不超过额定值。

    ``--duration`` 比额定速度要求的时间还短时不会照做——那正是会烧保护的用法
    ——而是放慢到额定速度并说明原因。``None`` 表示「按额定速度自动算」。
    """
    rad_to_mm = gripper.config.rad_to_mm or 105.26
    min_s = abs(distance_rad) * rad_to_mm / RATED_SPEED_MM_S
    if requested is None or requested <= 0.0:
        return min_s
    if requested < min_s:
        print(f"   ⚠️ --duration {requested:.2f} s 对应手指速度 "
              f"{abs(distance_rad) * rad_to_mm / requested:.0f} mm/s，"
              f"超过额定 {RATED_SPEED_MM_S:.0f} mm/s；已放慢到 {min_s:.2f} s")
        return min_s
    return max(requested, min_s)


def describe_code(error_code: int) -> str:
    """错误码 → 可读文本。

    ``describe_error`` 懒导入：``--dry-run`` 没装 SDK 也要能跑。
    """
    from litegrip.constants import describe_error

    return describe_error(error_code)


def fault_of(state) -> str | None:
    """状态里有故障就返回可读描述，否则 ``None``。

    电机故障（欠压/过流/过温）是**锁死**的：不报错也不动，位置照常回报。必须在
    循环里盯着，否则会一直对着一个已经不听话的电机发帧。
    """
    if not state.is_error:
        return None
    return f"{describe_code(state.error_code)} (0x{state.error_code:X})"


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


def read_real(gripper) -> tuple[float, float, bool, str | None] | None:
    """读一帧真机状态，返回 ``(开度, 夹持力 N, 是否在动, 故障)``，无真机返回 None。

    ``get_state(wait=False)`` 只做一次非阻塞 poll 再读缓存，不阻塞主循环。
    本样例的 CAN 收发全在主循环这一个线程里，没有第二个线程来抢帧。

    故障描述一起带出来，是因为**锁死的故障在状态帧上是看不出来的**：位置照常
    更新、``is_moving`` 照常为假，只有错误码变了。不盯着它就会一直发帧。
    """
    if gripper is None:
        return None
    state = gripper.get_state(wait=False)
    return (
        rad_to_fraction(gripper, state.position_rad),
        state.force_n,
        bool(state.is_moving),
        fault_of(state),
    )


def real_line(fraction: float, force_n: float, moving: bool) -> str:
    """真机的一行状态。真机只报 SDK 刻度，物理开口用仿真那套模型反推。"""
    return status_line(
        "真机", fraction=fraction, aperture_mm=fraction_to_aperture_mm(fraction),
        force_n=force_n, moving=moving,
    )


def run_status(args: argparse.Namespace) -> int:
    """``--status``：只连接、只读，诊断真机为什么「能读不能控」。

    正常情况下**一个 CAN 帧都不发**：``open_real_gripper(enable=False)`` 只做
    connect + load_calibration + 轮询读状态，不使能、不下发。所以这条路径不会让
    电机产生任何运动，可以在夹着工件、或手指在别人手里的时候安全地跑。

    加 ``--clear-fault`` 才会发帧，发的也只是 SDK 的故障清除序列——全程
    ``kp=0/kd=0/tau=0`` 的零力矩帧，**不命令任何运动**。但要说清楚：
    ``clear_fault()`` 内部是 disable → clear → enable，中间那一瞬间电机是失力
    的，手指可能因自重轻微滑动。夹着东西或需要保持位置时先托住再清。

    Returns:
        0 = 健康（或无故障）；1 = 有故障但没清（或清除失败）。
    """
    if args.dry_run:
        raise SystemExit("❌ --status 和 --dry-run 是两件事：前者要连真机看状态，"
                         "后者不碰真机。选一个。")
    if args.headless:
        raise SystemExit("❌ --status 本来就不开窗口，不用加 --headless。")

    print("样例 02 · --status：只连接、只读，不动电机")
    gripper = open_real_gripper(args, enable=False)
    cleared = 0
    try:
        state = gripper.get_state(wait=True)
        fraction = rad_to_fraction(gripper, state.position_rad)
        print("  " + status_line(
            "真机", fraction=fraction,
            aperture_mm=fraction_to_aperture_mm(fraction),
            sdk_mm=state.position_mm, force_n=state.force_n,
            moving=bool(state.is_moving),
        ))
        print(f"   错误码 0x{state.error_code:X} · "
              f"{describe_code(state.error_code)}")

        if not state.is_error:
            print("   ✅ 没有故障。真机能正常接受指令——想动它就直接跑本样例"
                  "（不带 --status）。")
            return 0

        print(f"   ❌ 真机处在锁死的故障态：{fault_of(state)}")
        print("      位置照样能读、但任何指令都不会被执行（这就是「能读不能控」）。")
        if not args.clear_fault:
            print("\n   要清掉它，加 --clear-fault 再跑一次：\n"
                  "       python3 examples/02_sim_to_real.py --status --clear-fault\n"
                  "   （只发零力矩帧，不命令运动；但清除流程会 disable→enable，\n"
                  "    那一瞬间电机失力，手指可能因自重滑动——先托住夹爪。）")
            return 1

        print("\n   正在清除故障（只发零力矩帧）...")
        if not gripper.clear_fault():
            print("   ❌ 清除失败，电机可能还在故障态。查供电（欠压常见于电源"
                  "带不动）后重试。")
            return 1
        cleared = 1
        after = gripper.get_state(wait=True)
        if after.is_error:
            print(f"   ❌ 清完还是故障态：{fault_of(after)}")
            return 1
        print(f"   ✅ 故障已清除，电机已重新使能（错误码 0x{after.error_code:X}）。"
              "现在可以跑本样例正常下发了。")
        return 0
    finally:
        gripper.disconnect()
        # 清完故障就停在「使能且零力矩」——这是它本来该有的样子，不再动它；
        # 只是把这次操作说清楚，免得用户以为还需要做什么。
        print("[真机] 已断开"
              + ("（已重新使能，电机保持当前姿态）" if cleared else
                 "（未使能，未发送任何运动指令）"))


def main() -> int:
    args = parse_args()
    if args.clear_fault and not args.status:
        raise SystemExit("❌ --clear-fault 要配合 --status 用（它只清故障，不驱动）：\n"
                         "   python3 examples/02_sim_to_real.py --status --clear-fault")
    if args.status:
        return run_status(args)
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
    speed_note = (f"{args.duration:g} s" if args.duration
                  else f"自动（≤{RATED_SPEED_MM_S:g} mm/s）")
    print(f"   拖滑条改目标 → 按 Enter/空格 下发（斜坡 {speed_note} + "
          f"保持 {DEFAULT_HOLD_S:g} s，{FRAME_HZ:g} Hz 发帧）→ Esc/Q 退出")
    if gripper is None:
        print("   （dry-run：按 Enter 只打印，不会真的下发）")

    move: StreamMove | None = None
    last_sent = 0.0
    last_print = 0.0
    sent_count = 0
    aborted = False    # 因电机故障中止

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
                    duration_s = args.duration or 1.0   # dry-run：跑得快些看流程
                    print(f"\n[dry-run #{sent_count}] 会下发："
                          f"开度 {target_fraction * 100:.1f}% · "
                          f"夹持力 {grip_n:.1f} N · 斜坡 {duration_s:g} s + "
                          f"保持 {DEFAULT_HOLD_S:g} s")
                else:
                    state = gripper.get_state(wait=False)
                    fault = fault_of(state)
                    if fault:
                        # 锁死的故障下，发什么都白搭，还会掩盖真正的原因
                        move = None
                        print(f"\n❌ 真机报故障：{fault}")
                        print("   故障是锁死的：位置照读，但电机不执行任何指令。"
                              "请先清故障再下发：")
                        print("   python3 examples/02_sim_to_real.py --status "
                              "--clear-fault")
                        continue
                    target_rad = fraction_to_target_rad(gripper, target_fraction)
                    # 前馈力矩是恒定推的，不分方向：张开时加力会顶住电机不让
                    # 它张开，所以只在**收拢**方向加——收拢时加力才是「夹紧」。
                    closing = target_rad > state.position_rad
                    tau_nm = grip_n * n_to_nm if closing else 0.0
                    duration_s = plan_duration(
                        gripper, target_rad - state.position_rad, args.duration)
                    move = StreamMove(
                        start_rad=state.position_rad, target_rad=target_rad,
                        duration_s=duration_s, hold_s=DEFAULT_HOLD_S,
                        kp=gripper.config.kp, kd=gripper.config.kd, tau_nm=tau_nm)
                    last_sent = 0.0
                    travel_mm = abs(target_rad - state.position_rad) \
                        * gripper.config.rad_to_mm
                    print(f"\n[下发 #{sent_count}] 开度 {target_fraction * 100:.1f}%"
                          f" → {target_rad:+.4f} rad · "
                          f"{'收拢' if closing else '张开'} · "
                          f"走 {travel_mm:.1f} mm / {duration_s:.2f} s "
                          f"（{travel_mm / duration_s:.0f} mm/s）· "
                          f"前馈 {tau_nm:.3f} Nm")

            if move is not None:
                if move.due(now, last_sent):
                    last_sent = now
                    if not move.send(gripper, now):
                        # send_mit_frame 返回 False = 没使能 / 没连接。旧版这里
                        # 直接吞掉了，于是「一条帧都没发出去」也报「发完 N 帧」。
                        if move.dropped == 1:
                            print("   ⚠️ 帧没发出去（send_mit_frame 返回 False）："
                                  "真机可能未使能或已断开")
                if move.in_hold(now) and not move.hold_announced:
                    move.hold_announced = True
                    print(f"   到位，开始加力顶住 {move.tau_nm:.3f} Nm")
                if move.expired(now):
                    print(f"   [#{sent_count}] "
                          f"{'发完' if not move.dropped else '只发出'} "
                          f"{move.frames} 帧"
                          + (f"（丢 {move.dropped} 帧）" if move.dropped else ""))
                    move = None

            real = read_real(gripper)
            if real is None:
                sim.status_text(f"命令 {target_fraction * 100:5.1f}%  （dry-run）")
            else:
                real_fraction, real_force_n, real_moving, fault = real
                if fault and move is not None:
                    # 走着走着进了故障态：立刻停发，别再对着死电机发帧了
                    print(f"\n❌ 真机中途报故障：{fault}")
                    print("   已停止发帧。清故障（不动电机）："
                          "examples/02_sim_to_real.py --status --clear-fault")
                    move = None
                    aborted = True
                    break

                if move is not None:
                    phase = "加力中" if move.in_hold(now) else "斜坡中"
                else:
                    phase = "运动中" if real_moving else "停住"
                sim.status_text(
                    f"命令 {target_fraction * 100:5.1f}%   "
                    f"真机 {real_fraction * 100:5.1f}%   "
                    f"力 {real_force_n:5.2f} N   {phase}"
                )
                if now - last_print >= PRINT_DT:
                    last_print = now
                    print("  " + real_line(real_fraction, real_force_n,
                                           real_moving))

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

    if aborted:
        return 1
    if sent_count == 0:
        print("\n一条指令都没下发过：滑条调好后要按 Enter 才下发。")
    print("完成。反向的（真机 → 仿真）见 examples/03_real_to_sim.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
