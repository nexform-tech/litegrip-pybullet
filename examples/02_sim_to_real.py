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
  4. **Esc / Q** 退出，退出前会**明确失能**（0xFD）：电机不再出力，手指会松、
     夹着的工件会掉，但不会在电机上留下通信超时故障（原因见下）。

⚠️ 会驱动真机！第一次跑务必先 dry-run：

    python3 examples/02_sim_to_real.py --dry-run    # 只开窗口，绝不碰 CAN

⚠️ **每次都要先选定这台夹爪的标定文件**（标定文件由上位机标定后保存得到：

    litegrip-studio / litegrip-console，或 SDK 自带的 tools/gui/litegrip_gui.py
）。

不给 ``--calib`` 就会在终端里列出候选让你选；选不出来（非交互、没有候选）直接
退出——**不会**去用 SDK 的默认标定，更不会回退出厂标定。标定的角度和毫米刻度
是一台机器一个值，拿别人的算目标角，轻则夹不住、重则一条指令撞限位。``--dry-run``
也要选：它虽然不碰 CAN，但走的就是这套参数。

真机跑：

    python3 examples/02_sim_to_real.py --calib ~/.litegrip/litegrip_calibration.json
    python3 examples/02_sim_to_real.py                # 不给就当场从候选里选
    python3 examples/02_sim_to_real.py --force 20 --duration 2
    python3 examples/02_sim_to_real.py --channel can1        # 换 CAN 口

**真机「能读不能控」怎么办**（位置读得到、发指令不动、驱动板红灯闪）：

    python3 examples/02_sim_to_real.py --status             # 只连、只读，不发一帧
    python3 examples/02_sim_to_real.py --status --clear-fault   # 清掉锁死的故障

红灯闪 + 位置照读 + 指令无效，是电机进了**锁死**的故障态，而 ``--status`` 打的
那个错误码就是它的名字（0xD = 通信丢失、0x9 = 欠压、0xA = 过流、0xB/0xC = 过温
……）。两类原因最常见：

  * **没人喂帧**：使能态的电机静默约 :data:`MEASURED_COMM_LOSS_S` 就报 0xD。所以
    空闲也得持续发帧（见 :class:`IdleKeeper`）。
  * **一条接不住的指令**：MIT 的 kp 是位置刚度，一整段行程的阶跃会让电机在第一帧
    就被要求输出 ``kp × 1.845 rad`` 那么大的力矩。SDK 默认 kp=100 Nm/rad，那是
    185 Nm，而额定只有 ~10 Nm；本机标定现在是 5.0，同样的阶跃约 9 Nm。但 kp 是
    标定文件里的一项、随时可能被改回去，所以本样例发的是**斜坡**
    （见 :class:`StreamMove`），和 SDK 自己的 ``goto_rad`` 一样，不押在某个 kp 上。

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
    FRESH_WAIT_S,
    NOMINAL_STROKE_MM,
    SAFETY_BANNER,
    STATUS_WAIT_S,
    add_common_args,
    add_hardware_args,
    calibration_summary,
    check_calibration_values,
    choose_calibration_file,
    fraction_to_target_rad,
    fresh_state,
    import_litegrip,
    open_real_gripper,
    rad_to_fraction,
    read_calibration_file,
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
                    help="不连真机、不下发任何指令（只开窗口看流程）；"
                         "标定照样要先选——它决定目标角和毫米刻度")
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

    MIT 的位置增益是标定文件里的 ``kp``：一帧要的力矩就是 ``kp × 目标与实测的
    差``。SDK 默认 kp=100 Nm/rad，一整段行程（1.845 rad）的阶跃会在第一帧要求
    ``100 × 1.845 ≈ 185 Nm``，而 DM4310 额定只有 ~10 Nm——电流瞬间拉满，电机进
    欠压/过流保护并**锁死**，红灯闪烁，之后就不再执行任何指令（位置照常回报，
    所以看起来是「能读、不能控」）。本机标定现在把 kp 调到 5.0，同样的阶跃约
    9 Nm、落在额定之内，但 kp 是标定里的一项、随时可能被改回去，所以不押它：
    斜坡限制的是**位置目标**的跳变，与 kp 取多少无关。SDK 自己的 ``goto_rad`` /
    ``move_at_speed`` 也都是线性斜坡，这里跟它保持一致。

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

#: 实测：**使能态**的电机静默约这么久就闩锁 0xD 通信丢失故障（SDK 在真机上量到的）。
#:
#: ⚠️ 别拿 ``TIMEOUT`` 寄存器（RID 9）当依据：它读到过 8000 ms，也读到过 0＝当前
#: 不生效，和实测的 ~0.9 s 都对不上，SDK 自己把这条标成「待查」。行为按实测走
#: ——空闲也持续发帧。之前照寄存器那个 8000 ms 推出来的结论是错的。
MEASURED_COMM_LOSS_S = 0.9

#: 空闲时也必须持续发帧 [Hz]，和运动时同频。
#:
#: ⚠️ 这不是可选的优化：使能态的电机静默约 :data:`MEASURED_COMM_LOSS_S` 就锁进
#: 通信丢失故障——红灯闪、位置照读、指令一律不执行。空闲不发帧 = 在窗口里多看
#: 一眼就把真机看哑了。SDK 的 ``control_mit_stream`` 和 LiteGrip 控制台都是持续
#: 发帧的，正是为此。
IDLE_HZ = FRAME_HZ

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


#: 没装 SDK（或者 SDK 比这颗错误码还老）时的兜底表，内容抄自
#: ``litegrip.constants.ERROR_DESCRIPTIONS`` 的当前版本。0x8/0xD/0xE 是 SDK 后来
#: 补上的；旧版会把它们报成「未知错误」，而本机最容易闩上的恰恰是 0xD，所以这里
#: 必须自己认识它。
FALLBACK_ERRORS = {
    0x0: "已失能",
    0x1: "已使能",
    0x8: "过压故障 (OV)",
    0x9: "欠压故障 (UV)",
    0xA: "过流故障 (OC)",
    0xB: "MOS 过温故障",
    0xC: "线圈过温故障",
    0xD: "通讯丢失 (CAN 超时)",
    0xE: "过载故障",
}


def describe_code(error_code: int) -> str:
    """错误码 → 可读文本。**没装 SDK 也必须能说人话。**

    故障路径是最不该抛异常的地方：负责报故障的代码自己崩了，操作员就只剩一个
    回溯，看不到电机报的到底是哪一条。所以 ``describe_error`` 是懒导入且**带
    兜底**的——``--dry-run`` 和 CI 没装 SDK 也照样能翻译错误码。

    认得这个码就用 SDK 的说法（0xD 已经在它的 ``ERROR_DESCRIPTIONS`` 里了）；
    SDK 回「未知错误」＝它的表里没有这个码（那是 ``describe_error`` 唯一的兜底
    话术），这时才退回本表。反过来先查本表是不行的：两处描述会慢慢走偏。
    """
    try:
        from litegrip.constants import describe_error
    except ImportError:
        return FALLBACK_ERRORS.get(error_code, f"未知错误 (0x{error_code:X})")
    text = describe_error(error_code)
    if text.startswith("未知错误"):
        return FALLBACK_ERRORS.get(error_code, text)
    return text


def fault_of(state) -> str | None:
    """状态里有故障就返回可读描述，否则 ``None``。

    电机的故障（0xD 通信丢失、0x9 欠压、0xA 过流、0xB/0xC 过温）是**锁死**的：
    不报错也不动，位置照常回报。必须在循环里盯着，否则会一直对着一个已经不听
    话的电机发帧。
    """
    if not state.is_error:
        return None
    return f"{describe_code(state.error_code)} (0x{state.error_code:X})"


def hold_frame(
    gripper, request: bool = False
) -> tuple[float, float, float, float, float] | None:
    """「锁在当前位置」的一帧：``(q, kp, kd, dq, tau)``；读不到就返回 ``None``。

    目标就是电机**现在**的位置、零速度、零前馈——命令出来的一瞬间误差为零，所以
    不会产生任何运动，只是让手指有刚度、并且把电机的通信超时计数器喂上。

    这是空闲时的保活帧，也是 :meth:`LiteGrip.exit_zero_gravity` 说的「锁在当前
    位」。用它而不是 ``kp=0``：``kp=0`` 会让手指变软，可能在自重下自己出溜。

    ⚠️ 「现在的位置」必须真的**是现在**，所以走 :func:`fresh_state`（等到一帧新的
    状态帧）而不是 ``get_state(wait=False)`` 的缓存。缓存里没读到过位置时是 SDK
    的初值 ``0.0``——拿它当目标发出去，就是一条指向 0 rad 的**阶跃**指令，电机按
    标定里的 ``kp``（SDK 默认 100 Nm/rad）去追那个根本不存在的误差。少发一帧不会
    让电机乱动，发错目标会，所以读不到就返回 ``None``，调用方负责不发。

    Args:
        request: 等之前先发一帧 READ-ONLY 的 ``0xCC`` 状态请求（见
            :func:`_common.request_status_frame`）——电机不会自己发状态帧，不喂
            它就不回话，所以「读到之前什么都不发」会自己把自己饿死。0xCC 不带
            任何位置/力矩目标，是这里唯一能既不发控制帧、又让电机开口的招。
    """
    state = fresh_state(gripper, request=request)
    if state is None:
        return None
    cfg = gripper.config
    return (state.position_rad, cfg.kp, cfg.kd, 0.0, 0.0)


class IdleKeeper:
    """空闲保活：没有指令在走的时候，照样按 :data:`IDLE_HZ` 接着发帧。

    ⚠️ 这不是可选的优化，是必须的。**使能态**的电机静默约
    :data:`MEASURED_COMM_LOSS_S` 就锁进通信丢失故障——位置照读、指令不执行、
    红灯闪；而且在窗口里多看一眼就够触发，**发再多帧也解不开**（得显式清故障）。
    别拿 ``TIMEOUT`` 寄存器推算这个时长，它和实测对不上，见
    :data:`MEASURED_COMM_LOSS_S`。SDK 的 ``control_mit_stream`` 和 LiteGrip
    控制台都是持续发帧的，正是为此。

    发什么：优先「接着上一条指令的最后一帧发」——这样夹持力不会因为空闲而松掉；
    还没下发过指令时发一条锁位帧（目标 = 实测位置、零前馈），不命令任何运动。

    ⚠️ 一条已经定下来的帧可以一直重发，**没读过位置就不该造新帧**。这两件事的
    区别就是安全与危险的分界：重发同一帧，电机的目标不动，最坏也只是它没跟上；
    而拿一个旧读数（或 ``0.0``）现造一帧，就是把「读数坏了」变成一条指向别处的
    阶跃指令。所以本类只在初始化时定一次目标，之后一直重发；那次读不到就不发，
    下一拍再试，读到了才开始。
    """

    def __init__(self, gripper, hz: float = IDLE_HZ) -> None:
        self.gripper = gripper
        self.frame = hold_frame(gripper, request=True)   # None = 还没读到可信位置
        self.interval = 1.0 / hz
        self.last_sent = float("-inf")
        self.frames = 0
        self.dropped = 0
        self.starved = 0                      # 因为读不到位置而没发帧的次数

    def remember(self, move: "StreamMove") -> None:
        """记住这条指令的最后一帧，之后接着按它发。"""
        self.frame = (move.target_rad, move.kp, move.kd, 0.0, move.tau_nm)

    def maybe_send(self, now: float) -> bool | None:
        """到点就发一帧。

        Returns:
            True/False = 发出去了/没发出去（读不到位置、未使能、已断开）；
            None = 还没到点。
        """
        if now - self.last_sent < self.interval:
            return None
        if self.frame is None:
            # 还没有一个可信的目标——宁可这一拍不发，也不拿缓存的伪值现造一帧。
            # 但不能干等：电机不会自己发状态帧，等下去就是一直等。所以带上一帧
            # 只读的 0xCC 请求把它叫醒，下一拍就有位置可锁了。
            self.frame = hold_frame(self.gripper, request=True)
            if self.frame is None:
                self.starved += 1
                return False
        self.last_sent = now
        q, kp, kd, dq, tau = self.frame
        if self.gripper.send_mit_frame(q=q, kp=kp, kd=kd, dq=dq, tau=tau):
            self.frames += 1
            return True
        self.dropped += 1
        return False


def make_sliders(gripper, default_force_n: float) -> tuple[int, int]:
    """建两个滑条：目标开度 % / 夹持力 N。返回 ``(开度 id, 力 id)``。

    开度滑条从真机当前开度起步，这样一上来不会因为滑条值≠真机位置而「跳」。
    """
    start_pct = 100.0
    if gripper is not None:
        state = fresh_state(gripper, request=True)   # 刚使能，先把它叫醒再读
        if state is None:
            # 读不到就别假装知道：起点留 100%，并说清楚它不是真机现在的开度
            # （滑条值本身不会下发，按 Enter 才发，所以只是预览不准）。
            print(f"   ⚠️ 读不到真机状态帧（等了 {FRESH_WAIT_S * 1000:.0f} ms）："
                  "滑条起点只能用 100%，不代表真机现在的开度")
        else:
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


def read_registers(gripper) -> dict[str, float]:
    """读几个 DM 寄存器，读不到的跳过（**只发读请求，不是运动指令**）。

    ``TIMEOUT`` 是电机对看门狗**声明**的时长；实测的闩锁时间
    （:data:`MEASURED_COMM_LOSS_S`）和它 对不上，见 ``--status`` 的说明。这里读
    它只是为了把真值打出来，逻辑上不依赖它。
    """
    from litegrip.can.protocol import DM_REG

    wanted = ("TIMEOUT", "CTRL_MODE", "UV_Value", "OC_Value", "OT_Value")
    out: dict[str, float] = {}
    for name in wanted:
        rid = getattr(DM_REG, name, None)
        if rid is None:
            continue
        try:
            out[name] = float(gripper.read_param(int(rid), timeout_s=0.5))
        except Exception:
            pass          # 读不到就算了，不能因为一个寄存器把诊断搞挂
    return out


def run_status(args: argparse.Namespace) -> int:
    """``--status``：只连接、只读，诊断真机为什么「能读不能控」。

    这条路径**不使能、不发运动指令、不开窗口**：``open_real_gripper(enable=False)``
    只做 connect + load_calibration，之后发出去的只有询问帧——DM 的读请求
    （0x33）和一次 ``0xCC`` 状态刷新（SDK 的原话："Does not change motor
    output"），都不带位置/力矩目标。所以电机不会产生任何运动，可以在夹着工件、
    或手指在别人手里的时候安全地跑。

    标定照样要先选（``--calib`` 或当场从候选里选）：读回来的位置要换成开度，
    靠的就是标定的角度和 ``rad_to_mm``——用别台机器的刻度换算，打出来的百分比
    是错的，而这条路径存在的意义就是让这个百分比可信。

    加 ``--clear-fault`` 才会写：发的也只是 SDK 的故障清除序列——全程
    ``kp=0/kd=0/tau=0`` 的零力矩帧，**不命令任何运动**。但要说清楚：
    ``clear_fault()`` 内部是 disable → clear → enable，中间那一瞬间电机是失力
    的，手指可能因自重轻微滑动。夹着东西或需要保持位置时先托住再清。

    Returns:
        0 = 健康（或无故障）；1 = 有故障但没清、清除失败，或者**压根读不到状态
        帧**（这种情况说「健康」是撒谎，退出码也不该是 0）。
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
        # 没使能的电机不会自己发帧，所以先请它回一帧（0xCC，只读、不改输出）
        # 再读。少了这一步，等不到的 poll 会让 get_state() 返回 MotorState 的初值
        # 0.0——打印出来就是「5.5% / 6.63 mm」这种**伪造**读数（真值实测是
        # −0.370 rad ≈ 27.5%），拿来判断故障只会把人带偏。
        state = fresh_state(gripper, timeout_s=STATUS_WAIT_S, request=True)
        if state is None:
            print(f"   ❌ 读不到状态帧：已经请它回一帧（0xCC，不改电机输出）"
                  f"并等了 {STATUS_WAIT_S:g} s。")
            print("      **这种情况下没有可信的位置，也没有可信的故障码**：")
            print("      SDK 现在会把这件事说出来——GripperState.has_data 为假、"
                  "is_stale 为真——而 get_state() 返回的 position 只是它构造时的"
                  "初值 0.0。打出来看着像「夹爪在 5.5%」，「从没读到过」才是它的"
                  "真意。")
            print("      查这几处：")
            print("        · 夹爪是否上电；CAN_H/CAN_L 有没有接反；120Ω 终端电阻；")
            print(f"        · 接口是否真的起来：ip -details link show {args.channel}")
            print("        · 总线上是不是已经有别的程序在发帧（两个主控会互相打架，"
                  "谁都控不住）")
            return 1

        fraction = rad_to_fraction(gripper, state.position_rad)
        print("  " + status_line(
            "真机", fraction=fraction,
            aperture_mm=fraction_to_aperture_mm(fraction),
            sdk_mm=state.position_mm, force_n=state.force_n,
            moving=bool(state.is_moving),
        ))
        print(f"   错误码 0x{state.error_code:X} · "
              f"{describe_code(state.error_code)}"
              f"（刚要到的一帧实测值，不是缓存）")

        registers = read_registers(gripper)
        if registers:
            shown = " · ".join(f"{k}={v:g}" for k, v in registers.items())
            print(f"   寄存器 {shown}")
        timeout_ms = registers.get("TIMEOUT")
        if timeout_ms:
            print(f"   ⏱  通信超时保护（TIMEOUT, RID 9）= {timeout_ms:g} ms")
        elif "TIMEOUT" in registers:
            print("   ⏱  通信超时保护（TIMEOUT, RID 9）= 0（这个寄存器当前不生效）")
        if "TIMEOUT" in registers:
            print(f"   ⚠️  别拿这个寄存器当依据：实测**使能态**的电机静默约 "
                  f"{MEASURED_COMM_LOSS_S:g} s 就锁 0xD 通信丢失故障，与寄存器读数"
                  "对不上（这台机器读到过 8000，也读到过 0），SDK 自己把这条标成"
                  "「待查」。")
            print("      所以 02/03 空闲时照 200 Hz 持续发帧，不赌这个数字；只读"
                  "不喂帧（或跑了别的只读脚本）同样会把它看哑。")

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
        # disconnect() 自己会先失能（0xFD）再关总线，而 clear_fault() 结束时电机
        # 是使能态的——所以清完故障退出，电机**不会**保持在原来的位置：手指会松。
        # 以前这里写的是「已重新使能，电机保持当前姿态」，与代码实际做的事相反。
        gripper.disconnect()
        print("[真机] 已断开"
              + ("（已清故障并失能：手指会松、夹着的工件会掉）" if cleared else
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
        # dry-run 也要先选标定：它走的正是这套参数（目标角、毫米刻度都由标定
        # 决定）。只读文件、不导入 SDK、不建 CAN 对象——所以这里用纯值版的自洽
        # 检查，SDK 那边的 config 此刻不存在。
        calib_path = choose_calibration_file(args.calib)
        calib = read_calibration_file(calib_path)
        check_calibration_values(
            float(calib["zero_position_rad"]), float(calib["max_position_rad"]),
            float(calib["rad_to_mm"]), NOMINAL_STROKE_MM,
        )
        print(f"   标定 {calib_path}")
        print(f"        {calibration_summary(calib)}")
        gripper = None
        n_to_nm = NOMINAL_N_TO_NM
    else:
        print(SAFETY_BANNER)
        print("\n样例 02 · 仿真控制真机")
        gripper = open_real_gripper(args)
        n_to_nm = import_litegrip().UnitConversion.N_TO_NM

    # ⚠️ 从这里起全部在 try 里：真机一旦使能（上面 open_real_gripper 干的事），
    # 任何异常都必须走到 finally 去 disable/disconnect。否则程序带着一个「已使能、
    # 但再没人喂帧」的电机退出——静默约 0.9 s 就锁通信超时故障（红灯闪），而且因为
    # 进程已经死了，连是哪一步炸的都看不到。窗口建得慢、滑条建不出来，都算在这里面。
    sim = None
    keeper: IdleKeeper | None = None
    move: StreamMove | None = None
    last_sent = 0.0
    last_print = 0.0
    sent_count = 0
    faulted = False    # 真机报故障：停发、不再对着不听话的电机发帧

    try:
        # 一定是开窗的：上面已经拦掉了 --headless，滑条只有 GUI 连接才有
        # （DIRECT 连接里 addUserDebugParameter 会返回 -1）。
        sim = GripperSim(urdf_path=args.urdf, gui=True, max_force_n=force_n)
        sim.focus_camera()
        target_id, force_id = make_sliders(gripper, force_n)
        if target_id < 0 or force_id < 0:
            raise SystemExit(
                "❌ 建不出滑条（PyBullet 的 addUserDebugParameter 只在 GUI 连接下"
                "可用）。\n   检查是否真的有可用显示，或改用 "
                "examples/01_sim_only.py。"
            )

        print(f"   仿真：{sim.urdf_path}")
        speed_note = (f"{args.duration:g} s" if args.duration
                      else f"自动（≤{RATED_SPEED_MM_S:g} mm/s）")
        print(f"   拖滑条改目标 → 按 Enter/空格 下发（斜坡 {speed_note} + "
              f"保持 {DEFAULT_HOLD_S:g} s，{FRAME_HZ:g} Hz 发帧）→ Esc/Q 退出")
        if gripper is None:
            print("   （dry-run：按 Enter 只打印，不会真的下发）")
        else:
            # 使能之后**立刻**喂一帧锁在当前位置，不等主循环第一圈。SDK 的
            # `enable()` 现在会自己抱在实测位置（`_enable_and_hold`：使能帧 →
            # 0.05 s 零力矩流 → 等一帧新状态 → 锁位），但那条流只有 50 ms，之后
            # 就没人喂了，而建窗口、建滑条要几百毫秒——使能态静默约
            # `MEASURED_COMM_LOSS_S` 就锁 0xD，等不起。所以先喂上，计数器归零。
            keeper = IdleKeeper(gripper)
            keeper.maybe_send(time.monotonic())
            print(f"   [真机] 已锁在当前位置（保活 {IDLE_HZ:g} Hz 已开始，"
                  f"目标 = 实测位置、零前馈，不命令运动）")

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
                    # ⚠️ 这里**必须**拿到新鲜的位置：斜坡的起点、方向和时长全按它
                    # 算。用缓存里的旧值（或从没读到过时的 0.0），起点就落在别处、
                    # 时长还能算成 ~0——合起来就是一条阶跃指令，正是「一开夹爪就
                    # 起飞」的形态。所以拿不到就拒绝下发，别猜。
                    state = fresh_state(gripper, request=True)
                    if state is None:
                        print(f"\n❌ 读不到真机的状态帧（先请它回了一帧，又等了 "
                              f"{FRESH_WAIT_S * 1000:.0f} ms），**不下发**：")
                        print("   斜坡的起点和时长都要按「现在」的位置算，拿旧读数"
                              "算出来的是一条阶跃指令，电机接不住。")
                        print("   先看真机怎么了："
                              "python3 examples/02_sim_to_real.py --status")
                        continue
                    fault = fault_of(state)
                    if fault:
                        # 锁死的故障下，发什么都白搭，还会掩盖真正的原因
                        move = None
                        faulted = True
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
                    # 空闲保活接着按这条指令的最后一帧发：夹持力不会因为
                    # 「空闲」而松掉，电机也不会因为收不到帧而锁超时故障。
                    keeper.remember(move)
                    move = None
            elif keeper is not None and not faulted:
                # 空闲保活。见 :class:`IdleKeeper`：停发 = 等电机的通信超时
                # 保护把真机锁成故障态。
                sent = keeper.maybe_send(now)
                if sent is False and keeper.dropped == 1:
                    print("   ⚠️ 空闲保活帧没发出去（send_mit_frame 返回 "
                          "False）：真机可能未使能或已断开")
                elif sent is False and keeper.starved and keeper.starved % 200 == 1:
                    # 不是「发失败」，是「不敢发」：保活帧的目标必须是实测位置，
                    # 读不到就不造这一帧（见 IdleKeeper）。每 200 次报一次免得刷屏。
                    print(f"   ⚠️ 读不到真机状态帧，保活帧发不出去（第 "
                          f"{keeper.starved} 次）：目标得按实测位置算，拿不到就不发。"
                          f"\n      查 CAN 连接和供电，或先跑 --status 看真机状态。")

            real = read_real(gripper)
            if real is None:
                sim.status_text(f"命令 {target_fraction * 100:5.1f}%  （dry-run）")
            else:
                real_fraction, real_force_n, real_moving, fault = real
                if fault and not faulted:
                    # 进了故障态：立刻停发，别再对着不听话的电机发帧了。
                    # 窗口留着不关，好让人把上面这些字读完。
                    print(f"\n❌ 真机报故障：{fault}")
                    print("   已停止发帧。清故障（不动电机）："
                          "examples/02_sim_to_real.py --status --clear-fault")
                    move = None
                    faulted = True

                if move is not None:
                    phase = "加力中" if move.in_hold(now) else "斜坡中"
                elif faulted:
                    phase = "故障"
                elif real_moving:
                    phase = "锁位·真机在动"
                else:
                    phase = "锁位中"
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
            # 退出时**明确失能**（0xFD），而不是只停发帧：
            #   · stop() 只发一帧 kp=0，电机还是「使能 + 没人喂帧」——实测这种
            #     状态静默约 0.9 s 就锁 0xD 通信丢失故障，而进程一退就没人能清它；
            #   · disable() 把电机放到「已失能」：它不再需要帧，也不会闩故障。
            # 代价说清楚：失能后手指是软的，夹着的工件会掉、手指可能因自重滑动。
            # 「退出把电机留成故障态」比松手严重得多，所以选失能。
            gripper.disable()
            gripper.disconnect()
            print("[真机] 已失能（0xFD）并断开：手指会松、夹着的工件会掉；"
                  "这样退出不会在电机上留下通信超时故障")
        if sim is not None:
            sim.disconnect()

    if faulted:
        return 1
    if sent_count == 0:
        print("\n一条指令都没下发过：滑条调好后要按 Enter 才下发。")
    print("完成。反向的（真机 → 仿真）见 examples/03_real_to_sim.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
