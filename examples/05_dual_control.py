#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 05 · 仿真控制真机：拖滑条，真机跟着走

方向是 **仿真 → 真机**：滑条是手里的操纵杆，真机跟着它动。

流程（全在 PyBullet 窗口里操作）：

  1. 启动后真机**不动**：本样例按 200 Hz 发「锁在实测位置」的保持帧，同时把真机
     的实测位置实时镜像进窗口——窗口显示的是真机**现在在哪**，不是你想让它去哪。
  2. 拖滑条「**目标开度**」——真机才开始走，**拖动直接生效**，不用再按 Enter。
     真机的目标是滑条值，但位置目标每帧最多前进「速度 %」允许的距离，收尾还会
     按 :data:`RAMP_DOWN_S` 减速（见 :class:`SliderDrive`），所以把滑条一拽到底，
     真机也是按速度滑条限定的速度走完全程，不会跟着你的手跳，也不会在到达时被
     速度前馈的台阶顶回来。
  3. 拖滑条「**速度 %**」——位置目标的最大速率，100% 就是手指额定
     :data:`RATED_SPEED_MM_S`。随时可调：拖快滑条真机也不会跟着快。
  4. 拖滑条「**夹持力**」——手指到位后用这个力矩顶住（夹住工件时的推力）。
  5. **Esc / Q** 退出，退出前会**明确失能**（0xFD）：电机不再出力，手指会松、
     夹着的工件会掉，但不会在电机上留下通信超时故障（原因见下）。

⚠️ 会驱动真机！**没有「确认」这一步了**：滑条被碰到（鼠标划过、方向键、触控板）
就会发指令，所以别把手放在滑条上，也别在真机动的时候去点窗口。第一次跑务必先
dry-run：

    python3 examples/05_dual_control.py --dry-run    # 只开窗口，绝不碰 CAN

⚠️ **每次都要先选定这台夹爪的标定文件**（标定文件由上位机标定后保存得到：

    litegrip-studio / litegrip-console，或 SDK 自带的 tools/gui/litegrip_gui.py
）。

不给 ``--calib`` 就会在终端里列出候选让你选；选不出来（非交互、没有候选）直接
退出——**不会**去用 SDK 的默认标定，更不会回退出厂标定。标定的角度和毫米刻度
是一台机器一个值，拿别人的算目标角，轻则夹不住、重则一条指令撞限位。``--dry-run``
也要选：它虽然不碰 CAN，但走的就是这套参数。

真机跑：

    python3 examples/05_dual_control.py --calib ~/.litegrip/litegrip_calibration.json
    python3 examples/05_dual_control.py                # 不给就当场从候选里选
    python3 examples/05_dual_control.py --force 20 --speed 40
    python3 examples/05_dual_control.py --channel can1        # 换 CAN 口

**真机「能读不能控」怎么办**（位置读得到、发指令不动、驱动板红灯闪）：

    python3 examples/05_dual_control.py --status             # 只连、只读，不发一帧
    python3 examples/05_dual_control.py --status --clear-fault   # 清掉锁死的故障

红灯闪 + 位置照读 + 指令无效，是电机进了**锁死**的故障态，而 ``--status`` 打的
那个错误码就是它的名字（0xD = 通信丢失、0x9 = 欠压、0xA = 过流、0xB/0xC = 过温
……）。两类原因最常见：

  * **没人喂帧**：使能态的电机静默约 :data:`MEASURED_COMM_LOSS_S` 就报 0xD。所以
    空闲也得持续发帧（见 :class:`IdleKeeper`）——本样例空闲时发的是锁位帧。
  * **一条接不住的指令**：MIT 的 kp 是位置刚度，一整段行程的阶跃会让电机在第一帧
    就被要求输出 ``kp × 1.845 rad`` 那么大的力矩。SDK 默认 kp=100 Nm/rad，那是
    185 Nm，而额定只有 ~10 Nm；本机标定现在是 5.0，同样的阶跃约 9 Nm。但 kp 是
    标定文件里的一项、随时可能被改回去，所以本样例限制的是**位置目标每帧走多远**
    （见 :class:`SliderDrive`），和 kp 取多少无关，也和滑条被拽得多快无关。

为什么要自己发帧：SDK 的 move_to()/goto_rad() 内部是 control_mit_stream()，
它自己 sleep 5 ms 循环、不让出控制权，PyBullet 窗口会卡住、也读不到按键。
所以这里用 SDK 公开的 send_mit_frame() + poll() 自己组循环——这正是它们被
公开出来的用途（自定义控制循环，自己管时序）。
"""
import argparse
import math
import sys
import time
from types import SimpleNamespace

from _common import (  # noqa: I001  (必须先于 litegrip_pybullet)
    FRESH_WAIT_S,
    NOMINAL_STROKE_MM,
    SAFETY_BANNER,
    STATUS_WAIT_S,
    add_common_args,
    add_hardware_args,
    calibration_config,
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
        description="样例 05 · 仿真控制真机：拖滑条，真机跟着走",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    add_common_args(ap)
    add_hardware_args(ap)
    ap.add_argument("--dry-run", action="store_true",
                    help="不连真机、不下发任何指令（只开窗口看流程）；"
                         "标定照样要先选——它决定目标角和毫米刻度")
    ap.add_argument("--force", type=float, default=10.0,
                    help=f"夹持力前馈 [N]（默认 10，上限 {MAX_GRIP_FORCE_N:g}）")
    ap.add_argument("--speed", type=float, default=100.0,
                    help=f"位置目标的最大速率 [%%]，100%% = 手指额定 "
                         f"{RATED_SPEED_MM_S:g} mm/s（默认 100，超过 100 按 100 "
                         f"处理）。这就是窗口里「速度 %%」滑条的起点")
    ap.add_argument("--status", action="store_true",
                    help="只连接、只读状态并诊断故障，**不打开窗口、不下发任何运动"
                         "指令**；真机「能读不能控」时先用它看看")
    ap.add_argument("--clear-fault", action="store_true",
                    help="配合 --status：清掉锁死的故障并重新使能（只发零力矩帧，"
                         "但清除流程含 disable→enable，电机会短暂失力）")
    return ap.parse_args()


#: 判「滑条被拖动了」的阈值 [%]。窗口里的滑条是连续量，鼠标拖一下至少是零点几
#: 个百分点，所以这个阈值只滤掉浮点噪声；也正因为如此，**手划过滑条就算拖动**。
SLIDER_EPS = 0.05

#: 收尾减速的时间尺度 [s]：速度从满速降到 0 大约用这么久。
#:
#: ⚠️ 为什么必须有这一段（实测现象：「先到位，然后回弹一下」，张开时尤其明显）。
#:
#: MIT 帧里的 ``dq`` 是**目标速度**，电机自己算
#: ``tau = kp×(q目标 − q实际) + kd×(dq目标 − dq实际) + 前馈``。不减速的话，位置
#: 目标到达的那一帧 ``dq`` 会从满速直接变成 0，阻尼项 ``kd×(dq目标 − dq实际)``
#: 于是整个反向——等于给电机一个 ``kd × v`` 的**力矩台阶**，方向与刚才的运动相反。
#: 本机标定 ``kp=5.0 kd=2.0``，满速 ``85 mm/s ÷ 61.01 = 1.39 rad/s``：
#:
#:     kd × v = 2.0 × 1.39 ≈ 2.8 Nm
#:
#: 而位置项要拿误差补出同样的力矩，需要 ``2.8 ÷ 5.0 = 0.56 rad``——那是整个行程
#: （1.41 rad）的 40%。也就是说位置环根本接不住这个台阶，电机只能靠机构自己往回
#: 让一点、把误差重新建立起来：看到的就是「先到位、再回弹」。它是**速度**的函数，
#: 不是距离的函数，所以行程越短越不明显、跑得越快越明显。
#:
#: 按 ``v = √(2·a·剩余距离)`` 收尾之后，``dq`` 在最后几帧连续降到 0，那个台阶就
#: 没有了。代价是到位晚那么几帧（默认 0.15 s 的量级）。
#:
#: 注意这不是 02 独有的毛病：SDK 自己的 ``_move_at_speed_rad`` 是同样的一刀切
#: （``dq = direction*speed if i < steps else 0.0``），直接调
#: ``gripper.move_at_speed()`` 收尾也会这样。
RAMP_DOWN_S = 0.15


class SliderDrive:
    """跟着滑条走的**限速**位置跟随器：一帧最多走 ``速度 × FRAME_DT``。

    ⚠️ 为什么必须限速，不能把滑条值直接当目标发：

    MIT 的位置增益是标定文件里的 ``kp``：一帧要的力矩就是 ``kp × 目标与实测的
    差``。SDK 默认 kp=100 Nm/rad，一整段行程（1.845 rad）的阶跃会在第一帧要求
    ``100 × 1.845 ≈ 185 Nm``，而 DM4310 额定只有 ~10 Nm——电流瞬间拉满，电机进
    欠压/过流保护并**锁死**，红灯闪烁，之后就不再执行任何指令（位置照常回报，
    所以看起来是「能读、不能控」）。本机标定现在把 kp 调到 5.0，同样的阶跃约
    9 Nm、落在额定之内，但 kp 是标定里的一项、随时可能被改回去，所以不押它：
    限速限制的是**位置目标每帧的增量**，与 kp 取多少无关。

    这也正是「拖动直接移动」还能安全的原因。``q_want`` 是滑条（＝你的手）指的
    位置，``q`` 是真正被命令的位置，两者之间隔着这道限速：手拽得再快，一帧也
    只有一个 :attr:`max_step_rad` 的增量，所以拽到底也只是让它按速度滑条限定的
    速度走完全程，而不是跟着手跳。

    一帧的 ``(q, dq, tau)``：

      ``q``    受限地朝 ``q_want`` 挪一步（挪到了就正好停在 ``q_want``）
      ``dq``   这一步的速度，作为前馈帮电机跟上（不额外使劲）。**收尾会减速**：
               快到目标时按 ``√(2·a·剩余距离)`` 走，见 :data:`RAMP_DOWN_S`
      ``tau``  只有**到位之后**才是夹持力前馈，运动中恒为 0——一边走一边顶，会在
               工件还没夹住之前就把力顶上去；这和 SDK ``set_force()`` 只在停住
               之后加力是同一个道理。

    ``q`` 只在**帧真的发出去之后**才推进（:meth:`send`）：发丢的帧不能算数，
    否则下一帧就得一次走两步，把这道限速自己绕过去。
    """

    def __init__(self, start_rad: float, speed_rad_s: float, kp: float,
                 kd: float) -> None:
        self.q = float(start_rad)         # 已经命令到的位置
        self.q_want = float(start_rad)    # 滑条现在指的位置
        self.speed_rad_s = max(0.0, float(speed_rad_s))
        self.kp = kp
        self.kd = kd
        #: 最后一次拖动是不是**收拢**方向（见 :meth:`retarget`）。张开时加力会
        #: 顶住电机不让它张开，所以只有收拢才允许加力。
        self.closing = False
        self.frames = 0
        self.dropped = 0                  # send_mit_frame 返回 False 的帧数
        self.last_move = time.monotonic()  # 最后一次真的挪动的时间

    @property
    def max_step_rad(self) -> float:
        """一帧允许走的最大角度。**这是整个样例的安全边界。**"""
        return self.speed_rad_s * FRAME_DT

    def arrived(self) -> bool:
        """已经到滑条指的位置了吗。"""
        return abs(self.q_want - self.q) <= 1e-12

    def retarget(self, q_want: float) -> None:
        """滑条又动了：换一个新目标。

        下一次 :meth:`send` 仍然只走一个 :attr:`max_step_rad`——目标换得再远，
        一帧的增量不变，所以「拖动」永远不会变成阶跃。
        """
        self.q_want = float(q_want)
        self.closing = self.q_want > self.q

    def frame(self, tau_nm: float = 0.0) -> tuple[float, float, float]:
        """这一帧会发出去的 ``(q, dq, tau)``。**纯函数，不改状态。**

        速度上限是 :attr:`max_step_rad`，但快到位时还会再低一档：剩余距离不够
        保持当前速度时按 ``v = √(2·a·剩余距离)`` 走（``a = 满速 / RAMP_DOWN_S``），
        于是 ``dq`` 在到达目标的过程中连续降到 0，而不是在最后一帧被一步切成 0。
        为什么必须这样，见 :data:`RAMP_DOWN_S`。

        ``v`` 恒 ≤ 满速，所以 :attr:`max_step_rad` 仍然是硬边界；减速只让收尾多花
        几帧，不改变任何一帧能走多远。
        """
        delta = self.q_want - self.q
        speed = self.speed_rad_s
        if delta != 0.0:
            decel = self.speed_rad_s / RAMP_DOWN_S
            speed = min(speed, math.sqrt(2.0 * decel * abs(delta)))
        step = speed * FRAME_DT
        moved = delta if abs(delta) <= step else math.copysign(step, delta)
        q = self.q + moved
        # 到位判据用**这一帧的 q**，不是提交后的：最后一步收在 q_want 上，那一帧
        # 就该带上夹持力，而不是等到下一帧才开始顶。
        return q, moved / FRAME_DT, (tau_nm if abs(self.q_want - q) <= 1e-12 else 0.0)

    def advance(self, now: float, tau_nm: float = 0.0) -> tuple[float, float, float]:
        """推进一帧并提交（**不发帧**）。dry-run 用它——那里没有真机可发。

        帧数照计：dry-run 报的是「本来会发出去多少帧」，和真机跑同一套节奏
        才有得比。
        """
        q, dq, tau = self.frame(tau_nm)
        self._commit(q, now)
        self.frames += 1
        return q, dq, tau

    def due(self, now: float, last_sent: float) -> bool:
        """该发下一帧了吗（按 :data:`FRAME_HZ` 限速）。"""
        return now - last_sent >= FRAME_DT

    def done(self, now: float, hold_s: float) -> bool:
        """到位之后又 ``hold_s`` 没动过——这条驱动可以交回保活了。

        判的是「真的没动」，不是「目标没变」：还在跟随的途中，不管滑条有没有在
        动，都不算结束。
        """
        return self.arrived() and (now - self.last_move) >= hold_s

    def send(self, gripper, now: float, tau_nm: float = 0.0) -> bool:
        """发一帧，返回是否发出去了。

        ``send_mit_frame`` 在「没连接」或「没使能」时返回 ``False``。这个返回
        值以前被丢掉了，于是帧全都没发出去也照样打印「发完 N 帧」——排查
        「能读不能控」时这会把人带偏，所以它会自己数着。
        """
        q, dq, tau = self.frame(tau_nm)
        if not gripper.send_mit_frame(q=q, kp=self.kp, kd=self.kd, dq=dq, tau=tau):
            self.dropped += 1
            return False
        self._commit(q, now)
        self.frames += 1
        return True

    def _commit(self, q: float, now: float) -> None:
        if q != self.q:
            self.q = q
            self.last_move = now


#: 手指额定速度 [SDK 刻度 mm/s]。限速的上限就是它，和 SDK 的 ``move_at_speed``
#: 同一把尺子。
RATED_SPEED_MM_S = 85.0


def plan_speed(gripper, percent: float) -> float:
    """「速度 %」→ 位置目标的最大角速度 [rad/s]。

    100% 就是额定手指速度 :data:`RATED_SPEED_MM_S`。超过 100% 不照做——按 100%
    处理并说明；那是会拉保护的用法，而且窗口里也没有更快的东西可比。
    """
    pct = float(percent)
    if pct > 100.0:
        print(f"   ⚠️ 速度 {pct:.0f}% 超过额定，已按 100%"
              f"（{RATED_SPEED_MM_S:g} mm/s）处理")
        pct = 100.0
    rad_to_mm = gripper.config.rad_to_mm or 105.26
    return RATED_SPEED_MM_S * max(0.0, pct) / 100.0 / rad_to_mm

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
        #: 记住的这一帧是不是收拢指令——只有收拢才允许带夹持力（见
        #: :meth:`SliderDrive.retarget`）。锁位帧不含运动，所以是 False。
        self.closing = False
        self.interval = 1.0 / hz
        self.last_sent = float("-inf")
        self.frames = 0
        self.dropped = 0
        self.starved = 0                      # 因为读不到位置而没发帧的次数

    def remember(self, drive: "SliderDrive") -> None:
        """记住这条驱动停住的位置，之后接着按它发。

        前馈力矩不写进这一帧：它跟着「夹持力」滑条走（见 :meth:`set_force`），
        所以夹住工件之后再拖力滑条，力道会跟着变，而不是停在拖动那一刻的值。
        """
        self.frame = (drive.q, drive.kp, drive.kd, 0.0, 0.0)
        self.closing = drive.closing

    def set_force(self, tau_nm: float) -> None:
        """更新保活帧的前馈力矩 [Nm]。0 = 这一帧只是锁位。"""
        if self.frame is None:
            return
        q, kp, kd, dq, _ = self.frame
        self.frame = (q, kp, kd, dq, max(0.0, float(tau_nm)))

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


def make_sliders(gripper, *, live: bool, default_force_n: float,
                 default_speed_pct: float) -> tuple[int, int, int, float]:
    """建三个滑条：目标开度 % / 速度 % / 夹持力 N。

    返回 ``(开度 id, 速度 id, 力 id, 起始开度)``。

    开度滑条从真机**当前**开度起步：判「你在拖滑条」看的是滑条值**变没变**，起点
    对不上真机就会在启动那一帧被当成一次拖动。所以起点要么是真机实测的，要么是
    一个明确说明过的名义值——**但无论哪个都不会让电机动**：拖动检测只认变化，
    静止的滑条值永远不会下发。
    """
    start_fraction = 1.0
    if live:
        state = fresh_state(gripper, request=True)   # 刚使能，先把它叫醒再读
        if state is None:
            print(f"   ⚠️ 读不到真机状态帧（等了 {FRESH_WAIT_S * 1000:.0f} ms）："
                  "开度滑条起点只能用 100%，不代表真机现在的开度"
                  "（起点只是为了让「拖动」有个基准，不会下发）")
        else:
            start_fraction = rad_to_fraction(gripper, state.position_rad)
    target_id = p.addUserDebugParameter("目标开度 %", 0.0, 100.0,
                                        start_fraction * 100.0)
    speed_id = p.addUserDebugParameter("速度 %", 0.0, 100.0,
                                       min(default_speed_pct, 100.0))
    force_id = p.addUserDebugParameter("夹持力 N", 0.0, MAX_GRIP_FORCE_N,
                                       min(default_force_n, MAX_GRIP_FORCE_N))
    return target_id, speed_id, force_id, start_fraction


def read_real(gripper, *, live: bool) -> tuple[float, float, bool, str | None] | None:
    """读一帧真机状态，返回 ``(开度, 夹持力 N, 是否在动, 故障)``，无真机返回 None。

    ``get_state(wait=False)`` 只做一次非阻塞 poll 再读缓存，不阻塞主循环。
    本样例的 CAN 收发全在主循环这一个线程里，没有第二个线程来抢帧。

    故障描述一起带出来，是因为**锁死的故障在状态帧上是看不出来的**：位置照常
    更新、``is_moving`` 照常为假，只有错误码变了。不盯着它就会一直发帧。
    """
    if not live:
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

    print("样例 05 · --status：只连接、只读，不动电机")
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
            print("      所以 04/05 空闲时照 200 Hz 持续发帧，不赌这个数字；只读"
                  "不喂帧（或跑了别的只读脚本）同样会把它看哑。")

        if not state.is_error:
            print("   ✅ 没有故障。真机能正常接受指令——想动它就直接跑本样例"
                  "（不带 --status）。")
            return 0

        print(f"   ❌ 真机处在锁死的故障态：{fault_of(state)}")
        print("      位置照样能读、但任何指令都不会被执行（这就是「能读不能控」）。")
        if not args.clear_fault:
            print("\n   要清掉它，加 --clear-fault 再跑一次：\n"
                  "       python3 examples/05_dual_control.py --status --clear-fault\n"
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
                         "   python3 examples/05_dual_control.py --status --clear-fault")
    if args.status:
        return run_status(args)
    if args.headless:
        raise SystemExit(
            "❌ 样例 05 靠窗口里的滑条来设目标，没有窗口就没法操作。\n"
            "   想看无窗口的纯仿真请用 examples/01_sim_only.py；\n"
            "   想看真机 → 仿真（不需要操作）请用 examples/04_mirror_real.py。"
        )

    force_n = max(0.0, min(MAX_GRIP_FORCE_N, args.force))
    if args.dry_run:
        print("样例 05 · 仿真控制真机（--dry-run：不碰真机，只走流程）")
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
        # 没有真机，但要算目标角／把角度换回开度，就得有一个 ``.config``——
        # 用刚校验过的那份标定装一个，字段名和 SDK 的 ``gripper.config`` 一致。
        gripper = SimpleNamespace(config=calibration_config(calib))
        live = False
        n_to_nm = NOMINAL_N_TO_NM
    else:
        print(SAFETY_BANNER)
        print("\n样例 05 · 仿真控制真机")
        gripper = open_real_gripper(args)
        live = True
        n_to_nm = import_litegrip().UnitConversion.N_TO_NM

    # ⚠️ 从这里起全部在 try 里：真机一旦使能（上面 open_real_gripper 干的事），
    # 任何异常都必须走到 finally 去 disable/disconnect。否则程序带着一个「已使能、
    # 但再没人喂帧」的电机退出——静默约 0.9 s 就锁通信超时故障（红灯闪），而且因为
    # 进程已经死了，连是哪一步炸的都看不到。窗口建得慢、滑条建不出来，都算在这里面。
    sim = None
    keeper: IdleKeeper | None = None
    drive: SliderDrive | None = None
    last_sent = 0.0
    last_print = 0.0
    last_target_pct: float | None = None   # None = 还没读到过滑条，第一帧只对齐基准
    drags = 0          # 拖动次数 = 建过几条驱动
    faulted = False    # 真机报故障：停发、不再对着不听话的电机发帧
    #: dry-run 下的「当前位置」：没有真机可读，就用上一条驱动停住的位置接上。
    dry_rad = 0.0

    try:
        # 一定是开窗的：上面已经拦掉了 --headless，滑条只有 GUI 连接才有
        # （DIRECT 连接里 addUserDebugParameter 会返回 -1）。
        sim = GripperSim(urdf_path=args.urdf, gui=True, max_force_n=force_n)
        sim.focus_camera()
        max_speed_rad_s = plan_speed(gripper, args.speed)
        target_id, speed_id, force_id, start_fraction = make_sliders(
            gripper, live=live, default_force_n=force_n,
            default_speed_pct=args.speed)
        if min(target_id, speed_id, force_id) < 0:
            raise SystemExit(
                "❌ 建不出滑条（PyBullet 的 addUserDebugParameter 只在 GUI 连接下"
                "可用）。\n   检查是否真的有可用显示，或改用 "
                "examples/01_sim_only.py。"
            )
        dry_rad = fraction_to_target_rad(gripper, start_fraction)

        print(f"   仿真：{sim.urdf_path}")
        print(f"   拖动「目标开度」→ 真机跟着走（限速 {args.speed:g}%，"
              f"≤{args.speed / 100.0 * RATED_SPEED_MM_S:g} mm/s；"
              f"「速度 %」滑条随时可调）→ Esc/Q 退出")
        print(f"   位置目标每帧最多走 {max_speed_rad_s * FRAME_DT:.6f} rad"
              f"（= 速度 × {FRAME_DT * 1000:g} ms），{FRAME_HZ:g} Hz 发帧——"
              "拖得再快也不会变成一条阶跃指令")
        if not live:
            print("   （dry-run：不连真机、不发任何帧，窗口里走的是命令值）")
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
            target_pct = p.readUserDebugParameter(target_id)
            speed_pct = p.readUserDebugParameter(speed_id)
            grip_n = p.readUserDebugParameter(force_id)

            real = read_real(gripper, live=live)
            if real is None:
                real_fraction, real_force_n, real_moving, fault = 0.0, 0.0, False, None
            else:
                real_fraction, real_force_n, real_moving, fault = real
                if fault and not faulted:
                    # 进了故障态：立刻停发，别再对着不听话的电机发帧了。
                    # 窗口留着不关，好让人把上面这些字读完。
                    print(f"\n❌ 真机报故障：{fault}")
                    print("   已停止发帧。清故障（不动电机）："
                          "examples/05_dual_control.py --status --clear-fault")
                    drive = None
                    faulted = True

            # 第一帧只对齐基准：滑条值没变过就不算「拖动了」。
            if last_target_pct is None:
                last_target_pct = target_pct
            dragged = abs(target_pct - last_target_pct) > SLIDER_EPS

            # ⚠️ 顺序是先拖动再发帧：拖动改的是**目标**，这一帧的位置目标仍然只
            # 走一个 max_step，所以「拖到哪」和「这一帧走到哪」是两件事。
            if dragged:
                last_target_pct = target_pct
            if dragged and not faulted:
                target_rad = fraction_to_target_rad(gripper, target_pct / 100.0)
                if drive is None:
                    # 每条驱动都要有实测的起点：拖动之前发的保活帧只是「锁在
                    # 电机说的位置」，还没有一条指令。拿旧读数当起点，限速本身就
                    # 没有意义了——一步就是从错的地方走到目标。所以拿不到就拒绝，
                    # 别猜。
                    start_rad = None
                    if live:
                        state = fresh_state(gripper, request=True)
                        if state is None:
                            print(f"\n❌ 读不到真机的状态帧（先请它回了一帧，又等了 "
                                  f"{FRESH_WAIT_S * 1000:.0f} ms），**不下发**：")
                            print("   限速要按「现在」的位置算，拿旧读数算出来的"
                                  "是一条阶跃指令，电机接不住。")
                            print("   先看真机怎么了："
                                  "python3 examples/05_dual_control.py --status")
                            continue
                        fault = fault_of(state)
                        if fault:
                            # 锁死的故障下，发什么都白搭，还会掩盖真正的原因
                            drive = None
                            faulted = True
                            print(f"\n❌ 真机报故障：{fault}")
                            print("   故障是锁死的：位置照读，但电机不执行任何指令。"
                                  "请先清故障再下发：")
                            print("   python3 examples/05_dual_control.py --status "
                                  "--clear-fault")
                            continue
                        start_rad = state.position_rad
                        travel_mm = abs(target_rad - start_rad) \
                            * gripper.config.rad_to_mm
                        print(f"\n[拖动 #{drags + 1}] 开度 {target_pct:.1f}% · "
                              f"从实测 {start_rad:+.4f} rad 起步 · "
                              f"路程 {travel_mm:.1f} mm · "
                              f"限速 {speed_pct:.0f}% "
                              f"（{speed_pct / 100.0 * RATED_SPEED_MM_S:.0f} mm/s）")
                    else:
                        # dry-run：没有真机可读，接着上一条驱动停住的位置走。
                        start_rad = dry_rad
                        print(f"\n[拖动 #{drags + 1}] 开度 {target_pct:.1f}% · "
                              f"（dry-run：起点用命令值 {start_rad:+.4f} rad）")
                    drive = SliderDrive(
                        start_rad=start_rad, speed_rad_s=plan_speed(gripper, speed_pct),
                        kp=gripper.config.kp, kd=gripper.config.kd)
                    drags += 1
                    last_sent = 0.0
                drive.retarget(target_rad)
                # 「速度 %」随时可调：它改的是**位置目标每帧的增量**，不是这一条
                # 驱动开跑时的速度。
                drive.speed_rad_s = plan_speed(gripper, speed_pct)

            if faulted:
                pass                       # 故障态一帧都不发：发了也不执行
            elif drive is not None:
                # 前馈力矩是恒定推的，不分方向：张开时加力会顶住电机不让它张开，
                # 所以只在**收拢**方向加——收拢时加力才是「夹紧」。而且只有到位
                # 之后才加（SliderDrive.frame 里判的）。
                tau_nm = grip_n * n_to_nm if drive.closing else 0.0
                if drive.due(now, last_sent):
                    last_sent = now
                    if live:
                        sent = drive.send(gripper, now, tau_nm)
                    else:
                        drive.advance(now, tau_nm)     # dry-run：推进，不发帧
                        sent = True
                    if not sent and drive.dropped == 1:
                        # send_mit_frame 返回 False = 没使能 / 没连接。旧版这里
                        # 直接吞掉了，于是「一条帧都没发出去」也报「发完 N 帧」。
                        print("   ⚠️ 帧没发出去（send_mit_frame 返回 False）："
                              "真机可能未使能或已断开")
                if drive.done(now, DEFAULT_HOLD_S):
                    print(f"   [#{drags}] 停住 {DEFAULT_HOLD_S:g} s，"
                          f"{'发出' if live else '（dry-run）模拟'} "
                          f"{drive.frames} 帧"
                          + (f"（丢 {drive.dropped} 帧）" if drive.dropped else "")
                          + "，接着按这个位置保活")
                    # 交回保活：夹持力不会因为「停住」而松掉，电机也不会因为
                    # 收不到帧而锁超时故障。下一次拖动会重新读一次实测位置。
                    if keeper is not None:
                        keeper.remember(drive)
                    dry_rad = drive.q
                    drive = None
            elif keeper is not None:
                # 空闲保活。见 :class:`IdleKeeper`：停发 = 等电机的通信超时
                # 保护把真机锁成故障态。
                keeper.set_force(grip_n * n_to_nm if keeper.closing else 0.0)
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

            # 窗口显示的是**真机在哪**（dry-run 下是命令值）。用 reset_fraction 而
            # 不是 command_fraction：前者是运动学瞬移，只把模型摆到那个位置，不驱
            # 动任何东西——这不是「仿真控制真机」的那条路径，是镜像。
            shown_rad = drive.q if drive is not None else dry_rad
            sim.reset_fraction(
                real_fraction if live else rad_to_fraction(gripper, shown_rad))

            if faulted:
                phase = "故障"
            elif drive is not None and not drive.arrived():
                phase = f"跟随中 → {target_pct:.0f}%"
            elif drive is not None and drive.closing and grip_n > 0.0:
                phase = f"到位·加力 {grip_n:g} N"
            elif drive is not None:
                phase = "到位"
            elif not live:
                phase = "空闲"
            elif real_moving:
                phase = "锁位·真机在动"
            else:
                phase = "锁位"
            commanded = rad_to_fraction(gripper, shown_rad) * 100
            if live:
                sim.status_text(
                    f"真机 {real_fraction * 100:5.1f}%   "
                    f"命令 {commanded:5.1f}%   "
                    f"力 {real_force_n:5.2f} N   {phase}"
                )
            else:
                sim.status_text(f"命令 {commanded:5.1f}%   （dry-run：没有真机）   "
                                f"{phase}")
            if live and now - last_print >= PRINT_DT:
                last_print = now
                print("  " + real_line(real_fraction, real_force_n, real_moving))

            if not sim.step():
                break
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C")
    finally:
        if live:
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
    if drags == 0:
        print("\n一条指令都没下发过：拖动「目标开度」滑条真机才会动，"
              "启动之后它一直锁在当前位置。")
    print("完成。反向的（真机 → 仿真）见 examples/04_mirror_real.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
