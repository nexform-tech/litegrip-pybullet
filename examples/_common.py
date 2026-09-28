#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""_common.py — LiteGrip PyBullet 样例的共用启动样板。

三个样例（01/02/03）共享这里的东西：

  ensure_deps()      缺 pybullet 时自动改用仓库自带 .venv 重跑
  import_litegrip()  导入真机 SDK（已安装 / 同级 lite-grip 仓库 / $LITEGRIP_SDK_DIR）
  check_sdk_api()    核对 SDK 有没有本仓库依赖的公开接口，缺了就在启动时停下
  add_common_args()  --urdf / --headless
  add_hardware_args() --channel / --can-id / --mst-id / --calib
  open_real_gripper() 连接 → 载入标定 → 使能，失败时给出可读的提示
  fresh_state()      等到一帧**新**的状态帧再读位置；等不到返回 None
                     （读真机位置只该走这里，别直接读 get_state() 的缓存）

⚠️ 02/03 会驱动真机！真机的两个手指会真的闭合。首次跑请：
  1) 把夹爪拿在手上或固定在台面上，**手指行程内不要放任何东西**；
  2) 手放在电源开关旁边；
  3) 先用 --dry-run（02）跑一遍看看流程。

真机跑之前确认 CAN 已配置好：

    sudo ip link set can0 up type can bitrate 1000000
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

__all__ = [
    "REQUIRED_SDK_API",
    "SAFETY_BANNER",
    "add_common_args",
    "add_hardware_args",
    "bootstrap_src",
    "check_calibration",
    "check_sdk_api",
    "ensure_deps",
    "fraction_to_target_rad",
    "fresh_state",
    "import_litegrip",
    "missing_sdk_api",
    "open_real_gripper",
    "rad_to_fraction",
    "request_status_frame",
    "sdk_dir",
    "status_line",
]

#: 真机样例开跑前打印的横幅。
SAFETY_BANNER = """\
⚠️  即将驱动真机：夹爪两个手指会真实运动。
    请确认行程内无遮挡、人员远离，并让电源开关触手可及。
    随时按 Esc / Q 停止（会等当前这条指令走完再退出）。"""

_REPO_ROOT = Path(__file__).resolve().parent.parent
_SRC = _REPO_ROOT / "src"
_VENV_PY = _REPO_ROOT / ".venv" / "bin" / "python"
_REEXEC_FLAG = "LITEGRIP_PYBULLET_REEXEC"


def bootstrap_src() -> None:
    """让仓库 ``src/`` 下的 litegrip_pybullet 可导入（未 pip install 时）。"""
    if str(_SRC) not in sys.path:
        sys.path.insert(0, str(_SRC))


def ensure_deps() -> None:
    """当前 python 缺 pybullet 时，自动改用仓库自带 .venv 重跑。

    这样 ``python3 examples/01_sim_only.py`` 开箱即用，不必先 pip install。
    """
    if _REEXEC_FLAG in os.environ:  # 已经重跑过一次，别再套娃
        return
    try:
        import pybullet  # noqa: F401
        return
    except Exception:
        pass
    script = os.path.abspath(sys.argv[0])  # 被跑的样例脚本，不是 _common.py
    if _VENV_PY.exists() and Path(sys.executable) != _VENV_PY:
        print(f"[hint] 当前 python 缺 pybullet，改用 {_VENV_PY}")
        os.environ[_REEXEC_FLAG] = "1"
        os.execv(str(_VENV_PY), [str(_VENV_PY), script] + sys.argv[1:])


def sdk_dir() -> Path | None:
    """真机 SDK（``litegrip`` 包）所在目录，找不到返回 ``None``。

    顺序：``$LITEGRIP_SDK_DIR`` → 已安装的 ``litegrip`` → 同级 ``lite-grip`` 仓库。
    """
    env = os.environ.get("LITEGRIP_SDK_DIR")
    if env:
        return Path(env).expanduser()
    try:
        import litegrip  # noqa: F401

        return Path(litegrip.__file__).resolve().parent.parent
    except Exception:
        pass
    for name in ("lite-grip", "litegrip"):
        sibling = _REPO_ROOT.parent / name
        if (sibling / "litegrip" / "__init__.py").is_file():
            return sibling
    return None


def import_litegrip():
    """导入真机 SDK，失败时给出安装提示并退出。

    Returns:
        已导入的 ``litegrip`` 模块。
    """
    directory = sdk_dir()
    if directory is not None and str(directory) not in sys.path:
        sys.path.insert(0, str(directory))
    try:
        import litegrip
    except ImportError as exc:
        raise SystemExit(
            "❌ 找不到真机 SDK（litegrip 包）。\n"
            "   litegrip **没有发布到 PyPI**，`pip install litegrip` 装的不是它；\n"
            "   本仓库要的是带 refresh_status() / GripperState.data_age_s 这些公开\n"
            "   接口的检出。三种任选其一：\n"
            "   1) python3 -m pip install -e /path/to/lite-grip\n"
            "   2) export LITEGRIP_SDK_DIR=/path/to/lite-grip\n"
            "   3) 把 lite-grip 仓库克隆到本仓库的同级目录\n"
            f"   （原始错误：{exc}）"
        ) from exc
    return litegrip


#: 本仓库依赖的 SDK 公开接口。清单化而不是散在各调用处：缺哪一个就在**启动时**
#: 说清楚该换哪份 SDK，而不是在发帧的循环里抛 AttributeError。
#:
#: 这些接口目前**没有任何发行版带**（litegrip 也不在 PyPI 上），所以
#: ``pip install litegrip`` 装到的那份一定缺它们——这正是要拦的情况。
REQUIRED_SDK_API: tuple[tuple[str, str], ...] = (
    ("LiteGrip.refresh_status",
     "发一帧只读的 0xCC 并等回应，电机不发帧时唯一能读到位置的公开路径"),
    ("GripperState.data_age_s",
     "这一帧有多旧（从没收到过是 inf）——判断读数能不能用"),
    ("GripperState.has_data", "到底收到过状态帧没有"),
    ("GripperState.is_stale", "这一帧是不是已经过期"),
)


def missing_sdk_api(litegrip) -> list[str]:
    """已导入的 SDK 里缺哪些必需接口（按 :data:`REQUIRED_SDK_API` 的顺序）。"""
    missing: list[str] = []
    for path, _why in REQUIRED_SDK_API:
        owner, _, attr = path.partition(".")
        if not hasattr(getattr(litegrip, owner, None), attr):
            missing.append(path)
    return missing


def check_sdk_api(litegrip) -> None:
    """缺必需接口就带着「该用哪份 SDK」退出（``SystemExit``）。

    **不保留降级路径**：没有这些接口，要么只能去够 SDK 的内部实现（以前就是
    靠 ``gripper._can._controller`` 借的），要么只能猜「这一帧是不是现在的」。
    真机样例宁可不跑，也不拿一个猜出来的位置去算目标角——那正是一条指向别处的
    阶跃指令的成因。
    """
    missing = missing_sdk_api(litegrip)
    if not missing:
        return
    why = dict(REQUIRED_SDK_API)
    directory = sdk_dir()
    raise SystemExit(
        "❌ 这份 litegrip SDK 缺少本仓库必须的公开接口：\n"
        + "".join(f"     • {path} —— {why[path]}\n" for path in missing)
        + "   litegrip **没有发布到 PyPI**（`pip install litegrip` 装到的不是这份"
          "代码），\n"
          "   带这些接口的版本也还没发布过。请改用本地检出：\n"
          "     python3 -m pip install -e /path/to/lite-grip          # 或\n"
          "     export LITEGRIP_SDK_DIR=/path/to/lite-grip\n"
        + (f"   （这次导入到的是：{directory}）" if directory is not None else "")
    )


def add_common_args(parser: argparse.ArgumentParser) -> None:
    """加 ``--urdf`` / ``--headless``。"""
    parser.add_argument(
        "--urdf", default=None,
        help="自定义 URDF/xacro 路径（默认用仓库自带的一份，"
             "也可用 $LITEGRIP_URDF_PATH / $LITEGRIP_URDF_DIR 指定）",
    )
    parser.add_argument(
        "--headless", action="store_true",
        help="不开 PyBullet 窗口（无显示环境必须加；运动学完全一致）",
    )


def add_hardware_args(parser: argparse.ArgumentParser) -> None:
    """加真机连接参数 ``--channel`` / ``--can-id`` / ``--mst-id`` / ``--calib``。"""
    parser.add_argument(
        "--channel", default="can0",
        help="SocketCAN 接口名（默认 can0）",
    )
    parser.add_argument(
        "--can-id", default=0x08, type=lambda s: int(s, 0),
        help="夹爪的 CAN ID（默认 0x08）",
    )
    parser.add_argument(
        "--mst-id", default=0x18, type=lambda s: int(s, 0),
        help="主控（达妙电机 MIT 协议）ID（默认 0x18）",
    )
    parser.add_argument(
        "--calib", default=None,
        help="用户标定文件路径（默认用 SDK 的默认路径，"
             "没有则退回出厂标定）",
    )


def open_real_gripper(args: argparse.Namespace, enable: bool = True):
    """按命令行参数连接真机夹爪：connect → load_calibration → enable。

    任一步失败都打印可读的原因并 ``SystemExit(1)``，不会抛裸异常。

    Args:
        args: 命令行参数（``--channel`` / ``--can-id`` / ``--mst-id`` / ``--calib``）。
        enable: 是否使能。``False`` 时只连接并载入标定，**一个 CAN 帧都不发**，
            电机保持原状——用来在不动电机的前提下先看看状态。

    Returns:
        ``litegrip.LiteGrip``（``enable=True`` 时已使能）。
    """
    litegrip = import_litegrip()
    check_sdk_api(litegrip)   # 缺公开接口就别连——宁可现在停，也别在循环里才发现

    print(f"[真机] 连接 {args.channel} · can_id={args.can_id:#04x} · "
          f"mst_id={args.mst_id:#04x}")
    gripper = litegrip.LiteGrip(
        channel=args.channel, can_id=args.can_id, mst_id=args.mst_id
    )
    try:
        if not gripper.connect():
            raise SystemExit(
                f"❌ 连不上 {args.channel}。检查：\n"
                f"   1) 接口是否存在且已起来 —— "
                f"sudo ip link set {args.channel} up type can bitrate 1000000\n"
                f"   2) ip -details link show {args.channel}\n"
                f"   3) 夹爪是否已上电、CAN_H/CAN_L 是否接对、"
                f"终端电阻（120Ω）是否装了"
            )
        # 标定必须在 enable 之前载入：SDK 的毫米刻度依赖它
        gripper.load_calibration(args.calib)
        check_calibration(gripper)   # 标定不对的话，下面的目标角就没意义
        if not enable:
            print("[真机] 已连接、已载入标定（未使能，不发送任何帧）")
            return gripper
        if not gripper.enable():
            raise SystemExit("❌ 使能失败：夹爪可能处于错误状态或未上电")
    except SystemExit:
        gripper.disconnect()
        raise
    except Exception as exc:  # SDK 的各种 *Error
        gripper.disconnect()
        raise SystemExit(f"❌ 初始化真机失败：{exc}") from exc

    cfg = gripper.config
    # kp/kd 一起打出来：它们是标定文件里的值（也是保持帧的刚度），改了标定之后
    # 「手感怎么变了」这个问题，第一件要看的就是这两个数。
    print(f"[真机] 已使能 · 行程 {cfg.max_stroke_mm:.1f} mm（SDK 刻度）"
          f" · rad_to_mm={cfg.rad_to_mm:.2f}"
          f" · kp={cfg.kp:g} kd={cfg.kd:g}")
    return gripper


def fraction_to_target_rad(gripper, fraction: float) -> float:
    """归一化开度 → 真机的电机目标角 [rad]。

    和 SDK 的 ``goto(position_mm)`` 用同一套换算，只是分两步写出来：

        position_mm  = fraction × max_stroke_mm
        position_rad = pos_closed_rad − position_mm / rad_to_mm

    注意 SDK 的毫米是**标定过的刻度**（``max_stroke_mm`` 名义 120），不是物理
    钳口间隙；两侧唯一对齐的量是这里的归一化开度。

    钳位沿用 ``goto`` 的写法，也就继承了它的前提：``pos_open_rad`` 必须是更**负**
    的那个（标定后如此）。SDK 的出厂默认配置把 ``pos_open_rad`` 写成 ``+1.14``，
    与 ``goto`` 的符号约定相矛盾，此时结果没有意义——所以真机样例在使能前会用
    :func:`check_calibration` 挡掉这种配置，而不是硬发一条越界的角度。
    """
    cfg = gripper.config
    position_mm = max(0.0, min(1.0, float(fraction))) * cfg.max_stroke_mm
    position_rad = cfg.pos_closed_rad - position_mm / cfg.rad_to_mm
    return max(cfg.pos_open_rad, min(cfg.pos_closed_rad, position_rad))


def check_calibration(gripper) -> None:
    """确认标定值自洽，不自洽就带着原因退出。

    ``goto`` 的换算要求 ``pos_closed_rad`` 是更大的那个（闭合 → 角度更正），
    ``pos_open_rad`` 更负。SDK 的出厂默认配置恰好相反，说明这台机器还没跑过
    ``calibrate()``／没载入标定文件——此时任何目标角都是瞎猜的，直接停下比发出去
    让手指撞限位好。
    """
    cfg = gripper.config
    travel = cfg.pos_closed_rad - cfg.pos_open_rad
    if travel <= 0.0 or cfg.rad_to_mm <= 0.0 or cfg.max_stroke_mm <= 0.0:
        raise SystemExit(
            "❌ 夹爪的标定值不合法，先做标定再跑：\n"
            f"   pos_closed_rad={cfg.pos_closed_rad:+.4f} "
            f"pos_open_rad={cfg.pos_open_rad:+.4f} "
            f"rad_to_mm={cfg.rad_to_mm:.2f} max_stroke_mm={cfg.max_stroke_mm:.1f}\n"
            "   闭合位应当比张开位角度更大（pos_closed_rad > pos_open_rad）。\n"
            "   常见原因：没载入标定文件，还在用 SDK 的出厂默认值。\n"
            "   试：gripper.calibrate() 生成标定，或用 --calib 指定标定文件。"
        )


def rad_to_fraction(gripper, position_rad: float) -> float:
    """真机的电机角 [rad] → 归一化开度（:func:`fraction_to_target_rad` 的逆）。"""
    cfg = gripper.config
    position_mm = (cfg.pos_closed_rad - float(position_rad)) * cfg.rad_to_mm
    if cfg.max_stroke_mm <= 0.0:
        return 0.0
    return max(0.0, min(1.0, position_mm / cfg.max_stroke_mm))


#: 读真机状态时最多等一帧状态帧的时间 [s]。SDK 的 ``get_state(wait=True)`` 内部
#: 也是等 50 ms，这里对齐它。
FRESH_WAIT_S = 0.05

#: 只读路径（``--status``）等一帧的时间 [s]：那条路径没有别人的帧可等（不使能
#: → 电机不主动发帧），得先发一帧**只读的** 0xCC 请它回话，所以给得比
#: :data:`FRESH_WAIT_S` 宽。
STATUS_WAIT_S = 0.5


def request_status_frame(gripper, timeout_s: float = FRESH_WAIT_S) -> bool:
    """请电机主动回一帧状态（``LiteGrip.refresh_status``：0xCC，**不改变输出**）。

    SDK 把它做成了公开方法，而且它**自己就把等待做完了**：发一帧 0xCC 刷新请求，
    然后等到真的收到一帧新的状态帧为止。所以这里只是把「传输层出错也算没有帧」
    包住，让调用方拿到一个布尔值——**不再额外 poll**：同一个时间预算花两遍，等于
    把主循环的最坏停顿翻倍。

    为什么需要它：**没使能的电机不会自己发帧**，光 poll 永远等不到状态帧，于是
    ``get_state()`` 返回的是 ``MotorState._position`` 的初值 ``0.0``——那不是
    「夹爪在 0 弧度」，是「从没读到过」。只读诊断想看真实位置，就得主动要一帧
    （实测：电机答 0xCC 时读到 −0.370 rad，而缓存说是 0.0）。

    Args:
        timeout_s: 最多等多久 [s]。默认沿用 :data:`FRESH_WAIT_S`，与
            ``get_state(wait=True)`` 的 50 ms 对齐；``--status`` 那条路用
            :data:`STATUS_WAIT_S`。

    Returns:
        True = ``timeout_s`` 内确实收到了一帧**新**状态帧。False = 没收到——电机
        没答、总线出错，对调用方是同一件事：**拿不到新鲜读数，不许下发**。
    """
    try:
        return bool(gripper.refresh_status(timeout_s=timeout_s))
    except Exception:      # 传输层掉了等等：没有帧就是没有帧，别把调用方带崩
        return False


def fresh_state(gripper, timeout_s: float = FRESH_WAIT_S, *, request: bool = False):
    """等到一帧**新**的状态帧再读缓存；等不到返回 ``None``。

    这是本仓库读真机位置的正确入口（02/03 都用它）。「这份位置是不是*现在*的」
    这件事，SDK 的公开接口里有**两个互补**的回答，这里两道门都过才算数：

    * :meth:`LiteGrip.poll` —— True 表示「刚收到的这一帧是我们电机的状态帧」
      （读寄存器的应答不算）。它回答「**刚**有没有帧」；
    * ``GripperState.has_data`` / ``is_stale`` / ``data_age_s`` —— 这份快照到底
      有没有被数据支撑。它回答「**这份快照**是不是量出来的」。

    为什么要两道：``data_age_s`` 量的是「距上次**读到**帧」，时间戳在 SDK 的
    transport 里打，滞留在接收队列里的旧帧读出来照样显得新鲜（SDK 自己把这条标了
    「待查」）；而 ``poll`` 信任的是 SDK 对「状态帧」的判定。合起来才是「这一帧
    是现在的」。

    为什么非要问这一句：缓存里可能是 ``MotorState._position`` 的初值 ``0.0``，
    或者一个冻结的旧值。拿它当「现在的位置」去算目标角和斜坡时长，算出来的是一
    条指向别处的**阶跃**指令——电机按标定里的 ``kp``（SDK 默认 100 Nm/rad）去追
    一个不存在的误差，就是「一开夹爪就起飞」的形态。所以拿不到新鲜读数时，调用方
    应当**拒绝下发**，而不是猜一个值。

    Args:
        timeout_s: 最多等多久 [s]。
        request: 等之前先发一帧只读的 ``0xCC`` 请求并等它回，见
            :func:`request_status_frame`——电机没在主动发帧时（未使能、或者
            没人喂帧）才需要。这条路径**不再**额外 poll：``refresh_status`` 自己
            就把这个预算花完了。

    Returns:
        ``GripperState``；``timeout_s`` 内没有新的状态帧则 ``None``。
    """
    if request:
        if not request_status_frame(gripper, timeout_s=timeout_s):
            return None
    elif not gripper.poll(timeout_s=timeout_s):
        return None
    state = gripper.get_state(wait=False)
    if not state.has_data or state.is_stale:
        # 来了帧，但这份快照不是「现在」的（从没被数据支撑过，或者已经过期）。
        # 少发一帧不会让电机乱动，拿它算目标会。
        return None
    return state


def status_line(
    label: str,
    *,
    fraction: float,
    aperture_mm: float,
    sdk_mm: float | None = None,
    force_n: float | None = None,
    moving: bool | None = None,
) -> str:
    """一行状态文本，三个样例共用，保证口径一致。

    Args:
        label: 行首标签（``仿真`` / ``真机``）。
        fraction: 归一化开度（0 闭合 … 1 张开）——两侧唯一可比的量。
        aperture_mm: 物理钳口间隙 [mm]。
        sdk_mm: SDK 刻度位置 [mm]，真机才有。
        force_n: 夹持力 [N]。
        moving: 是否在动，真机才有。
    """
    bar_width = 20
    filled = int(round(bar_width * max(0.0, min(1.0, fraction))))
    bar = "█" * filled + "·" * (bar_width - filled)
    parts = [f"[{label}] {bar} {fraction * 100:5.1f}%", f"开口 {aperture_mm:5.2f} mm"]
    if sdk_mm is not None:
        parts.append(f"SDK {sdk_mm:6.2f} mm")
    if force_n is not None:
        parts.append(f"力 {force_n:5.2f} N")
    if moving is not None:
        parts.append("运动中" if moving else "已停住")
    return " · ".join(parts)


# 导入本模块时就准备好环境，这样样例只要一句 `from _common import ...` 即可，
# 不需要记住「必须先 import _common 再 import litegrip_pybullet」这个顺序。
bootstrap_src()
ensure_deps()
