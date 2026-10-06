#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""_common.py — LiteGrip PyBullet 样例的共用启动样板。

五个样例（01–05）共享这里的东西：

  ensure_deps()      缺 pybullet 时自动改用仓库自带 .venv 重跑
  import_litegrip()  导入真机 SDK（$LITEGRIP_SDK_DIR / 同级 litegrip-python
                     仓库 / 已安装的 litegrip）
  check_sdk_api()    核对 SDK 有没有本仓库依赖的公开接口，缺了就在启动时停下
  add_common_args()  --urdf / --headless
  add_hardware_args() --channel / --can-id / --mst-id / --calib
  factory_calibration_path()  SDK 包里那份出厂标定文件的路径（跟着包走）
  choose_calibration_file() 定下这次用**哪一份**标定：--calib 指定 → SDK 出厂
                     标定 → 两个都没有才当场从候选里选
  calibration_config()  把标定文件装成 ``gripper.config`` 的形状（--dry-run 用）
  open_real_gripper() 连接 → 载入并核实标定 → 使能，失败时给出可读的提示
  fresh_state()      等到一帧**新**的状态帧再读位置；等不到返回 None
                     （读真机位置只该走这里，别直接读 get_state() 的缓存）

三个真机样例（03/04/05）用**同一份** SDK 检出：带轨迹录制/回放的那份
（``nexform-tech/litegrip-python``）。它不在 PyPI 上，`pip install litegrip`
装到的是别的代码，所以要从检出装或把目录指出来——见 :func:`import_litegrip`。

04/05 会驱动真机！真机的两个手指会真的闭合。首次跑请：
  1) 把夹爪拿在手上或固定在台面上，**手指行程内不要放任何东西**；
  2) 手放在电源开关旁边；
  3) 先用 --dry-run（05）跑一遍看看流程。

不指定 ``--calib`` 时用的是 SDK 包里那份**出厂标定**——它是台架夹具的实测参数，
而标定的角度/毫米刻度本该是每台夹爪单独量的。拿不准就用 ``--calib`` 指这台夹爪
自己的那份：上位机 ``litegrip-studio`` / ``litegrip-console`` 标定后保存，或
SDK 自带的 ``tools/gui/litegrip_gui.py``。

真机跑之前确认 CAN 已配置好：

    sudo ip link set can0 up type can bitrate 1000000
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path
from types import SimpleNamespace

__all__ = [
    "CALIBRATIONS_DIR",
    "NOMINAL_KD",
    "NOMINAL_KP",
    "NOMINAL_STROKE_MM",
    "REQUIRED_CALIB_KEYS",
    "REQUIRED_SDK_API",
    "SAFETY_BANNER",
    "SIM_CALIBRATION_MARKER",
    "add_common_args",
    "add_hardware_args",
    "bootstrap_src",
    "calibration_candidates",
    "calibration_config",
    "calibration_summary",
    "check_calibration",
    "check_calibration_matches_args",
    "check_calibration_values",
    "check_sdk_api",
    "choose_calibration_file",
    "ensure_deps",
    "factory_calibration_path",
    "fraction_to_target_rad",
    "fresh_state",
    "import_litegrip",
    "load_chosen_calibration",
    "missing_sdk_api",
    "open_real_gripper",
    "rad_to_fraction",
    "read_calibration_file",
    "sdk_dir",
    "status_line",
]

#: 真机样例开跑前打印的横幅。
SAFETY_BANNER = """\
即将驱动真机：夹爪两个手指会真实运动。
    请确认行程内无遮挡、人员远离，并让电源开关触手可及。
    再确认一次下面打印的那份标定文件：它是**这台夹爪**标出来的，还是 SDK 自带的
    出厂标定（台架夹具的实测参数）。用出厂那份驱动，行程端点可能与这台对不上。
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

    这样 ``python3 examples/01_hello_sim.py`` 开箱即用，不必先 pip install。
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

    顺序：``$LITEGRIP_SDK_DIR`` → 同级 ``litegrip-python`` 仓库 → 已安装的
    ``litegrip``。

    为什么同级检出排在**已安装的**前面：本仓库要的是带轨迹录制/回放的那份
    （``litegrip-python``），而机器上装着的 ``litegrip`` 可能是另一个仓库的同名
    包——两份的 ``__version__`` 都是 ``2.2.0``，光看版本号分不出来。同级目录就在
    眼前、名字点得很明确，优先信它。
    """
    env = os.environ.get("LITEGRIP_SDK_DIR")
    if env:
        return Path(env).expanduser()
    # 同级检出：``litegrip-python`` 是 src 布局（包在 src/litegrip），也接受把包
    # 直接放在仓库根下的布局——两种都试，免得只认一种。
    sibling = _REPO_ROOT.parent / "litegrip-python"
    for candidate in (sibling / "src", sibling):
        if (candidate / "litegrip" / "__init__.py").is_file():
            return candidate
    try:
        import litegrip  # noqa: F401

        return Path(litegrip.__file__).resolve().parent.parent
    except Exception:
        return None


def import_litegrip():
    """导入真机 SDK，失败时给出安装提示并退出。

    先找目录再**显式加载**，不能靠 ``sys.path`` 顺序：``pip install -e`` 装的那份
    会注册一个 meta path finder，优先级高于 ``sys.path``，把目录插到最前面也没用
    ——理由见 :func:`_load_package_from`。

    ``$LITEGRIP_SDK_DIR`` 指错目录时**不悄悄换一份**：本机装着的 ``litegrip`` 可能
    是另一个仓库的同名包（两份的 ``__version__`` 都是 2.2.0），悄悄换过去会让人以
    为「指了却能跑」，实际跑的是别的代码。

    Returns:
        已导入的 ``litegrip`` 模块。
    """
    directory = sdk_dir()
    if directory is not None:
        try:
            module = _load_package_from(directory)
        except Exception as exc:           # 那份检出自己炸了（语法错误等）
            raise SystemExit(f"从 {directory} 加载 litegrip 失败：{exc}") from exc
        if module is not None:
            return module
        requested = os.environ.get("LITEGRIP_SDK_DIR")
        if requested:
            raise SystemExit(
                f"找不到真机 SDK（litegrip 包）：LITEGRIP_SDK_DIR 指到 {requested}，"
                f"但那里没有 litegrip/__init__.py。\n"
                "   这个变量要指到**包所在目录**（src 布局就是 "
                "<仓库>/litegrip-python/src），不是仓库根、也不是包目录本身。\n"
                "   不想指定就 unset LITEGRIP_SDK_DIR；否则三种任选其一：\n"
                "   1) python3 -m pip install -e /path/to/litegrip-python\n"
                "   2) export LITEGRIP_SDK_DIR=/path/to/litegrip-python/src\n"
                "   3) 把 litegrip-python 仓库克隆到本仓库的同级目录"
            )
        # 走到这里说明是自己找到的目录（同级检出 / 已安装包）里没有包——同级目录
        # 名字没对上而已，落到下面的 import 再看。
    try:
        import litegrip
    except ImportError as exc:
        raise SystemExit(
            "找不到真机 SDK（litegrip 包）。\n"
            "   litegrip **没有发布到 PyPI**，`pip install litegrip` 装的不是它；\n"
            "   本仓库要的是带轨迹录制/回放的那份检出\n"
            "   （nexform-tech/litegrip-python）。三种任选其一：\n"
            "   1) python3 -m pip install -e /path/to/litegrip-python\n"
            "   2) export LITEGRIP_SDK_DIR=/path/to/litegrip-python/src\n"
            "   3) 把 litegrip-python 仓库克隆到本仓库的同级目录\n"
            f"   （原始错误：{exc}）"
        ) from exc
    return litegrip


#: 本仓库依赖的 SDK 公开接口。清单化而不是散在各调用处：缺哪一个就在**启动时**
#: 说清楚该换哪份 SDK，而不是在发帧的循环里抛 AttributeError。
#:
#: 三个真机样例（03/04/05）用的是**同一份**检出，所以这里是一份并集：轨迹那几个是
#: 03 要的，其余是 04/05 要的。分两份清单是上一版的事——那时「新鲜度」接口
#: （``refresh_status`` / ``GripperState.data_age_s``）和轨迹接口分在两个仓库里，
#: 只能按能力分辨（两份的 ``__version__`` 都是 2.2.0）。现在统一到带轨迹那份，
#: 它**没有**那几个新鲜度接口，读取新鲜度只靠 :func:`fresh_state` 里的 ``poll()``
#: ——理由写在那里。
#:
#: 这些接口目前没有任何发行版带（litegrip 也不在 PyPI 上），所以
#: ``pip install litegrip`` 装到的那份一定缺它们——这正是要拦的情况。
#:
#: 写法是 ``名字.属性`` 或裸的模块级名字（如 ``trajectory_dir``），两种都认。
REQUIRED_SDK_API: tuple[tuple[str, str], ...] = (
    ("LiteGrip.connect", "连上 CAN 并注册夹爪"),
    ("LiteGrip.enable", "使能电机——不使能就一个运动指令都发不出去"),
    ("LiteGrip.disable", "退出前把电机放回失力状态"),
    ("LiteGrip.disconnect", "关掉 CAN 连接"),
    ("LiteGrip.poll",
     "等一帧**状态帧**；fresh_state() 靠它区分「刚量到的」和「缓存里的」"),
    ("LiteGrip.get_state", "读一次位置/速度/力矩/错误码快照"),
    ("LiteGrip.send_mit_frame", "发一帧 MIT 指令：锁位帧和零力矩帧都走它"),
    ("LiteGrip.read_param", "按 RID 读电机寄存器（只读诊断用）"),
    ("LiteGrip.clear_fault", "清锁存的故障码"),
    ("LiteGrip.enter_zero_gravity", "进零重力：03/04 靠它让人手拖动手指"),
    ("LiteGrip.exit_zero_gravity", "退出零重力并锁在当前位"),
    ("LiteGrip.move_at_speed", "按速度走一段（05 的滑条路径）"),
    ("LiteGrip.load_calibration", "载入标定文件——毫米刻度和行程端点都从它来"),
    ("LiteGrip.record_start",
     "在后台开始录制；zero_gravity=True 时由它自己流零力矩帧"),
    ("LiteGrip.record_stop", "停止录制并取回轨迹"),
    ("LiteGrip.play_start",
     "在后台开始回放——主循环才腾得出手同步刷仿真（play() 会阻塞到放完）"),
    ("LiteGrip.play_stop", "停止回放，并把夹爪留在最后一个目标位上"),
    ("LiteGrip.trajectory_status",
     "录制/回放的进度：active / completed / openness"),
    ("GripperState.position_rad", "状态快照里的电机角，开度换算的输入"),
    ("Trajectory.load", "读回一段 ``.lgt``（纯文件 I/O，不碰 CAN）"),
    ("Trajectory.openness_at", "按时间取归一化开度——离线 --play 靠它驱动仿真"),
    ("trajectory_dir", "轨迹默认存在哪（``~/.litegrip/trajectories``）"),
)


def missing_sdk_api(litegrip) -> list[str]:
    """已导入的 SDK 里缺哪些必需接口（按 :data:`REQUIRED_SDK_API` 的顺序）。"""
    missing: list[str] = []
    for path, _why in REQUIRED_SDK_API:
        owner, sep, attr = path.partition(".")
        if sep:
            found = hasattr(getattr(litegrip, owner, None), attr)
        else:
            found = hasattr(litegrip, owner)   # 模块级的名字，如 trajectory_dir
        if not found:
            missing.append(path)
    return missing


def check_sdk_api(litegrip) -> None:
    """缺必需接口就带着「该用哪份 SDK」退出（``SystemExit``）。

    **不保留降级路径**：这些接口没有替代品——录制是 SDK 在后台线程里按自己的节拍
    采样和发帧的，自己拿 ``send_mit_frame`` 拼一个循环只会得到一份节拍对不上的
    样本。真机样例宁可不跑，也不拿一个猜出来的位置去算目标角——那正是一条指向别处
    的阶跃指令的成因。
    """
    missing = missing_sdk_api(litegrip)
    if not missing:
        return
    why = dict(REQUIRED_SDK_API)
    directory = sdk_dir()
    raise SystemExit(
        "这份 litegrip SDK 缺少本仓库必须的公开接口：\n"
        + "".join(f"     • {path} —— {why[path]}\n" for path in missing)
        + "   litegrip **没有发布到 PyPI**（`pip install litegrip` 装到的不是这份"
          "代码）。\n"
          "   本仓库要的是带轨迹录制/回放的那份检出，请指到它：\n"
          "     python3 -m pip install -e /path/to/litegrip-python  # 或\n"
          "     export LITEGRIP_SDK_DIR=/path/to/litegrip-python/src\n"
        + (f"   （这次导入到的是：{directory}）" if directory is not None else "")
    )


def _load_package_from(directory):
    """从指定目录显式加载 ``litegrip`` 包，不走 ``import litegrip``。

    为什么必须显式加载：``pip install -e`` 装的那份会注册一个 **meta path
    finder**，它的优先级高于 ``sys.path``——所以「把要用的那份插到 sys.path 最
    前面」在装了 editable 版的机器上一点用都没有，``import litegrip`` 拿到的还是
    装的哪份。同级目录 checkout 与已安装的包同名时，只能按目录点名加载。

    副作用是 ``sys.modules["litegrip"]`` 被换掉（包括它已经导入过的子模块）：这个
    进程从这一句起就用这一份，这正是想要的。
    """
    import importlib.util

    init = Path(directory) / "litegrip" / "__init__.py"
    if not init.is_file():
        return None
    for name in [m for m in sys.modules if m == "litegrip"
                 or m.startswith("litegrip.")]:
        del sys.modules[name]
    spec = importlib.util.spec_from_file_location(
        "litegrip", init, submodule_search_locations=[str(init.parent)])
    module = importlib.util.module_from_spec(spec)
    sys.modules["litegrip"] = module       # 子模块与 dataclass 都要先看到它
    try:
        spec.loader.exec_module(module)
    except Exception:                      # 导入一半失败：别留下半个包
        sys.modules.pop("litegrip", None)
        raise
    return module


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
        help="标定文件路径。不给就用 SDK 包里那份**出厂标定**（台架夹具的实测"
             "参数，跟着 SDK 包走，换电脑也指得到）；要按这**台**夹爪自己的尺寸"
             "驱动，就得给出它的标定。出厂文件也读不出来时才在终端里列候选让你"
             "选。标定文件由 litegrip-studio / litegrip-console 对着真机标定后"
             "保存得到",
    )


#: 用户标定文件所在目录（SDK 的 ``save_calibration`` 默认也写这里）。
CALIBRATIONS_DIR = Path.home() / ".litegrip"

#: 上位机给**仿真**后端单独存的那一份的后缀（``litegrip_calibration.sim.json``）。
#: 它记录的是仿真里的刻度，拿它去算真机的目标角是错的——上位机刻意与真机文件分开
#: 命名，这里也必须分开对待：候选里不收，显式指定也拒。
SIM_CALIBRATION_MARKER = ".sim.json"

#: SDK 载入标定时**无保护**索引的三个键（``gripper.load_calibration`` 里直接
#: ``data["zero_position_rad"]`` 这样取）。少了它们 SDK 会抛 KeyError，所以这里先
#: 自己查一遍，好把话说明白。
REQUIRED_CALIB_KEYS = ("zero_position_rad", "max_position_rad", "rad_to_mm")

#: 标定文件的键 → ``GripperConfig`` 上的字段。用来核实「载入的确实是这份文件」，
#: 以及比对文件里记的连接参数和这次命令行给的一致不一致。
_CALIB_FIELDS: tuple[tuple[str, str], ...] = (
    ("zero_position_rad", "pos_closed_rad"),
    ("max_position_rad", "pos_open_rad"),
    ("rad_to_mm", "rad_to_mm"),
    ("kp", "kp"),
    ("kd", "kd"),
    ("can_id", "can_id"),
    ("mst_id", "mst_id"),
    ("channel", "can_channel"),
)

#: SDK ``GripperConfig.max_stroke_mm`` 的默认值 [mm]。标定文件里**没有**这一项
#: （它是「名义行程」而不是量出来的尺寸），所以只读文件、拿不到 config 的
#: ``--dry-run`` 用这个值做自洽性检查。
#:
#: 开度换算**不**用它。归一化开度按标定行程归一，见 :func:`rad_to_fraction`：
#: 这个名义值和标定文件里 ``rad_to_mm`` 那套刻度是可以对不上的。
NOMINAL_STROKE_MM = 120.0

#: SDK ``GripperConfig`` 里 ``kp``/``kd`` 的默认值。标定文件里这两项**可选**，
#: 缺了 SDK 就留着 config 上的原值（``load_calibration`` 只对出现在文件里的键
#: ``setattr``），也就是这两个数。
NOMINAL_KP = 100.0
NOMINAL_KD = 2.0


def calibration_config(data: dict, *, max_stroke_mm: float = NOMINAL_STROKE_MM):
    """把一份标定文件装成 ``gripper.config`` 的形状。

    ``--dry-run`` 不连真机、也就没有 SDK 的 ``GripperConfig``，但它走的正是这套
    参数（两个端角、kp/kd），照样需要一个 config 来算。这里按
    :data:`_CALIB_FIELDS` 那张表装，字段名不另写一份；缺的可选字段沿用 SDK 的
    默认值（见 :data:`NOMINAL_KP`）。``max_stroke_mm`` 不在标定文件里，用名义值
    ——它只进 :func:`check_calibration` 的自洽性检查，不进开度换算。

    只用于「不碰真机、但要算同一套数」的场合。真机路径上用的永远是 SDK 自己那份
    ``gripper.config``——那份是 :func:`load_chosen_calibration` 核实过的。
    """
    values: dict[str, object] = {"kp": NOMINAL_KP, "kd": NOMINAL_KD}
    values.update({attr: data[key] for key, attr in _CALIB_FIELDS if key in data})
    values["max_stroke_mm"] = max_stroke_mm
    return SimpleNamespace(**values)


def factory_calibration_path(litegrip) -> Path:
    """SDK 包里那份**出厂标定**的路径。

    跟着 ``litegrip`` 包所在目录解析（``<包目录>/factory_calibration.json``），
    所以换电脑、换虚拟环境、换 SDK 检出都指得到——**不要**把它写成某个本机绝对
    路径，这正是这个函数存在的理由。

    SDK 自己把这份文件当作 ``load_calibration()`` 的最后一级兜底（它内部叫
    ``_FACTORY_CALIB``，是私有的，这里按包目录自己拼，不碰私有名字）。

    这份文件是**台架夹具的实测参数**，不是每台夹爪各自量的：它是一份能用的默认
    值，不是「这台夹爪的标定」。要按这台夹爪自己的尺寸驱动，用 ``--calib`` 指
    上位机保存的那份。
    """
    return Path(litegrip.__file__).resolve().parent / "factory_calibration.json"


def calibration_candidates(directories=None) -> list[Path]:
    """真机上可用的标定文件候选，按修改时间从新到旧。

    只收 ``*.json``，并且排除两类：

    * ``*.sim.json`` —— 上位机给仿真后端单独存的那份（见
      :data:`SIM_CALIBRATION_MARKER`），刻度不是真机的；
    * ``*.bak`` —— 备份/历史，不是现在生效的那份。名字形如
      ``litegrip_calibration.json.20260928.bak``，本来就不以 ``.json`` 结尾，
      这里再显式排一次，免得哪天备份改了命名习惯就混进来。

    Args:
        directories: 去哪里找；默认是 :data:`CALIBRATIONS_DIR`，外加
            ``$LITEGRIP_CALIB`` 所在目录（SDK 认这个变量，操作员也可能把标定
            放在别处再从那里指过来）。
    """
    if directories is None:
        directories = [CALIBRATIONS_DIR]
        env = os.environ.get("LITEGRIP_CALIB")
        if env:
            directories = list(directories) + [Path(env).expanduser().parent]
    found: list[Path] = []
    for directory in directories:
        directory = Path(directory)
        if not directory.is_dir():
            continue
        for path in directory.glob("*.json"):
            name = path.name
            if name.endswith(SIM_CALIBRATION_MARKER) or name.endswith(".bak"):
                continue
            if path not in found:
                found.append(path)
    return sorted(found, key=lambda p: p.stat().st_mtime, reverse=True)


#: 摘要里打哪几个键、怎么打。ID 按十六进制——命令行的 ``--mst-id 0x18`` 和日志
#: 里的 ``0x18`` 都是这么写的，十进制 24 只会让人多换算一次（而把 0x18 写成 18
#: 正是这批文件里真出过的事故）。
_CALIB_SUMMARY: tuple[tuple[str, str], ...] = (
    ("zero_position_rad", "closed {v:+.4f}"),
    ("max_position_rad", "open {v:+.4f}"),
    ("rad_to_mm", "rad_to_mm {v:g}"),
    ("kp", "kp {v:g}"),
    ("kd", "kd {v:g}"),
    ("mst_id", "mst_id {v:#04x}"),
    ("can_id", "can_id {v:#04x}"),
)


def calibration_summary(data: dict) -> str:
    """一行摘要：这份标定的关键值（文件里有什么就打什么）。

    打出来是为了让操作员在下发前能认出「这就是我刚在这台机器上标出来的那份」。
    SDK 自己**不记录**用了哪个文件（``load_calibration`` 只往日志写一行），
    所以来源这件事只能由调用方说清楚。
    """
    shown: list[str] = []
    for key, template in _CALIB_SUMMARY:
        if key not in data:
            continue
        try:
            value = float(data[key])
        except (TypeError, ValueError):
            shown.append(f"{key}={data[key]!r}")
            continue
        shown.append(template.format(v=int(value) if "#04x" in template
                                     else value))
    return " · ".join(shown)


def read_calibration_file(path) -> dict:
    """读一份标定文件并查它能用；不能就带着原因退出（``SystemExit``）。

    为什么要自己读一遍：SDK 的 ``load_calibration(path)`` 在**文件不存在或 JSON
    坏了**的时候，会**静默回退到打包的出厂标定并且返回 True**——调用方从返回值
    上分不出来。于是「--calib 指了个打错的路径」会变成「悄悄用出厂尺寸驱动真机」。
    所以路径必须自己验，载入之后还要再核一次（见 :func:`load_chosen_calibration`）。
    """
    path = Path(path).expanduser()
    if str(path).endswith(SIM_CALIBRATION_MARKER):
        raise SystemExit(
            f"这是一份**仿真**标定：{path}\n"
            f"   上位机给仿真后端单独存一份（名字以 {SIM_CALIBRATION_MARKER} 结尾），"
            "里面的刻度是仿真里的，\n"
            "   不是这台真机量出来的。拿它算真机的目标角，轻则夹不住、重则撞限位。\n"
            "   真机的标定请在上位机里对着真机做一遍，另存为不带 "
            f"{SIM_CALIBRATION_MARKER} 的名字。"
        )
    if not path.is_file():
        raise SystemExit(
            f"标定文件不存在：{path}\n"
            "   （SDK 在这种情况会**静默改用出厂标定**，所以这里宁可停下。）\n"
            "   标定文件由上位机标定后保存：litegrip-studio / litegrip-console，\n"
            "   或 SDK 自带的 tools/gui/litegrip_gui.py。"
        )
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SystemExit(f"标定文件读不出来：{path}\n   （{exc}）") from exc
    if not isinstance(data, dict):
        raise SystemExit(f"标定文件不是一份 JSON 对象：{path}")
    missing = [key for key in REQUIRED_CALIB_KEYS if key not in data]
    if missing:
        raise SystemExit(
            f"标定文件缺字段：{path}\n"
            f"   缺 {'、'.join(missing)}（SDK 载入时直接取这几个键，缺了会抛 "
            "KeyError）。\n"
            "   看起来不是上位机保存的标定文件；重新标定并另存一份。"
        )
    return data


def choose_calibration_file(requested=None, *, factory=None, candidates=None,
                            ask=input, out=print) -> Path:
    """定下这次用哪份标定文件。三档，**顺序就是优先级**。

    * ``--calib <路径>``（``requested``）：直接用，不提问；只做校验。
    * ``factory`` 给了且读得出来：用 SDK 包里那份出厂标定，**不提问**，只打一行
      说明。这是默认路径——``factory`` 由调用方用
      :func:`factory_calibration_path` 算出来，所以它跟着包走，换电脑也一样。
    * 出厂标定也读不出来（SDK 装得残缺、文件被删）：才回到选择器——列出候选让
      操作员当场选；非交互（stdin 不是 tty、EOF）或没有候选就直接退出。

    出厂标定是**台架夹具的实测参数**，不是每台夹爪各自量的：它是一份能用的默认
    值，不是「这台夹爪的标定」。要按这台夹爪自己的尺寸驱动，用 ``--calib`` 指
    上位机保存的那份——所以第 2 档那行提示必须把这句话说出来，不能让人以为默认
    值就是自己的标定。

    Args:
        requested: ``--calib`` 的值（``None`` = 没给）。
        factory: 出厂标定文件的路径；``None`` 表示调用方拿不到 SDK，直接进选择器。
        candidates: 候选文件；默认 :func:`calibration_candidates`。
        ask: 取输入的函数（默认 ``input``）——测试注入用。
        out: 打印函数（默认 ``print``）——测试注入用。

    Returns:
        选中的标定文件路径（已校验存在、可解析、字段齐、不是仿真那份）。
    """
    if requested:
        path = Path(requested).expanduser()
        read_calibration_file(path)
        return path

    if factory is not None:
        path = Path(factory).expanduser()
        try:
            read_calibration_file(path)
        except SystemExit:
            pass          # 出厂文件不在/读不出来：落到下面的选择器，别把它当终局
        else:
            out(f"未指定 --calib：使用 SDK 自带的出厂标定 {path}")
            out("   （台架夹具的实测参数，不是这台夹爪自己量的。"
                "换 --calib <路径> 指这台夹爪的那份。）")
            return path

    found = list(calibration_candidates() if candidates is None else candidates)
    listing = "".join(f"     {i}) {p}\n" for i, p in enumerate(found, 1))
    if not found:
        raise SystemExit(
            f"没有指定标定文件，SDK 自带的出厂标定也读不出来，"
            f"候选里也没有（{CALIBRATIONS_DIR} 下没有 *.json）。\n"
            "   请先用上位机对着真机标定并保存：\n"
            "     litegrip-studio / litegrip-console（或 SDK 自带 "
            "tools/gui/litegrip_gui.py）\n"
            "   然后重跑；也可以直接指路径：--calib /path/to/"
            "litegrip_calibration.json"
        )
    if not sys.stdin.isatty():
        raise SystemExit(
            "需要先选定标定文件，但当前不是交互终端（stdin 不是 tty），"
            "没法让你选。\n"
            "   SDK 自带的出厂标定也读不出来，所以没得默认。\n"
            "   脚本/非交互请显式指定：--calib <路径>\n"
            "   找到的候选：\n"
            + listing
            + "   标定文件在上位机里标定后保存得到（litegrip-studio / "
              "litegrip-console）。"
        )

    out("SDK 自带的出厂标定读不出来，请选定**这台夹爪**的标定文件"
        "（标定文件在上位机里标定后保存：\n"
        "litegrip-studio / litegrip-console，或 SDK 自带的 "
        "tools/gui/litegrip_gui.py）。")
    out("  找到这些候选：")
    for i, path in enumerate(found, 1):
        try:
            stamp = time.strftime("%Y-%m-%d %H:%M",
                                  time.localtime(path.stat().st_mtime))
        except OSError:
            stamp = "?"
        try:
            summary = calibration_summary(read_calibration_file(path))
        except SystemExit:
            summary = "读不出（不是可用的标定文件）"
        out(f"    {i}) {path}   {stamp}")
        out(f"       {summary}")

    while True:
        try:
            answer = ask(f"  选哪个 [1-{len(found)}]（也可直接输入路径；"
                         "回车/q 退出）: ").strip()
        except (EOFError, KeyboardInterrupt):
            raise SystemExit(
                "没选标定文件就退出了——不会替你挑一份。\n"
                "   下次直接指路径：--calib <路径>"
            )
        if not answer or answer.lower() in ("q", "quit", "exit"):
            raise SystemExit(
                "没有选定标定文件——不会替你挑一份。\n"
                "   下一步：在上位机里对这台夹爪标定并保存，再重跑；"
                "或 --calib <路径>。"
            )
        if answer.isdigit() and 1 <= int(answer) <= len(found):
            path = found[int(answer) - 1]
            read_calibration_file(path)
            return path
        # 不是编号就当路径试试——上位机「另存为」可能把标定存到别处了
        candidate = Path(answer).expanduser()
        if candidate.is_file():
            read_calibration_file(candidate)
            return candidate
        out(f"   看不懂 {answer!r}：既不是 1-{len(found)} 的编号，"
            "也不是一个存在的文件路径。再试一次。")


def check_calibration_matches_args(data: dict, *, channel=None, can_id=None,
                                   mst_id=None) -> list[str]:
    """文件里记的 ID 和这次命令行给的对不对得上；对不上就退出。

    在**连总线之前**调用：选错文件（另一台夹爪）是能在发帧前就发现的错误，没有理由
    带着它往下走。只比对文件里确实记了的键——旧标定文件可能没有
    ``can_id``/``mst_id``。

    为什么值得拦：这批文件是真出过事的——``mst_id`` 被写成十进制 ``18``（应为
    ``0x18``）时，SDK 会把它当 ``0x12`` 用，RX 过滤器就绑到了没人应答的 ID 上，
    主控变成「聋子但一直发」。

    ``channel`` **不**在这里拦：文件里记的是上次上位机连的那条总线，换口是很正常
    的事（``--channel can1``），拦下来只会挡路。不同的话以一行提示返回，让调用方
    打出来——返回值是这些提示，可能为空。

    Returns:
        要提醒的话（``channel`` 对不上之类），没有就是空列表。
    """
    checks = (("can_id", can_id), ("mst_id", mst_id))
    wrong: list[str] = []
    for key, given in checks:
        if key not in data or given is None:
            continue
        try:
            same = int(data[key]) == int(given)
        except (TypeError, ValueError):
            same = False
        if not same:
            wrong.append(f"   {key}: 文件里是 {data[key]!r}，这次命令行给的是 "
                         f"{int(given):#04x}")
    if wrong:
        raise SystemExit(
            "选中的标定文件不是这台夹爪的：\n"
            + "\n".join(wrong) + "\n"
            "   要么文件拿错了（另一台机器的标定），要么 ID 参数变了。\n"
            "   确认是同一台夹爪的话，在上位机里重新标定并保存一份，"
            "或用 --calib 指另一份文件。"
        )
    notes: list[str] = []
    if channel is not None and "channel" in data and \
            str(data["channel"]) != str(channel):
        notes.append(f"   注意：标定文件里记的接口是 {data['channel']}，"
                     f"这次用的是 {channel}——同一台夹爪换口没问题，"
                     "别是另一台。")
    return notes


def load_chosen_calibration(gripper, path, data: dict | None = None) -> dict:
    """把选中的标定文件载入 ``gripper``，并证明**生效的确实是这一份**。

    ``load_calibration`` 会在文件读不出来时静默改用出厂标定、并且照样返回
    ``True``，所以「调用成功」不等于「用上了我选的文件」。这里逐个字段把
    ``gripper.config`` 和文件对一遍：对不上就说明 SDK 用了别的来源，停下比拿
    出厂尺寸驱动真机好。

    Returns:
        文件的内容（省得调用方再读一遍）。
    """
    data = read_calibration_file(path) if data is None else data
    if not gripper.load_calibration(str(path)):
        raise SystemExit(f"SDK 载入标定失败：{path}")
    cfg = gripper.config
    applied = [(key, data[key], getattr(cfg, attr))
               for key, attr in _CALIB_FIELDS if key in data]
    differing = [(key, want, got) for key, want, got in applied
                 if not _same_value(want, got)]
    if differing:
        detail = "\n".join(f"   {key}: 文件里是 {want!r}，实际用的是 {got!r}"
                           for key, want, got in differing)
        raise SystemExit(
            f"SDK 没有用这份文件，而是用了别的标定：{path}\n"
            f"{detail}\n"
            "   （``load_calibration`` 在文件读不出来时会静默回退到打包的出厂"
            "标定，\n"
            "     并且照样返回 True——所以这里要自己核一遍。常见原因：路径写错、"
            "文件里有\n"
            "     它不认识的键、或者它读到的是别的文件。）\n"
            "   停下来是故意的：拿出厂标定的刻度去算真机的目标角，"
            "不是「差一点」，是另一个尺寸。"
        )
    check_calibration(gripper)   # 标定不对的话，下面的目标角就没意义
    return data


def _same_value(want, got) -> bool:
    """标定文件里的值和 config 上的值算不算同一个（数字按数值比，其余按等值）。"""
    try:
        return math.isclose(float(want), float(got), rel_tol=1e-12,
                            abs_tol=1e-12)
    except (TypeError, ValueError):
        return want == got


def open_real_gripper(args: argparse.Namespace, enable: bool = True):
    """连接真机夹爪：选标定 → connect → 载入并核实标定 → enable。

    标定在**连接之前**就定下来（:func:`choose_calibration_file`）：``--calib``
    给的优先，没给就用 SDK 包里那份出厂标定，出厂文件也读不出来才在终端里选。
    ``load_chosen_calibration`` 之后还会逐个字段核实「生效的确实是这一份」——
    SDK 在文件读不出来时会**静默**改用出厂标定并照样返回 True，光看返回值不够。

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

    calib_path = choose_calibration_file(
        args.calib, factory=factory_calibration_path(litegrip))
    calib = read_calibration_file(calib_path)
    # 文件记的是哪台夹爪：对不上就在碰总线之前停
    notes = check_calibration_matches_args(calib, channel=args.channel,
                                           can_id=args.can_id,
                                           mst_id=args.mst_id)
    print(f"[真机] 标定 {calib_path}")
    print(f"       {calibration_summary(calib)}")
    for note in notes:
        print(note)
    print(f"[真机] 连接 {args.channel} · can_id={args.can_id:#04x} · "
          f"mst_id={args.mst_id:#04x}")
    gripper = litegrip.LiteGrip(
        channel=args.channel, can_id=args.can_id, mst_id=args.mst_id
    )
    try:
        if not gripper.connect():
            raise SystemExit(
                f"连不上 {args.channel}。检查：\n"
                f"   1) 接口是否存在且已起来 —— "
                f"sudo ip link set {args.channel} up type can bitrate 1000000\n"
                f"   2) ip -details link show {args.channel}\n"
                f"   3) 夹爪是否已上电、CAN_H/CAN_L 是否接对、"
                f"终端电阻（120Ω）是否装了"
            )
        # 标定必须在 enable 之前载入：SDK 的毫米刻度依赖它
        load_chosen_calibration(gripper, calib_path, calib)
        if not enable:
            # 只说「不发送运动指令」：--status 走这条路（未使能），但读寄存器仍要
            # 发读请求帧，说「一帧都不发」就把话说大了。04 的 --passive 才是真的
            # 一帧不发，它自己会这么说。
            print("[真机] 已连接、已载入并核实标定（未使能，不发送任何运动指令）")
            return gripper
        if not gripper.enable():
            raise SystemExit("使能失败：夹爪可能处于错误状态或未上电")
    except SystemExit:
        gripper.disconnect()
        raise
    except Exception as exc:  # SDK 的各种 *Error
        gripper.disconnect()
        raise SystemExit(f"初始化真机失败：{exc}") from exc

    cfg = gripper.config
    # kp/kd 一起打出来：它们是标定文件里的值（也是保持帧的刚度），改了标定之后
    # 「手感怎么变了」这个问题，第一件要看的就是这两个数。
    #
    # 行程打的是**实测的那两个角之差**，不是 ``max_stroke_mm``：后者是 SDK 的名义
    # 默认值，``load_calibration`` 不写它，所以它和这份标定对不对得上完全看标定是
    # 怎么做的（见 :func:`rad_to_fraction`）。打一个可能对不上的名义值，等于把
    # 操作员往错误的方向引。
    print(f"[真机] 已使能 · 行程 {cfg.pos_closed_rad - cfg.pos_open_rad:.4f} rad"
          f"（{cfg.pos_closed_rad:+.4f} → {cfg.pos_open_rad:+.4f}）"
          f" · rad_to_mm={cfg.rad_to_mm:.2f}"
          f" · kp={cfg.kp:g} kd={cfg.kd:g}")
    return gripper


def fraction_to_target_rad(gripper, fraction: float) -> float:
    """归一化开度 → 真机的电机目标角 [rad]。

    **归一化开度就是标定行程的百分比**：``0`` 是标定出来的闭合位，``1`` 是标定
    出来的张开位，中间线性。

        position_rad = pos_closed_rad − fraction × (pos_closed_rad − pos_open_rad)

    这里**不经过毫米**，也不碰 ``cfg.max_stroke_mm``。这正是它与
    ``goto(position_mm)`` 的唯一区别，理由见 :func:`rad_to_fraction`：
    ``position_mm`` 那把尺子是按**标定时那个** ``max_stroke_mm`` 定的，而
    ``load_calibration()`` 从不写这个字段，它一直是 SDK 的默认值——两者对不上
    的时候，走毫米的换算会在中途饱和。按行程归一没有这个前提。

    前提是标定自洽（``pos_closed_rad`` 比 ``pos_open_rad`` 更**正**，标定后如此）。
    SDK 的出厂默认配置把 ``pos_open_rad`` 写成 ``+1.14``，与 ``goto`` 的符号约定
    相矛盾，此时两个函数的结果都没有意义——所以真机样例在使能前会用
    :func:`check_calibration` 挡掉这种配置，而不是硬发一条越界的角度。
    """
    cfg = gripper.config
    travel = cfg.pos_closed_rad - cfg.pos_open_rad
    fraction = max(0.0, min(1.0, float(fraction)))
    return cfg.pos_closed_rad - fraction * travel


def check_calibration_values(pos_closed_rad: float, pos_open_rad: float,
                             rad_to_mm: float, max_stroke_mm: float) -> None:
    """确认一组标定值自洽，不自洽就带着原因退出。

    ``goto`` 的换算要求 ``pos_closed_rad`` 是更大的那个（闭合 → 角度更正），
    ``pos_open_rad`` 更负。SDK 的出厂默认配置恰好相反，说明这台机器还没跑过
    ``calibrate()``／没载入标定文件——此时任何目标角都是瞎猜的，直接停下比发出去
    让手指撞限位好。

    纯值版本：``--dry-run`` 那条路不连真机、不导入 SDK，只有一份文件里的数，
    也要能查（见 :func:`check_calibration` 是它在 config 上的包装）。
    """
    travel = pos_closed_rad - pos_open_rad
    if travel <= 0.0 or rad_to_mm <= 0.0 or max_stroke_mm <= 0.0:
        raise SystemExit(
            "夹爪的标定值不合法，先做标定再跑：\n"
            f"   pos_closed_rad={pos_closed_rad:+.4f} "
            f"pos_open_rad={pos_open_rad:+.4f} "
            f"rad_to_mm={rad_to_mm:.2f} max_stroke_mm={max_stroke_mm:.1f}\n"
            "   闭合位应当比张开位角度更大（pos_closed_rad > pos_open_rad）。\n"
            "   常见原因：没载入标定文件，还在用 SDK 的出厂默认值。\n"
            "   试：gripper.calibrate() 生成标定，或用 --calib 指定标定文件。"
        )


def check_calibration(gripper) -> None:
    """:func:`check_calibration_values` 在 ``gripper.config`` 上的包装。"""
    cfg = gripper.config
    check_calibration_values(cfg.pos_closed_rad, cfg.pos_open_rad,
                             cfg.rad_to_mm, cfg.max_stroke_mm)


def rad_to_fraction(gripper, position_rad: float) -> float:
    """真机的电机角 [rad] → 归一化开度（:func:`fraction_to_target_rad` 的逆）。

    按**标定行程**归一，不按毫米——理由在下面，值得读完再改成「除以
    ``max_stroke_mm``」的写法。

    为什么不走毫米。SDK 的毫米刻度由 ``rad_to_mm`` 定，而 ``rad_to_mm`` 是
    标定那一刻用**当时那个** ``max_stroke_mm`` 算出来的（``gripper.calibrate()``
    里就是 ``max_stroke_mm / travel``）。可是 ``load_calibration()`` 从不写
    ``config.max_stroke_mm``，它一直是 ``GripperConfig`` 的默认值 120.0。两者只要
    对不上，``position_mm / max_stroke_mm`` 这条映射就会在中途**饱和**：本机
    2026-09-28 那份标定文件的 ``rad_to_mm`` 对应 86 mm 刻度，于是行程走到
    ``86 / 120 = 72%`` 就顶住了——滑条再往上推目标角不再变化，窗口里的仿真手指
    也张不到底，而且拖到 50% 实际给的是全行程的 70%。按行程归一没有这个前提：
    它只用 ``pos_closed_rad`` / ``pos_open_rad``，而这两个角每次标定都实测。

    （标定自洽、即 ``rad_to_mm × travel == max_stroke_mm`` 时，两种写法结果相同；
    差别只在它们对不上的时候。）

    行程非正时返回 ``0.0``：标定本身不自洽，真机路径上 :func:`check_calibration`
    会先把它挡掉，这里只是不做除法。
    """
    cfg = gripper.config
    travel = cfg.pos_closed_rad - cfg.pos_open_rad
    if travel <= 0.0:
        return 0.0
    return max(0.0, min(1.0, (cfg.pos_closed_rad - float(position_rad)) / travel))


#: 读真机状态时最多等一帧状态帧的时间 [s]。SDK 的 ``get_state(wait=True)`` 内部
#: 也是等 50 ms，这里对齐它。
FRESH_WAIT_S = 0.05

#: 只读路径（``--status``）等一帧的时间 [s]。那条路径**什么都不发**（只发寄存器的
#: 读请求，不是运动指令），而 DM 电机只为收到的指令帧回一帧、不会自己持续发帧，所以
#: 等不到是常态而不是故障——等久一点只是给「别的程序刚放过帧」留点余地，不是指望它
#: 一定回话。
STATUS_WAIT_S = 0.5


def fresh_state(gripper, timeout_s: float = FRESH_WAIT_S):
    """等到一帧**新**的状态帧再读快照；等不到返回 ``None``。

    这是本仓库读真机位置的正确入口（03/04/05 都用它）。判据只有一条：

    :meth:`LiteGrip.poll` 为真 ⟹ **这次调用里**解出了一帧本电机的状态帧
    （SDK 自己会把读寄存器的参数应答帧排除掉），于是紧随其后的
    ``get_state(wait=False)`` 读到的就是刚才那一帧。为假就是没有新帧，返回
    ``None``。

    为什么非要问这一句：缓存里可能是 ``MotorState._position`` 的初值 ``0.0``，
    或者一个冻结的旧值。拿它当「现在的位置」去算目标角和斜坡时长，算出来的是一
    条指向别处的**阶跃**指令——电机按标定里的 ``kp`` 去追一个不存在的误差，就是
    「一开夹爪就起飞」的形态。所以拿不到新鲜读数时，调用方应当**拒绝下发**，而
    不是猜一个值。

    **不要再加第二道门。** 上一版这里还查 ``GripperState.has_data`` /
    ``is_stale`` / ``data_age_s``（防「快照没有数据支撑」），这三个属性在现在用的
    那份 SDK（nexform-tech/litegrip-python）里**不存在**，而且 ``poll`` 为真时
    它们要防的两种情况——从没读过、读到的是冻结的旧值——本来就不成立。哪天 SDK
    把「这份快照是不是量出来的」做成公开接口，再加回来。

    **帧只可能来自调用方自己的指令流**：DM 电机只为收到的指令帧回一帧，不会自己
    持续发帧——使能态也一样（实测真机：停发之后连等 4 个 50 ms 窗口，一帧都没有）。
    所以「这一拍先等帧、又不发帧」的调用方式会永远等下去，05 的拖动分支就踩过这个
    坑：那里的正确顺序是**先补发一条已经定下来的保活帧，再等帧**。原来还能发一帧只读
    的 0xCC 把电机叫醒，那条路已经去掉了。

    没有帧流的时候（``--status`` 这种什么都不发的只读路径、电机未使能、适配器掉线），
    这里就会一直返回 ``None``，调用方必须把它当成正常结果处理，不要报错——见
    ``examples/05_dual_control.py`` 的 ``--status``。

    Args:
        timeout_s: 最多等多久 [s]。

    Returns:
        ``GripperState``；``timeout_s`` 内没有新的状态帧则 ``None``。
    """
    if not gripper.poll(timeout_s=timeout_s):
        return None
    return gripper.get_state(wait=False)


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
