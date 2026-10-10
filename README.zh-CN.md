# litegrip-pybullet

**LiteGrip 轻量机械爪系列**的 PyBullet 仿真环境：夹爪的物理模型，说的开度和真机是
同一个量；外加五个可以直接跑的程序，从读模型一路走到用手在真机上手教它一段动作、
再同时放给仿真和夹爪两边。

[English](README.md) · **简体中文**

| 属性 | 值 |
| --- | --- |
| 产品 | LiteGrip 轻量机械爪系列 |
| 仓库定位 | PyBullet 仿真环境 |
| 模型 | 两个平行的移动副手指，各 42.726 mm 行程 |
| 钳口开度 | 闭合 1.548 mm … 张开 87.0 mm |
| 手指额定速度 | 85 mm/s |
| 真机 SDK | 一份 `litegrip-python` 检出，03/04/05 三个样例共用 |
| Python | 3.8+ |

## 目录

- [功能特性](#功能特性)
- [系统架构](#系统架构)
- [安装](#安装)
- [快速开始](#快速开始)
- [核心 API](#核心-api)
- [五个样例](#五个样例)
- [标定](#标定)
- [安全机制](#安全机制)
- [验证状态](#验证状态)
- [故障排查](#故障排查)
- [相关仓库](#相关仓库)
- [文档](#文档)
- [开发](#开发)
- [许可](#许可)
- [联系](#联系)

## 功能特性

### 仿真

- **模型随包发布**：`litegrip-description` 里上游那份 ROS 2 xacro，在加载时归一化成 PyBullet
  能读的形状（改网格路径、去掉没有质量的根连杆），整个打进包里，克隆下来就能用。
- **一个开度，两边通用**：仿真和真机唯一说得通的量是归一化开度（0 闭合 … 1 张到
  最大）。其余写法——关节行程、钳口间隙、SDK 的标定毫米——都由它推出来，换算集中在
  `litegrip_pybullet.model` 一处。
- **限速运动**：单指按额定的 42.73 mm/s 起斜坡，所以一次全行程约 1 s，走到位不超调。
- **力上限与摩擦夹持**：`command_fraction(..., force_n=...)` 限夹持力；`add_box()` 加
  工件、`contacts()` 取接触，测试据此检查「夹得住」和「拽得动就滑」。
- **运动学镜像**：`reset_fraction()` 直接把手指瞬移过去（不跑动力学、没有跟随延迟），
  用仿真显示真机此刻在哪。
- **开窗与无窗口**：`gui=False` 走 PyBullet 的 DIRECT 连接，物理完全一样，CI 和
  `--headless` 用的就是它。

### 真机样例

- **录制与回放（03）**：在零重力下用手教一段动作、存成 `.lgt`，再同时放给真机和窗口；
  单加 `--play` 则只放给窗口——不连接、不使能、一帧都不发。
- **镜像（04）**：真机驱动仿真，每帧读一次；`--zero-gravity` 可以手推着看，`--passive`
  让别的程序驱动它。
- **双向控制（05）**：滑条下发指令，仿真手指镜像真机**实测**的位置；带速度上限和可选
  的前馈夹持力。
- **空闲保活**：三个真机样例在不动的时候都持续发保持帧——使能态的电机静默就会锁通信
  丢失故障，见[安全机制](#安全机制)。
- **目标走斜坡**：位置目标每帧只走一个 tick，到位前先减速，所以一帧要多大扭矩跟标定里
  的 `kp` 无关、也跟滑条被拽得多快无关。
- **诊断不动电机**：`05 --status` 只读错误码和 DM 寄存器，不使能、不发运动指令。

### 标定与启动检查

- **默认出厂标定**：不给 `--calib` 时用 SDK 包里那份出厂标定，路径跟着包目录解析，换
  台电脑、换个虚拟环境还是同一条命令。
- **显式覆盖**：`--calib <路径>` 用这台夹爪自己那份。
- **先核实再用**：文件会被自己读一遍、逐字段确认确实生效——SDK 自带的加载器在路径读不
  出来时会**静默**改用打包的出厂值。
- **不扫盘**：不给 `--calib`、出厂文件也读不出来时就直接停下，让你显式给 `--calib`。
  样例不去猜机器上哪份 JSON 是这台夹爪的；需要找路径时用 `--list-calibrations` 列候选。
- **SDK 接口自检**：每个真机样例启动时先核对 SDK 有没有它要调的成员，缺了就直接停下并
  列出缺哪些，而不是在控制循环里才炸。

## 系统架构

```text
┌──────────────────────────────────────────────────────────────┐
│  你的程序 · examples/01–05 · examples/_common.py               │
└──────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌──────────────────────────────────────────────────────────────┐
│  litegrip_pybullet.sim — GripperSim                           │
│  ├── command_fraction / command_joint / reset_fraction        │
│  ├── settle / run_for / step                                  │
│  ├── fraction / aperture_mm / finger_force_n                  │
│  ├── add_box / contacts / grasp_center / link_aabb            │
│  └── keyboard_events / mouse_events / status_text             │
└──────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌──────────────────────────────────────────────────────────────┐
│  litegrip_pybullet.model — 单位换算的边界                      │
│  归一化开度 ↔ 关节行程 ↔ 钳口间隙 ↔ SDK 毫米                    │
└──────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌──────────────────────────────────────────────────────────────┐
│  litegrip_pybullet.urdf — 查找与归一化                         │
│  随包一份 / $LITEGRIP_URDF_PATH / $LITEGRIP_URDF_DIR          │
└──────────────────────────────────────────────────────────────┘
                              │
                              ▼
                 PyBullet（GUI 或 DIRECT）→ URDF + STL
```

三个真机样例在 `GripperSim` 之上还多一层：它们自己组 CAN 循环，用的是 SDK 公开的帧级
接口（`send_mit_frame()` + `poll()`）。

### 核心模块

| 模块 | 职责 |
| --- | --- |
| `src/litegrip_pybullet/sim.py` | `GripperSim`、`pressed()`、`clicked()`、事件与按键常量 |
| `src/litegrip_pybullet/model.py` | `STROKE_M`、`N_FINGERS`、几个开度常量和各种写法之间的换算 |
| `src/litegrip_pybullet/urdf.py` | URDF 查找（`resolve_urdf()`、`bundle_dir()`）与面向 PyBullet 的归一化（`normalize_urdf()`） |
| `src/litegrip_pybullet/assets/litegrip_urdf/` | 随包发布的 xacro 与 STL 网格 |
| [`examples/_common.py`](examples/_common.py) | 参数解析、SDK 查找、标定选择、连接/使能、单位换算、状态行 |
| [`examples/03_trajectory.py`](examples/03_trajectory.py) | 真机录制、两边一起回放 |
| [`examples/04_mirror_real.py`](examples/04_mirror_real.py) | 真机 → 仿真 |
| [`examples/05_dual_control.py`](examples/05_dual_control.py) | 仿真 → 真机 |

## 安装

### 方式一：从检出装（开发用）

```bash
git clone https://github.com/nexform-tech/litegrip-pybullet.git
cd litegrip-pybullet
python3 -m venv .venv && source .venv/bin/activate
python3 -m pip install -e ".[test]"
python3 -m pytest -q
```

### 方式二：普通安装

```bash
python3 -m pip install .
```

### 方式三：构建 wheel

```bash
python3 -m pip install build
python3 -m build --wheel
# 产物在 dist/
```

### 依赖说明

- **`pybullet>=3.2`** —— 仿真器。
- **`numpy>=1.20`** —— 显式要求，因为 pybullet 的 wheel 自己不声明依赖，但没有 numpy
  时 import 就会失败。
- **`pytest>=7.0`**（`test` extra）—— 测试。

模型不需要别的：URDF、xacro 和 STL 网格都在包里。

### 真机 SDK（03、04、05 三个样例）

`litegrip` **没有发布到 PyPI**——`pip install litegrip` 装到的不是它，已发布的版本里也
没有带这套接口的。三个真机样例共用**一份**检出，
[`nexform-tech/litegrip-python`](https://github.com/nexform-tech/litegrip-python)，
按下面的顺序找：

1. `$LITEGRIP_SDK_DIR` —— 指到**包含** `litegrip` 包的那层目录（src 布局就是
   `<仓库>/src`）。
2. 本仓库同级的检出：`../litegrip-python`，先试 `src/`，再试仓库根。
3. 当前解释器里装着的 `litegrip`。

```bash
python3 -m pip install -e /path/to/litegrip-python
# 或者
export LITEGRIP_SDK_DIR=/path/to/litegrip-python/src
```

**别**把 `LITEGRIP_SDK_DIR` 指到一个没有 `litegrip` 包的目录：样例会带着说明直接停下，
而不是悄悄从别处导入一个同名的包。另一个仓库也发过一个叫 `litegrip` 的包，`__version__`
同样是 2.2.0，但它没有轨迹接口——所以版本号分不出来，能分出来的是启动时的接口自检，它
会把缺的接口逐个列出来。

## 快速开始

### 只跑仿真

```bash
python3 examples/01_hello_sim.py --headless
```

```python
from litegrip_pybullet import GripperSim

sim = GripperSim()                  # 默认不开窗口；要窗口传 gui=True
sim.command_fraction(0.0)           # 0 = 闭合，1 = 张到最大
sim.settle()                        # 约 1 s，受额定 85 mm/s 限速
print(sim.aperture_mm(), sim.fraction(), sim.finger_force_n())
sim.disconnect()
```

### 配 CAN

每个真机样例都要一条 1 Mbit/s 的 SocketCAN 接口（默认 `can0`）。

**这一步不用你手动做**：样例 03、04、05 在连接前会用 `ip -details link show` 读一次
接口，只有状态**真的不对**时才去跑那几条需要特权的 `ip` 命令。接口本来就是对的，就
一条命令都不跑、也不问密码。要自己管接口，加 `--no-can-setup`。

手动那份配方，也是自动那一步真正跑的东西：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100 fd off
sudo ip link set can0 up
ip -details link show can0
```

`restart-ms` 不能省。内核默认是 `restart-ms 0`，意思是控制器进了 bus-off 之后**不会
自己恢复**：一帧坏帧就能让接口处于「up 着、比特率也对、却什么都发不出去」的状态。有些
USB 适配器不认这个选项，样例只会去掉它重试那一条命令，别的一概不动。

### 上真机

```bash
python3 examples/04_mirror_real.py                     # 只读着镜像
python3 examples/05_dual_control.py --dry-run           # 整套流程，不发 CAN
python3 examples/05_dual_control.py --calib ~/.litegrip/litegrip_calibration.json
python3 examples/03_trajectory.py                       # 先录、再放
```

> **样例 03、04、05 会驱动真机。** 先读[安全机制](#安全机制)和
> [examples/README.zh-CN.md](examples/README.zh-CN.md#上真机之前)。

## 核心 API

### GripperSim

| 方法 | 说明 |
| --- | --- |
| `GripperSim(urdf_path=None, *, gui=False, gravity=(0,0,-9.81), max_force_n=..., velocity_m_s=...)` | 连接 PyBullet 并加载夹爪；`gui=True` 开窗口，`gui=False` 走 DIRECT |
| `command_fraction(fraction, *, force_n=None, velocity_m_s=None)` | 按归一化开度下命令，钳到 `[0, 1]` |
| `command_joint(joint, *, force_n=None, velocity_m_s=None)` | 按单指行程（米）下命令，钳到 URDF 限位 |
| `reset_fraction(fraction)` | 运动学瞬移——镜像走的就是它 |
| `step(n=1)` | 推进一步；窗口被关掉时返回 `False` |
| `run_for(seconds)` | 按墙钟时间推进 |
| `settle(*, tolerance_m=..., timeout_s=3.0)` | 一直走到目标，返回 `(秒数, 到了没)` |
| `fraction()` / `aperture_mm()` / `joint_values()` / `finger_force_n()` | 当前状态 |
| `add_box(half_extents, position, ...)` | 放一个可夹的工件 |
| `contacts(other_body)` | 工件与夹爪之间的接触 |
| `status_text(text)` | 往窗口里写一行字 |
| `keyboard_events()` / `mouse_events()` | 取输入；`pressed()`、`clicked()` 负责解读 |
| `disconnect()` | 关窗口、断开 PyBullet 连接 |

### 单位换算

仿真和真机在归一化开度上碰面，所以换算只在一个模块里，不散落在各样例里。

| 函数 | 说明 |
| --- | --- |
| `fraction_to_joint(fraction)` / `joint_to_fraction(joint)` | 归一化开度 ↔ 单指行程（米） |
| `fraction_to_aperture_mm(fraction)` / `joint_to_aperture_mm(joint)` | 钳口间隙（mm），卡尺量出来的那个距离 |
| `sdk_mm_to_fraction(mm, max_stroke_mm=120.0)` / `fraction_to_sdk_mm(...)` | SDK 的**标定**毫米 |
| `clamp_fraction(fraction)` | 钳到 `[0, 1]` |

| 常量 | 值 | 含义 |
| --- | --- | --- |
| `STROKE_M` | `0.042726` m | 单指满行程 |
| `N_FINGERS` | `2` | 平行手指数量 |
| `APERTURE_CLOSED_MM` / `APERTURE_OPEN_MM` | `1.548` / `87.0` mm | 两端钳口间隙 |
| `MAX_GRIP_FORCE_N` | `40.0` N | 本库的力上限 |
| `DEFAULT_MAX_STROKE_MM` | `120.0` mm | SDK 默认的标定刻度 |

**SDK 毫米和钳口间隙是两个不同的量，不是差一个单位换算。** 表里的 `120` 是 SDK 的
`max_stroke_mm` **默认值**；标定文件里存的是那次标定自己的刻度，而 `rad_to_mm` 是标定
当时按行程算出来的、`load_calibration()` 从不把行程写回这个字段。所以样例在弧度和归一化
开度之间直接换算。见[「一个开度，三种写法」](examples/README.zh-CN.md#一个开度三种写法)。

### URDF 查找

`resolve_urdf(path)` 依次试：显式路径 → `$LITEGRIP_URDF_PATH` → `$LITEGRIP_URDF_DIR` →
随包一份 → 同级的 `litegrip-description` 检出。也就是说换模型是改环境变量，不是改代码。找不到
会抛 `UrdfError`。

## 五个样例

| 文件 | 方向 | 需要真机？ |
| --- | --- | --- |
| [`examples/01_hello_sim.py`](examples/01_hello_sim.py) | 只读状态，不动 | **不需要** |
| [`examples/02_move_sim.py`](examples/02_move_sim.py) | 只跑仿真 | **不需要** |
| [`examples/03_trajectory.py`](examples/03_trajectory.py) | 真机录、真机+仿真一起放 | 录制和真机回放都需要 |
| [`examples/04_mirror_real.py`](examples/04_mirror_real.py) | 真机 → 仿真 | 要真的镜像就需要 |
| [`examples/05_dual_control.py`](examples/05_dual_control.py) | 仿真 → 真机 | 要真的动就需要 |

01 一个动作都不做：`tests/test_examples_cli.py` 会解析它，不许它出现任何运动调用。03 的
`--play` 是唯一不需要真机就能跑的真机样例路径，也是真机样例里 CI 唯一跑得了的（而且要
有检出才写得出那段 `.lgt`）。

### 建议学习路径

```text
01_hello_sim.py        读模型，一个动作都不做        （不要真机）
  ↓
02_move_sim.py         限速运动                      （不要真机）
  ↓
03_trajectory.py --play <文件>   离线回放            （不要真机）
  ↓
04_mirror_real.py      真机驱动窗口                  （真机，基本只读）
  ↓
05_dual_control.py --dry-run     走一遍流程，不发 CAN
  ↓
05_dual_control.py     窗口驱动真机                  （真机，会动）
  ↓
03_trajectory.py       手教一遍，两边一起放          （真机，会动）
```

每个样例演示什么、以及它们共用的那套记号，见
[examples/README.zh-CN.md](examples/README.zh-CN.md)。

## 标定

每条碰真机的路径都要先定下用哪份标定，因为角度和毫米刻度是**每台夹爪各自量的**。

### 默认用的那份出厂标定

不给 `--calib` 时，样例用 SDK 包里那份出厂标定——`factory_calibration.json`，就在
`litegrip` 包的 `__init__.py` 旁边。路径是从包目录推出来的，没有写死过任何本机路径，
所以换台电脑、换个虚拟环境还是同一条命令。

**那份文件里是台架夹具的实测参数，不是你这台夹爪的参数。** 它是一份能用的默认值，而它
记录的行程端点未必和你面前这台对得上。除了先看一眼效果，请加 `--calib`。

### 覆盖它

```bash
python3 examples/05_dual_control.py --calib ~/.litegrip/litegrip_calibration.json
```

标定文件是用上位机（`litegrip-studio` / `litegrip-console`，或 SDK 自带的
`tools/gui/litegrip_gui.py`）对着**这台**夹爪标定后保存出来的。样例自己读这份文件、逐
字段核实它确实生效，会拒绝 `can_id`/`mst_id` 指向另一台电机的文件，角度互相矛盾的文件
也会被拦下来并给出说明。这件事不交给 SDK 的 `load_calibration()`：它在路径读不出来时会
**静默**改用打包的出厂值，而且照样返回 `True`——路径打错一个字母，就会拿另一台机器的
角度去驱动电机。

### 没有选择器了

换标定只有一个办法：显式给出文件名 `--calib <路径>`。样例不扫盘、不提问。`--calib` 和
可读的出厂文件都没有时，程序直接停下并告诉你传 `--calib`——SDK 装得不完整，不是拿一份
没人选过的参数去驱动电机的理由。

`--list-calibrations` 把 `~/.litegrip` 下的候选连同关键值（闭合/张开角度、`rad_to_mm`、
`kp`、`mst_id`）打出来，然后退出 0。它是一次**查询**：不连真机、不发帧、也不改变默认
标定。上位机给仿真后端单独存的 `*.sim.json` 和 `*.bak` 备份**不会**出现在列表里，显式
指定 `*.sim.json` 也会被拒——那份刻度是仿真里的。

## 安全机制

### 速度与力上限

| 限制 | 值 | 在哪一层施加 |
| --- | --- | --- |
| 手指速度 | 单指 42.73 mm/s，钳口 85 mm/s | `GripperSim` 的 `velocity_m_s`，默认就是 42.73 mm/s |
| 夹持力 | 默认 10 N，额定上限 40 N | `DEFAULT_MAX_FORCE_N`、`MAX_GRIP_FORCE_N`；可用 `force_n` 逐条覆盖 |
| 位置目标跳变 | 每帧一个斜坡 tick | 样例 03、04、05 |

### 保持帧与 0xD 通信丢失故障

**使能态**的电机连续约 **0.9 s** 收不到任何帧，就锁进通信丢失故障（`0xD`，SDK 的
`describe_error` 现在自己叫它 `通讯丢失 (CAN 超时)`）。症状就是大家报的那句「位置读得
到，指令不执行，红灯闪」：位置照常回报，指令一律被忽略。**一段安静的空闲期本身就是故障
原因。**

这 0.9 s 是本机硬件上量出来的。**别**拿 `TIMEOUT` 寄存器（RID 9）推它：那个寄存器一回
读到 8000 ms、一回读到 0（当前不生效），SDK 自己把这条标成「待查」。`--status` 会把
寄存器的原值打出来存档，并在旁边说明这一点。

所以三个真机样例在所有「没有别人喂帧」的空档里都按 200 Hz 持续发保持帧——目标 = 实测
位置、零前馈、不命令任何运动。03 发在它录制与回放两个相位之间；04 的 `--passive` 则是
在一帧不发、也不使能的前提下让别的程序驱动总线。

录制或回放进行中时，是 SDK 自己的后台线程在流总线。**别**在这时候往同一条线上插帧：
第二路流会把轨迹打散。

### 阶跃指令与 0x9/0xA 故障

MIT 的位置项是 `kp × (q目标 − q实际)`，而 `kp` 是标定文件里的一项、不是常数。一帧就把
目标发到底，等于一次性向电机要 `kp × 1.845 rad`——在 SDK 默认的 `kp = 100` 下约
185 Nm，而这颗电机额定才 10 Nm 左右。电流瞬间拉满，电机锁进欠压/过流保护（0x9/0xA），
之后位置照常回报、指令一律不执行。

样例 05 限的是**位置目标**：每帧只比上一帧走一个 tick。这样一帧要多大扭矩与 `kp` 配成
多少无关——本机 `kp = 5.0` 下同样的阶跃约 9 Nm。它还会在到位前先减速，因为把目标速度
从满速一刀切到 `0` 的那一帧，阻尼项会反号成一个 `kd × v` 的力矩阶跃。

这两种故障都是**锁死**的：断电重启之前不会自己消失。可以在不动电机的前提下读到并清掉：

```bash
python3 examples/05_dual_control.py --status                  # 只读，不发运动指令
python3 examples/05_dual_control.py --status --clear-fault    # disable → clear → enable
```

`--status` 不开窗口、不使能、也不发任何指令帧。它还会**读**一次 CAN 接口，但**不修**：
读到什么就打印什么，接口原样留在那儿——替它把接口改掉，恰好把 `--status` 存在的意义
抹掉了。04 的 `--passive` 同样如此。

状态帧是**回**出来的——DM 电机收到一条指令帧才回一帧，使能与否都一样——一条不发就
没有可回的东西，所以它**根本读不到位置**。
这时它报的是「判不了」、退出码 0，而不是故障；DM 寄存器照读。只有 `--clear-fault`
会发帧，而它的 disable → clear → enable 中间那一瞬间手指是失力的。

## 验证状态

哪些验证过、哪些没有：

| 能力 | 状态 | 依据 |
| --- | --- | --- |
| URDF/xacro 归一化 | ✅ 已验证 | `tests/test_urdf.py`：上游 ROS 2 xacro 能在 PyBullet 里加载，2 个移动副关节，网格齐全 |
| 模型几何（行程 ↔ 开度） | ✅ 已验证 | `tests/test_model.py`，其中一条用例检查打包 xacro 的 `stroke` 默认值与 `STROKE_M` 一致 |
| 限速运动 | ✅ 已验证 | `tests/test_sim.py`：全行程 1.00 s 斜坡 + 约 0.13 s 伺服稳定，无超调 |
| 力上限与摩擦夹持 | ✅ 已验证 | `tests/test_sim.py`：10 N 夹持力下扛得住 5 N，15 N 会滑；且只有手指碰到工件 |
| 样例 01–02（纯仿真） | ✅ 已验证 | 两个都无窗口跑通、exit 0，输出内容都在 `tests/test_examples_cli.py` 里断言；01 还会被解析一遍，确认它没有任何运动调用 |
| 样例 03 的 `--play`（离线回放） | ✅ 已验证 | 读一段 `.lgt` 推进窗口、一帧都不发：无窗口跑通、exit 0，端到端断言在 `tests/test_example03_loop.py`，另一条用真实文件跑的在 `tests/test_examples_cli.py` |
| 单份 SDK、默认出厂标定 | ✅ 已验证（不碰真机） | `tests/test_common.py` 覆盖查找顺序（`$LITEGRIP_SDK_DIR` → 同级 → 已安装）、`LITEGRIP_SDK_DIR` 指错目录时会明确报错、`factory_calibration_path()` 从包目录解析，以及标定的两档选择；`tests/test_examples_cli.py` 端到端覆盖这些路径 |
| 样例 03 的录制与真机回放 | ⚠️ **未验证** | 两条都没在真机上跑过：它们都会让真机真的动。采样率、位姿换算、保持帧和「录制结束要确认才回放」的闸门由 `tests/test_example03_loop.py` 用桩 SDK 覆盖 |
| 样例 03 回放前的「点一下窗口」 | ⚠️ **部分验证** | 键盘那一路有单测；鼠标那一路只在真窗口里确认过 `getMouseEvents()` 调得通（0 个事件），**没人真的点过**——`clicked()` 判断的 5 元组布局抄自 pybullet 自带的 `pybullet_examples/createVisualShapeArray.py` |
| 样例 04（真机 → 仿真） | ⚠️ **部分验证** | 只读镜像路径在 `can0` 的真实夹爪上跑过，但用的是**另一份** SDK 检出——本仓库现在不再用的那份。`--zero-gravity` 和 `--passive` 完全没跑过，这些路径也没有对着 `litegrip-python` 重跑过 |
| 样例 05（仿真 → 真机） | ⚠️ **部分验证** | 在 `can0` 的真实夹爪上跑过，触发了电机故障；斜坡修复有 `tests/test_example05_stream.py` 覆盖，但**修复本身还没上过真机**。改成拖动驱动之后（无下发键、速度滑条）、按行程归一的滑条映射、到位前的减速段，都只在测试里跑过 |
| 样例 05 的 `--status` 诊断 | ⚠️ **部分验证** | 2026-09-28 在 `can0` 的真实夹爪上，它读到一帧新状态、位置、错误码和 DM 寄存器，全程未使能、未动电机（exit 0）——同样是另一份 SDK 检出。**锁死故障**的报出、清除，以及新的「未使能读不到位置也退出 0」行为，都还没在真机上试过 |
| 真机运动指令 | ⚠️ **部分验证** | 早期版本的样例 05 向真机发过阶跃指令流，把电机打进了故障态；换成斜坡之后还没跑过 |
| 空闲保活（`IdleKeeper` / 04 的锁位帧） | ⚠️ **部分验证** | 有单测覆盖发帧节奏与间隔上限；**没上过真机** |
| CAN 探测（`ensure_can_link` / `--no-can-setup`） | ⚠️ **部分验证** | 「读」这一半验证过：解析器钉的是 `ip -details link show` 的真输出（含一份每个标志位都正常、其实是 bus-off 的样本），并且 `probe_can_link('can0')` 在本机只读地跑过、读得对。「拉起」那一半——那串把不对的接口配好的 `sudo ip` 命令——只对着替身 `run` 跑过，**没人看着它修好过一个真接口** |

仿真动力学来自 PyBullet，惯量用的是 URDF 自带的。手指限速和力上限由本库施加；报告出来
的伺服稳定时间是仿真里位置伺服的实测特性，不是真机测量值。

## 故障排查

### CAN 起不来

要带 `-details` 读。只看标志位不够——下面两种成因，不看 `can state` 分不出来：

```bash
ip -details link show can0
```

- **接口没起来。** 标志位是 `<NOARP>`，没有 `UP`。样例 03/04/05 连接前会自己修好；
  手动修就是[「配 CAN」](#配-can)那份配方。
- **控制器 bus-off。** 标志位是 `<NOARP,UP,LOWER_UP>`、比特率也对，**每个标志位看着
  都正常**，但一帧都发不出去。只有 `can state BUS-OFF` 看得出来。`restart-ms 0` 时它
  会一直这样，直到有东西重新配置接口。

`enable()` 报 `[Errno 100] Network is down`，那是**主机侧的链路问题，不是夹爪**。CAN
的 socket 在 down 的接口上照样 bind 得上，所以 `connect()` 会成功，直到发出第一帧才
暴露——这就是它以前被报成「夹爪可能未上电」的原因。现在样例会直接点明是链路；动接线
之前先看 `can state`。

样例默认用 `can0`、CAN ID `0x08`，`--channel` 和 `--can-id` 可以改。USB-CAN 适配器在
运行中途掉线时，`enable()` 会报 `ENOBUFS`——那是适配器的问题，不是夹爪。

### 「位置读得到，指令不执行」，红灯闪

两种长得一样的原因，都在[安全机制](#安全机制)里讲了：阶跃指令（0x9/0xA），或者静默
超过看门狗（0xD）。不动电机就能诊断：

```bash
python3 examples/05_dual_control.py --status
python3 examples/05_dual_control.py --status --clear-fault
```

### 总线上有第二个主设备

同一条接口上两路流会互相打架，谁也控制不了。跑真机样例之前先确认没有别的程序占着它：

```bash
pgrep -af python3 | grep -i litegrip        # 谁在占 CAN
ip -details -statistics link show can0      # 你什么都没跑，计数器还在涨？
```

`ip link set can0 down` / `up` **赶不走**那个程序——它的 socket 还在，接口一回来它接着
发。得让那个进程退出。

### 样例找不到 SDK

```text
找不到真机 SDK（litegrip 包）：LITEGRIP_SDK_DIR 指到 /path,但那里没有 litegrip/__init__.py。
```

这个变量要指到**包含** `litegrip` 包的那层目录，不是包目录本身、也不是仓库根。要么
unset 它，要么指到 `<仓库>/src`。`$LITEGRIP_TRAJ_SDK_DIR` 已经没有了：三个真机样例现在
共用同一份检出，还在设它的脚本请改设 `LITEGRIP_SDK_DIR`。

## 相关仓库

| 仓库 | 定位 |
| --- | --- |
| [litegrip-python](https://github.com/nexform-tech/litegrip-python) | Python SDK |
| [litegrip-cpp](https://github.com/nexform-tech/litegrip-cpp) | C++ SDK |
| [litegrip-ros2](https://github.com/nexform-tech/litegrip-ros2) | ROS 2 驱动 |
| [litegrip-description](https://github.com/nexform-tech/litegrip-description) | URDF/xacro 描述包 |
| [litegrip-docs](https://github.com/nexform-tech/litegrip-docs) | 产品文档 |

## 文档

- [样例说明](examples/README.zh-CN.md) —— 五个样例各自演示什么、共用的那套记号，以及
  上真机之前要过的清单。

## 开发

```bash
python3 -m pip install -e ".[test]"
python3 -m pytest -q
```

测试既不需要 CAN 也不需要显示器：真机部分整段跳过，PyBullet 无窗口跑。「没有 SDK」也是
测试的一部分——哪里都没有检出时，`tests/test_examples_cli.py` 仍然会断言每个样例带着
可读的说明停下，而不是抛一串 traceback。CI 跑的就是这个状态：那边根本没装 `litegrip`。

## 仓库规范

本仓库遵循 NEXFORM ROBOTICS 的共享仓库规范：代理操作规则见 [AGENTS.md](AGENTS.md)，
提交信息用 Conventional Commits，合并到 `main` 后由 semantic-release 自动发版。

## 许可

Copyright © 2026 NEXFORM ROBOTICS. 以
[Apache License 2.0](LICENSE) 授权。

## 联系

- 仓库：[github.com/nexform-tech/litegrip-pybullet](https://github.com/nexform-tech/litegrip-pybullet)
- 问题反馈：[github.com/nexform-tech/litegrip-pybullet/issues](https://github.com/nexform-tech/litegrip-pybullet/issues)
