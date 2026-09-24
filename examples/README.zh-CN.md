# LiteGrip PyBullet 样例

三个可以直接跑的程序，从三个方向看同一个夹爪模型：只跑仿真、仿真驱动真机、
真机驱动仿真。

[English](README.md) · **简体中文**

| 文件 | 方向 | 需要真机？ |
| --- | --- | --- |
| [`01_sim_only.py`](01_sim_only.py) | —（纯 PyBullet） | **不需要** |
| [`02_sim_to_real.py`](02_sim_to_real.py) | 仿真 → 真机 | 要真的动就需要 |
| [`03_real_to_sim.py`](03_real_to_sim.py) | 真机 → 仿真 | 要真的镜像就需要 |

## 运行前准备

```bash
python3 -m pip install pybullet          # 01–03 都要
python3 -m pip install litegrip          # 02–03（要接真机）
```

真机 SDK 也可以从同级的 `lite-grip` 仓库或 `$LITEGRIP_SDK_DIR` 找到，所以直接
用源码检出也能跑：

```bash
export LITEGRIP_SDK_DIR=/path/to/lite-grip
```

缺 pybullet 时，样例会自动改用 `./.venv/bin/python` 重跑（存在的话）。除此之外
不需要别的：URDF 和网格已经打包在 `src/litegrip_pybullet/assets/litegrip_urdf/`
里，克隆下来就能跑。

02/03 之前先把 CAN 起起来：

```bash
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0
```

> **样例 02 和 03 会驱动真机。** 先读[「上真机之前」](#上真机之前)。

## 一个开度，三种写法

夹爪的开度有三种说法，混起来用是困惑的主要来源：

| 写法 | 范围 | 含义 |
| --- | --- | --- |
| 关节值 | `0 … 0.042726` m | 单个手指的移动行程；`0` 是张到最大 |
| **归一化开度** | `0 … 1` | **唯一两边通用的量**；`0` 闭合，`1` 张到最大 |
| SDK 毫米 | `0 … 120` mm | 真机报的**标定刻度**，不是钳口间隙 |
| 钳口间隙 | `1.548 … 87` mm | 两个指面的物理距离 |

SDK 毫米是夹爪自己的 `goto(mm)` 用的刻度，钳口间隙是拿卡尺量出来的距离。这两个
数不是同一个数，也不是差一个单位换算——只有归一化开度在两边指的是同一件事，所以
所有样例都从它换算。

## 样例说明

### 01 —— 仅仿真

不接真机、不碰 CAN、不用 SDK。四段走完模型的全貌：

1. **速度受限的行程** —— 一次全行程约 1 s，因为斜坡被限制在 85 mm/s（真机的额定
   速度）。两个手指是对冲的，所以**开口**变化的速度是这个的两倍。
2. **中间位定位** —— 按归一化开度下命令，`settle()` 等它真的走到。
3. **夹持** —— 两指中间放个方块，按力上限收爪，读回接触点和电机推力。这里
   `settle()` 返回 `False` 才是对的，不是失败：手指被工件顶住了。
4. **下拽** —— 给工件加向下的力，找摩擦力的极限。5 N 夹得住，15 N 就滑了。

```bash
python3 examples/01_sim_only.py                 # 开窗口，跑整套演示
python3 examples/01_sim_only.py --headless      # 无窗口，exit 0
python3 examples/01_sim_only.py --object-mm 60 --force 20
python3 examples/01_sim_only.py --slip 40       # 一定会滑的拉力
```

第 3 步的夹持是真的摩擦，不是「搁在台沿上」：工件悬在两指之间，而且**只碰到两根
手指**。怎么做到的见[「说明」](#说明)。

### 02 —— 仿真控制真机

在 PyBullet 窗口里设目标，按 Enter 才下发。

1. 拖**开度**滑条。仿真手指实时跟着走——这是**预览**，它同样受 85 mm/s 限制，
   所以你看到多快，真机就多快。此时真机还没动。
2. 拖**夹持力**滑条。下发时它作为前馈力矩跟着帧一起发出去。
3. 按 **Enter** 或 **空格** 才真的下发：真机以 200 Hz 朝目标流式走 `--duration`
   秒。仿真显示的是**命令值**，旁边对照的是真机的实测值；两者的差就是跟随误差，
   被工件顶住时这个差会一直留着——正好用来判断「夹到了没有」。
4. 按 **Esc** 或 **Q** 退出。停止发帧，电机保持当前位置。

```bash
python3 examples/02_sim_to_real.py --dry-run         # 只开窗口，绝不碰 CAN
python3 examples/02_sim_to_real.py                   # can0，10 N，1 s
python3 examples/02_sim_to_real.py --force 20 --duration 2
python3 examples/02_sim_to_real.py --channel can1    # 换一个 CAN 口
```

`--headless` 会被拒绝：滑条就是输入设备，而 DIRECT 连接里没有滑条。无窗口请用 01。

这个样例自己组 CAN 循环，用的是 SDK 公开的 `send_mit_frame()` + `poll()`，而**不是**
`move_to()`/`goto_rad()`。后两者内部跑 `control_mit_stream()`，它自己 sleep 在一个
5 ms 循环里不让出控制权——PyBullet 窗口会卡住，按键也读不到。公开的帧级 API 就是
为这种场合准备的。

### 03 —— 真机控制仿真

真机是「主」，仿真只是显示器。每帧读一次真机位置，用 `reset_fraction()` 把仿真
手指瞬移过去——纯运动学、不跑动力学——所以窗口显示的就是真机此刻的样子，没有跟随
延迟，也不会自己漂。

两种用法：

- **用手推着看**（`--zero-gravity`，推荐）：电机失力，可以用手推动手指，窗口跟着
  你的手走。运行中按 **Z** 可以随时切换失力/使能。
- **看别人的程序驱动**：不加 `--zero-gravity` 时真机自己保持位置；如果有别的程序
  在给它发指令，窗口同样会跟着显示。

```bash
python3 examples/03_real_to_sim.py --zero-gravity   # 用手推
python3 examples/03_real_to_sim.py                  # 只镜像，不下发
python3 examples/03_real_to_sim.py --headless       # 只看终端读数
python3 examples/03_real_to_sim.py --duration 10    # 10 s 后自动退出
```

默认路径除了保持电机使能之外是只读的：它不发任何位置指令。改变真机行为的是
`--zero-gravity`（或按 Z）——它让夹爪变**软**，手指可以被推动，也会因为重力自己滑。
使能之前先托住夹爪。

退出时会恢复使能（`exit_zero_gravity()`），让夹爪自己保持住位置，而不是松掉。

## 上真机之前

两个真机样例都会先打印安全横幅，使用姿势是：夹爪拿在手上或固定在台面上，
**行程内不放任何东西**，电源开关触手可及。02 的第一次跑应当是 `--dry-run`。

动之前先确认 CAN 接口和标定：

```bash
ip -details link show can0
```

样例自己会检查标定，不一致就拒绝启动——SDK 的出厂默认值里的张开角度与它自己
`goto()` 的符号约定相矛盾，所以未标定的机器会被拦下来并给出说明，而不是拿一个
没有意义的角度去驱动它。

## 说明

**为什么合爪的时候要「扶住」工件。** 合爪大约要 1 s，而一个 40 mm 的工件放在抓取
中心后，这 1 s 里会自由落体约 24 mm 掉到夹爪底座上。那之后它就是**坐在底座上**，
不是被手指夹住的，对它做摩擦测试什么也测不出来。所以样例 01 在合爪期间把工件的
质量设为 0（PyBullet 会把它当静态体），合上之后再恢复——此后就只有手指碰得到它。
这是仿真里的技巧，不是物理现象；`tests/test_sim.py::close_in` 用的是同一招。

**抓取中心在哪。** `[0.0, 0.0, 0.0665]` m——在两指中间，离底座顶面 22.5 mm。
`getAABB` 会把每个 link 放大约 3 mm，所以不要用它读开口，用 `aperture_mm()`。

**共用的部分。** [`_common.py`](_common.py) 放着参数解析、SDK 查找、连接/使能流程、
单位换算和状态行。它不是第四个样例——另外三个都 import 它，所以每个样例都是先
`from _common import ...` 再 import `litegrip_pybullet`。
