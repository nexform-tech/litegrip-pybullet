# litegrip-pybullet

**LiteGrip 轻量机械爪系列**的 PyBullet 仿真环境——夹爪的物理模型，加上三个样例，
从两个方向演示怎么驱动它。

[English](README.md) · **简体中文**

| 属性 | 值 |
| --- | --- |
| 产品 | LiteGrip 轻量机械爪系列 |
| 仓库定位 | PyBullet 仿真环境 |
| 模型 | 两个平行的移动副手指，各 42.726 mm 行程 |
| 钳口开度 | 闭合 1.548 mm … 张开 87.0 mm |
| 手指额定速度 | 85 mm/s |
| Python | 3.8+ |

## 安装

```bash
python3 -m pip install pybullet      # 仿真
python3 -m pip install -e .          # 本包
python3 -m pip install litegrip      # 可选：要驱动真机时
```

夹爪模型（URDF + STL 网格）已经打包在仓库里，加载它不需要别的东西。

## 快速开始

```bash
python3 examples/01_sim_only.py --headless
```

```python
from litegrip_pybullet import GripperSim

sim = GripperSim()                  # 默认开窗口
sim.command_fraction(0.0)           # 0 = 闭合，1 = 张到最大
sim.settle()                        # 约 1 s，受额定 85 mm/s 限速
print(sim.aperture_mm(), sim.fraction(), sim.finger_force_n())
sim.disconnect()
```

## 三个样例

| 文件 | 方向 | 需要真机？ |
| --- | --- | --- |
| [`examples/01_sim_only.py`](examples/01_sim_only.py) | —（纯 PyBullet） | **不需要** |
| [`examples/02_sim_to_real.py`](examples/02_sim_to_real.py) | 仿真 → 真机 | 要真的动就需要 |
| [`examples/03_real_to_sim.py`](examples/03_real_to_sim.py) | 真机 → 仿真 | 要真的镜像就需要 |

每个样例演示什么、以及它们共用的那套记号，见
[examples/README.zh-CN.md](examples/README.zh-CN.md)。

## 一个开度，三种写法

把它们混起来用是最常见的困惑来源，所以库统一从一个归一化开度换算
（0 闭合 … 1 张到最大）：

| 写法 | 范围 | 含义 |
| --- | --- | --- |
| 关节值 | `0 … 0.042726` m | 单个手指的移动行程；`0` 是张到最大 |
| 归一化开度 | `0 … 1` | 唯一两边通用的量 |
| SDK 毫米 | `0 … 120` mm | 真机报的**标定刻度** |
| 钳口间隙 | `1.548 … 87` mm | 两个指面的物理距离 |

SDK 毫米和钳口间隙之间不是单位换算的关系，它们是两个不同的量。只有归一化开度在
两边指的是同一件事。

## 状态

哪些验证过、哪些没有：

| 能力 | 状态 | 依据 |
| --- | --- | --- |
| URDF/xacro 归一化 | ✅ 已验证 | `tests/test_urdf.py`：上游 ROS 2 xacro 能在 PyBullet 里加载，2 个移动副关节，网格齐全 |
| 模型几何（行程 ↔ 开度） | ✅ 已验证 | `tests/test_model.py`，其中一条用例检查打包 xacro 的 `stroke` 默认值与 `STROKE_M` 一致 |
| 限速运动 | ✅ 已验证 | `tests/test_sim.py`：全行程 1.00 s 斜坡 + 约 0.13 s 伺服稳定，无超调 |
| 力上限与摩擦夹持 | ✅ 已验证 | 10 N 夹持力下扛得住 5 N，15 N 会滑；且只有手指碰到工件 |
| 样例 01 | ✅ 已验证 | 无窗口跑通、exit 0，四段演示的内容都在 `tests/test_examples_cli.py` 里断言 |
| 样例 02（仿真 → 真机） | ⚠️ **未验证** | 从未对真机执行过。只验证了 `--dry-run` 路径和命令行 |
| 样例 03（真机 → 仿真） | ⚠️ **部分验证** | 只读镜像路径在 `can0` 的真实夹爪上跑过；`--zero-gravity` 没跑过 |
| 真机运动指令（`goto`、力前馈） | ⚠️ **未验证** | 本仓库还没有向真机发过运动指令 |

仿真动力学来自 PyBullet，惯量用的是 URDF 自带的。手指限速和力上限由本库施加；
报告出来的伺服稳定时间是仿真里位置伺服的实测特性，不是真机测量值。

## 相关仓库

| 仓库 | 定位 |
| --- | --- |
| [litegrip-python](https://github.com/nexform-tech/litegrip-python) | Python SDK |
| [litegrip-cpp](https://github.com/nexform-tech/litegrip-cpp) | C++ SDK |
| [litegrip-ros2](https://github.com/nexform-tech/litegrip-ros2) | ROS 2 驱动 |
| [litegrip-urdf](https://github.com/nexform-tech/litegrip-urdf) | URDF/xacro 描述包 |
| [litegrip-docs](https://github.com/nexform-tech/litegrip-docs) | 产品文档 |

## 开发

```bash
python3 -m pip install -e ".[test]"
python3 -m pytest -q
```

URDF 默认用仓库自带的那一份。要换成别的模型，按优先级依次是：显式传入
`urdf_path` → `$LITEGRIP_URDF_PATH` → `$LITEGRIP_URDF_DIR` → 自带的一份 →
同级的 `litegrip-urdf` 检出。

## 仓库规范

本仓库遵循 NEXFORM ROBOTICS 的共享仓库规范：代理操作规则见 [AGENTS.md](AGENTS.md)，
提交信息用 Conventional Commits，合并到 `main` 后由 semantic-release 自动发版。

## 许可

Copyright © 2026 NEXFORM ROBOTICS. 以
[Apache License 2.0](LICENSE) 授权。
