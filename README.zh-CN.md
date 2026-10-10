# SimCompat（中文说明）

[English](README.md)

**你的机器人模型文件，在每个物理引擎里被仿真成同一个东西了吗？**

机器人描述文件不等于仿真。每个引擎的导入器都会把 URDF 转成*它自己的*模型：套用默认值、丢掉不支持的字段，偶尔还有 bug，而且全程不报错。SimCompat 把同一个文件加载进多个引擎（目前是 MuJoCo、PyBullet，以 Pinocchio 作参考），读回每个引擎**实际构建出的模型**，然后用直白的语言告诉你：哪个引擎仿真的东西和文件不一样、原因是什么、怎么修。

```bash
pip install "simcompat[all] @ git+https://github.com/isospin01/simcompat"    # Python 3.10-3.12
simcompat check path/to/robot.urdf      # 也可以是 robot.urdf.xacro、包目录，或 rd:go2
```

输出示例（Unitree Go2）：

```text
✗ FAIL  Some engines simulate something different from what the file declares.

1. ✗ Engines simulate different mass, center of mass or inertia than the file declares
   pybullet  FL_calf_joint   mass 2.194 kg vs declared 0.194 kg; COM off by 30.9 mm; ...
   mujoco    FL_calf_joint   mass 0.241352 kg vs declared 0.194 kg; COM off by 6.51 mm; ...
   Fix: These bodies include links without <inertial>, ...
   Also explains: dynamics.mass_matrix (mujoco, pybullet), rollout.rigid ...

2. ! Moving links have no <inertial>, so each engine invents its own mass for them
   file  FL_calflower   moved by 'FL_calf_joint' but has no <inertial>
```

问题按**根因**归组：能被惯量或导入问题解释的动力学/轨迹差异，会挂在对应根因下面（"Also explains"），而不是单独刷屏。在公开机器人上，原始比较的差异中位数是 25 条，报告里只需读 2 个问题。退出码：`0` 表示所有引擎都与文件一致，`1` 表示有引擎不一致（加 `--strict` 时警告也算），`2` 表示输入或参数有误，可以直接作为 CI 关卡，在训练策略之前拦住"被某个引擎悄悄改掉"的模型。

## 常用操作

| 你想…… | 命令 |
|---|---|
| 检查一个机器人 | `simcompat check robot.urdf` |
| 生成可分享的报告 | `simcompat check robot.urdf -o report.html`（也支持 `.md`、`.json`、`.txt`） |
| 检查 xacro（不需要 ROS） | `simcompat check robot.urdf.xacro --xacro-arg prefix:=left_` |
| 在公开机器人上试用 | `simcompat robots unitree`，然后 `simcompat check rd:go2` |
| 看懂某个检查项 | `simcompat explain inertia.clusters`（`simcompat explain` 列出全部） |
| 只跑部分检查 | `--only dynamics`、`--skip rollout`、`--only 'import.*'` |
| 给仓库加 CI 关卡 | `simcompat init`：生成 `simcompat.toml` 和 `.github/workflows/simcompat.yml` |
| 比较引擎配置 | `simcompat check robot.urdf -b mujoco -b pybullet -b pybullet:inertia_from_file=1` |
| 批量检查多个机器人 | `simcompat matrix rd:go2 rd:g1 my_arm.urdf -o matrix.html` |
| 查看全部细节和关节顺序表 | 加 `-v` |

输入会自动解析：`package://` 网格路径会向上查找所在的包（必要时用 `--package-dir NAME=PATH`），xacro 用 `xacrodoc` 展开（无需安装 ROS），目录会自动找到其中唯一的 URDF。各种笔误会在运行前就被拦下并给出建议，例如 `-b mujoko` 会提示 "Did you mean 'mujoco'?"，`rd:go22` 会建议 `rd:go2`，`simcompat.toml` 里的拼写错误会指出文件和键名。

项目配置文件（`simcompat init` 会生成一个带注释的）可以设置容差、调整检查项的严重级别，例如代码按名字映射关节时可设 `"import.joint_order" = "info"`。调整一个根因的级别时，它所解释的症状也会随之调整：设置 `"inertia.clusters" = "warn"` 后，由惯量差异引起的动力学和轨迹差异也只算警告，已知问题就不会再让 CI 失败。

Python 接口：

```python
import simcompat

report = simcompat.check("robot.urdf", backends=["pinocchio", "mujoco"])
report.ok                                  # 有引擎与文件不一致时为 False
for problem in report.problems:            # 按根因归组，最严重的在前
    print(problem.severity, problem.headline, problem.engines, problem.hints)
report.save("report.html")
```

[`examples/`](examples/) 里有真实报告（HTML、Markdown、JSON、终端输出）和常见机器人的兼容性矩阵。

## 检查内容

| 层级 | 比较什么 | 对照 |
|---|---|---|
| 文件 | 缺少 `<inertial>` 的连杆、不可能的惯量、有歧义的关节范围、非单位轴、找不到的网格 | （会导致引擎分歧的文件属性） |
| 导入 | 能否加载、自由度、**关节顺序**（并识别每个引擎用的遍历约定）、浮动关节、限位、力矩/速度限制、读到的阻尼/摩擦、引擎注入的 armature、mimic 耦合 | 文件 |
| 惯量 | 每个关节带动的所有连杆的复合质量、质心、惯量（先合并固定子连杆，会合并静态体的引擎也能公平比较） | 文件 |
| 运动学 | 采样构型下子连杆位姿和关节轴方向（能抓出反向的轴） | 文件 |
| 动力学 | M(q)、g(q)、C(q,q̇)q̇，固定基座、无接触 | Pinocchio（参考） |
| 轨迹 | 零力矩轨迹；**声明的阻尼/摩擦是否真的生效**（每个引擎和自己做消融对比） | 参考 / 第一个仿真器 |

设计原则：文件就是真值（simcompat 自己解析 URDF，不经过任何引擎）；确定性的物理用严格容差（导入正确时各引擎能对齐到约 1e-12，所以 FAIL 就意味着导入问题）；报告根因而不是症状；引擎默认行为全部公开（例如 PyBullet 的默认速度电机、零重力、连杆阻尼和 100 rad/s 限速，都会写进报告）。

## 在真实数据集上的验证

我们在 [robot_descriptions](https://github.com/robot-descriptions/robot_descriptions.py) 的**全部 132 个 URDF 模型**上运行了 SimCompat，涵盖机械臂、四足、人形、灵巧手和移动操作平台。方法、数据和复现方式见 [VALIDATION.md](VALIDATION.md)。摘要：

| 问题 | 结果 |
|---|---|
| 能在真实模型上跑通吗？ | 132 个中 131 个完整跑通，0 次崩溃，四种引擎配置每个机器人中位耗时 1.0 秒。唯一的例外是上游 xacro 本身有错，被报告为输入错误。 |
| 真值可靠吗？ | 解析器与独立实现 yourdfpy 在全部 4,012 个连杆上一致：质量完全相同，位置误差不超过 1.4e-8 m；Pinocchio 在 131/131 个模型上与之一致。 |
| 能抓到真实的导入 bug 吗？ | 往 109 个机器人的 MuJoCo 编译模型中注入 1,028 个 bug，其中 1,027 个被对应的检查项抓到、定位到正确的关节，并作为独立问题报告。bug 类型来自真实 issue：质量/质心/惯量错误、惯量坐标系旋转丢失、关节轴反向或倾斜、原点偏移、阻尼/摩擦/限位丢失、注入 armature、广度优先关节顺序。唯一漏掉的一个低于 1e-9 kg·m² 的惯量下限（一个 2 mg 的占位连杆）。 |
| 给出的原因对吗？ | 公开模型上的全部 904 处动力学和轨迹差异都被归到了某个根因，没有一处无法解释。其中 871 处来自惯量类根因，我们对这些归因做了反事实验证：把 simcompat 指出的原因改掉再跑一遍，214 个案例中差异全部消失。 |
| 有实际价值吗？ | 66% 的公开模型在 MuJoCo 或 PyBullet 中被仿真成与文件不同的东西；其中 20 个 MuJoCo 直接拒绝加载，40% 的模型各引擎关节编号不同，19 个模型声明的关节摩擦被 PyBullet 读到但没有生效。 |

## 为什么需要它

sim2sim（例如 Isaac Lab → MuJoCo）已是上真机前的标准关卡，多后端训练也越来越普遍。策略换个引擎就不工作，很多时候不是物理问题，而是**同一个文件被读成了不同的模型**，而且这类问题既无声又难查：

| 案例 | 发生了什么 |
|---|---|
| [Isaac Sim #841](https://github.com/isaac-sim/IsaacSim/issues/841) | Isaac Sim 6.1 静默丢弃了 URDF 的阻尼和摩擦（NVIDIA 确认是回归 bug） |
| [Genesis #2398](https://github.com/Genesis-Embodied-AI/genesis-world/discussions/2398) | Genesis 与 MuJoCo 的轨迹对不上，花了约 5 周才查到是默认 armature |
| [Isaac Lab #4383](https://github.com/isaac-sim/IsaacLab/issues/4383) | 自定义关节顺序没生效："机器人在 MuJoCo 里立刻摔倒" |
| [humanoid-locomotion-sim2sim](https://github.com/earosenfeld/humanoid-locomotion-sim2sim) | 对齐 G1 在 Menagerie（0.01）和 Isaac Lab（0.03）中的 armature 后，跟踪速度从 0.63 变为 0.80 m/s |

引擎团队只对自己的导入器、用精选模型做对齐测试；URDF 静态检查器只看文件，不看引擎实际构建了什么。完整调研见 [docs/PROBLEM.md](docs/PROBLEM.md)。

## 引擎与扩展

| 后端 | 角色 | 选项（`-b name:option=value`，`simcompat backends` 可查看） |
|---|---|---|
| `pinocchio` | 参考（解析的 CRBA/RNEA/ABA），本身也会与文件对照检查 | 无 |
| `mujoco` | 仿真器（内置 URDF 导入器） | `fusestatic`、`discardvisual`、`balanceinertia`、`inertiafromgeom`、`boundmass`、`boundinertia` |
| `pybullet` | 仿真器（`loadURDF`） | `inertia_from_file`、`merge_fixed_links` |

计划支持：Isaac Sim/Lab（PhysX 与 Newton）、Genesis、Drake、Gazebo，见 [ROADMAP.md](ROADMAP.md)。新增一个引擎只需写一个约十个小方法的类，第三方包也可以通过 entry point 注册，见 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md#adding-a-backend)。

## 局限

- 输入目前只支持 URDF 和 xacro，MJCF、USD 在计划中。
- 比较基于固定基座；浮动关节只报告各引擎如何构建（`import.multi_dof`），不比较运动。
- 尚未比较接触、执行器和 PD 控制器，这是下一阶段的工作。

## 参与贡献

```bash
git clone https://github.com/isospin01/simcompat && cd simcompat
pip install -e ".[dev]" && pytest && ruff check .
```

先读 [CONTRIBUTING.md](CONTRIBUTING.md)，再读 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)（十分钟代码导览）。测试中包含一个不依赖任何引擎、可注入导入 bug 的假后端，以及固定每条引擎行为的引擎测试。许可证 Apache-2.0。
