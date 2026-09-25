# Caster 机械臂动力学参数辨识 — 完整指南

## 目录

- [系统总览](#系统总览)
- [目录结构](#目录结构)
- [数据流架构](#数据流架构)
- [MCU 端：真机数据采集](#mcu-端真机数据采集)
- [PC 端：离线参数辨识](#pc-端离线参数辨识)
- [需要修改的参数一览](#需要修改的参数一览)
- [编译与运行指令](#编译与运行指令)
- [辨识算法说明](#辨识算法说明)
- [故障排查](#故障排查)

---

## 系统总览

整个辨识流程分为 **MCU 端采集** 和 **PC 端辨识** 两部分：

```
┌─────────────────────────────────────────────────────────────────┐
│  MCU 端 (PinyCustom / STM32)                                    │
│                                                                 │
│  ParamIdentIntegration::trigger()                               │
│       │                                                         │
│       ▼                                                         │
│  ┌──────────┐    到位     ┌──────────┐   30s后   ┌──────────┐  │
│  │ MOVE_TO  │ ─────────▶ │ EXCITING │ ────────▶ │   DONE   │  │
│  │  HOME    │            │ Fourier  │           │ 清零力矩  │  │
│  │ PD回零位 │            │ PD跟踪   │           │          │  │
│  └──────────┘            │ +VOFA记录│           └──────────┘  │
│                          └──────────┘                           │
│                               │                                 │
│                          VOFA+ 串口                              │
│                               │                                 │
│                          output.csv                              │
│                          (time, q×6, qd×6, tau×6)               │
└───────────────────────────────┬─────────────────────────────────┘
                                │
                    复制 CSV 到 PC
                                │
┌───────────────────────────────▼─────────────────────────────────┐
│  PC 端 (Robot-Parameter-Indentification-Simulation)             │
│                                                                 │
│  ./build/identify --data-file data/output.csv --robot caster    │
│       │                                                         │
│       ▼                                                         │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐       │
│  │data_loader│─▶│preprocess│─▶│  Caster  │─▶│ OLS/WLS/ │       │
│  │ 读CSV    │  │ 数值微分  │  │ Regressor│  │ ML/...   │       │
│  │          │  │ 算qdd    │  │ 构建W矩阵│  │ 求解β    │       │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘       │
│                                              │                  │
│                                         results/                │
│                                    identification.yaml         │
└─────────────────────────────────────────────────────────────────┘
```

---

## 目录结构

```
Robot-Parameter-Indentification-Simulation/
├── Caster/                          # Caster 机械臂模型
│   ├── urdf/
│   │   ├── caster2.xml              # MuJoCo MJCF 模型
│   │   └── scene.xml               # MuJoCo 场景文件
│   └── asserts/
│       ├── base_link.STL            # 7 个网格文件
│       ├── Link1.STL ~ Link6.STL
│
├── config/
│   └── identification.yaml          # 辨识配置（可选，也可命令行指定）
│
├── data/
│   └── output.csv                   # 真机采集的 CSV 数据
│
├── results/
│   └── identification.yaml          # 辨识输出结果
│
├── src/identification/
│   ├── include/
│   │   ├── identification/
│   │   │   ├── identification.hpp   # 辨识主类
│   │   │   ├── algorithms.hpp       # 8种辨识算法
│   │   │   └── data_loader.hpp      # CSV 加载器
│   │   ├── mujoco_caster_dynamics.hpp   # ★ Caster 动力学
│   │   ├── mujoco_caster_regressor.hpp  # ★ Caster 回归器
│   │   ├── mujoco_piper_dynamics.hpp    # Piper 动力学（参考）
│   │   ├── mujoco_piper_regressor.hpp   # Piper 回归器（参考）
│   │   └── robot/
│   │       ├── robot_model.hpp      # 通用机器人模型接口
│   │       └── regressor.hpp        # 通用回归器（DH参数版）
│   │
│   └── src/
│       ├── main.cpp                 # identify 可执行文件入口
│       ├── identification.cpp       # 辨识核心逻辑
│       ├── data_loader.cpp          # CSV 解析
│       ├── algorithms.cpp           # OLS/WLS/IRLS/TLS/EKF/ML/CLOE/NLS
│       └── robot/
│           ├── mujoco_caster_dynamics.cpp   # ★ Caster 动力学实现
│           └── mujoco_caster_regressor.cpp  # ★ Caster 回归器实现
│
└── CMakeLists.txt
```

**★ 标记的文件是为 Caster 新增的，修改机器人参数时主要改这些文件。**

---

## 数据流架构

### CSV 数据格式

MCU 端 `DataRecorder` 输出 19 列 CSV：

```csv
time,q0,q1,q2,q3,q4,q5,qd0,qd1,qd2,qd3,qd4,qd5,tau0,tau1,tau2,tau3,tau4,tau5
```

| 列 | 含义 | 单位 |
|---|---|---|
| `time` | 时间戳 | s |
| `q0~q5` | 关节位置 | rad |
| `qd0~qd5` | 关节速度 | rad/s |
| `tau0~tau5` | 关节力矩（PD 控制器输出） | Nm |

> **注意**：CSV 中**没有 qdd（加速度）**。`data_loader` 会检测到 19 列格式，自动用数值微分从 qd 计算 qdd。

### 辨识公式

```
τ = W · β
```

- **τ**：测量力矩向量（来自 CSV 的 tau 列）
- **W**：观测/回归矩阵（由 Caster Regressor 根据 q, qd, qdd 构建）
- **β**：待辨识的惯性参数向量（72 个参数 = 6 个 body × 10 个参数 + 6 个 armature + 6 个 damping）

### 代码调用链

```
main.cpp::main()
  │
  ├── loadConfig()                    # 读取 config 或命令行参数
  │
  ├── DataLoader::loadCSV()           # 解析 CSV → ExperimentData {q, qd, tau, time}
  │
  ├── Identification(robot_type)      # 创建辨识对象
  │     └── 初始化 MuJoCoCasterRegressor（加载 Caster body 参数）
  │
  ├── identifier.preprocess(data)     # 数值微分计算 qdd
  │
  ├── MuJoCoCasterDynamics            # 计算 MuJoCo 基准 RMSE（可选对比）
  │     └── computeInverseDynamics(q, qd, qdd) → 预测力矩
  │
  ├── identifier.solve(data, algo)    # 核心辨识
  │     ├── computeObservationMatrix() → W 矩阵
  │     │     └── MuJoCoCasterRegressor::computeObservationMatrix()
  │     │           └── 对每个样本调用 computeRegressorMatrix(q, qd, qdd)
  │     │                 └── computeBodyRegressorBlock() × 6 个 body
  │     │
  │     ├── 过滤异常值（qdd 阈值）
  │     │
  │     └── OLS/WLS/...::solve(W, tau) → β 参数向量
  │
  └── saveResults()                   # 输出到 results/identification.yaml
```

---

## MCU 端：真机数据采集

### 关键配置文件

**`/code/PinyCustom/Src/App/RobotModules/Arm/ParamIdent/ParamIdentIntegration.cpp`**

```cpp
config_ = {
    .homePosition      = {0.f, -3.1f, -3.1f, 0.f, 0.f, 0.f},  // ★ 回零目标位
    .homeSpeedRate     = 0.5f,
    .homeTolerance     = 0.05f,                                 // ★ 到位容差 rad
    .trajectoryPeriod  = 10.f,                                  // ★ 轨迹周期 s
    .trajectoryScale   = 0.12f,                                 // ★ 振幅缩放
    .trajectorySeed    = 42,                                    // ★ 随机种子
    .trajectoryDuration = 30.f,                                 // ★ 采集时长 s
    .recordDiv         = 10,                                    // ★ 分频（1kHz/10=100Hz）
    .kp                = {10.f, 22.f, 10.f, 0.5f, 0.8f, 0.5f}, // ★ PD kp
    .kd                = {0.8f, 4.f, 1.f, 0.2f, 0.35f, 0.2f},  // ★ PD kd
    .maxTorque         = {6.f, 6.f, 6.f, 0.8f, 0.8f, 0.8f},    // ★ 力矩限幅 Nm
    .homeTimeoutSec    = 30.f,
    .jointMin          = {-1.57f, -3.1f, -3.1f, -2.2f, -1.7f, -1.9f}, // ★ 关节下限
    .jointMax          = { 1.5f,  0.f,  0.f,   3.f,  1.56f,   3.f },  // ★ 关节上限
    .maxVelocity       = {1.f, 1.f, 1.f, 1.f, 1.f, 1.f},
    .eStopTorque       = {5.5f, 8.f, 5.5f, 0.7f, 0.8f, 0.7f},  // ★ 急停力矩阈值
};
```

### 傅立叶轨迹参数

**`/code/PinyCustom/Src/App/RobotModules/Arm/ParamIdent/ExcitationTrajectory.hpp`**

```
q_i(t)  = q0_i + Σ_{j=1}^{5} [ A_ij/(ω·j) · sin(ω·j·t) - B_ij/(ω·j) · cos(ω·j·t) ]
qd_i(t) = Σ_{j=1}^{5} [ A_ij · cos(ω·j·t) + B_ij · sin(ω·j·t) ]
```

- `ω = 2π/T`，`T = trajectoryPeriod`
- 系数 A, B 由确定性 LCG 伪随机数（seed=42）生成
- 施加初始约束：q(0)=home, qd(0)=0, qdd(0)=0

---

## PC 端：离线参数辨识

### Caster Body 参数（核心！修改机器人时必须更新）

**`src/identification/src/robot/mujoco_caster_dynamics.cpp`** 和 **`mujoco_caster_regressor.cpp`**

两个文件中的 `initBodies()` 函数包含完全相同的 body 参数，**必须保持一致**。

```cpp
// 以 Link1 为例：
bodies_[1].name = "Link1";
bodies_[1].pos = Vector3d(0.065, 0.0055, 0.061078);     // ★ 相对于父body的位置
bodies_[1].quat = Quaterniond(0.499998, 0.5, 0.5, -0.500002); // ★ 相对于父body的姿态
bodies_[1].mass = 0.17456;                                // ★ 质量 kg
bodies_[1].com = Vector3d(3.1394e-05, -0.0055261, 0.040465); // ★ 质心位置 m
bodies_[1].Ixx = 0.0001154398387;                         // ★ 惯量张量
bodies_[1].Iyy = 0.0001199199839;                         //   kg·m²
bodies_[1].Izz = 0.0001295898918;
bodies_[1].Ixy = -4.74914118e-08;
bodies_[1].Ixz = -4.126868193e-09;
bodies_[1].Iyz = 2.256879731e-05;
bodies_[1].has_joint = true;
bodies_[1].joint_axis = Vector3d(0.0, 0.0, 1.0);         // ★ 关节旋转轴
bodies_[1].armature = 0.1;                                // ★ 电机反映惯量
bodies_[1].damping = 1.0;                                 // ★ 粘性阻尼系数
```

### Body 参数来源

当前参数从 **Caster URDF**（SolidWorks 导出）的惯性数据转换而来：

| Body | 来源 |
|---|---|
| `pos`, `quat` | URDF `<joint>` 的 `origin xyz + rpy` → 四元数 |
| `mass` | URDF `<inertial>` 的 `mass value` |
| `com` | URDF `<inertial>` 的 `origin xyz` |
| `Ixx~Izz, Ixy~Iyz` | 从 MuJoCo XML 的 `diaginertia` + `inertial quat` 反算 |
| `joint_axis` | URDF `<joint>` 的 `axis xyz` |
| `armature` | MuJoCo XML 默认值 `0.1` |
| `damping` | MuJoCo XML 默认值 `1.0` |

### base_link 特殊处理

Caster 的 MuJoCo 模型有一个 `mount` body 做全局旋转（Z 轴朝上）：
```xml
<body name="mount" quat="0.707107 -0.707107 0 0">
```
这个旋转被合并到了 `bodies_[0].quat` 中。

---

## 需要修改的参数一览

### 场景1：更换机械臂（修改连杆质量、长度等）

| 修改内容 | 文件 |
|---|---|
| Body 参数（pos/quat/mass/com/inertia） | `src/identification/src/robot/mujoco_caster_dynamics.cpp` 的 `initBodies()` |
| Body 参数（同上，必须一致） | `src/identification/src/robot/mujoco_caster_regressor.cpp` 的 `initBodies()` |
| 关节轴方向 | 同上两个文件的 `joint_axis` |
| armature / damping | 同上两个文件 |
| MuJoCo 模型（仿真用） | `Caster/urdf/caster2.xml` |

### 场景2：修改辨识配置

| 修改内容 | 方法 |
|---|---|
| 机械臂类型 | 命令行 `--robot caster` 或 `config/identification.yaml` |
| 数据文件路径 | 命令行 `--data-file data/output.csv` |
| 辨识算法 | 命令行 `--algorithm 1`（1=OLS, 0=全部） |
| 输出文件路径 | 命令行 `--output-file results/xxx.yaml` |

### 场景3：修改真机采集参数

| 修改内容 | 文件 |
|---|---|
| 回零位、PD增益、力矩限幅 | `ParamIdentIntegration.cpp` 的 `IDENT_CONFIG` |
| 轨迹周期、振幅、种子 | 同上 |
| 采集时长、采样率 | 同上 |
| 关节软限位 | 同上 `jointMin` / `jointMax` |

### 场景4：添加新机器人

1. 创建 `mujoco_xxx_dynamics.hpp` + `.cpp`（复制 caster 版本，修改 `initBodies()`）
2. 创建 `mujoco_xxx_regressor.hpp` + `.cpp`（同上）
3. 在 `identification.hpp` 中添加 `#include` 和成员变量
4. 在 `identification.cpp` 的 `numParameters()` / `getGroundTruthParameters()` / `computeObservationMatrix()` 中添加 `else if (robot_type_ == "xxx")` 分支
5. 在 `main.cpp` 的 `robotDof()` 和 MuJoCo 基准计算中添加分支
6. 在 `CMakeLists.txt` 的 `identification_core` 中添加新的 `.cpp` 文件

---

## 编译与运行指令

### 编译

```bash
cd /code/Dynamic_Parameter_Identification/Robot-Parameter-Indentification-Simulation

# 配置
cmake -S . -B build

# 编译（并行）
cmake --build build --parallel

# 只编译 identify
cmake --build build --target identify
```

### 运行辨识

```bash
# 跑全部8种算法（推荐，可以看到哪种效果最好）
./build/identify --data-file data/output.csv --robot caster

# 只跑 OLS
./build/identify --data-file data/output.csv --robot caster --algorithm 1

# 只跑 ML（通常效果最好）
./build/identify --data-file data/output.csv --robot caster --algorithm 6

# 只跑 NLS_FRICTION（含非线性摩擦辨识）
./build/identify --data-file data/output.csv --robot caster --algorithm 8

# 指定输出文件
./build/identify --data-file data/output.csv --robot caster --output results/caster_v2.yaml
```

### 算法编号对照

| 编号 | 算法 | 说明 |
|---|---|---|
| 0 | 全部 | 跑完8种，按 RMSE 排序 |
| 1 | OLS | 普通最小二乘 |
| 2 | WLS | 加权最小二乘 |
| 3 | IRLS | 迭代重加权（Huber鲁棒） |
| 4 | TLS | 总体最小二乘 |
| 5 | EKF | 扩展卡尔曼滤波 |
| 6 | ML | 最大似然（通常最好） |
| 7 | CLOE | 负载在线辨识（降级为OLS） |
| 8 | NLS_FRICTION | 非线性摩擦联合辨识 |

---

## 辨识算法说明

### 输出格式 (`results/identification.yaml`)

```yaml
calibration_date: "2026-09-21 17:31:22"
algorithm: "OLS"
evaluation_results:
  torque_rmse: 0.146874      # 力矩残差 RMSE (Nm)
  torque_max_error: 0.453325 # 最大残差 (Nm)
parameters:                  # 72 个参数
  - 0           # body1: m
  - -222.157    # body1: mx
  - 44.2599     # body1: my
  - ...         # (每 body 10 个参数: m, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz)
  # 共 6 body × 10 = 60 个惯性参数
  # + 6 个 armature
  # + 6 个 damping
```

### 参数向量 β 的结构

```
β = [body1(10), body2(10), ..., body6(10), armature(6), damping(6)]
    |___________________________|   |________|   |________|
           60 惯性参数                6 电枢       6 阻尼
```

每个 body 的 10 个参数：
- `m`: 质量 (kg)
- `mx, my, mz`: 一阶矩 = mass × com_position (kg·m)
- `Ixx, Ixy, Ixz, Iyy, Iyz, Izz`: 惯量张量 (kg·m²)

---

## 故障排查

### CSV 列数不匹配

```
CSV 列数与配置的机械臂自由度不匹配: 25 列, 期望 25 或 19 列
```

**原因**：表头和数据行列数不一致。`data_loader` 按**数据行**判断格式。

**解决**：确保表头列数 = 数据行列数。对于 Caster 6-DOF，应为 19 列：
```csv
time,q0,q1,q2,q3,q4,q5,qd0,qd1,qd2,qd3,qd4,qd5,tau0,tau1,tau2,tau3,tau4,tau5
```

### 过滤了太多异常值

```
Filtered 56 outlier samples. 32 valid samples remain.
```

**原因**：qdd 数值微分产生了很大加速度（>10 rad/s² 的样本被过滤）。

**解决**：
1. 采集更长时间的数据（建议 10~30 秒）
2. 确保采样率足够高（当前 100Hz）
3. 检查关节速度是否平滑（无突变）

### MuJoCo 基准 RMSE 很大

```
MuJoCo 动力学基准 RMSE: 0.98 Nm
```

**含义**：MuJoCo 默认参数预测的力矩与实测力矩的偏差。这是正常的——说明默认参数不准确，需要辨识。

### 编译错误：undefined reference to `MuJoCoCasterDynamics`

**原因**：CMakeLists.txt 中没有添加新的 `.cpp` 文件。

**解决**：确保 `identification_core` target 中包含：
```cmake
src/identification/src/robot/mujoco_caster_dynamics.cpp
src/identification/src/robot/mujoco_caster_regressor.cpp
```

新 URDF
  │
  ├─→ Caster/urdf/caster2.xml          (MuJoCo 模型)
  ├─→ mujoco_caster_dynamics.cpp       (辨识动力学)
  ├─→ mujoco_caster_regressor.cpp      (辨识回归器)
  └─→ GravityCompensation.hpp          (MCU 重力补偿)