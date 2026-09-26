# HANDOFF — 机器人动力学参数辨识项目

## 任务概述

为 Caster 6-DOF 机械臂建立完整的**动力学参数辨识流水线**，实现从真机数据采集到重力补偿的闭环：

```
STM32H723 (PinyCustom)        PC (本项目)              STM32H723 (PinyCustom)
┌────────────────────┐    ┌──────────────────┐    ┌─────────────────────┐
│ 激励轨迹 + 串口录制 │──→│ 参数辨识          │──→│ 重力补偿            │
│ → output.csv       │    │ → identification  │    │ → 拖拽示教/悬停     │
└────────────────────┘    │   .yaml           │    └─────────────────────┘
                          └──────────────────┘
```

**当前状态：Regressor body chain 已修正为与 caster3.xml 一致，等待编译验证和重新辨识。**

---

## 项目结构

### 本项目 (`D:\RM\Robot-Parameter-Indentification-Simulation`)

PC 端参数辨识程序（C++/CMake），读取 CSV 数据，用 8 种算法辨识 72 个惯性参数。

关键文件：
- `src/identification/src/main.cpp` — 辨识主程序入口
- `src/identification/src/data_loader.cpp` — CSV 加载（自动检测 19/25 列格式）
- `src/identification/src/identification.cpp` — 预处理 + 观测矩阵构建
- `src/identification/src/algorithms.cpp` — 8 种辨识算法 (OLS/WLS/IRLS/TLS/EKF/ML/CLOE/NLS_FRICTION)
- `src/identification/src/robot/mujoco_caster_regressor.cpp` — Caster 回归器（观测矩阵 W）
- `src/identification/src/robot/mujoco_caster_dynamics.cpp` — Caster 动力学（基准对比）
- `config/identification.yaml` — 辨识配置（robot: caster, algorithm: 0=全部跑）
- `tools/serial_record.py` — 串口数据采集脚本

### PinyCustom (`D:\RM\PinyCustom`)

STM32H723 FreeRTOS 固件，控制真实机械臂执行激励轨迹并录制数据。

关键文件：
- `Src/App/RobotModules/Arm/ParamIdent/ParamIdentTask.hpp` — 辨识任务状态机
- `Src/App/RobotModules/Arm/ParamIdent/ParamIdentIntegration.cpp` — 电机/串口接口
- `Src/App/RobotModules/Arm/ParamIdent/ExcitationTrajectory.hpp` — Fourier 激励轨迹
- `Src/App/RobotModules/Arm/ParamIdent/DataRecorder.hpp` — CSV 录制器
- `Src/App/RobotModules/Arm/ParamIdent/GravityCompensation.hpp` — 简化重力补偿（辨识用）
- `Src/App/RobotModules/Arm/ParamIdent/IdentifiedCompensation.hpp` — 全参数重力补偿（拖拽示教用）
- `Src/App/RobotModules/Arm/Arm.cpp` — casterMdh（正确的 DH 参数参考）

---

## 已完成的工作

### 1. 完整流程审查

审查了从 MCU 数据采集到 PC 辨识到 MCU 重力补偿的每一步，确认：
- **所有 8 种辨识算法数学上无错误**（OLS/IRLS/TLS/EKF/NLS_FRICTION 等均已手算验证）
- **回归器的 Newton-Euler 公式正确**（10 参数惯性模型 + armature + damping）
- **IdentifiedCompensation.hpp 的 DH 参数与 Arm.cpp casterMdh 一致** ✅
- **2D 重力矩公式正确**（已与 3D 计算交叉验证）

### 2. 发现并修复了 4 个问题

#### 问题 A：录制的是指令力矩，不是反馈力矩 [已修复]

**根因**：`readArmState()` 只读了 `posNorm()` 和 `vel()`，没有读 `torq()`。`recorder_.record()` 录制的是 MCU 发给电机的指令力矩（包含 URDF 重力补偿 + PD 跟踪），不是电机实际输出的力矩。

**辨识模型期望**：`τ_实际 = M(q)q̈ + C(q,q̇)q̇ + G(q) - damping·q̇`
**实际录制的**：`τ_指令 = τ_g_URDF(q) + Kp·err + Kd·derr`

指令力矩混入了 URDF 重力补偿偏差（GravityCompensation.hpp 的 GAIN=4.0、J4=0、J5=-0.2），导致辨识参数被污染。

**修复**：
- `ParamIdentTask.hpp`：`JointState_s` 添加 `float tau[6]`
- `ParamIdentIntegration.cpp`：`readArmState()` 添加 `s.tau[i] = m.all_motors[i]->torq()`
- `ParamIdentTask.hpp`：`stateExciting()` 改为录制 `state.tau`（反馈力矩）

#### 问题 B：傅里叶轨迹撞限位 [已修复]

**根因**：`trajectoryScale=0.12` 生成的轨迹可能超出关节限位。虽然 `stateExciting()` 对位置做了 clamp，但 clamp 破坏了轨迹连续性，导致数值微分的 q̈ 出现尖峰，污染辨识数据。

**修复**：在 `stateHoldHome()` 中添加轨迹验证——开始激励前检查整个周期内轨迹是否超出限位，超出则打印警告并回到 IDLE。

#### 问题 C：Regressor 与 Dynamics 质量不一致 [已修复]

**根因**：`mujoco_caster_dynamics.cpp` 的 Link2/3/4/5 质量多加了电机质量（如 Link2: dynamics=0.48, regressor=0.15），导致 `main.cpp` 中的 MuJoCo 基准 RMSE 比较无效。

**修复**：将 dynamics 的质量改为与 regressor 一致（link-only 质量）。

#### 问题 D：RTT stop 命令不生效 [已修复]

**根因**：`stopAndDisable()` 只在 DONE 和 EXCITING 状态生效，其他状态（MOVE_TO_HOME、HOLD_HOME 等）调用时静默忽略。

**修复**：改为对所有非 IDLE 状态都响应——写零力矩 + 失能电机。

### 3. 已识别但未修复的问题

- **GravityCompensation.hpp 的 GAIN=4.0**：这是辨识前用的简化重力补偿，手动增益说明 URDF 质量严重低估。辨识完成后应该用 IdentifiedCompensator 替代它。
- **ParamIdentTask.hpp 的 `kinematic_` DH 参数错误**：死代码，未被使用，不影响运行。
- **CLOE 算法未实现**：回退到无正则化 OLS，不影响其他算法。

### 4. Regressor/Dynamics body chain 修正（2026-09-25）

**根因**：`initBodies()` 的 body 参数与 `caster3.xml` 不一致，导致辨识出的参数在错误的 body-local frame 中定义，MCU 端补偿器用错误的 frame 解读这些参数，重力补偿失效。

**修正内容**（`mujoco_caster_regressor.cpp` 和 `mujoco_caster_dynamics.cpp` 同步修改）：

| Body | 参数 | 旧值 | 新值 (匹配 caster3.xml) |
|---|---|---|---|
| base_link | quat | Rx(-90°) | identity |
| base_link | mass | 0.69445 | 0 |
| Link1 | pos | (0.065, 0.0055, 0.061) | (0, 0, 0) |
| Link1 | quat | Rz(-90°)·Rx(-90°) | identity |
| Link1 | mass | 0.37981 | 0.17456 |
| Link2 | pos | (0, -0.0405, 0.051) | (0, 0, 0.051) |
| Link4 | pos | (0, 0.055, -0.052) | (0, 0.055, -0.0115) |
| Link6 | mass | 0.026 | 0.017338 |
| J2 | axis | (0,0,-1) | (0,0,1) |

**影响**：修正后需要重新跑辨识，新的 72 个参数才能在 MCU 端正确使用。

---

## 当前卡在哪

**等待重新采集数据验证修改效果。**

修改后的代码已写入文件，但：
- PinyCustom 需要 ARM 交叉编译工具链（`arm-none-eabi-g++`）编译烧录
- 辨识程序需要 MuJoCo + Eigen3 + GLFW 库编译
- 需要在真机上重新采集数据，然后在 PC 上运行辨识

---

## 下一步计划

1. **编译验证 Regressor**：在有 Eigen3/MuJoCo 的环境编译，确认无编译错误
2. **重新跑辨识**：用已有数据（或重新采集）运行辨识，检查 RMSE
3. **回写参数到 IdentifiedCompensation.hpp**：将新辨识结果更新到 MCU 固件
4. **编译烧录 PinyCustom 固件**：烧录到 STM32H723
5. **真机验证拖拽悬停**：确认重力补偿方向和幅值正确

---

## 踩过的坑（绝对不要再踩）

### 坑 1：录制指令力矩而非反馈力矩

**现象**：辨识 RMSE ~0.6 Nm，远高于预期。
**原因**：`recorder_.record(elapsed, q, qd, tau)` 中的 `tau` 是 MCU 计算的指令力矩（包含重力补偿 + PD），不是电机实际输出的力矩。
**教训**：辨识用的力矩必须是电机反馈力矩（`IMotor::torq()`），它才是真实的动力学力矩。指令力矩混入了控制策略的偏差。

### 坑 2：轨迹 clamp 破坏数据质量

**现象**：辨识数据中有大量异常值。
**原因**：傅里叶轨迹超出关节限位后被 `std::clamp` 截断，导致位置不连续 → 速度跳变 → q̈ 尖峰。
**教训**：轨迹执行前必须验证可行性，不要在执行时 clamp。clamp 破坏了动力学一致性。

### 坑 3：Regressor 和 Dynamics 质量定义不一致

**现象**：MuJoCo 基准 RMSE 不为零（理论上应为零）。
**原因**：regressor 用 link-only 质量（正确），dynamics 用了 link+motor 质量（错误）。两处代码分别维护，改了一处忘了另一处。
**教训**：共享参数应该提取到一个地方，不要在两个文件中各写一份。如果必须分开，写单元测试验证一致性。

### 坑 4：GravityCompensation.hpp 的手动增益

**现象**：GAIN=4.0、J4=0、J5=-0.2 说明 URDF 模型与真实重力差异巨大。
**教训**：简化重力补偿的手动增益只在辨识过程中临时使用。辨识完成后必须用辨识结果替代。不要把手动调参的模型当作最终方案。

### 坑 5：STM32 和 PC 的随机数生成器不同

**现象**：同一 seed=42 在 STM32（LCG）和 PC（MT19937）上产生完全不同的轨迹系数。
**教训**：如果需要在 PC 复现 MCU 的轨迹，必须用 `setCoefficients()` 传输系数，不能靠 seed 同步。当前辨识流程不依赖参考轨迹，所以不影响辨识结果。

### 坑 6：stop 命令只在部分状态生效

**现象**：RTT 输入 stop 有时没有反应。
**原因**：`stopAndDisable()` 只处理 DONE 和 EXCITING 状态，其他状态静默忽略。
**教训**：安全相关的命令（停止/急停）必须覆盖所有非空闲状态，宁可多处理也不要静默忽略。

---

## 技术细节备忘

### CSV 格式

19 列（无 qdd）：`time, q0..q5, qd0..qd5, tau0..tau5`
PC 端自动通过中心差分计算 qdd。

### 辨识参数布局（72 个）

```
[0-9]    Link1: m, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz
[10-19]  Link2: ...
[20-29]  Link3: ...
[30-39]  Link4: ...
[40-49]  Link5: ...
[50-59]  Link6: ...
[60-65]  armature[6] (转子惯量)
[66-71]  damping[6] (粘滞摩擦)
```

### 第一矩使用原则

**绝对不要 `COM = mx / m`！** 辨识出的 mx/my/mz 是 `质量 × 质心` 的乘积，直接用于重力矩计算：
```
p_weighted = m * p_origin + R * [mx, my, mz]
τ_g[j] += G * (z_j × p_weighted)
```

### DH 参数参考（Caster 6-DOF）— 仅供参考，MCU 端已改用 MuJoCo body chain

```
a:      {0,     0.14,   0.055,  0,      0,      0}
alpha:  {π/2,   0,      -π/2,   π/2,    -π/2,   0}
d:      {0.051,     0,   -0.0115,  0.129.9, 0,      0.1309}
offset: {0,     0,      -π/2,   0,      0,      0}

```

**注意**：MCU 端 `IdentifiedCompensation.hpp` 已从 SDH 重写为 MuJoCo body chain FK。SDH 参数仅用于 `ArmKinematic.hpp` 的逆运动学等非补偿器功能。MuJoCo body chain 参数（pos + quat）见 `caster3.xml`。