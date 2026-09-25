/**
 * @file mujoco_caster_regressor.cpp
 * @brief 基于 MuJoCo 运动学的 Caster 机器人回归矩阵计算实现
 */

#include "mujoco_caster_regressor.hpp"
#include <iostream>

namespace mujoco_dynamics {

// ============================================================================
// MuJoCoCasterRegressor 构造函数
// ============================================================================

MuJoCoCasterRegressor::MuJoCoCasterRegressor() { initBodies(); }

void MuJoCoCasterRegressor::initBodies() {
  // base_link (link=0.36445kg + motor=0.33kg = 0.69445kg)
  bodies_[0].name = "base_link";
  bodies_[0].quat = Quaterniond(0.707107, -0.707107, 0, 0);
  bodies_[0].mass = 0.69445;
  bodies_[0].com = Vector3d(0.0007243342069, -0.0006176415941, -0.0007868908201);
  bodies_[0].Ixx = 0.0014801;
  bodies_[0].Iyy = 0.0017385;
  bodies_[0].Izz = 0.003218;
  bodies_[0].Ixy = 0;
  bodies_[0].Ixz = 0;
  bodies_[0].Iyz = 0;
  bodies_[0].has_joint = false;

  // Link1 (link=0.04981kg + motor=0.33kg = 0.37981kg)
  bodies_[1].name = "Link1";
  bodies_[1].pos = Vector3d(0.065, 0.0055, 0.061078);
  bodies_[1].quat = Quaterniond(0.499998, 0.5, 0.5, -0.500002);
  bodies_[1].mass = 0.37981;
  bodies_[1].com = Vector3d(4.567283647e-06, -0.03596375972, 0.04973514334);
  bodies_[1].Ixx = 5.006274946e-05;
  bodies_[1].Iyy = 5.252226745e-05;
  bodies_[1].Izz = 5.491786233e-05;
  bodies_[1].Ixy = -1.002105787e-08;
  bodies_[1].Ixz = -4.200281484e-09;
  bodies_[1].Iyz = 5.591104523e-06;
  bodies_[1].has_joint = true;

  // Link2 (link=0.15000kg + motor=0.33kg = 0.48000kg)
  bodies_[2].name = "Link2";
  bodies_[2].pos = Vector3d(0, -0.0405, 0.051);
  bodies_[2].quat = Quaterniond(0.707105, 0.707108, 0, 0);
  bodies_[2].mass = 0.15000;
  bodies_[2].com = Vector3d(0.1181465625, -2.89639375e-08, 0.01500084375);
  bodies_[2].Ixx = 1.849692314e-05;
  bodies_[2].Iyy = 0.0002242325554;
  bodies_[2].Izz = 0.0002337932893;
  bodies_[2].Ixy = 0;
  bodies_[2].Ixz = 7.15118426e-09;
  bodies_[2].Iyz = 0;
  bodies_[2].has_joint = true;

  // Link3 (link=0.70600kg + motor=0.15kg = 0.85600kg)
  bodies_[3].name = "Link3";
  bodies_[3].pos = Vector3d(0.14, 0, 0);
  bodies_[3].quat = Quaterniond(1, 0, 0, 0);
  bodies_[3].mass = 0.70600;
  bodies_[3].com = Vector3d(0.01097714533, 0.03424524953, -0.0142572235);
  bodies_[3].Ixx = 0.0001407486393;
  bodies_[3].Iyy = 0.0002312706286;
  bodies_[3].Izz = 0.0002280238399;
  bodies_[3].Ixy = 3.676811989e-06;
  bodies_[3].Ixz = 1.502064617e-06;
  bodies_[3].Iyz = 1.251930986e-05;
  bodies_[3].has_joint = true;

  // Link4 (link=0.19500kg + motor=0.15kg = 0.34500kg)
  bodies_[4].name = "Link4";
  bodies_[4].pos = Vector3d(0, 0.055, -0.052);
  bodies_[4].quat = Quaterniond(0.499998, -0.5, 0.500002, -0.5);
  bodies_[4].mass = 0.19500;
  bodies_[4].com = Vector3d(1.927351739e-05, 0.001791914348, 0.1265872609);
  bodies_[4].Ixx = 2.056062295e-05;
  bodies_[4].Iyy = 2.046667585e-05;
  bodies_[4].Izz = 1.482324858e-05;
  bodies_[4].Ixy = -2.717049414e-09;
  bodies_[4].Ixz = -4.311138767e-09;
  bodies_[4].Iyz = -1.897821059e-06;
  bodies_[4].has_joint = true;

  // Link5 (link=0.17400kg + motor=0.15kg = 0.32400kg)
  bodies_[5].name = "Link5";
  bodies_[5].pos = Vector3d(0, 0, 0.1299);
  bodies_[5].quat = Quaterniond(0.707105, 0.707108, 0, 0);
  bodies_[5].mass = 0.20700;
  bodies_[5].com = Vector3d(2.229095741e-05, 0.02488656481, 0.002743963889);
  bodies_[5].Ixx = 1.906853156e-05;
  bodies_[5].Iyy = 1.375311978e-05;
  bodies_[5].Izz = 1.270412739e-05;
  bodies_[5].Ixy = -2.948905512e-08;
  bodies_[5].Ixz = 2.943545303e-08;
  bodies_[5].Iyz = 6.172689611e-06;
  bodies_[5].has_joint = true;

  // Link6 (link=0.02600kg + motor=0kg = 0.02600kg)
  bodies_[6].name = "Link6";
  bodies_[6].pos = Vector3d(0, 0, 0);
  bodies_[6].quat = Quaterniond(0.707105, -0.707108, 0, 0);
  bodies_[6].mass = 0.02600;
  bodies_[6].com = Vector3d(-0.000249497, -0.000229533, 0.100505);
  bodies_[6].Ixx = 1.769993054e-06;
  bodies_[6].Iyy = 1.772660687e-06;
  bodies_[6].Izz = 4.193943671e-07;
  bodies_[6].Ixy = -1.541162505e-08;
  bodies_[6].Ixz = -1.844751313e-08;
  bodies_[6].Iyz = -1.690750795e-08;
  bodies_[6].has_joint = true;

  // Joint axes and parameters
  bodies_[1].joint_axis = Vector3d(0, 0, 1);
  bodies_[2].joint_axis = Vector3d(0, 0, -1);
  bodies_[3].joint_axis = Vector3d(0, 0, -1);
  bodies_[4].joint_axis = Vector3d(0, 0, 1);
  bodies_[5].joint_axis = Vector3d(0, 0, 1);
  bodies_[6].joint_axis = Vector3d(0, 0, 1);

  for (std::size_t i = 1; i < N_BODIES; ++i) {
    bodies_[i].armature = 0.1;
    bodies_[i].damping = 1.0;
  }
}

// ============================================================================
// 辅助函数
// ============================================================================

MuJoCoCasterRegressor::Matrix3d MuJoCoCasterRegressor::skew(const Vector3d &v) {
  Matrix3d S;
  S << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return S;
}

MuJoCoCasterRegressor::Matrix4d
MuJoCoCasterRegressor::poseToTransform(const Vector3d &pos, const Quaterniond &quat) {
  Matrix4d T = Matrix4d::Identity();
  T.block<3, 3>(0, 0) = quat.toRotationMatrix();
  T.block<3, 1>(0, 3) = pos;
  return T;
}

// ============================================================================
// 运动学
// ============================================================================

std::vector<MuJoCoCasterRegressor::Matrix4d>
MuJoCoCasterRegressor::computeBodyTransforms(const VectorXd &q) const {
  std::vector<Matrix4d> transforms(N_BODIES + 1);

  // link0 在世界坐标系
  transforms[0] = poseToTransform(bodies_[0].pos, bodies_[0].quat);

  std::size_t joint_idx = 0;

  for (std::size_t i = 1; i <= N_BODIES; ++i) {
    const auto &body = bodies_[i];

    // 相对于父 body 的基础变换
    Matrix4d T_parent_body = poseToTransform(body.pos, body.quat);

    // 如果有关节，添加关节旋转
    if (body.has_joint && joint_idx < N_DOF) {
      double angle = q(joint_idx);
      Quaterniond joint_rot =
          Quaterniond(Eigen::AngleAxisd(angle, body.joint_axis));

      Matrix4d T_joint = Matrix4d::Identity();
      T_joint.block<3, 3>(0, 0) = joint_rot.toRotationMatrix();

      transforms[i] = transforms[i - 1] * T_parent_body * T_joint;
      ++joint_idx;
    } else {
      transforms[i] = transforms[i - 1] * T_parent_body;
    }
  }

  return transforms;
}

MuJoCoCasterRegressor::Vector3d
MuJoCoCasterRegressor::computeBodyCOM(std::size_t body_idx, const VectorXd &q) const {
  auto transforms = computeBodyTransforms(q);
  Vector3d com_local = bodies_[body_idx].com;
  Matrix4d T = transforms[body_idx];
  return T.block<3, 3>(0, 0) * com_local + T.block<3, 1>(0, 3);
}

/**
 * @brief 计算 Body 原点的雅可比矩阵 (用于回归矩阵)
 *
 * 注意：这与 computeBodyJacobian 不同！
 * - computeBodyJacobian: 计算 COM 的雅可比 (用于动力学)
 * - computeBodyOriginJacobian: 计算 Body 原点的雅可比 (用于回归矩阵)
 *
 * 回归矩阵使用标准惯性参数 (m, mc, I_origin)，惯量定义在 Body 原点
 */
MuJoCoCasterRegressor::MatrixXd
MuJoCoCasterRegressor::computeBodyOriginJacobian(std::size_t body_idx,
                                           const VectorXd &q) const {
  MatrixXd J = MatrixXd::Zero(6, N_DOF);

  auto transforms = computeBodyTransforms(q);

  // Body 原点位置 (不是 COM！)
  Vector3d p_origin = transforms[body_idx].block<3, 1>(0, 3);

  // 对每个关节
  std::size_t joint_count = 0;
  for (std::size_t i = 1; i <= body_idx && i <= N_BODIES; ++i) {
    if (bodies_[i].has_joint) {
      // 关节轴在世界坐标系中的方向
      Vector3d z_axis = transforms[i].block<3, 3>(0, 0) * bodies_[i].joint_axis;

      // 关节原点位置
      Vector3d p_joint = transforms[i].block<3, 1>(0, 3);

      // 线速度雅可比: z × (p_origin - p_joint)
      J.block<3, 1>(0, joint_count) = z_axis.cross(p_origin - p_joint);

      // 角速度雅可比: z
      J.block<3, 1>(3, joint_count) = z_axis;

      ++joint_count;
    }
  }

  return J;
}

MuJoCoCasterRegressor::MatrixXd
MuJoCoCasterRegressor::computeBodyJacobian(std::size_t body_idx,
                                     const VectorXd &q) const {
  MatrixXd J = MatrixXd::Zero(6, N_DOF);

  auto transforms = computeBodyTransforms(q);
  Vector3d p_body = computeBodyCOM(body_idx, q);

  // 对每个关节
  std::size_t joint_count = 0;
  for (std::size_t i = 1; i <= body_idx && i <= N_BODIES; ++i) {
    if (bodies_[i].has_joint) {
      // 关节轴在世界坐标系中的方向
      Vector3d z_axis = transforms[i].block<3, 3>(0, 0) * bodies_[i].joint_axis;

      // 关节原点位置
      Vector3d p_joint = transforms[i].block<3, 1>(0, 3);

      // 线速度雅可比: z × (p_body - p_joint)
      J.block<3, 1>(0, joint_count) = z_axis.cross(p_body - p_joint);

      // 角速度雅可比: z
      J.block<3, 1>(3, joint_count) = z_axis;

      ++joint_count;
    }
  }

  return J;
}

MuJoCoCasterRegressor::MatrixXd MuJoCoCasterRegressor::computeBodyOriginJacobianDerivative(
    std::size_t body_idx, const VectorXd &q, const VectorXd &qd) const {
  // 数值微分
  const double eps = 1e-7;
  VectorXd q_plus = q + eps * qd;
  VectorXd q_minus = q - eps * qd;

  MatrixXd J_plus = computeBodyOriginJacobian(body_idx, q_plus);
  MatrixXd J_minus = computeBodyOriginJacobian(body_idx, q_minus);

  return (J_plus - J_minus) / (2.0 * eps);
}

MuJoCoCasterRegressor::MatrixXd MuJoCoCasterRegressor::computeBodyJacobianDerivative(
    std::size_t body_idx, const VectorXd &q, const VectorXd &qd) const {
  // 数值微分
  const double eps = 1e-7;
  VectorXd q_plus = q + eps * qd;
  VectorXd q_minus = q - eps * qd;

  MatrixXd J_plus = computeBodyJacobian(body_idx, q_plus);
  MatrixXd J_minus = computeBodyJacobian(body_idx, q_minus);

  return (J_plus - J_minus) / (2.0 * eps);
}

// ============================================================================
// 参数向量
// ============================================================================

std::size_t MuJoCoCasterRegressor::numParameters(MuJoCoParamFlags flags) const {
  std::size_t params = N_BODIES * MuJoCoInertialParams::PARAMS_PER_BODY;

  if (hasFlag(flags, MuJoCoParamFlags::ARMATURE)) {
    params += N_DOF;
  }

  if (hasFlag(flags, MuJoCoParamFlags::DAMPING)) {
    params += N_DOF;
  }

  return params;
}

MuJoCoCasterRegressor::VectorXd
MuJoCoCasterRegressor::computeParameterVector(MuJoCoParamFlags flags) const {
  const std::size_t num_params = numParameters(flags);
  VectorXd theta = VectorXd::Zero(num_params);

  // 填充惯性参数 (link1-6)
  for (std::size_t i = 0; i < N_BODIES; ++i) {
    std::size_t body_idx = i + 1;
    auto sip = MuJoCoInertialParams::fromMuJoCoBody(bodies_[body_idx]);
    auto sip_vec = sip.toVector();

    std::size_t offset = i * MuJoCoInertialParams::PARAMS_PER_BODY;
    theta.segment(offset, MuJoCoInertialParams::PARAMS_PER_BODY) = sip_vec;
  }

  std::size_t current_offset = N_BODIES * MuJoCoInertialParams::PARAMS_PER_BODY;

  // Armature 参数
  if (hasFlag(flags, MuJoCoParamFlags::ARMATURE)) {
    for (std::size_t i = 0; i < N_DOF; ++i) {
      theta(current_offset + i) = bodies_[i + 1].armature;
    }
    current_offset += N_DOF;
  }

  // Damping 参数
  if (hasFlag(flags, MuJoCoParamFlags::DAMPING)) {
    for (std::size_t i = 0; i < N_DOF; ++i) {
      theta(current_offset + i) = bodies_[i + 1].damping;
    }
  }

  return theta;
}

std::vector<std::string>
MuJoCoCasterRegressor::getParameterNames(MuJoCoParamFlags flags) const {
  std::vector<std::string> names;

  const char *body_names[] = {"Link1", "Link2", "Link3",
                              "Link4", "Link5", "Link6"};
  const char *param_names[] = {"m",   "mx",  "my",  "mz",  "Ixx",
                               "Ixy", "Ixz", "Iyy", "Iyz", "Izz"};

  for (std::size_t i = 0; i < N_BODIES; ++i) {
    for (int j = 0; j < 10; ++j) {
      names.push_back(std::string(body_names[i]) + "_" + param_names[j]);
    }
  }

  if (hasFlag(flags, MuJoCoParamFlags::ARMATURE)) {
    for (std::size_t i = 0; i < N_DOF; ++i) {
      names.push_back("armature_" + std::to_string(i + 1));
    }
  }

  if (hasFlag(flags, MuJoCoParamFlags::DAMPING)) {
    for (std::size_t i = 0; i < N_DOF; ++i) {
      names.push_back("damping_" + std::to_string(i + 1));
    }
  }

  return names;
}

// ============================================================================
// 回归矩阵
// ============================================================================

MuJoCoCasterRegressor::MatrixXd MuJoCoCasterRegressor::computeBodyRegressorBlock(
    std::size_t body_idx, const VectorXd &q, const VectorXd &qd,
    const VectorXd &qdd) const {

  /**
   * 标准惯性参数回归矩阵推导（Body 局部坐标系）
   *
   * 参数向量: θ = [m, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]
   * - (mx, my, mz) = m * (cx, cy, cz) 是一阶矩（局部坐标系）
   * - I_origin 是在 Body 原点的惯量张量（局部坐标系）
   *
   * 动力学方程（在 Body 局部坐标系中）：
   *   f_local = m * (a_local - g_local) + K * mc_local
   *   n_local = [mc_local]× * (a_local - g_local) + I_origin * α_local +
   * [ω_local]× * I_origin * ω_local
   *
   * 其中 K = [α_local]× + [ω_local]× * [ω_local]×
   *
   * 关节扭矩：τ = J_v_local^T * f_local + J_w_local^T * n_local
   */

  auto transforms = computeBodyTransforms(q);
  Matrix3d R = transforms[body_idx].block<3, 3>(0, 0); // Body 到 World 旋转
  Vector3d p_origin = transforms[body_idx].block<3, 1>(0, 3); // Body 原点位置

  // ========== 计算 Body 原点的雅可比 (世界坐标系) ==========
  MatrixXd J_world = MatrixXd::Zero(6, N_DOF);
  std::size_t joint_count = 0;
  for (std::size_t i = 1; i <= body_idx && i <= N_BODIES; ++i) {
    if (bodies_[i].has_joint) {
      Vector3d z_axis = transforms[i].block<3, 3>(0, 0) * bodies_[i].joint_axis;
      Vector3d p_joint = transforms[i].block<3, 1>(0, 3);
      J_world.block<3, 1>(0, joint_count) = z_axis.cross(p_origin - p_joint);
      J_world.block<3, 1>(3, joint_count) = z_axis;
      ++joint_count;
    }
  }

  // 雅可比导数（数值微分）
  const double dt = 1e-7;
  VectorXd q_plus = q + dt * qd;
  auto transforms_plus = computeBodyTransforms(q_plus);
  Vector3d p_origin_plus = transforms_plus[body_idx].block<3, 1>(0, 3);

  MatrixXd J_world_plus = MatrixXd::Zero(6, N_DOF);
  joint_count = 0;
  for (std::size_t i = 1; i <= body_idx && i <= N_BODIES; ++i) {
    if (bodies_[i].has_joint) {
      Vector3d z_axis =
          transforms_plus[i].block<3, 3>(0, 0) * bodies_[i].joint_axis;
      Vector3d p_joint = transforms_plus[i].block<3, 1>(0, 3);
      J_world_plus.block<3, 1>(0, joint_count) =
          z_axis.cross(p_origin_plus - p_joint);
      J_world_plus.block<3, 1>(3, joint_count) = z_axis;
      ++joint_count;
    }
  }
  MatrixXd J_world_dot = (J_world_plus - J_world) / dt;

  Eigen::Matrix<double, 3, Eigen::Dynamic> Jv_world = J_world.topRows(3);
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jw_world = J_world.bottomRows(3);
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jv_dot_world =
      J_world_dot.topRows(3);
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jw_dot_world =
      J_world_dot.bottomRows(3);

  // ========== 世界坐标系中的运动学量 ==========
  Vector3d a_origin_world = Jv_world * qdd + Jv_dot_world * qd;
  Vector3d omega_world = Jw_world * qd;
  Vector3d alpha_world = Jw_world * qdd + Jw_dot_world * qd;

  // ========== 转换到 Body 局部坐标系 ==========
  Matrix3d Rt = R.transpose();
  Vector3d a_local = Rt * a_origin_world;
  Vector3d omega_local = Rt * omega_world;
  Vector3d alpha_local = Rt * alpha_world;
  Vector3d g_local = Rt * gravity_;
  Vector3d b_local = a_local - g_local;

  // 局部坐标系中的雅可比
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jv_local = Rt * Jv_world;
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jw_local = Rt * Jw_world;

  // ========== 构建回归矩阵块 ==========
  MatrixXd Y_block = MatrixXd::Zero(N_DOF, 10);

  // K 矩阵: K = [α]× + [ω]× * [ω]×
  Matrix3d alpha_skew = skew(alpha_local);
  Matrix3d omega_skew = skew(omega_local);
  Matrix3d K = alpha_skew + omega_skew * omega_skew;

  double ox = omega_local(0), oy = omega_local(1), oz = omega_local(2);
  double ax = alpha_local(0), ay = alpha_local(1), az = alpha_local(2);

  // 1. 质量 m: f = m * b, n = 0
  Y_block.col(0) = Jv_local.transpose() * b_local;

  // 2. 一阶矩 (mx, my, mz): f = K * e_i, n = e_i × b
  for (int i = 0; i < 3; ++i) {
    Vector3d e_i = Vector3d::Zero();
    e_i(i) = 1.0;
    Y_block.col(1 + i) = Jv_local.transpose() * K.col(i) +
                         Jw_local.transpose() * e_i.cross(b_local);
  }

  // 3. 惯量张量: n = E * α + [ω]× * E * ω
  Y_block.col(4) = Jw_local.transpose() * Vector3d(ax, ox * oz, -ox * oy);
  Y_block.col(5) = Jw_local.transpose() *
                   Vector3d(ay - ox * oz, ax + oy * oz, ox * ox - oy * oy);
  Y_block.col(6) = Jw_local.transpose() *
                   Vector3d(az + ox * oy, oz * oz - ox * ox, ax - oy * oz);
  Y_block.col(7) = Jw_local.transpose() * Vector3d(-oy * oz, ay, ox * oy);
  Y_block.col(8) = Jw_local.transpose() *
                   Vector3d(oy * oy - oz * oz, az - ox * oy, ay + ox * oz);
  Y_block.col(9) = Jw_local.transpose() * Vector3d(oy * oz, -ox * oz, az);

  return Y_block;
}

MuJoCoCasterRegressor::MatrixXd
MuJoCoCasterRegressor::computeRegressorMatrix(const VectorXd &q, const VectorXd &qd,
                                        const VectorXd &qdd,
                                        MuJoCoParamFlags flags) const {

  const std::size_t num_params = numParameters(flags);
  MatrixXd Y = MatrixXd::Zero(N_DOF, num_params);

  // 计算每个 Body 的回归矩阵块
  for (std::size_t i = 0; i < N_BODIES; ++i) {
    std::size_t body_idx = i + 1;
    MatrixXd Y_body = computeBodyRegressorBlock(body_idx, q, qd, qdd);

    std::size_t offset = i * MuJoCoInertialParams::PARAMS_PER_BODY;
    Y.block(0, offset, N_DOF, MuJoCoInertialParams::PARAMS_PER_BODY) = Y_body;
  }

  std::size_t current_offset = N_BODIES * MuJoCoInertialParams::PARAMS_PER_BODY;

  // Armature 回归: τ_armature = armature * q̈
  if (hasFlag(flags, MuJoCoParamFlags::ARMATURE)) {
    for (std::size_t i = 0; i < N_DOF; ++i) {
      Y(i, current_offset + i) = qdd(i);
    }
    current_offset += N_DOF;
  }

  // Damping 回归: 在 MuJoCo 中 damping 是被动项
  // τ_ctrl = M*q̈ + C*q̇ + G - damping*q̇
  // 所以回归项是 -q̇
  if (hasFlag(flags, MuJoCoParamFlags::DAMPING)) {
    for (std::size_t i = 0; i < N_DOF; ++i) {
      Y(i, current_offset + i) = -qd(i);
    }
  }

  return Y;
}

MuJoCoCasterRegressor::MatrixXd
MuJoCoCasterRegressor::computeObservationMatrix(const MatrixXd &Q, const MatrixXd &Qd,
                                          const MatrixXd &Qdd,
                                          MuJoCoParamFlags flags) const {

  const std::size_t K = Q.cols(); // 样本数
  const std::size_t num_params = numParameters(flags);

  MatrixXd W = MatrixXd::Zero(N_DOF * K, num_params);

#ifdef IDENTIFICATION_USE_OPENMP
#pragma omp parallel for schedule(static)
#endif
  for (Eigen::Index k = 0; k < static_cast<Eigen::Index>(K); ++k) {
    VectorXd q = Q.col(k);
    VectorXd qd = Qd.col(k);
    VectorXd qdd = Qdd.col(k);

    MatrixXd Y_k = computeRegressorMatrix(q, qd, qdd, flags);
    W.block(k * static_cast<Eigen::Index>(N_DOF), 0,
            static_cast<Eigen::Index>(N_DOF),
            static_cast<Eigen::Index>(num_params)) = Y_k;
  }

  return W;
}

} // namespace mujoco_dynamics
