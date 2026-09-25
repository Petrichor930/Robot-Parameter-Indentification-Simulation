#include "mujoco_caster_dynamics.hpp"

namespace mujoco_dynamics {

MuJoCoCasterDynamics::MuJoCoCasterDynamics() { initBodies(); }

void MuJoCoCasterDynamics::initBodies() {
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
  bodies_[2].mass = 0.48000;
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
  bodies_[3].mass = 0.85600;
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
  bodies_[4].mass = 0.34500;
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
  bodies_[5].mass = 0.32400;
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

Matrix3d MuJoCoCasterDynamics::skew(const Vector3d &v) {
  Matrix3d S;
  S << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0.0;
  return S;
}

Matrix4d MuJoCoCasterDynamics::poseToTransform(const Vector3d &pos,
                                              const Quaterniond &quat) {
  Matrix4d T = Matrix4d::Identity();
  T.block<3, 3>(0, 0) = quat.normalized().toRotationMatrix();
  T.block<3, 1>(0, 3) = pos;
  return T;
}

std::vector<Matrix4d>
MuJoCoCasterDynamics::computeBodyTransforms(const VectorXd &q) const {
  std::vector<Matrix4d> transforms(N_BODIES);
  transforms[0] = poseToTransform(bodies_[0].pos, bodies_[0].quat);

  for (std::size_t i = 1; i < N_BODIES; ++i) {
    Matrix4d T_parent_body = poseToTransform(bodies_[i].pos, bodies_[i].quat);
    Quaterniond joint_rot(Eigen::AngleAxisd(q(static_cast<Eigen::Index>(i - 1)),
                                            bodies_[i].joint_axis));
    Matrix4d T_joint = Matrix4d::Identity();
    T_joint.block<3, 3>(0, 0) = joint_rot.toRotationMatrix();
    transforms[i] = transforms[i - 1] * T_parent_body * T_joint;
  }

  return transforms;
}

Vector3d MuJoCoCasterDynamics::computeBodyCOM(std::size_t body_idx,
                                             const VectorXd &q) const {
  const auto transforms = computeBodyTransforms(q);
  const Matrix4d &T = transforms[body_idx];
  return T.block<3, 3>(0, 0) * bodies_[body_idx].com + T.block<3, 1>(0, 3);
}

MatrixXd MuJoCoCasterDynamics::computeBodyJacobian(std::size_t body_idx,
                                                  const VectorXd &q) const {
  MatrixXd J = MatrixXd::Zero(6, N_DOF);
  const auto transforms = computeBodyTransforms(q);
  const Vector3d p_body = computeBodyCOM(body_idx, q);

  for (std::size_t i = 1; i <= body_idx && i < N_BODIES; ++i) {
    const Vector3d z_axis =
        transforms[i].block<3, 3>(0, 0) * bodies_[i].joint_axis;
    const Vector3d p_joint = transforms[i].block<3, 1>(0, 3);
    const Eigen::Index col = static_cast<Eigen::Index>(i - 1);
    J.block<3, 1>(0, col) = z_axis.cross(p_body - p_joint);
    J.block<3, 1>(3, col) = z_axis;
  }

  return J;
}

Eigen::Matrix<double, 6, 6>
MuJoCoCasterDynamics::computeSpatialInertia(std::size_t body_idx) const {
  const auto &body = bodies_[body_idx];
  const Vector3d c = body.com;
  const double m = body.mass;

  Matrix3d I_com;
  I_com << body.Ixx, body.Ixy, body.Ixz, body.Ixy, body.Iyy, body.Iyz,
      body.Ixz, body.Iyz, body.Izz;

  const Matrix3d c_skew = skew(c);
  const Matrix3d I_origin = I_com - m * c_skew * c_skew;

  Eigen::Matrix<double, 6, 6> M_spatial;
  M_spatial.setZero();
  M_spatial.block<3, 3>(0, 0) = m * Matrix3d::Identity();
  M_spatial.block<3, 3>(0, 3) = m * c_skew.transpose();
  M_spatial.block<3, 3>(3, 0) = m * c_skew;
  M_spatial.block<3, 3>(3, 3) = I_origin;
  return M_spatial;
}

MatrixXd MuJoCoCasterDynamics::computeInertiaMatrix(const VectorXd &q) const {
  MatrixXd M = MatrixXd::Zero(N_DOF, N_DOF);

  for (std::size_t i = 1; i < N_BODIES; ++i) {
    const MatrixXd J_i = computeBodyJacobian(i, q);
    M += J_i.transpose() * computeSpatialInertia(i) * J_i;
  }

  for (std::size_t i = 0; i < N_DOF; ++i) {
    M(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(i)) +=
        bodies_[i + 1].armature;
  }

  return (M + M.transpose()) / 2.0;
}

MatrixXd MuJoCoCasterDynamics::computeCoriolisMatrix(const VectorXd &q,
                                                    const VectorXd &qd) const {
  const double eps = 1e-7;
  MatrixXd C = MatrixXd::Zero(N_DOF, N_DOF);

  for (std::size_t k = 0; k < N_DOF; ++k) {
    VectorXd q_plus = q;
    VectorXd q_minus = q;
    q_plus(static_cast<Eigen::Index>(k)) += eps;
    q_minus(static_cast<Eigen::Index>(k)) -= eps;
    const MatrixXd dM_dqk =
        (computeInertiaMatrix(q_plus) - computeInertiaMatrix(q_minus)) /
        (2.0 * eps);
    C += 0.5 * dM_dqk * qd(static_cast<Eigen::Index>(k));
  }

  return C;
}

VectorXd MuJoCoCasterDynamics::computeGravityVector(const VectorXd &q) const {
  VectorXd G = VectorXd::Zero(N_DOF);

  for (std::size_t i = 1; i < N_BODIES; ++i) {
    const MatrixXd J_i = computeBodyJacobian(i, q);
    G -= J_i.topRows(3).transpose() * (bodies_[i].mass * gravity_);
  }

  return G;
}

VectorXd MuJoCoCasterDynamics::computeInverseDynamics(const VectorXd &q,
                                                     const VectorXd &qd,
                                                     const VectorXd &qdd) const {
  const MatrixXd M = computeInertiaMatrix(q);
  const MatrixXd C = computeCoriolisMatrix(q, qd);
  const VectorXd g = computeGravityVector(q);

  VectorXd damping_force = VectorXd::Zero(N_DOF);
  for (std::size_t i = 0; i < N_DOF; ++i) {
    damping_force(static_cast<Eigen::Index>(i)) =
        bodies_[i + 1].damping * qd(static_cast<Eigen::Index>(i));
  }

  return M * qdd + C * qd + g - damping_force;
}

} // namespace mujoco_dynamics
