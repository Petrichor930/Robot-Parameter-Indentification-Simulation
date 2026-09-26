/**
 * fk_compare.cpp — FK comparison: MuJoCo XML body chain vs SDH (casterMdh)
 *
 * Verifies that the SDH parameters used by IdentifiedCompensator produce
 * the same joint origins and z-axes as the MuJoCo XML body chain.
 *
 * Build: cmake -B build && ninja -C build fk_compare
 * Run:   ./build/fk_compare
 */

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <cstdio>
#include <array>
#include <string>

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector4d;

// ─── MuJoCo XML body chain (from caster3.xml) ────────────────────────────────

struct MuJoCoBody {
    std::string name;
    Vector3d pos{0, 0, 0};
    Matrix3d rot{Matrix3d::Identity()};
    Vector3d joint_axis{0, 0, 1};
    bool has_joint = false;
};

struct MuJoCoChain {
    std::array<MuJoCoBody, 7> bodies; // [0]=base_link, [1..6]=Link1..Link6

    MuJoCoChain()
    {
        // base_link: no transform (identity in world frame)
        bodies[0].name = "base_link";
        bodies[0].has_joint = false;

        // Link1: no pos/quat offset from base, joint J1 about local Z
        bodies[1].name = "Link1";
        bodies[1].pos = Vector3d(0, 0, 0);
        bodies[1].rot = Matrix3d::Identity();
        bodies[1].joint_axis = Vector3d(0, 0, 1);
        bodies[1].has_joint = true;

        // Link2: pos="0 0 0.051", quat=(0.707105, 0.707108, 0, 0) = Rx(+90°)
        bodies[2].name = "Link2";
        bodies[2].pos = Vector3d(0, 0, 0.051);
        bodies[2].rot = (Matrix3d() << 1, 0, 0, 0, 0, -1, 0, 1, 0).finished();
        bodies[2].joint_axis = Vector3d(0, 0, 1);
        bodies[2].has_joint = true;

        // Link3: pos="0.14 0 0", quat=identity
        bodies[3].name = "Link3";
        bodies[3].pos = Vector3d(0.14, 0, 0);
        bodies[3].rot = Matrix3d::Identity();
        bodies[3].joint_axis = Vector3d(0, 0, 1);
        bodies[3].has_joint = true;

        // Link4: pos="0 0.055 -0.0115", quat=(0.499998, -0.5, 0.500002, -0.5)
        bodies[4].name = "Link4";
        bodies[4].pos = Vector3d(0, 0.055, -0.0115);
        bodies[4].rot = quatToMatrix(0.499998, -0.5, 0.500002, -0.5);
        bodies[4].joint_axis = Vector3d(0, 0, 1);
        bodies[4].has_joint = true;

        // Link5: pos="0 0 0.1299", quat=(0.707105, 0.707108, 0, 0) = Rx(+90°)
        bodies[5].name = "Link5";
        bodies[5].pos = Vector3d(0, 0, 0.1299);
        bodies[5].rot = (Matrix3d() << 1, 0, 0, 0, 0, -1, 0, 1, 0).finished();
        bodies[5].joint_axis = Vector3d(0, 0, 1);
        bodies[5].has_joint = true;

        // Link6: no pos, quat=(0.707105, -0.707108, 0, 0) = Rx(-90°)
        bodies[6].name = "Link6";
        bodies[6].pos = Vector3d(0, 0, 0);
        bodies[6].rot = (Matrix3d() << 1, 0, 0, 0, 0, 1, 0, -1, 0).finished();
        bodies[6].joint_axis = Vector3d(0, 0, 1);
        bodies[6].has_joint = true;
    }

    /// Compute 4×4 body transforms for joints 1..6 (transforms[1..6])
    std::array<Matrix4d, 7> computeTransforms(const double q[6]) const
    {
        std::array<Matrix4d, 7> T;
        T[0] = Matrix4d::Identity(); // base_link at world origin

        int ji = 0;
        for (int i = 1; i <= 6; ++i) {
            const auto &b = bodies[i];
            // T_parent_body: parent → body (before joint rotation)
            Matrix4d T_body = Matrix4d::Identity();
            T_body.block<3, 3>(0, 0) = b.rot;
            T_body.block<3, 1>(0, 3) = b.pos;

            if (b.has_joint && ji < 6) {
                // Joint rotation about body-local axis
                double angle = q[ji];
                Matrix3d R_joint =
                    Eigen::AngleAxisd(angle, b.joint_axis).toRotationMatrix();
                Matrix4d T_joint = Matrix4d::Identity();
                T_joint.block<3, 3>(0, 0) = R_joint;

                T[i] = T[i - 1] * T_body * T_joint;
                ++ji;
            } else {
                T[i] = T[i - 1] * T_body;
            }
        }
        return T;
    }

private:
    static Matrix3d quatToMatrix(double w, double x, double y, double z)
    {
        Quaterniond q(w, x, y, z);
        return q.toRotationMatrix();
    }
};

// ─── SDH FK (Standard DH, from casterMdh) ────────────────────────────────────

struct SDHChain {
    // casterMdh parameters
    static constexpr double A[6] = {0, 0.14, 0.055, 0, 0, 0};
    static constexpr double ALPHA[6] = {M_PI / 2, 0, -M_PI / 2, M_PI / 2, -M_PI / 2, 0};
    static constexpr double D[6] = {0, 0, 0.052, 0.0115, 0, 0.131};
    static constexpr double OFFSET[6] = {0, 0, -M_PI / 2, 0, 0, 0};

    /// SDH transform: T = Rz(q+offset) · Tz(d) · Tx(a) · Rx(alpha)
    static Matrix4d dhTransform(double a, double alpha, double d, double theta)
    {
        double ct = cos(theta), st = sin(theta);
        double ca = cos(alpha), sa = sin(alpha);
        Matrix4d T = Matrix4d::Zero();
        T(0, 0) = ct;
        T(0, 1) = -st * ca;
        T(0, 2) = st * sa;
        T(0, 3) = a * ct;
        T(1, 0) = st;
        T(1, 1) = ct * ca;
        T(1, 2) = -ct * sa;
        T(1, 3) = a * st;
        T(2, 1) = sa;
        T(2, 2) = ca;
        T(2, 3) = d;
        T(3, 3) = 1.0;
        return T;
    }

    /// Compute transforms[0..5] for joints 1..6
    static std::array<Matrix4d, 6> computeTransforms(const double q[6])
    {
        std::array<Matrix4d, 6> T;
        Matrix4d Tcum = Matrix4d::Identity();
        for (int i = 0; i < 6; ++i) {
            double theta = q[i] + OFFSET[i];
            Matrix4d Ti = dhTransform(A[i], ALPHA[i], D[i], theta);
            Tcum = Tcum * Ti;
            T[i] = Tcum;
        }
        return T;
    }
};

// ─── SDH FK variant with A[0]=0 (from IdentifiedCompensator) ─────────────────

struct SDHChainComp {
    // IdentifiedCompensator parameters (A[0]=0 instead of 0.051)
    static constexpr double A[6] = {0, 0.14, 0.055, 0, 0, 0};
    static constexpr double ALPHA[6] = {M_PI / 2, 0, -M_PI / 2, M_PI / 2, -M_PI / 2, 0};
    static constexpr double D[6] = {0, 0, 0.052, 0.0115, 0, 0.131};
    static constexpr double OFFSET[6] = {0, 0, -M_PI / 2, 0, 0, 0};

    static std::array<Matrix4d, 6> computeTransforms(const double q[6])
    {
        std::array<Matrix4d, 6> T;
        Matrix4d Tcum = Matrix4d::Identity();
        for (int i = 0; i < 6; ++i) {
            double theta = q[i] + OFFSET[i];
            Matrix4d Ti = SDHChain::dhTransform(A[i], ALPHA[i], D[i], theta);
            Tcum = Tcum * Ti;
            T[i] = Tcum;
        }
        return T;
    }
};

// ─── Comparison utilities ────────────────────────────────────────────────────

struct JointComparison {
    Vector3d p_mj;   // MuJoCo position
    Vector3d p_sdh;  // SDH position
    Vector3d z_mj;   // MuJoCo z-axis
    Vector3d z_sdh;  // SDH z-axis
    double pos_err;  // position error (m)
    double z_err;    // z-axis angle error (deg)
};

struct EEComparison {
    Vector3d pos_mj;
    Vector3d pos_sdh;
    Matrix3d rot_mj;
    Matrix3d rot_sdh;
    double pos_err;
    double rot_err_deg;
};

double vecAngleDeg(const Vector3d &a, const Vector3d &b)
{
    double c = a.normalized().dot(b.normalized());
    c = std::clamp(c, -1.0, 1.0);
    return std::acos(c) * 180.0 / M_PI;
}

/// Extract z-axis column and position from 4×4 transform
void extractZAxisAndPos(const Matrix4d &T, Vector3d &z, Vector3d &p)
{
    z = T.block<3, 1>(0, 2);
    p = T.block<3, 1>(0, 3);
}

// ─── Test configurations ─────────────────────────────────────────────────────

struct TestConfig {
    std::string name;
    std::array<double, 6> q;
};

std::vector<TestConfig> getTestConfigs()
{
    return {
        {"home (all zeros)", {0, 0, 0, 0, 0, 0}},
        {"typical working pose", {0, -M_PI / 4, -M_PI / 2, 0, 0, 0}},
        {"reaching forward", {M_PI / 4, -M_PI / 3, -M_PI / 2, 0.3, 0, 0}},
        {"left side", {-M_PI / 4, -M_PI / 6, -M_PI / 3, -0.5, 0.2, 0}},
        {"elbow up", {0, -M_PI / 2, -M_PI / 4, M_PI / 4, M_PI / 6, 0}},
        {"extended", {M_PI / 6, -1.0, -1.5, 0.5, -0.3, 0.5}},
        {"MCU homePosition", {0, -2.9, -3.1, 0, 0.1, 0}},
        {"wide reach", {0.5, -1.5, -2.0, 1.0, 0.5, 1.0}},
        {"symmetric", {-0.5, -0.5, -0.5, -0.5, -0.5, -0.5}},
        {"folded", {1.0, -2.0, -2.5, 0, 0, 0}},
        {"q3 at DH offset", {0, 0, 0, 0, 0, 0}},  // q3=0 → SDH uses offset=-π/2
        {"J4 negative", {0, -1.0, -1.5, -1.0, 0, 0}},
        {"all positive", {0.5, -0.5, -0.5, 0.5, 0.5, 0.5}},
    };
}

// ─── Main ────────────────────────────────────────────────────────────────────

int main()
{
    printf("=== FK Comparison: MuJoCo XML body chain vs SDH (casterMdh) ===\n\n");

    MuJoCoChain mj_chain;
    const auto configs = getTestConfigs();

    int total_joints = 0;
    int joints_pos_ok = 0;
    int joints_z_ok = 0;
    int ee_ok = 0;

    for (size_t ci = 0; ci < configs.size(); ++ci) {
        const auto &cfg = configs[ci];
        printf("────────────────────────────────────────────────────────\n");
        printf("Config %zu: %s\n", ci + 1, cfg.name.c_str());
        printf("  q = {%.4f, %.4f, %.4f, %.4f, %.4f, %.4f}\n", cfg.q[0], cfg.q[1],
               cfg.q[2], cfg.q[3], cfg.q[4], cfg.q[5]);

        // MuJoCo FK
        auto T_mj = mj_chain.computeTransforms(cfg.q.data());
        // SDH FK (casterMdh: a[0]=0.051)
        auto T_sdh = SDHChain::computeTransforms(cfg.q.data());
        // SDH FK (Compensator: A[0]=0)
        auto T_comp = SDHChainComp::computeTransforms(cfg.q.data());

        printf("\n  Joint-by-joint comparison (MuJoCo vs SDH casterMdh):\n");
        printf("  %-8s  %-28s  %-28s  %-10s  %-28s  %-28s  %-10s\n", "Joint",
               "p_mj (x,y,z)", "p_sdh (x,y,z)", "Δp (mm)",
               "z_mj (x,y,z)", "z_sdh (x,y,z)", "Δz (deg)");
        printf("  %-8s  %-28s  %-28s  %-10s  %-28s  %-28s  %-10s\n", "─────",
               "────────────────────────────", "────────────────────────────",
               "──────────", "────────────────────────────",
               "────────────────────────────", "──────────");

        for (int j = 0; j < 6; ++j) {
            Vector3d z_mj, p_mj, z_sdh, p_sdh;
            // MuJoCo: joint j is at body j+1 transform (body[1]=Link1=J1)
            extractZAxisAndPos(T_mj[j + 1], z_mj, p_mj);
            extractZAxisAndPos(T_sdh[j], z_sdh, p_sdh);

            double pos_err_mm = (p_mj - p_sdh).norm() * 1000.0;
            double z_err_deg = vecAngleDeg(z_mj, z_sdh);

            printf("  J%-7d  (%+8.4f,%+8.4f,%+8.4f)  (%+8.4f,%+8.4f,%+8.4f)  %8.3f  "
                   "(%+7.4f,%+7.4f,%+7.4f)  (%+7.4f,%+7.4f,%+7.4f)  %8.4f\n",
                   j + 1, p_mj.x(), p_mj.y(), p_mj.z(), p_sdh.x(), p_sdh.y(), p_sdh.z(),
                   pos_err_mm, z_mj.x(), z_mj.y(), z_mj.z(), z_sdh.x(), z_sdh.y(),
                   z_sdh.z(), z_err_deg);

            total_joints++;
            if (pos_err_mm < 0.5)
                joints_pos_ok++;
            if (z_err_deg < 0.1)
                joints_z_ok++;
        }

        // End-effector comparison
        Vector3d ee_pos_mj = T_mj[6].block<3, 1>(0, 3);
        Vector3d ee_pos_sdh = T_sdh[5].block<3, 1>(0, 3);
        Matrix3d ee_rot_mj = T_mj[6].block<3, 3>(0, 0);
        Matrix3d ee_rot_sdh = T_sdh[5].block<3, 3>(0, 0);

        double ee_pos_err_mm = (ee_pos_mj - ee_pos_sdh).norm() * 1000.0;
        Matrix3d R_diff = ee_rot_mj.transpose() * ee_rot_sdh;
        double ee_rot_err_deg = std::acos(std::clamp((R_diff.trace() - 1.0) / 2.0, -1.0, 1.0)) * 180.0 / M_PI;

        printf("\n  End-effector:\n");
        printf("    p_mj  = (%+.6f, %+.6f, %+.6f)\n", ee_pos_mj.x(), ee_pos_mj.y(),
               ee_pos_mj.z());
        printf("    p_sdh = (%+.6f, %+.6f, %+.6f)\n", ee_pos_sdh.x(), ee_pos_sdh.y(),
               ee_pos_sdh.z());
        printf("    Δp    = %.3f mm\n", ee_pos_err_mm);
        printf("    Δrot  = %.4f deg\n", ee_rot_err_deg);

        if (ee_pos_err_mm < 1.0 && ee_rot_err_deg < 0.5)
            ee_ok++;

        // Also show IdentifiedCompensator variant (A[0]=0)
        printf("\n  IdentifiedCompensator (A[0]=0) vs MuJoCo:\n");
        for (int j = 0; j < 6; ++j) {
            Vector3d z_mj, p_mj, z_comp, p_comp;
            extractZAxisAndPos(T_mj[j + 1], z_mj, p_mj);
            extractZAxisAndPos(T_comp[j], z_comp, p_comp);
            double pos_err_mm = (p_mj - p_comp).norm() * 1000.0;
            double z_err_deg = vecAngleDeg(z_mj, z_comp);
            printf("    J%d: Δp=%.3f mm, Δz=%.4f deg\n", j + 1, pos_err_mm, z_err_deg);
        }
    }

    // ─── Summary ──────────────────────────────────────────────────────────
    printf("\n════════════════════════════════════════════════════════\n");
    printf("SUMMARY\n");
    printf("  Joint origins (Δp < 0.5mm):  %d / %d\n", joints_pos_ok, total_joints);
    printf("  Joint z-axes  (Δz < 0.1°):   %d / %d\n", joints_z_ok, total_joints);
    printf("  End-effector  (Δp<1mm, ΔR<0.5°): %d / %zu\n", ee_ok, configs.size());

    if (joints_pos_ok == total_joints && joints_z_ok == total_joints) {
        printf("\n  ✅ SDH (casterMdh) matches MuJoCo XML body chain.\n");
    } else {
        printf("\n  ❌ MISMATCH detected. Check DH parameters.\n");
    }

    return 0;
}