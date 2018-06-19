#ifndef MODULE_MOTION_HUMANOIDMODEL_H
#define MODULE_MOTION_HUMANOIDMODEL_H

#include <Eigen/Dense>

namespace module {
namespace motion {

    static constexpr double EPSILON = 0.0000001;

    /**
     * All combinations
     * of Euler angles types
     * in same order as rotation application
     *
     * EulerYawPitchRoll is built as
     * Roll * Pitch * Yaw.
     */
    enum EulerType {
        EulerYawPitchRoll,
        EulerYawRollPitch,
        EulerRollPitchYaw,
        EulerRollYawPitch,
        EulerPitchRollYaw,
        EulerPitchYawRoll,
    };

    /**
     * Leg motor result positions
     */
    struct IKWalkOutputs {
        double left_hip_yaw;
        double left_hip_roll;
        double left_hip_pitch;
        double left_knee;
        double left_ankle_pitch;
        double left_ankle_roll;
        double right_hip_yaw;
        double right_hip_roll;
        double right_hip_pitch;
        double right_knee;
        double right_ankle_pitch;
        double right_ankle_roll;
    };

    struct Position {
        Position() : theta(Eigen::Matrix<double, 6, 1>::Zero()) {}
        Position(const Eigen::Matrix<double, 6, 1>& theta) : theta(theta) {}
        Position(double hip_yaw,
                 double hip_roll,
                 double hip_pitch,
                 double knee,
                 double ankle_pitch,
                 double ankle_roll) {
            theta << hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll;
        }

        Eigen::Matrix<double, 6, 1> theta;
    };

    struct Frame3D {
    private:
        std::vector<Eigen::Vector3d> frame;

    public:
        /* canonical frame */
        Frame3D() : frame({{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}) {}
        Frame3D(double psi, double theta, double phi) {
            /* Ordre habituel des angles d'euler : precession, nutation et rotation propre */
            frame.emplace_back(std::cos(phi) * std::cos(psi) - std::sin(phi) * std::cos(theta) * std::sin(psi),
                               std::cos(phi) * std::sin(psi) + std::sin(phi) * std::cos(theta) * std::cos(psi),
                               std::sin(phi) * std::sin(theta));
            frame.emplace_back(-std::sin(phi) * std::cos(psi) - std::cos(phi) * std::cos(theta) * std::sin(psi),
                               -std::sin(phi) * std::sin(psi) + std::cos(phi) * std::cos(theta) * std::cos(psi),
                               std::cos(phi) * std::sin(theta));
            frame.emplace_back(std::sin(theta) * std::sin(psi), -std::sin(theta) * std::cos(psi), std::cos(theta));
        }
        Frame3D(const Eigen::Vector3d& e1, const Eigen::Vector3d& e2, const Eigen::Vector3d& e3)
            : frame({e1, e2, e3}) {}
        Frame3D(const Eigen::Matrix3d& mat) : frame({mat.leftCols<1>(), mat.middleCols<1>(1), mat.rightCols<1>()}) {}

        Eigen::Vector3d& operator[](size_t pos) {
            return frame[pos];
        }
        const Eigen::Vector3d& operator[](size_t pos) const {
            return frame[pos];
        }
    };

    class IK {
        Eigen::Vector3d L;

    public:
        IK(double L0, double L1, double L2) : L(L0, L1, L2) {}
        IK(const Eigen::Vector3d& L) : L(L) {}

        bool compute(Eigen::Vector3d C, Frame3D orientation, Position& result) {
            if (utility::math::almost_equal(L.x(), 0.0, EPSILON) || utility::math::almost_equal(L.y(), 0.0, EPSILON)) {
                return false;
            }

            /* step 1 : calcul de B */
            Eigen::Vector3d B = C + L.z() * orientation[2];
            double B_len      = B.norm();
            if (B.z() >= 0 || utility::math::almost_equal(B_len, 0.0, EPSILON)) {
                return false;
            }

            /* step 2 : calcul de phi */
            Eigen::Vector3d phi;
            if (!utility::math::almost_equal(orientation[0].z(), 0.0, EPSILON)) {
                double a_pp = 1.0;
                double b_pp = -B.z() / orientation[0].z();
                phi         = (a_pp * B) + (b_pp * orientation[0]);
                phi.normalize();
            }

            else {
                phi = orientation[0];
                phi.normalize();
            }

            /* phi est orienté vers l'avant ou sur la gauche */
            if (!(phi.x() > 0 || (utility::math::almost_equal(phi.x(), 0.0, EPSILON) && phi.y() >= 0))) {
                phi = -1.0 * phi;
            }

            /* step 3 : calcul de \theta_0 */
            result.theta[0] = std::atan2(phi[1], phi[0]);

            /* step 4 : calcul de G */
            Eigen::Vector3d G = B.dot(phi) * phi;

            /* step 5 : calcul de \theta_1 */
            Eigen::Vector3d zeta = -1.0 * phi.cross(Eigen::Vector3d::UnitZ());
            result.theta[1]      = std::atan2((B - G).dot(zeta), -B.z());

            /* step 6 : calcul de \theta_3 */
            double q = (L.x() * L.x() + L.y() * L.y() - B_len * B_len) / (2.0 * L.x() * L.y());
            if (q < (-1.0 - EPSILON) || q > (1.0 + EPSILON)) {
                return false;
            }
            utility::math::clamp(-1.0, q, 1.0);
            result.theta[3] = M_PI - std::acos(q);

            /* step 7 : calcul de \omega */
            Eigen::Vector3d omega(-std::sin(result.theta[0]) * std::sin(result.theta[1]),
                                  std::cos(result.theta[0]) * std::sin(result.theta[1]),
                                  -std::cos(result.theta[1]));

            /* step 8 : calcul de alpha */
            q = B.dot(omega) / B_len;
            utility::math::clamp(-1.0, q, 1.0); /* on a toujours |q| <= 1 */
            double alpha = utility::math::sign(B.cross(omega).dot(zeta)) * std::acos(q);

            /* step 9 : calcul de l'angle (A \Omega B) */
            q = (L.x() * L.x() + B_len * B_len - L.y() * L.y()) / (2.0 * L.x() * B_len);
            if (q < (-1.0 - EPSILON) || q > (1.0 + EPSILON)) {
                return false;
            }
            utility::math::clamp(-1.0, q, 1.0);
            double A_omega_B = std::acos(q);

            /* step 10 : calcul de theta_2 */
            result.theta[2] = alpha + A_omega_B;

            /* step 11 : calcul de theta_4 */
            q = phi.dot(orientation[0]);
            utility::math::clamp(-1.0, q, 1.0);
            double beta     = -utility::math::sign(phi.cross(orientation[0]).dot(zeta)) * std::acos(q);
            result.theta[4] = beta + result.theta[3] - result.theta[2];

            /* step 12 : calcul de theta_5 */
            Eigen::Vector3d tau = phi.cross(omega);
            q                   = tau.dot(orientation[1]);
            utility::math::clamp(-1.0, q, 1.0);
            result.theta[5] = utility::math::sign(tau.cross(orientation[1]).dot(orientation[0])) * std::acos(q);

            return true;
        }
    };

    /**
     * HumanoidModel
     */
    class HumanoidModel {
    public:
        /**
         * Initialize the model with given
         * typical leg length
         */
        HumanoidModel(double legHipToKnee, double legKneeToAnkle, double legAnkleToGround, double legLateral)
            : legHipToKnee(legHipToKnee)
            , legKneeToAnkle(legKneeToAnkle)
            , legAnkleToGround(legAnkleToGround)
            , legLateral(legLateral) {}

        /**
         * Run analytical inverse kinematics LegIK and
         * assign outputs motor positions.
         * Target footPos and angles are given in
         * special frame when all leg motors are in zero position.
         * Frame centre is located on left or right foot tip.
         * True is returned if angles are updated and inverse
         * kinematics is successful, else false is returned.
         */
        bool legIK(const Eigen::Vector3d& footPos,
                   const Eigen::Vector3d& angles,
                   const EulerType& eulerType,
                   bool isLeftLeg,
                   IKWalkOutputs& outputs) {

            // LegIK initialization
            IK ik(legHipToKnee, legKneeToAnkle, legAnkleToGround);

            // Convert foot position from given
            // target to LegIK base
            Eigen::Vector3d legIKTarget = buildTargetPos(footPos);

            // Convert orientation from given frame
            // to LegIK base
            Frame3D legIKMatrix = buildTargetOrientation(angles, eulerType);

            // Run inverse kinematics
            Position result;
            bool isSuccess = ik.compute(legIKTarget, legIKMatrix, result) && checkNaN(result);

            // Update degrees of freedom
            setIKResult(result, isLeftLeg, outputs);

            return isSuccess;
        }

        /**
         * Return the initial vertical distance
         * from trunk frame to foot tip frame (Z)
         */
        double legsLength() const {
            return legHipToKnee + legKneeToAnkle + legAnkleToGround;
        }

        /**
         * Return the initial lateral distance
         * between each feet
         */
        double feetDistance() const {
            return legLateral;
        }

    private:
        /**
         * Leg segments lengths used by
         * inverse kinematics
         */
        double legHipToKnee;
        double legKneeToAnkle;
        double legAnkleToGround;
        double legLateral;

        /**
         * Convert given euler angle of given
         * convention type to rotation matrix
         */
        Eigen::Matrix3d eulersToMatrix(const Eigen::Vector3d& angles, const EulerType& eulerType) const {
            switch (eulerType) {
                case EulerYawPitchRoll: {
                    Eigen::AngleAxisd yawRot(angles(0), Eigen::Vector3d::UnitZ());
                    Eigen::AngleAxisd pitchRot(angles(1), Eigen::Vector3d::UnitY());
                    Eigen::AngleAxisd rollRot(angles(2), Eigen::Vector3d::UnitX());
                    return (rollRot * pitchRot * yawRot).matrix();
                }
                case EulerYawRollPitch: {
                    Eigen::AngleAxisd yawRot(angles(0), Eigen::Vector3d::UnitZ());
                    Eigen::AngleAxisd pitchRot(angles(2), Eigen::Vector3d::UnitY());
                    Eigen::AngleAxisd rollRot(angles(1), Eigen::Vector3d::UnitX());
                    return (pitchRot * rollRot * yawRot).matrix();
                }
                case EulerRollPitchYaw: {
                    Eigen::AngleAxisd yawRot(angles(2), Eigen::Vector3d::UnitZ());
                    Eigen::AngleAxisd pitchRot(angles(1), Eigen::Vector3d::UnitY());
                    Eigen::AngleAxisd rollRot(angles(0), Eigen::Vector3d::UnitX());
                    return (yawRot * pitchRot * rollRot).matrix();
                }
                case EulerRollYawPitch: {
                    Eigen::AngleAxisd yawRot(angles(1), Eigen::Vector3d::UnitZ());
                    Eigen::AngleAxisd pitchRot(angles(2), Eigen::Vector3d::UnitY());
                    Eigen::AngleAxisd rollRot(angles(0), Eigen::Vector3d::UnitX());
                    return (pitchRot * yawRot * rollRot).matrix();
                }
                case EulerPitchRollYaw: {
                    Eigen::AngleAxisd yawRot(angles(2), Eigen::Vector3d::UnitZ());
                    Eigen::AngleAxisd pitchRot(angles(0), Eigen::Vector3d::UnitY());
                    Eigen::AngleAxisd rollRot(angles(1), Eigen::Vector3d::UnitX());
                    return (yawRot * rollRot * pitchRot).matrix();
                }
                case EulerPitchYawRoll: {
                    Eigen::AngleAxisd yawRot(angles(1), Eigen::Vector3d::UnitZ());
                    Eigen::AngleAxisd pitchRot(angles(0), Eigen::Vector3d::UnitY());
                    Eigen::AngleAxisd rollRot(angles(2), Eigen::Vector3d::UnitX());
                    return (rollRot * yawRot * pitchRot).matrix();
                }
                default: return Eigen::Matrix3d::Identity();
            }
        }

        /**
         * Compute and return the IK position reference
         * vector and orientation reference matrix
         */
        Eigen::Vector3d buildTargetPos(const Eigen::Vector3d& footPos) {
            // Special frame where foot tip in zero position
            return {footPos.x(), footPos.y(), footPos.z() - legsLength()};
        }

        Frame3D buildTargetOrientation(const Eigen::Vector3d& angles, const EulerType& eulerType) {
            return eulersToMatrix(angles, eulerType);
        }

        /**
         * Assign model leg DOF to given IK results
         */
        void setIKResult(const Position& result, bool isLeftLeg, IKWalkOutputs& outputs) {
            if (isLeftLeg) {
                outputs.left_hip_yaw     = result.theta[0];
                outputs.left_hip_roll    = result.theta[1];
                outputs.left_hip_pitch   = -result.theta[2];
                outputs.left_knee        = result.theta[3];
                outputs.left_ankle_pitch = -result.theta[4];
                outputs.left_ankle_roll  = result.theta[5];
            }
            else {
                outputs.right_hip_yaw     = result.theta[0];
                outputs.right_hip_roll    = result.theta[1];
                outputs.right_hip_pitch   = -result.theta[2];
                outputs.right_knee        = result.theta[3];
                outputs.right_ankle_pitch = -result.theta[4];
                outputs.right_ankle_roll  = result.theta[5];
            }
        }

        /**
         * Check inverse kinematics computed value
         * and throw an error in case of NaN
         */
        bool checkNaN(const Position& result) const {
            // Ensure all results are finite
            return (std::isfinite(result.theta[0]) && std::isfinite(result.theta[1]) && std::isfinite(result.theta[2])
                    && std::isfinite(result.theta[3]) && std::isfinite(result.theta[4])
                    && std::isfinite(result.theta[5]));
        }
    };


}  // namespace motion
}  // namespace module

#endif
