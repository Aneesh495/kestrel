#pragma once

#include "KestrelMathTypes.h"

namespace Kestrel {

/**
 * Singularity-free Quaternion Kinematics and SO(3) Lie Algebra primitives.
 * Provides exact attitude propagation, shortest-path geodesic error formulation,
 * and exponential/logarithmic mappings between Lie group SO(3) and Lie algebra so(3).
 */
class KestrelQuaternionMath {
public:
    /**
     * Exponential map from Lie algebra so(3) rotation vector to unit quaternion.
     * Computes exp(theta/2) using Taylor series expansion for small angles.
     *
     * @param omegaRotationVector Angular rotation vector in radians
     * @return Unit quaternion representation of the rotation
     */
    static Quat Exp(const Vector3& omegaRotationVector);

    /**
     * Logarithmic map from unit quaternion to Lie algebra so(3) rotation vector.
     * Inverse of the exponential map.
     *
     * @param q Unit quaternion
     * @return Angular rotation vector in radians
     */
    static Vector3 Log(const Quat& q);

    /**
     * Closed-form discrete integration of orientation quaternion given angular rate.
     * Preserves exact unit norm without numerical damping.
     *
     * @param currentOrientation Current unit quaternion
     * @param angularVelocity Body frame angular velocity vector (rad/s)
     * @param dt Integration step size in seconds
     * @return Updated unit quaternion
     */
    static Quat IntegrateAngularVelocity(const Quat& currentOrientation,
                                         const Vector3& angularVelocity,
                                         double dt);

    /**
     * Computes the geodesic attitude error vector between current and target orientation.
     * Returns an error vector in body frame suitable for PD/PID attitude control.
     *
     * @param currentAttitude Current vehicle orientation
     * @param targetAttitude Desired target orientation
     * @return Body-frame attitude error vector in radians
     */
    static Vector3 ComputeAttitudeError(const Quat& currentAttitude,
                                        const Quat& targetAttitude);

    /**
     * Spherical Linear Interpolation (SLERP) between two unit quaternions.
     * Automatically handles antipodal flipping to guarantee shortest rotation path.
     *
     * @param q0 Start quaternion
     * @param q1 End quaternion
     * @param alpha Interpolation parameter in [0.0, 1.0]
     * @return Interpolated unit quaternion
     */
    static Quat Slerp(const Quat& q0, const Quat& q1, double alpha);

    /**
     * Generates a 3x3 skew-symmetric cross-product matrix [v]x.
     */
    static Matrix3x3 SkewSymmetric(const Vector3& v);

    /**
     * Checks if quaternion is normalized within numerical tolerance.
     */
    static bool IsNormalized(const Quat& q, double tolerance = 1e-6);
};

} // namespace Kestrel
