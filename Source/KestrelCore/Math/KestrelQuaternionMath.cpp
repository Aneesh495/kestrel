#include "KestrelQuaternionMath.h"

namespace Kestrel {

Quat KestrelQuaternionMath::Exp(const Vector3& omegaRotationVector) {
    double angle = omegaRotationVector.Norm();
    double halfAngle = 0.5 * angle;

    if (angle < 1e-8) {
        // First-order Taylor expansion for near-zero rotation
        double factor = 0.5 - (angle * angle) / 48.0;
        return Quat(1.0 - (halfAngle * halfAngle) * 0.5,
                    omegaRotationVector.x * factor,
                    omegaRotationVector.y * factor,
                    omegaRotationVector.z * factor).Normalized();
    }

    double s = std::sin(halfAngle) / angle;
    return Quat(std::cos(halfAngle),
                omegaRotationVector.x * s,
                omegaRotationVector.y * s,
                omegaRotationVector.z * s);
}

Vector3 KestrelQuaternionMath::Log(const Quat& q) {
    Quat qUnit = q.Normalized();
    // Enforce positive scalar part for shortest geodesic distance
    if (qUnit.w < 0.0) {
        qUnit = Quat(-qUnit.w, -qUnit.x, -qUnit.y, -qUnit.z);
    }

    double vNorm = std::sqrt(qUnit.x * qUnit.x + qUnit.y * qUnit.y + qUnit.z * qUnit.z);
    if (vNorm < 1e-8) {
        return Vector3(2.0 * qUnit.x, 2.0 * qUnit.y, 2.0 * qUnit.z);
    }

    double angle = 2.0 * std::atan2(vNorm, qUnit.w);
    double factor = angle / vNorm;
    return Vector3(qUnit.x * factor, qUnit.y * factor, qUnit.z * factor);
}

Quat KestrelQuaternionMath::IntegrateAngularVelocity(const Quat& currentOrientation,
                                                    const Vector3& angularVelocity,
                                                    double dt) {
    Vector3 deltaTheta = angularVelocity * dt;
    Quat deltaQ = Exp(deltaTheta);
    // Body-frame update: q_new = q_current * delta_q
    return (currentOrientation * deltaQ).Normalized();
}

Vector3 KestrelQuaternionMath::ComputeAttitudeError(const Quat& currentAttitude,
                                                   const Quat& targetAttitude) {
    // Relative error quaternion: q_error = q_current^-1 * q_target
    Quat qError = currentAttitude.Conjugate() * targetAttitude;
    
    // Disambiguate double cover of SO(3): choose shortest rotation direction
    if (qError.w < 0.0) {
        qError = Quat(-qError.w, -qError.x, -qError.y, -qError.z);
    }

    // Geodesic error rotation vector in body coordinates
    return 2.0 * Vector3(qError.x, qError.y, qError.z);
}

Quat KestrelQuaternionMath::Slerp(const Quat& q0, const Quat& q1, double alpha) {
    double dot = q0.w * q1.w + q0.x * q1.x + q0.y * q1.y + q0.z * q1.z;
    Quat target = q1;

    // Flip sign if negative dot product to guarantee acute angle interpolation
    if (dot < 0.0) {
        target = Quat(-q1.w, -q1.x, -q1.y, -q1.z);
        dot = -dot;
    }

    // Linear interpolation fallback when quaternions are collinear
    if (dot > 0.9995) {
        Quat result = (q0 * (1.0 - alpha)) + (target * alpha);
        return result.Normalized();
    }

    double theta0 = std::acos(dot);
    double theta = theta0 * alpha;
    double sinTheta = std::sin(theta);
    double sinTheta0 = std::sin(theta0);

    double s0 = std::cos(theta) - dot * sinTheta / sinTheta0;
    double s1 = sinTheta / sinTheta0;

    return ((q0 * s0) + (target * s1)).Normalized();
}

Matrix3x3 KestrelQuaternionMath::SkewSymmetric(const Vector3& v) {
    return Matrix3x3(
         0.0, -v.z,  v.y,
         v.z,  0.0, -v.x,
        -v.y,  v.x,  0.0
    );
}

bool KestrelQuaternionMath::IsNormalized(const Quat& q, double tolerance) {
    return std::abs(q.NormSq() - 1.0) < tolerance;
}

} // namespace Kestrel
