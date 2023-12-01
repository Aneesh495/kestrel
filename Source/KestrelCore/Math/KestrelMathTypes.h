#pragma once

#include <cmath>
#include <string>
#include <vector>
#include <array>
#include <cstdint>

#if defined(KESTREL_STANDALONE) || !defined(UE_GAME) && !defined(UE_EDITOR)
    #ifndef USTRUCT
    #define USTRUCT(...)
    #endif
    #ifndef UPROPERTY
    #define UPROPERTY(...)
    #endif
    #ifndef GENERATED_BODY
    #define GENERATED_BODY()
    #endif
    #ifndef UCLASS
    #define UCLASS(...)
    #endif
    #ifndef UFUNCTION
    #define UFUNCTION(...)
    #endif
    #ifndef KESTREL_API
    #define KESTREL_API
    #endif
    #ifndef KESTRELCORE_API
    #define KESTRELCORE_API
    #endif
#else
    #include "CoreMinimal.h"
    #include "KestrelMathTypes.generated.h"
#endif

namespace Kestrel {

constexpr double PI = 3.14159265358979323846;
constexpr double DEG2RAD = PI / 180.0;
constexpr double RAD2DEG = 180.0 / PI;
constexpr double GRAVITY_ACCEL = 9.80665; // Standard gravitational acceleration (m/s^2)
constexpr double AIR_DENSITY_SEA_LEVEL = 1.225; // Standard sea-level air density (kg/m^3)

/**
 * 3D Vector for robotics dynamics and kinematics.
 * Follows standard North-East-Down (NED) aeronautical frame or Unreal (XYZ) coordinate frame.
 */
struct Vector3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    constexpr Vector3() = default;
    constexpr Vector3(double inX, double inY, double inZ) : x(inX), y(inY), z(inZ) {}

    Vector3 operator+(const Vector3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vector3 operator-(const Vector3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vector3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vector3 operator/(double s) const { return {x / s, y / s, z / s}; }
    Vector3 operator-() const { return {-x, -y, -z}; }

    Vector3& operator+=(const Vector3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vector3& operator-=(const Vector3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vector3& operator*=(double s) { x *= s; y *= s; z *= s; return *this; }
    Vector3& operator/=(double s) { x /= s; y /= s; z /= s; return *this; }

    bool operator==(const Vector3& o) const {
        return std::abs(x - o.x) < 1e-9 && std::abs(y - o.y) < 1e-9 && std::abs(z - o.z) < 1e-9;
    }

    double Dot(const Vector3& o) const { return x * o.x + y * o.y + z * o.z; }

    Vector3 Cross(const Vector3& o) const {
        return {
            y * o.z - z * o.y,
            z * o.x - x * o.z,
            x * o.y - y * o.x
        };
    }

    double NormSq() const { return x * x + y * y + z * z; }
    double Norm() const { return std::sqrt(NormSq()); }

    Vector3 Normalized() const {
        double n = Norm();
        if (n < 1e-12) return {0.0, 0.0, 0.0};
        return *this / n;
    }

    static Vector3 Zero() { return {0.0, 0.0, 0.0}; }
    static Vector3 UnitX() { return {1.0, 0.0, 0.0}; }
    static Vector3 UnitY() { return {0.0, 1.0, 0.0}; }
    static Vector3 UnitZ() { return {0.0, 0.0, 1.0}; }
};

inline Vector3 operator*(double s, const Vector3& v) { return v * s; }

/**
 * 3x3 Matrix for rigid body inertia tensors and coordinate transformations.
 */
struct Matrix3x3 {
    double m[3][3]{{1,0,0},{0,1,0},{0,0,1}};

    constexpr Matrix3x3() = default;
    constexpr Matrix3x3(double m00, double m01, double m02,
                        double m10, double m11, double m12,
                        double m20, double m21, double m22)
        : m{{m00, m01, m02}, {m10, m11, m12}, {m20, m21, m22}} {}

    Vector3 operator*(const Vector3& v) const {
        return {
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
        };
    }

    Matrix3x3 operator*(const Matrix3x3& o) const {
        Matrix3x3 res{0,0,0, 0,0,0, 0,0,0};
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                for (int k = 0; k < 3; ++k) {
                    res.m[i][j] += m[i][k] * o.m[k][j];
                }
            }
        }
        return res;
    }

    Matrix3x3 Transpose() const {
        return {
            m[0][0], m[1][0], m[2][0],
            m[0][1], m[1][1], m[2][1],
            m[0][2], m[1][2], m[2][2]
        };
    }

    double Determinant() const {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
             - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
             + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }

    Matrix3x3 Inverse() const {
        double det = Determinant();
        if (std::abs(det) < 1e-15) return Matrix3x3();
        double invDet = 1.0 / det;

        return {
            (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * invDet,
            (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invDet,
            (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invDet,

            (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invDet,
            (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invDet,
            (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * invDet,

            (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * invDet,
            (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * invDet,
            (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * invDet
        };
    }

    static Matrix3x3 Identity() {
        return {1,0,0, 0,1,0, 0,0,1};
    }

    static Matrix3x3 Diagonal(double d0, double d1, double d2) {
        return {d0,0,0, 0,d1,0, 0,0,d2};
    }
};

/**
 * Unit Quaternion for singularity-free 3D attitude orientation.
 * Convention: Hamilton quaternion q = w + x*i + y*j + z*k.
 */
struct Quat {
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};

    constexpr Quat() = default;
    constexpr Quat(double inW, double inX, double inY, double inZ)
        : w(inW), x(inX), y(inY), z(inZ) {}

    Quat operator+(const Quat& o) const { return {w + o.w, x + o.x, y + o.y, z + o.z}; }
    Quat operator-(const Quat& o) const { return {w - o.w, x - o.x, y - o.y, z - o.z}; }
    Quat operator*(double s) const { return {w * s, x * s, y * s, z * s}; }

    Quat operator*(const Quat& o) const {
        return {
            w * o.w - x * o.x - y * o.y - z * o.z,
            w * o.x + x * o.w + y * o.z - z * o.y,
            w * o.y - x * o.z + y * o.w + z * o.x,
            w * o.z + x * o.y - y * o.x + z * o.w
        };
    }

    Quat Conjugate() const { return {w, -x, -y, -z}; }

    double NormSq() const { return w * w + x * x + y * y + z * z; }
    double Norm() const { return std::sqrt(NormSq()); }

    Quat Normalized() const {
        double n = Norm();
        if (n < 1e-12) return {1.0, 0.0, 0.0, 0.0};
        double inv = 1.0 / n;
        return {w * inv, x * inv, y * inv, z * inv};
    }

    Vector3 Rotate(const Vector3& v) const {
        Quat qv(0.0, v.x, v.y, v.z);
        Quat res = (*this) * qv * Conjugate();
        return {res.x, res.y, res.z};
    }

    Vector3 RotateInverse(const Vector3& v) const {
        return Conjugate().Rotate(v);
    }

    Matrix3x3 ToRotationMatrix() const {
        double xx = x * x, yy = y * y, zz = z * z;
        double xy = x * y, xz = x * z, yz = y * z;
        double wx = w * x, wy = w * y, wz = w * z;

        return {
            1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy),
            2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
            2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)
        };
    }

    static Quat FromEuler(double roll, double pitch, double yaw) {
        double cr = std::cos(roll * 0.5);
        double sr = std::sin(roll * 0.5);
        double cp = std::cos(pitch * 0.5);
        double sp = std::sin(pitch * 0.5);
        double cy = std::cos(yaw * 0.5);
        double sy = std::sin(yaw * 0.5);

        return {
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        };
    }

    void ToEuler(double& outRoll, double& outPitch, double& outYaw) const {
        // Roll (x-axis rotation)
        double sinr_cosp = 2.0 * (w * x + y * z);
        double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        outRoll = std::atan2(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        double sinp = 2.0 * (w * y - z * x);
        if (std::abs(sinp) >= 1.0) {
            outPitch = std::copysign(PI / 2.0, sinp); // 90 degrees at gimbal poles
        } else {
            outPitch = std::asin(sinp);
        }

        // Yaw (z-axis rotation)
        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        outYaw = std::atan2(siny_cosp, cosy_cosp);
    }

    static Quat Identity() { return {1.0, 0.0, 0.0, 0.0}; }
};

/**
 * 6-DOF Complete Rigid Body Dynamic State.
 */
struct RigidBodyState {
    Vector3 position{0.0, 0.0, 0.0};       // Inertial world position (m)
    Vector3 velocity{0.0, 0.0, 0.0};       // Inertial world linear velocity (m/s)
    Vector3 acceleration{0.0, 0.0, 0.0};   // Inertial world linear acceleration (m/s^2)
    Quat attitude{1.0, 0.0, 0.0, 0.0};     // World-to-body orientation quaternion
    Vector3 angularVelocity{0.0, 0.0, 0.0};// Body frame angular velocity (rad/s)
    Vector3 angularAccel{0.0, 0.0, 0.0};   // Body frame angular acceleration (rad/s^2)
};

/**
 * Multirotor Aerodynamic and Motor Physical Parameters.
 */
struct DronePhysicalParams {
    double mass{1.5};                       // Vehicle mass (kg)
    Matrix3x3 inertiaTensor{
        0.015, 0.0,   0.0,
        0.0,   0.015, 0.0,
        0.0,   0.0,   0.025
    };                                      // Moments of inertia (kg * m^2)
    double armLength{0.225};                // Motor arm radius from center of gravity (m)
    double thrustCoefficient{1.5e-5};       // Thrust coefficient Ct (N / (rad/s)^2)
    double torqueCoefficient{2.5e-7};       // Torque drag coefficient Cq (N*m / (rad/s)^2)
    double maxMotorRPM{9500.0};             // Maximum motor speed (RPM)
    double motorTimeConstant{0.035};        // ESC and motor response lag time constant (s)
    double dragLinear{0.12};                // Linear translational aerodynamic drag (N / (m/s))
    double dragAngular{0.02};               // Angular aerodynamic damping (N*m / (rad/s))
    int rotorCount{4};                      // Number of propulsion rotors (4 = quad, 6 = hex)
};

/**
 * Actuator Command (Normalized 0.0 to 1.0 or PWM).
 */
struct MotorCommands {
    std::array<double, 8> motorPWM{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    int activeCount{4};
};

/**
 * Sensor Suite Telemetry Frame.
 */
struct SensorTelemetry {
    uint64_t timestampMicros{0};
    Vector3 accelRaw{0.0, 0.0, -9.80665};   // Accelerometer with gravity (m/s^2)
    Vector3 gyroRaw{0.0, 0.0, 0.0};         // Gyroscope body rates (rad/s)
    Vector3 magRaw{0.22, 0.02, 0.42};       // Magnetometer flux (Gauss)
    double baroAltitude{0.0};               // Barometric pressure derived altitude (m)
    double baroPressurePa{101325.0};        // Static atmospheric pressure (Pa)
    Vector3 gpsPosition{0.0, 0.0, 0.0};     // Geodetic or local NED position (m)
    Vector3 gpsVelocity{0.0, 0.0, 0.0};     // GNSS ground velocity (m/s)
    double opticalFlowVx{0.0};              // Visual odometry / flow x-velocity (m/s)
    double opticalFlowVy{0.0};              // Visual odometry / flow y-velocity (m/s)
    double sonarDistance{0.0};              // Downward acoustic/lidar distance (m)
    bool bGpsFix{true};
    int satellitesVisible{14};
};

} // namespace Kestrel
