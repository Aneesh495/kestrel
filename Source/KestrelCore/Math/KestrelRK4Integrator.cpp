#include "KestrelRK4Integrator.h"
#include "KestrelQuaternionMath.h"

namespace Kestrel {

RigidBodyState KestrelRK4Integrator::Step(const RigidBodyState& state,
                                         double dt,
                                         const DynamicsFunction& dynamics,
                                         double currentTime) {
    if (dt <= 0.0) return state;

    // Stage 1: Initial state slope
    StateDerivative k1 = dynamics(state, currentTime);

    // Stage 2: Midpoint slope based on k1
    RigidBodyState state2;
    state2.position = state.position + k1.linearVelocity * (0.5 * dt);
    state2.velocity = state.velocity + k1.linearAcceleration * (0.5 * dt);
    state2.attitude = KestrelQuaternionMath::IntegrateAngularVelocity(state.attitude, k1.angularVelocity, 0.5 * dt);
    state2.angularVelocity = state.angularVelocity + k1.angularAcceleration * (0.5 * dt);
    StateDerivative k2 = dynamics(state2, currentTime + 0.5 * dt);

    // Stage 3: Midpoint slope based on k2
    RigidBodyState state3;
    state3.position = state.position + k2.linearVelocity * (0.5 * dt);
    state3.velocity = state.velocity + k2.linearAcceleration * (0.5 * dt);
    state3.attitude = KestrelQuaternionMath::IntegrateAngularVelocity(state.attitude, k2.angularVelocity, 0.5 * dt);
    state3.angularVelocity = state.angularVelocity + k2.angularAcceleration * (0.5 * dt);
    StateDerivative k3 = dynamics(state3, currentTime + 0.5 * dt);

    // Stage 4: Endpoint slope based on k3
    RigidBodyState state4;
    state4.position = state.position + k3.linearVelocity * dt;
    state4.velocity = state.velocity + k3.linearAcceleration * dt;
    state4.attitude = KestrelQuaternionMath::IntegrateAngularVelocity(state.attitude, k3.angularVelocity, dt);
    state4.angularVelocity = state.angularVelocity + k3.angularAcceleration * dt;
    StateDerivative k4 = dynamics(state4, currentTime + dt);

    // Weighted Simpson's rule integration
    RigidBodyState nextState;
    nextState.position = state.position + (k1.linearVelocity + 2.0 * k2.linearVelocity + 2.0 * k3.linearVelocity + k4.linearVelocity) * (dt / 6.0);
    nextState.velocity = state.velocity + (k1.linearAcceleration + 2.0 * k2.linearAcceleration + 2.0 * k3.linearAcceleration + k4.linearAcceleration) * (dt / 6.0);
    nextState.acceleration = (k1.linearAcceleration + 2.0 * k2.linearAcceleration + 2.0 * k3.linearAcceleration + k4.linearAcceleration) * (1.0 / 6.0);

    // Average angular velocity for quaternion Lie algebra integration
    Vector3 avgOmega = (k1.angularVelocity + 2.0 * k2.angularVelocity + 2.0 * k3.angularVelocity + k4.angularVelocity) * (1.0 / 6.0);
    nextState.attitude = KestrelQuaternionMath::IntegrateAngularVelocity(state.attitude, avgOmega, dt);

    nextState.angularVelocity = state.angularVelocity + (k1.angularAcceleration + 2.0 * k2.angularAcceleration + 2.0 * k3.angularAcceleration + k4.angularAcceleration) * (dt / 6.0);
    nextState.angularAccel = (k1.angularAcceleration + 2.0 * k2.angularAcceleration + 2.0 * k3.angularAcceleration + k4.angularAcceleration) * (1.0 / 6.0);

    return nextState;
}

StateDerivative KestrelRK4Integrator::ComputeRigidBodyDerivative(const RigidBodyState& state,
                                                                const Vector3& bodyForces,
                                                                const Vector3& bodyTorques,
                                                                const DronePhysicalParams& params) {
    StateDerivative deriv;

    // Linear kinematics: d/dt(position) = velocity
    deriv.linearVelocity = state.velocity;

    // Rotate body-frame forces to inertial frame using current attitude quaternion
    Vector3 inertialForces = state.attitude.Rotate(bodyForces);
    // Add downward gravity acceleration: in NED coordinates, gravity is +Z
    Vector3 gravityForce(0.0, 0.0, params.mass * GRAVITY_ACCEL);
    Vector3 totalInertialForce = inertialForces + gravityForce;

    // Translational dynamics: F = m * a -> a = F / m
    deriv.linearAcceleration = totalInertialForce / params.mass;

    // Rotational kinematics: d/dt(orientation) driven by angular velocity
    deriv.angularVelocity = state.angularVelocity;

    // Euler's rotational equations of motion: J * dw/dt + w x (J * w) = Tau
    // dw/dt = J^-1 * (Tau - w x (J * w))
    Vector3 J_omega = params.inertiaTensor * state.angularVelocity;
    Vector3 gyroscopicMoment = state.angularVelocity.Cross(J_omega);
    Vector3 netTorque = bodyTorques - gyroscopicMoment;

    Matrix3x3 invInertia = params.inertiaTensor.Inverse();
    deriv.angularAcceleration = invInertia * netTorque;

    return deriv;
}

} // namespace Kestrel
