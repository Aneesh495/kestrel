#pragma once

#include "KestrelMathTypes.h"
#include <functional>

namespace Kestrel {

/**
 * State Derivative structure for 6-DOF dynamic equations of motion.
 */
struct StateDerivative {
    Vector3 linearVelocity{0.0, 0.0, 0.0};
    Vector3 linearAcceleration{0.0, 0.0, 0.0};
    Vector3 angularVelocity{0.0, 0.0, 0.0};
    Vector3 angularAcceleration{0.0, 0.0, 0.0};
};

/**
 * Fourth-Order Runge-Kutta (RK4) Numerical Dynamical Integrator.
 * Evaluates non-linear coupled translational and rotational rigid body dynamics
 * with O(dt^4) global error accuracy and strict quaternion norm preservation.
 */
class KestrelRK4Integrator {
public:
    using DynamicsFunction = std::function<StateDerivative(const RigidBodyState&, double)>;

    /**
     * Integrates state forward by dt using classical Runge-Kutta 4th-order method.
     *
     * @param state Initial dynamic state
     * @param dt Integration time step in seconds
     * @param dynamics Callable computing state derivatives f(x, t)
     * @param currentTime Current simulation time
     * @return New integrated dynamic state
     */
    static RigidBodyState Step(const RigidBodyState& state,
                               double dt,
                               const DynamicsFunction& dynamics,
                               double currentTime = 0.0);

    /**
     * Compute 6-DOF rigid body derivatives given applied forces and torques.
     *
     * @param state Current rigid body state
     * @param bodyForces Total force vector expressed in body frame (N)
     * @param bodyTorques Total torque vector expressed in body frame (N*m)
     * @param params Vehicle physical and inertia parameters
     * @return Evaluated state derivative
     */
    static StateDerivative ComputeRigidBodyDerivative(const RigidBodyState& state,
                                                      const Vector3& bodyForces,
                                                      const Vector3& bodyTorques,
                                                      const DronePhysicalParams& params);
};

} // namespace Kestrel
