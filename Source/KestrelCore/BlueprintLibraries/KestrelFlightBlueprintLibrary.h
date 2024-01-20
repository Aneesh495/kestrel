#pragma once

#if !defined(KESTREL_STANDALONE) && (defined(UE_GAME) || defined(UE_EDITOR))
#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "KestrelFlightBlueprintLibrary.generated.h"

UCLASS()
class KESTRELCORE_API UKestrelFlightBlueprintLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/** Converts North-East-Down (NED) robotics coordinates to Unreal Engine XYZ world coordinates */
	UFUNCTION(BlueprintPure, Category = "Kestrel|Coordinates")
	static FVector NEDToUnreal(const FVector& InNED);

	/** Converts Unreal Engine XYZ world coordinates to North-East-Down (NED) coordinates */
	UFUNCTION(BlueprintPure, Category = "Kestrel|Coordinates")
	static FVector UnrealToNED(const FVector& InUnreal);

	/** Calculates atmospheric density (kg/m^3) at geometric altitude (m) using US Standard Atmosphere 1976 */
	UFUNCTION(BlueprintPure, Category = "Kestrel|Aerodynamics")
	static double GetAirDensityAtAltitude(double AltitudeMeters);

	/** Solves Blade Element Momentum Theory (BEMT) rotor thrust and reaction torque */
	UFUNCTION(BlueprintPure, Category = "Kestrel|Aerodynamics")
	static void EvaluateRotorAerodynamics(double RotorOmegaRadPerS,
	                                     const FVector& RelativeWind,
	                                     double GroundClearanceMeters,
	                                     double AirDensity,
	                                     double& OutThrustNewtons,
	                                     double& OutTorqueNm,
	                                     double& OutInflowVelocity);

	/** Calculates shortest-path geodesic attitude error quaternion in body frame */
	UFUNCTION(BlueprintPure, Category = "Kestrel|Kinematics")
	static FVector ComputeGeodesicAttitudeError(const FQuat& CurrentOrientation, const FQuat& TargetOrientation);
};
#else
#include "Math/KestrelMathTypes.h"
#include "Aerodynamics/KestrelAtmosphere.h"
#include "Aerodynamics/KestrelBladeElement.h"
#include "Math/KestrelQuaternionMath.h"

namespace Kestrel {
    class KestrelFlightBlueprintLibrary {
    public:
        static Vector3 NEDToUnreal(const Vector3& ned) {
            // NED: X=North, Y=East, Z=Down -> Unreal: X=Forward(North)*100, Y=Right(East)*100, Z=Up(-Down)*100
            return Vector3(ned.x * 100.0, ned.y * 100.0, -ned.z * 100.0);
        }
        static Vector3 UnrealToNED(const Vector3& ue) {
            return Vector3(ue.x * 0.01, ue.y * 0.01, -ue.z * 0.01);
        }
        static double GetAirDensityAtAltitude(double altMeters) {
            return KestrelAtmosphere::GetStateAtAltitude(altMeters).densityKgPerM3;
        }
        static void EvaluateRotorAerodynamics(double omega, const Vector3& relWind, double clearance, double airDensity,
                                              double& outThrust, double& outTorque, double& outInflow) {
            KestrelBladeElement bemt;
            RotorState state;
            state.omegaRadPerS = omega;
            state.localAirflowVelocity = relWind;
            state.groundClearanceMeters = clearance;
            RotorAerodynamicOutput out = bemt.ComputeRotorForces(state, airDensity);
            outThrust = out.thrustNewtons;
            outTorque = out.torqueNewtonMeters;
            outInflow = out.inducedInflowVelocity;
        }
    };
}
#endif
