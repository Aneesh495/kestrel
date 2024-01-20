#include "KestrelFlightBlueprintLibrary.h"

#if !defined(KESTREL_STANDALONE) && (defined(UE_GAME) || defined(UE_EDITOR))
#include "Aerodynamics/KestrelAtmosphere.h"
#include "Aerodynamics/KestrelBladeElement.h"
#include "Math/KestrelQuaternionMath.h"

FVector UKestrelFlightBlueprintLibrary::NEDToUnreal(const FVector& InNED)
{
	// Scale from meters to centimeters: X=North, Y=East, Z=-Down
	return FVector(InNED.X * 100.0, InNED.Y * 100.0, -InNED.Z * 100.0);
}

FVector UKestrelFlightBlueprintLibrary::UnrealToNED(const FVector& InUnreal)
{
	return FVector(InUnreal.X * 0.01, InUnreal.Y * 0.01, -InUnreal.Z * 0.01);
}

double UKestrelFlightBlueprintLibrary::GetAirDensityAtAltitude(double AltitudeMeters)
{
	return Kestrel::KestrelAtmosphere::GetStateAtAltitude(AltitudeMeters).densityKgPerM3;
}

void UKestrelFlightBlueprintLibrary::EvaluateRotorAerodynamics(double RotorOmegaRadPerS,
                                                               const FVector& RelativeWind,
                                                               double GroundClearanceMeters,
                                                               double AirDensity,
                                                               double& OutThrustNewtons,
                                                               double& OutTorqueNm,
                                                               double& OutInflowVelocity)
{
	Kestrel::KestrelBladeElement solver;
	Kestrel::RotorState state;
	state.omegaRadPerS = RotorOmegaRadPerS;
	state.localAirflowVelocity = Kestrel::Vector3(RelativeWind.X, RelativeWind.Y, RelativeWind.Z);
	state.groundClearanceMeters = GroundClearanceMeters;

	Kestrel::RotorAerodynamicOutput result = solver.ComputeRotorForces(state, AirDensity);
	OutThrustNewtons = result.thrustNewtons;
	OutTorqueNm = result.torqueNewtonMeters;
	OutInflowVelocity = result.inducedInflowVelocity;
}

FVector UKestrelFlightBlueprintLibrary::ComputeGeodesicAttitudeError(const FQuat& CurrentOrientation, const FQuat& TargetOrientation)
{
	Kestrel::Quat qCur(CurrentOrientation.W, CurrentOrientation.X, CurrentOrientation.Y, CurrentOrientation.Z);
	Kestrel::Quat qTgt(TargetOrientation.W, TargetOrientation.X, TargetOrientation.Y, TargetOrientation.Z);

	Kestrel::Vector3 err = Kestrel::KestrelQuaternionMath::ComputeAttitudeError(qCur, qTgt);
	return FVector(err.x, err.y, err.z);
}
#endif
