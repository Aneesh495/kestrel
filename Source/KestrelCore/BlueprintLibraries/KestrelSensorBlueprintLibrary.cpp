#include "KestrelSensorBlueprintLibrary.h"

#if !defined(KESTREL_STANDALONE) && (defined(UE_GAME) || defined(UE_EDITOR))
#include "Aerodynamics/KestrelAtmosphere.h"
#include "Sensors/KestrelNoiseModel.h"

double UKestrelSensorBlueprintLibrary::ConvertPressureToAltitude(double PressurePa, double SeaLevelPressurePa)
{
	return Kestrel::KestrelAtmosphere::PressureToAltitude(PressurePa, SeaLevelPressurePa);
}

double UKestrelSensorBlueprintLibrary::GenerateGaussianNoise(double Mean, double StdDev)
{
	static Kestrel::KestrelNoiseModel noiseModel;
	return Mean + noiseModel.SampleGaussian(StdDev);
}

double UKestrelSensorBlueprintLibrary::CalculateCompassHeading(const FVector& MagReading)
{
	double headingRad = std::atan2(-MagReading.Y, MagReading.X);
	double headingDeg = headingRad * (180.0 / 3.14159265358979323846);
	if (headingDeg < 0.0) headingDeg += 360.0;
	return headingDeg;
}
#endif
