#pragma once

#if !defined(KESTREL_STANDALONE) && (defined(UE_GAME) || defined(UE_EDITOR))
#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "KestrelSensorBlueprintLibrary.generated.h"

UCLASS()
class KESTRELCORE_API UKestrelSensorBlueprintLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/** Converts barometric pressure reading in Pascals to estimated altitude above sea level */
	UFUNCTION(BlueprintPure, Category = "Kestrel|Sensors")
	static double ConvertPressureToAltitude(double PressurePa, double SeaLevelPressurePa = 101325.0);

	/** Samples Gaussian white noise with specified standard deviation */
	UFUNCTION(BlueprintPure, Category = "Kestrel|Sensors")
	static double GenerateGaussianNoise(double Mean, double StdDev);

	/** Calculates magnetic heading angle in degrees from magnetometer vector */
	UFUNCTION(BlueprintPure, Category = "Kestrel|Sensors")
	static double CalculateCompassHeading(const FVector& MagReading);
};
#else
#include "Math/KestrelMathTypes.h"
#include "Aerodynamics/KestrelAtmosphere.h"
#include "Sensors/KestrelNoiseModel.h"

namespace Kestrel {
    class KestrelSensorBlueprintLibrary {
    public:
        static double ConvertPressureToAltitude(double pressurePa, double qnhPa = 101325.0) {
            return KestrelAtmosphere::PressureToAltitude(pressurePa, qnhPa);
        }
        static double GenerateGaussianNoise(double mean, double stdDev) {
            static KestrelNoiseModel noiseModel;
            return mean + noiseModel.SampleGaussian(stdDev);
        }
        static double CalculateCompassHeading(const Vector3& mag) {
            double headingRad = std::atan2(-mag.y, mag.x);
            double headingDeg = headingRad * RAD2DEG;
            if (headingDeg < 0.0) headingDeg += 360.0;
            return headingDeg;
        }
    };
}
#endif
