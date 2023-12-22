#pragma once

#include "Math/KestrelMathTypes.h"

namespace Kestrel {

/**
 * Atmospheric state vector at a specific geometric altitude.
 */
struct AtmosphericState {
    double altitudeMeters{0.0};
    double temperatureKelvin{288.15};     // Standard temperature at sea level (15 deg C)
    double pressurePascals{101325.0};     // Standard sea-level pressure (1 atm)
    double densityKgPerM3{1.225};         // Air density
    double speedOfSoundMPerS{340.294};    // Acoustic velocity
    double dynamicViscosityPaS{1.789e-5}; // Dynamic air viscosity
};

/**
 * High-Fidelity US Standard Atmosphere 1976 Model.
 * Implements geopotential altitude, temperature lapse rates, hydrostatic barometric integration,
 * Sutherland viscosity interpolation, and hypsometric altitude inversion for barometric sensors.
 */
class KestrelAtmosphere {
public:
    // Physical constants
    static constexpr double SEA_LEVEL_PRESSURE = 101325.0;      // Pa
    static constexpr double SEA_LEVEL_TEMPERATURE = 288.15;     // K
    static constexpr double SEA_LEVEL_DENSITY = 1.225;          // kg/m^3
    static constexpr double TEMPERATURE_LAPSE_RATE = -0.0065;   // K/m (troposphere)
    static constexpr double GAS_CONSTANT_AIR = 287.05287;       // J / (kg * K)
    static constexpr double SPECIFIC_HEAT_RATIO = 1.4;          // gamma (diatomic air)
    static constexpr double SUTHERLAND_TEMP = 110.4;            // K (Sutherland's constant)
    static constexpr double SUTHERLAND_VISCOSITY = 1.7894e-5;   // Pa * s
    static constexpr double TROPOPAUSE_ALTITUDE = 11000.0;      // m

    /**
     * Computes atmospheric properties at geometric altitude.
     *
     * @param altitudeMeters Height above mean sea level (MSL) in meters
     * @return Complete atmospheric state
     */
    static AtmosphericState GetStateAtAltitude(double altitudeMeters);

    /**
     * Converts barometric pressure reading back to estimated altitude (hypsometric formula).
     * Used directly by onboard flight controller sensor fusion.
     *
     * @param pressurePa Measured static pressure in Pascals
     * @param qnhPressurePa Local altimeter setting / sea level reference pressure (Pa)
     * @return Estimated pressure altitude in meters
     */
    static double PressureToAltitude(double pressurePa, double qnhPressurePa = SEA_LEVEL_PRESSURE);

    /**
     * Converts true altitude to expected static barometric pressure.
     */
    static double AltitudeToPressure(double altitudeMeters, double qnhPressurePa = SEA_LEVEL_PRESSURE);
};

} // namespace Kestrel
