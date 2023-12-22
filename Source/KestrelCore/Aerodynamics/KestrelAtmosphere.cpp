#include "KestrelAtmosphere.h"
#include <algorithm>

namespace Kestrel {

AtmosphericState KestrelAtmosphere::GetStateAtAltitude(double altitudeMeters) {
    AtmosphericState state;
    // Bound altitude to troposphere and lower stratosphere
    double h = std::clamp(altitudeMeters, -500.0, 20000.0);
    state.altitudeMeters = altitudeMeters;

    if (h <= TROPOPAUSE_ALTITUDE) {
        // Troposphere (0 to 11 km): constant temperature lapse rate L = -6.5 K/km
        state.temperatureKelvin = SEA_LEVEL_TEMPERATURE + TEMPERATURE_LAPSE_RATE * h;
        double tempRatio = state.temperatureKelvin / SEA_LEVEL_TEMPERATURE;
        double exponent = -GRAVITY_ACCEL / (TEMPERATURE_LAPSE_RATE * GAS_CONSTANT_AIR);
        state.pressurePascals = SEA_LEVEL_PRESSURE * std::pow(tempRatio, exponent);
    } else {
        // Lower Stratosphere (11 km to 20 km): isothermal region
        double tropoTemp = SEA_LEVEL_TEMPERATURE + TEMPERATURE_LAPSE_RATE * TROPOPAUSE_ALTITUDE;
        state.temperatureKelvin = tropoTemp;
        double tempRatio = tropoTemp / SEA_LEVEL_TEMPERATURE;
        double exponent = -GRAVITY_ACCEL / (TEMPERATURE_LAPSE_RATE * GAS_CONSTANT_AIR);
        double tropoPressure = SEA_LEVEL_PRESSURE * std::pow(tempRatio, exponent);

        double deltaH = h - TROPOPAUSE_ALTITUDE;
        state.pressurePascals = tropoPressure * std::exp(-GRAVITY_ACCEL * deltaH / (GAS_CONSTANT_AIR * tropoTemp));
    }

    // Ideal gas law: rho = P / (R * T)
    state.densityKgPerM3 = state.pressurePascals / (GAS_CONSTANT_AIR * state.temperatureKelvin);

    // Speed of sound: a = sqrt(gamma * R * T)
    state.speedOfSoundMPerS = std::sqrt(SPECIFIC_HEAT_RATIO * GAS_CONSTANT_AIR * state.temperatureKelvin);

    // Dynamic viscosity (Sutherland's law)
    double tRatio = state.temperatureKelvin / SEA_LEVEL_TEMPERATURE;
    state.dynamicViscosityPaS = SUTHERLAND_VISCOSITY * std::pow(tRatio, 1.5) *
                                ((SEA_LEVEL_TEMPERATURE + SUTHERLAND_TEMP) / (state.temperatureKelvin + SUTHERLAND_TEMP));

    return state;
}

double KestrelAtmosphere::PressureToAltitude(double pressurePa, double qnhPressurePa) {
    if (pressurePa <= 0.0) return TROPOPAUSE_ALTITUDE;
    double pRatio = pressurePa / qnhPressurePa;
    // Invert barometric formula: h = (T0 / L) * ((P / P0)^(-R*L / g) - 1)
    double exponent = -TEMPERATURE_LAPSE_RATE * GAS_CONSTANT_AIR / GRAVITY_ACCEL;
    return (SEA_LEVEL_TEMPERATURE / -TEMPERATURE_LAPSE_RATE) * (1.0 - std::pow(pRatio, exponent));
}

double KestrelAtmosphere::AltitudeToPressure(double altitudeMeters, double qnhPressurePa) {
    double h = std::clamp(altitudeMeters, -500.0, TROPOPAUSE_ALTITUDE);
    double temp = SEA_LEVEL_TEMPERATURE + TEMPERATURE_LAPSE_RATE * h;
    double exponent = -GRAVITY_ACCEL / (TEMPERATURE_LAPSE_RATE * GAS_CONSTANT_AIR);
    return qnhPressurePa * std::pow(temp / SEA_LEVEL_TEMPERATURE, exponent);
}

} // namespace Kestrel
