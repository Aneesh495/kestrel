#pragma once

#include "Math/KestrelMathTypes.h"
#include "Spatial/KestrelOctree.h"
#include <vector>
#include <random>

namespace Kestrel {

struct LiDARPoint {
    Vector3 localPoint{0.0, 0.0, 0.0};  // Relative to sensor origin (m)
    Vector3 worldPoint{0.0, 0.0, 0.0};  // Global world coordinate (m)
    double distance{0.0};                // Measured radial distance (m)
    double intensity{1.0};               // Echo reflectivity intensity [0.0, 1.0]
    int beamId{0};                       // Vertical channel laser index
};

struct LiDARScannerConfig {
    int verticalBeams{16};               // Number of vertical elevation channels
    double fovVerticalMinDeg{-15.0};     // Lower elevation boundary (deg)
    double fovVerticalMaxDeg{15.0};      // Upper elevation boundary (deg)
    int horizontalPointsPerScan{360};    // Azimuthal resolution (1 deg steps)
    double minRangeMeters{0.2};          // Blind zone threshold (m)
    double maxRangeMeters{60.0};         // Maximum laser range (m)
    double rangeNoiseStdDev{0.015};      // Gaussian distance measurement error (m)
    uint32_t seed{404};
};

/**
 * 3D Multi-Beam LiDAR Sensor Simulator.
 * Simulates mechanical spinning or solid-state LiDAR raycast scans against
 * the spatial Octree with distance attenuation and Gaussian beam divergence noise.
 */
class KestrelLiDARScanner {
public:
    KestrelLiDARScanner();
    explicit KestrelLiDARScanner(const LiDARScannerConfig& inConfig);

    /**
     * Executes a full 360-degree point cloud sweep against the spatial Octree.
     *
     * @param sensorWorldPos 3D world position of sensor
     * @param sensorWorldRot Orientation quaternion of sensor
     * @param worldOctree Spatial acceleration structure of the environment
     * @param outPointCloud Populated point cloud buffer
     * @return Number of valid laser returns
     */
    int Scan(const Vector3& sensorWorldPos,
             const Quat& sensorWorldRot,
             const KestrelOctree& worldOctree,
             std::vector<LiDARPoint>& outPointCloud);

private:
    LiDARScannerConfig cfg;
    std::mt19937 rng;
    std::normal_distribution<double> noiseDist{0.0, 1.0};
    std::vector<double> elevationAnglesRad;
};

} // namespace Kestrel
