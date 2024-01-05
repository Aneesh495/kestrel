#include "KestrelLiDARScanner.h"
#include <cmath>

namespace Kestrel {

KestrelLiDARScanner::KestrelLiDARScanner(const Config& inConfig)
    : cfg(inConfig)
    , rng(inConfig.seed)
{
    elevationAnglesRad.resize(cfg.verticalBeams);
    if (cfg.verticalBeams <= 1) {
        elevationAnglesRad[0] = 0.0;
    } else {
        double minElev = cfg.fovVerticalMinDeg * DEG2RAD;
        double maxElev = cfg.fovVerticalMaxDeg * DEG2RAD;
        double step = (maxElev - minElev) / (cfg.verticalBeams - 1);
        for (int i = 0; i < cfg.verticalBeams; ++i) {
            elevationAnglesRad[i] = minElev + i * step;
        }
    }
}

int KestrelLiDARScanner::Scan(const Vector3& sensorWorldPos,
                              const Quat& sensorWorldRot,
                              const KestrelOctree& worldOctree,
                              std::vector<LiDARPoint>& outPointCloud) {
    outPointCloud.clear();
    outPointCloud.reserve(cfg.verticalBeams * cfg.horizontalPointsPerScan);

    double azimuthStep = (2.0 * PI) / cfg.horizontalPointsPerScan;
    int validHits = 0;

    for (int b = 0; b < cfg.verticalBeams; ++b) {
        double elev = elevationAnglesRad[b];
        double cosElev = std::cos(elev);
        double sinElev = std::sin(elev);

        for (int h = 0; h < cfg.horizontalPointsPerScan; ++h) {
            double azimuth = h * azimuthStep;

            // Unit direction in sensor local frame
            Vector3 localDir(
                cosElev * std::cos(azimuth),
                cosElev * std::sin(azimuth),
                sinElev
            );

            // Transform ray direction to world frame
            Vector3 worldDir = sensorWorldRot.Rotate(localDir);

            RaycastHit hit;
            if (worldOctree.Raycast(sensorWorldPos, worldDir, cfg.maxRangeMeters, hit)) {
                if (hit.distance >= cfg.minRangeMeters) {
                    double noise = noiseDist(rng) * cfg.rangeNoiseStdDev;
                    double noisyDist = std::max(cfg.minRangeMeters, hit.distance + noise);

                    LiDARPoint pt;
                    pt.distance = noisyDist;
                    pt.localPoint = localDir * noisyDist;
                    pt.worldPoint = sensorWorldPos + worldDir * noisyDist;
                    // Inverse-square intensity attenuation normalized
                    pt.intensity = std::clamp(1.0 / (1.0 + 0.02 * noisyDist * noisyDist), 0.05, 1.0);
                    pt.beamId = b;

                    outPointCloud.push_back(pt);
                    validHits++;
                }
            }
        }
    }

    return validHits;
}

} // namespace Kestrel
