#pragma once

#include <param_config/param_config.h>


namespace intensity_map {
namespace intensity_map_data {
using namespace param_config;


/**
 * @brief Configuration parameters interface for image generation from point cloud intensity data.
 */
struct ParamConfig : public Config {
    virtual void fromConfig(const YAML::Node& n) override {
        loggingLevel = get_yaml<std::string>(n, "loggingLevel");
        workDir = get_yaml<std::string>(n, "workDir");

        filteringPreSegZMin = get_yaml<double>(n, "filteringPreSegZMin");
        filteringPreSegZMax = get_yaml<double>(n, "filteringPreSegZMax");
        filteringSegMaxDistance = get_yaml<double>(n, "filteringSegMaxDistance");
        filteringSegInlierWarningPerc = get_yaml<double>(n, "filteringSegInlierWarningPerc");

        generationROIWidth = get_yaml<double>(n, "generationROIWidth");
        generationROILength = get_yaml<double>(n, "generationROILength");
        generationImgResolution = get_yaml<double>(n, "generationImgResolution");
        generationIntensityMin = get_yaml<double>(n, "generationIntensityMin");
    }

    inline std::ostream& print(std::ostream& os) const override {
        return os << "Parameters:\n========================================"
                  << "\nloggingLevel: " << loggingLevel << "\nworkDir: " << workDir
                  << "\nfilteringPreSegZMin: " << filteringPreSegZMin
                  << "\nfilteringPreSegZMax: " << filteringPreSegZMax
                  << "\nfilteringSegMaxDistance: " << filteringSegMaxDistance
                  << "\nfilteringSegInlierWarningPerc: " << filteringSegInlierWarningPerc
                  << "\ngenerationROIWidth: " << generationROIWidth << "\ngenerationROILength: " << generationROILength
                  << "\ngenerationImgResolution: " << generationImgResolution
                  << "\ngenerationIntensityMin: " << generationIntensityMin
                  << "\n========================================" << std::endl;
    }

    /// Generic logger level.
    std::string loggingLevel{""};

    /// Work directory.
    std::string workDir{""};

    /// Point cloud filtering.
    double filteringPreSegZMin{-0.2};
    double filteringPreSegZMax{0.2};
    double filteringSegMaxDistance{0.2};
    double filteringSegInlierWarningPerc{0.6};

    /// Image generation.
    double generationROIWidth{10.0};
    double generationROILength{20.0};
    double generationImgResolution{0.05};
    double generationIntensityMin{0.0};
};
} // namespace intensity_map_data
} // namespace intensity_map
