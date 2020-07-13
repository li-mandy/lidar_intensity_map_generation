#pragma once

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tbb/tbb.h>
#include <pcl/filters/passthrough.h>

#include <generic_logger/generic_logger.hpp>
#include <trajectory/trajectory.hpp>


namespace intensity_map {
namespace intensity_map_data {


using Pose = trajectory::Pose;


/**
 * @brief A struct for image generation configuration parameters from point cloud intensity data.
 * @brief width: the width of the ROI region we want to use to generate topview image.
 * @brief length: the length of the ROI region we want to use to generate topview image.
 * @brief intensityMin: the minimal intensity for filtering data.
 */
struct ImgGenerationCfg {

    ImgGenerationCfg(const double& widthInit,
                     const double& lengthInit,
                     const double& resolutionInit,
                     const double& intensityMinInit)
            : width(widthInit), length(lengthInit), resolution(resolutionInit), intensityMin(intensityMinInit) {
    }

    double width{10.0};
    double length{20.0};
    double resolution{0.05};
    double intensityMin{0.0};
};


/**
 * @brief A class to generate image from the point cloud intensity data.
 * @brief Using the sensor pose in the world coordinate system.
 * @brief Usage: 1. ImgGenerater<> generater(cfg, cloud, pose);
 *               2. generater.process();
 *               3. auto img = generater.getGeneratedImg();
 * @tparam PointT_: the template argument for pcl point type.
 */
template <typename PointT_ = pcl::PointXYZI> // NOLINT
class ImgGenerater {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Point = PointT_;
    using Cloud = pcl::PointCloud<Point>;


    /**
     * @brief Constructor using source point cloud.
     * @param cloud: the source point cloud.
     */
    ImgGenerater(const ImgGenerationCfg& cfg, const typename Cloud::Ptr& cloud, const Pose& pose)
            : cfg_(cfg), cloud_(cloud), pose_(pose) {
    }


    /**
     * @brief Get the configuration parameters.
     * @return ImgGenerationCfg: the configuration parameters.
     */
    ImgGenerationCfg getConfig() const {
        return cfg_;
    }

    /**
     * @brief Get the source input point cloud.
     * @return Cloud::Ptr: the source input point cloud.
     */
    typename Cloud::Ptr getSrcCloud() const {
        return cloud_;
    }

    /**
     * @brief Get the sensor pose.
     * @return Pose: the sensor pose.
     */
    Pose getPose() const {
        return pose_;
    }

    /**
     * @brief Get the pre filtered point cloud.
     * @return Cloud::Ptr: the pre filtered point cloud.
     */
    typename Cloud::Ptr getFilteredCloud() const {
        checkStatus();
        return cloudFiltered_;
    }

    /**
     * @brief Get the filtered point cloud in the sensor frame.
     * @return Cloud::Ptr: the filtered point cloud in the sensor frame.
     */
    typename Cloud::Ptr getSensorCloud() const {
        checkStatus();
        return cloudSensor_;
    }

    /**
     * @brief Get the generated image.
     * @return cv::Mat: the generated image.
     */
    cv::Mat getGeneratedImg() const {
        checkStatus();
        return img_;
    }


    /**
     * @brief Generate an image using point cloud and pose.
     * @return bool: return true if the process is successful.
     */
    bool process() {
        // Filter the point cloud using sensor pose and extension.
        if (filterCloud()) {
            INFO_STREAM("The point cloud pre filtering step is successful.");
        } else {
            ERROR_STREAM("The point cloud pre filtering step is not successful.");
        }

        // Transform the filtered point cloud from world frame into the sensor frame.
        if (transformCloud2SensorFrame()) {
            INFO_STREAM("The point cloud transformation step is successful.");
        } else {
            ERROR_STREAM("The point cloud transformation step is not successful.");
        }

        // Extract an image using the filtered point cloud, pose and configuration parameters.
        if (imgExtracting()) {
            INFO_STREAM("The image extracting step is successful.");
        } else {
            ERROR_STREAM("The image extracting step is not successful.");
        }

        // Set flag as true.
        isProcessed_ = true;
        return true;
    }


private:
    /**
     * @brief Check the status if the image generation is processed.
     * @brief If the algorithm is not processed, then throw a runtime error.
     */
    void checkStatus() const {
        if (!isProcessed_) {
            throw std::runtime_error(
                "The image is not yet generated and you need to run the method 'process()' firstly.");
        }
    }


    /**
     * @brief Filter the point cloud using the pose and x, y extension depending on width and length.
     * @return bool: return ture if the point cloud pre filtering process is successful.
     */
    bool filterCloud() {
        double extension = std::max(cfg_.width, cfg_.length);
        pcl::PassThrough<Point> pass;

        pass.setInputCloud(cloud_);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(pose_.translation()[0] - 2.0 * extension, pose_.translation()[0] + 2.0 * extension);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(pose_.translation()[1] - 2.0 * extension, pose_.translation()[1] + 2.0 * extension);
        pass.filter(*cloudFiltered_);

        return !static_cast<bool>(cloudFiltered_->empty());
    }

    /**
     * @brief Transform the point cloud from world frame to the sensor frame using sensor pose.
     * @return bool: return true if the tranformation process is successful.
     */
    bool transformCloud2SensorFrame() const {
        Point pclP;
        for (const auto& pt : *cloudFiltered_) {
            Eigen::Vector3d p{pt.x, pt.y, pt.z};
            auto pTf = pose_.inverse() * p;
            pclP.x = pTf[0];
            pclP.y = pTf[1];
            pclP.z = pTf[2];
            cloudSensor_->push_back(pclP);
        }

        return true;
    }


    /**
     * @brief Transform the pixel coordinates uv to sensor coordinates using resolution, width and length parameter.
     * @param u: The pixel u coordinate.
     * @param v: The pixel v coordinate.
     * @return Eigen::Vector2d: the transformed point in sensor coordinate system.
     */
    Eigen::Vector2d getXyFromUv(const int& u, const int& v) const {
        Eigen::Vector2d ptTf{Eigen::Vector2d::Zero()};

        ptTf[0] = 0.5 * cfg_.length / -static_cast<double>(v) * cfg_.resolution;
        ptTf[1] = 0.5 * cfg_.width / -static_cast<double>(u) * cfg_.resolution;

        return ptTf;
    }

    /**
     * @brief Get the pixel value from the point cloud intensity.
     * @param pixel: the pixel coordinate in the sensor coordinate system.
     * @return int: the pixel value.
     */
    int getPixelValue(const Eigen::Vector2d& pixel) const {
        double intensity{0.0};
        int value{0};
        int32_t count{0};

        for (const auto& pt : *cloudSensor_) {
            if (std::fabs(pt.x - pixel[0]) < 0.5 * cfg_.resolution &&
                std::fabs(pt.y - pixel[1]) < 0.5 * cfg_.resolution) {
                intensity += static_cast<double>(pt.intensity);
                count++;
            }
        }

        if (count == 0) {
            return value;
        }

        value = static_cast<int>((intensity * 255.0 / static_cast<double>(count)));
        return value;
    }


    /**
     * @brief Image generation using the segmented point cloud in sensor frame.
     * @return bool: return true if the image generation process is successful.
     */
    bool imgExtracting() {
        // Calculate the size of extracted image pixel wise.
        int imgExRows = int(cfg_.length / cfg_.resolution);
        int imgExCols = int(cfg_.width / cfg_.resolution);
        cv::Size imgExSize{imgExCols, imgExRows};
        cv::resize(img_, img_, imgExSize, 0, 0, cv::INTER_CUBIC);
        DEBUG_STREAM("imgExSize: " << imgExSize.width << " * " << imgExSize.height);

        // Calculate the pixel value using point cloud intensity.
        for (int u = 0; u < img_.rows; ++u) {
            for (int v = 0; v < img_.cols; ++v) {
                auto pixel = getXyFromUv(u, v);
                img_.at<char>(u, v) = getPixelValue(pixel);
            }
        }

        return true;
    }


    bool isProcessed_{false};
    ImgGenerationCfg cfg_;
    Pose pose_;
    typename Cloud::Ptr cloud_;
    typename Cloud::Ptr cloudFiltered_;
    typename Cloud::Ptr cloudSensor_;
    cv::Mat img_;
};
} // namespace intensity_map_data
} // namespace intensity_map