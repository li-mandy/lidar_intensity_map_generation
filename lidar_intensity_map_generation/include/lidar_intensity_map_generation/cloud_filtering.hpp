#pragma once

#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <generic_logger/generic_logger.hpp>


namespace intensity_map {
namespace intensity_map_data {


/**
 * @brief A struct for point cloud filtering configuration parameters.
 * @brief preSegZMin: the z min parameter for the naive pre segmentation only using z coordinate.
 * @brief preSegZMax: the z max parameter for the naive pre segmentation only using z coordinate.
 * @brief segMaxDistance: the max distance parameter for the RANSAC plane fitting.
 * @brief segInlierWarningPerc: the inlier percentage threshold to throw warnings.
 */
struct FilterCfg {

    FilterCfg(const double& preSegZMinInit,
              const double& preSegZMaxInit,
              const double& segMaxDistanceInit,
              const double& segInlierWarningPercInit)
            : preSegZMin(preSegZMinInit), preSegZMax(preSegZMaxInit), segMaxDistance(segMaxDistanceInit),
              segInlierWarningPerc(segInlierWarningPercInit) {
    }

    double preSegZMin{-0.2};
    double preSegZMax{0.2};
    double segMaxDistance{0.2};
    double segInlierWarningPerc{0.6};
};


/**
 * @brief A class to segment the ground points from a point cloud.
 * @brief The source input point cloud is firstly filtered using the 'z' coordinate.
 * @brief And then a plane is fitted from this filtered point cloud using RANSAC algorithm.
 * @brief After fitting, the inlier points will be used as result and saved in cloudSegmented_.
 * @brief Usage: 1. CloudFilter<pcl::PointXYZI> filter(cfg, srcCloud);
 *               2. filter.process();
 *               3. auto segmentedCloud = filter.getSegmentedCloud();
 * @tparam PointT_: the template argument for pcl point type.
 */
template <typename PointT_ = pcl::PointXYZI> // NOLINT
class CloudFilter {
public:
    using Point = PointT_;
    using Cloud = pcl::PointCloud<Point>;


    /**
     * @brief Constructor with configs and point cloud.
     * @param cfg: the configs for point cloud filtering.
     * @param cloud: the source point cloud.
     */
    CloudFilter(const FilterCfg& cfg, const typename Cloud::Ptr& cloud) : cfg_(cfg), cloud_(cloud) {
    }


    /**
     * @brief Get the configs.
     * @return filterCfg: the cloud filter configs.
     */
    FilterCfg getConfigs() const {

        return cfg_;
    }

    /**
     * @brief Get the source point cloud.
     * @return Cloud::Ptr: the source point cloud ptr.
     */
    typename Cloud::Ptr getSrcCloud() const {
        checkStatus();
        return cloud_;
    }

    /**
     * @brief Get the filtered point cloud.
     * @return Cloud::Ptr: the filtered point cloud ptr.
     */
    typename Cloud::Ptr getFilteredCloud() const {

        checkStatus();
        return cloudFiltered_;
    }

    /**
     * @brief Get the segmented point cloud.
     * @return Cloud::Ptr: the segmented point cloud ptr.
     */
    typename Cloud::Ptr getSegmentedCloud() const {
        checkStatus();
        return cloudSegmented_;
    }


    /**
     * @brief Process the whole filtering pipeline.
     * @brief Use the source point cloud and surface segmentation.
     * @brief 3 steps: filterCloud(), planeFit(), segment().
     * @return bool: return true if the process is successful.
     */
    bool process() {
        // Point cloud filtering.
        if (filterCloud()) {
            INFO_STREAM("The point cloud filter step is successful.");
        } else {
            ERROR_STREAM("The point cloud filter step is not successful.");
        }

//        // Plane fitting.
//        if (planeFit()) {
//            INFO_STREAM("The plane fit step is successful.");
//        } else {
//            ERROR_STREAM("The plane fit step is not successful.");
//        }
//
//        // Point cloud segmentation.
//        if (segment()) {
//            INFO_STREAM("The segment step is successful.");
//        } else {
//            ERROR_STREAM("The segment step is not successful.");
//        }

        // Set flag as true.
        isFiltered_ = true;
        return true;
    }


private:
    /**
     * @brief Filter the point cloud using the z min and z max.
     * @return void: the cloud_ will be filtered and saved into cloudFiltered.
     */
    bool filterCloud() {
        pcl::PassThrough<Point> pass;

        pass.setInputCloud(cloud_);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(cfg_.preSegZMin, cfg_.preSegZMax);
        pass.filter(*cloudFiltered_);

        return true;
    }

    /**
     * @brief Fit a plane using the points near ground.
     * @return bool: return true if the process is done.
     */
    bool planeFit() {
        pcl::SACSegmentation<Point> seg;

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(cfg_.segMaxDistance);
        seg.setInputCloud(cloudFiltered_);
        seg.segment(*planeInliers_, *planeCoeffs_);

        auto inlierPerc =
            static_cast<double>(planeInliers_->indices.size()) / static_cast<double>(cloudFiltered_->size());
        if (planeInliers_->indices.empty()) {
            throw std::runtime_error("The cloud after plane fitting are empty.");
        }
        if (inlierPerc < cfg_.segInlierWarningPerc) {
            WARN_STREAM("The inlier point percentage: " + std::to_string(inlierPerc) +
                        " is smaller then set: " + std::to_string(cfg_.segInlierWarningPerc));
        }
        return true;
    }

    /**
     * @brief Segment the point cloud using the fitted plane.
     * @return bool: return true if the process is done.
     */
    bool segment() {
        pcl::ExtractIndices<Point> extract;

        extract.setInputCloud(cloudFiltered_);
        extract.setIndices(planeInliers_);
        extract.setNegative(true);
        extract.filter(*cloudSegmented_);

        return true;
    }

    /**
     * @brief Check the status if the point cloud is processed.
     * @brief If the algorithm is not processed, then throw a runtime error.
     */
    void checkStatus() const {
        if (!isFiltered_) {
            throw std::runtime_error(
                "The point cloud is not yet filtered and you need to run the method 'process()' firstly.");
        }
    }


    FilterCfg cfg_;
    bool isFiltered_{false};
    typename Cloud::Ptr cloud_{new Cloud};
    typename Cloud::Ptr cloudFiltered_{new Cloud};
    typename Cloud::Ptr cloudSegmented_{new Cloud};
    pcl::ModelCoefficients::Ptr planeCoeffs_{new pcl::ModelCoefficients};
    pcl::PointIndices::Ptr planeInliers_{new pcl::PointIndices};
};
} // namespace intensity_map_data
} // namespace intensity_map