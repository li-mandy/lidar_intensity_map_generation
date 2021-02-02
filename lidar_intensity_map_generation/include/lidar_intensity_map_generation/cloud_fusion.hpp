#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tbb/tbb.h>

#include <generic_logger/generic_logger.hpp>
#include <trajectory/trajectory.hpp>


namespace intensity_map {
namespace intensity_map_data {


using Point3d = trajectory::Point3d;


/**
 * @brief Convert pcl point into eigen 3d point.
 * @tparam PointT: the template for pcl point type.
 * @param pclPt: the input pcl point.
 * @return Point3d: the converted eigen 3d point.
 */
template <typename PointT>
inline Point3d pclPoint2Point3d(const PointT& pclPt) {
    Point3d pt3d(pclPt.x, pclPt.y, pclPt.z);
    return pt3d;
}


/**
 * @brief A class to fuse stamped point clouds.
 * @brief Usage: 1. CloudFusion<> fusion(trajectory, clouds);
 *               2. fusion.fusing();
 *               3. auto fusedCloud = fusion.getFusedCloud();
 * @tparam PointT_: the template argument for pcl point type.
 * @tparam TrajectoryType_: the template argument for trajectory type.
 */
template <typename PointT_ = pcl::PointXYZI, typename TrajectoryType_ = trajectory::GraphTrajectory> // NOLINT
class CloudFusion {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Point = PointT_;
    using Cloud = pcl::PointCloud<Point>;
    using TrajectoryType = TrajectoryType_;
    using Trajectory = trajectory::Trajectory<TrajectoryType>;
    using TimePoint = trajectory::TimePoint;
    using StampedCloud = std::pair<TimePoint, typename Cloud::Ptr>;
    using StampedClouds = std::vector<StampedCloud>;
    using Pose = trajectory::Pose;


    /**
     * @brief Constructor with trajectory.
     * @param clouds: the stamped source point clouds.
     */
    CloudFusion(const Trajectory& trajectory, const StampedClouds& clouds)
            : trajectory_(trajectory), srcClouds_(clouds) {
    }


    /**
     * @brief Get the saved trajectory.
     * @return Trajectory: the trajectory.
     */
    Trajectory getTrajectory() const {
        return trajectory_;
    }

    /**
     * @brief Get the source stamped point clouds.
     * @return StampedClouds: the source stamped point clouds.
     */
    StampedClouds getSrcClouds() const {
        return srcClouds_;
    }

    /**
     * @brief Get the fused point cloud.
     * @return Cloud::Ptr: the fused point cloud.
     */
    typename Cloud::Ptr getFusedCloud() const {
        checkStatus();
        return cloudFused_;
    }


    /**
     * @brief Fuse the input point clouds using the trajectory.
     * @return bool: return true if the fusing process is successful.
     */
    bool fusing() {
        tbb::parallel_for(static_cast<size_t>(0), srcClouds_.size(), [&](size_t i) {
            INFO_STREAM("Point cloud fusion process: [" << i << "/" << srcClouds_.size() << "].");
            auto timePoint = srcClouds_[i].first;
            auto cloud = srcClouds_[i].second;
            Point ptTf;
            for (const auto& pt : *cloud) {
                auto pt3d = pclPoint2Point3d<Point>(pt);
                auto pt3dTf = trajectory_.transformPoint3d(pt3d, timePoint);
                ptTf.x = pt3dTf[0];
                ptTf.y = pt3dTf[1];
                ptTf.z = pt3dTf[2];
                ptTf.intensity = pt.intensity;
                cloudFused_->push_back(ptTf);
            }
        });
        isProcessed_ = true;
        return true;
    }


private:
    /**
     * @brief Check the status if the point cloud is processed.
     * @brief If the algorithm is not processed, then throw a runtime error.
     */
    void checkStatus() const {
        if (!isProcessed_) {
            throw std::runtime_error(
                "The point cloud is not yet fused and you need to run the method 'fusing()' firstly.");
        }
    }


    bool isProcessed_{false};
    StampedClouds srcClouds_{};
    typename Cloud::Ptr cloudFused_;
    Trajectory trajectory_;
};
} // namespace intensity_map_data
} // namespace intensity_map