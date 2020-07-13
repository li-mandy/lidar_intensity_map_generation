#include <chrono>
#include <filesystem>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <tbb/tbb.h>

#include <extracted_msgs_file_io/extracted_msgs_file_io.h>
#include <generic_logger/generic_logger.hpp>
#include <lidar_intensity_map_generation/cloud_filtering.hpp>
#include <lidar_intensity_map_generation/cloud_fusion.hpp>
#include <lidar_intensity_map_generation/config.hpp>
#include <trajectory/trajectory.hpp>
#include <spdlog/sinks/stdout_color_sinks.h>


namespace fs = std::filesystem;
namespace po = boost::program_options;
namespace io = extracted_msgs_file_io;
namespace im = intensity_map;
namespace imd = im::intensity_map_data;
using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;


void setEnvironment(const std::string& loggingLevel) {
    auto colorSink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    generic_logger::set_sink(colorSink);
    generic_logger::set_level(spdlog::level::from_str(loggingLevel));
}


int main(int argc, char* argv[]) { // NOLINT
    //![Program_Options]
    std::string cfgFile;
    std::string poseFile;
    std::string outputCloud;
    bool fusionClouds{true};
    po::options_description d("Usage: rosrun lidar_intensity_map_generation_tool cloud_filtering_and_fusion"
                              " -c <cfgFile> -p <poseFile> -o <outputCloud>, -f <fusionClouds>, where "
                              "cfgFile is the configuration file for parameters, "
                              "poseFile is the pose file path, "
                              "outputCloud is the file path to save the output point cloud pcd, "
                              "fusionClouds flag if fusion the filtered point clouds");
    d.add_options()("help,h", "Help messages")(
        "cfgFile,c", po::value<std::string>(&cfgFile), "The configuration file for parameters.")(
        "poseFile,p", po::value<std::string>(&poseFile), "The pose file path.")(
        "outputCloud,o", po::value<std::string>(&outputCloud), "The file path to save the output point cloud pcd.")(
        "fusionClouds,f", po::bool_switch(&fusionClouds), "The flag if fusion the filtered point clouds.");
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, d), vm);
    po::notify(vm);
    if (vm.count("help") != 0u) {
        std::cout << d << "\n";
        return 1;
    }
    if (vm.count("cfgFile") != 0u) {
        std::cout << "The configuration file was set to " << cfgFile << ".\n";
    } else {
        std::cout << "The configuration file was not set.\n";
    }
    if (vm.count("poseFile") != 0u) {
        std::cout << "The pose file was set to " << poseFile << ".\n";
    } else {
        std::cout << "The pose file was not set.\n";
    }
    if (vm.count("outputCloud") != 0u) {
        std::cout << "The path to save the output point cloud pcd file was set to " << outputCloud << ".\n";
    } else {
        std::cout << "The path to save the output point cloud pcd file was not set.\n";
    }
    if (vm.count("fusionClouds") != 0u) {
        std::cout << "The filtered clouds will be fused and saved into: " << outputCloud << ".\n";
    } else {
        std::cout << "The filtered clouds will not be fused.\n";
    }

    //![Get_Configuration]
    imd::ParamConfig cfg;
    cfg.fromConfigFile(cfgFile);

    //![Set_Logging_Level]
    setEnvironment(cfg.loggingLevel);

    //![Read_Message_Files]
    boost::filesystem::path messagePath(cfg.workDir + "/messages/");
    io::DataRead data(messagePath);

    //![Filtering_Clouds]
    auto elapsedStart = std::chrono::high_resolution_clock::now();
    imd::FilterCfg filterCfg(cfg.filteringPreSegZMin,
                             cfg.filteringPreSegZMax,
                             cfg.filteringSegMaxDistance,
                             cfg.filteringSegInlierWarningPerc);
    auto srcCloudsMsg = data.getMsgFromName(io::Name("pointcloud"));
    auto times = srcCloudsMsg.getTimes();
    std::vector<io::ID> ids;
    for_each(times.begin(), times.end(), [&ids](const auto& time) { ids.push_back(time.first); });
    tbb::parallel_for(static_cast<size_t>(0), ids.size(), [&](size_t i) {
        auto srcCloudPath = data.getFilePathInMsgDirectory(io::Name("pointcloud"), ids[i], 10);
        auto srcCloud = io::loadPointCloud<Point>(srcCloudPath);
        imd::CloudFilter<Point> filter(filterCfg, srcCloud);
        filter.process();
        auto cloudFiltered = filter.getSegmentedCloud();
        auto cloudFilterdPath = boost::filesystem::path(srcCloudPath.parent_path().parent_path().generic_string() +
                                                        "/filteredcloud/" + srcCloudPath.filename().generic_string());
        io::savePointCloud<Point>(cloudFiltered, cloudFilterdPath);
    });
    auto elapsedEnd = std::chrono::high_resolution_clock::now();
    INFO_STREAM(
        "Elapsed time for point clouds filtering: "
        << std::chrono::duration_cast<std::chrono::seconds>(elapsedEnd - elapsedStart).count() << " seconds, "
        << std::chrono::duration_cast<std::chrono::milliseconds>(
               elapsedEnd - elapsedStart - std::chrono::duration_cast<std::chrono::seconds>(elapsedEnd - elapsedStart))
               .count()
        << " milliseconds.");

    //![Fusion_Clouds]
    elapsedStart = std::chrono::high_resolution_clock::now();
    trajectory::Trajectory<trajectory::GraphTrajectory> poses;
    poses.loadTrajectoryFromFile(fs::path(poseFile));
    auto filteredCloudsMsg = data.getMsgFromName(io::Name("filteredcloud"));
    times = filteredCloudsMsg.getTimes();
    imd::CloudFusion<Point, trajectory::GraphTrajectory>::StampedClouds stampedClouds;
    for (const auto& time : times) {
        auto filteredCloudPath = data.getFilePathInMsgDirectory(io::Name("filteredcloud"), time.first, 10);
        auto filteredCloud = io::loadPointCloud<Point>(filteredCloudPath);
        imd::CloudFusion<Point, trajectory::GraphTrajectory>::StampedCloud stampedCloud{
            std::make_pair(trajectory::timeToTimePoint(time.second), filteredCloud)};
        stampedClouds.push_back(stampedCloud);
    }
    imd::CloudFusion<Point, trajectory::GraphTrajectory> fusion(poses, stampedClouds);
    fusion.fusing();
    auto cloudFused = fusion.getFusedCloud();
    elapsedEnd = std::chrono::high_resolution_clock::now();
    INFO_STREAM(
        "Elapsed time for point clouds fusion: "
        << std::chrono::duration_cast<std::chrono::seconds>(elapsedEnd - elapsedStart).count() << " seconds, "
        << std::chrono::duration_cast<std::chrono::milliseconds>(
               elapsedEnd - elapsedStart - std::chrono::duration_cast<std::chrono::seconds>(elapsedEnd - elapsedStart))
               .count()
        << " milliseconds.");

    //![Save_Fused_Cloud]
    boost::filesystem::path outputCloudPath(outputCloud);
    io::savePointCloud<Point>(cloudFused, outputCloudPath);

    return 1;
}
