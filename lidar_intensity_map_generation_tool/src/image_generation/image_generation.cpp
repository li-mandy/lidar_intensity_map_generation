#include <chrono>
#include <filesystem>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <opencv2/opencv.hpp>
#include <tbb/tbb.h>

#include <extracted_msgs_file_io/extracted_msgs_file_io.h>
#include <generic_logger/generic_logger.hpp>
#include <lidar_intensity_map_generation/config.hpp>
#include <lidar_intensity_map_generation/image_generation.hpp>
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
    po::options_description d("Usage: rosrun lidar_intensity_map_generation_tool cloud_filtering_and_fusion"
                              " -c <cfgFile> where cfgFile is the configuration file for parameters.");
    d.add_options()("help,h", "Help messages")(
        "cfgFile,c", po::value<std::string>(&cfgFile), "The configuration file for parameters.");
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

    //![Get_Configuration]
    imd::ParamConfig cfg;
    cfg.fromConfigFile(cfgFile);

    //![Set_Logging_Level]
    setEnvironment(cfg.loggingLevel);

    //![Image_Generation]
    auto elapsedStart = std::chrono::high_resolution_clock::now();
    imd::ImgGenerationCfg imgGenerationCfg(
        cfg.generationROIWidth, cfg.generationROILength, cfg.generationImgResolution, cfg.generationIntensityMin);
    boost::filesystem::path cloudPath(cfg.workDir + "/fused_cloud.pcd");
    fs::path posePath(cfg.workDir + "/poses.txt");
    fs::path imageDir(cfg.workDir + "/topviews/");
    auto cloud = io::loadPointCloud<Point>(cloudPath);
    trajectory::Trajectory<trajectory::GraphTrajectory> poses;
    poses.loadTrajectoryFromFile(posePath);
    auto posesData = poses.getTrajectory().getTrajectory();
    std::vector<std::pair<io::TIME, imd::Pose>> stampedPoseVec;
    for_each(posesData.begin(), posesData.end(), [&stampedPoseVec](const auto& poseData) {
        stampedPoseVec.push_back(std::make_pair(poseData.first, poseData.second.second));
    });
    tbb::parallel_for(static_cast<size_t>(0), stampedPoseVec.size(), [&](size_t i) {
        imd::ImgGenerater<Point> generater(imgGenerationCfg, cloud, stampedPoseVec[i].second);
        generater.process();
        auto img = generater.getGeneratedImg();
        fs::path imageFile(imageDir.generic_string() + "/" + std::to_string(stampedPoseVec[i].first) + ".png");
        cv::imwrite(imageFile.generic_string(), img);
    });
    auto elapsedEnd = std::chrono::high_resolution_clock::now();
    INFO_STREAM(
        "Elapsed time for image generation: "
        << std::chrono::duration_cast<std::chrono::seconds>(elapsedEnd - elapsedStart).count() << " seconds, "
        << std::chrono::duration_cast<std::chrono::milliseconds>(
               elapsedEnd - elapsedStart - std::chrono::duration_cast<std::chrono::seconds>(elapsedEnd - elapsedStart))
               .count()
        << " milliseconds.");

    return 1;
}
