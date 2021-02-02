#include <chrono>
#include <ostream>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tbb/tbb.h>

#include <extracted_msgs_file_io/extracted_msgs_file_io.h>
#include <generic_logger/generic_logger.hpp>
#include <spdlog/sinks/stdout_color_sinks.h>


namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace io = extracted_msgs_file_io;
using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;


void setEnvironment(const std::string& loggingLevel) {
    auto colorSink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    generic_logger::set_sink(colorSink);
    generic_logger::set_level(spdlog::level::from_str(loggingLevel));
}


Cloud::Ptr kittiBin2Pcd(const fs::path& path) {
    std::fstream file(path, std::ios::in | std::ios::binary);
    if (!file.good()) {
        throw std::runtime_error("The file path is invalid: " + path.generic_string());
    }

    file.seekg(0, std::ios::beg);
    Cloud::Ptr cloud(new Cloud);
    do {
        Point pt;
        file.read((char*)&pt.x, 3 * sizeof(float));     // NOLINT
        file.read((char*)&pt.intensity, sizeof(float)); // NOLINT
        cloud->push_back(pt);
    } while (file.good());

    file.close();
    DEBUG_STREAM("Load " << cloud->size() << " points from file: " << path);
    return cloud;
}


int main(int argc, char* argv[]) { // NOLINT
    //![Program_Options]
    std::string srcDir;
    std::string dstDir;
    std::string loggingLevel;
    po::options_description d("Usage: rosrun lidar_intensity_map_generation_tool convert_bins_2_pcds"
                              " -s <srcDir> -d <dstDir>, -l <loggingLevel> where "
                              "srcDir is the root directory of the source bin files, "
                              "dstDir is the root directory of the destination pcd files.");
    d.add_options()("help,h", "Help messages")(
        "srcDir,s", po::value<std::string>(&srcDir), "The root directory of the source bin files.")(
        "dstDir,d", po::value<std::string>(&dstDir), "The root directory of the destination pcd files.")(
        "loggingLevel,l", po::value<std::string>(&loggingLevel)->default_value("info"), "The logging level.");
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, d), vm);
    po::notify(vm);
    if (vm.count("help") != 0u) {
        std::cout << d << "\n";
        return 1;
    }
    if (vm.count("srcDir") != 0u) {
        std::cout << "The root directory of the source bin files was set to " << srcDir << ".\n";
    } else {
        std::cout << "The root directory of the source bin files was not set.\n";
    }
    if (vm.count("dstDir") != 0u) {
        std::cout << "The root directory of the destination pcd files was set to " << srcDir << ".\n";
    } else {
        std::cout << "The root directory of the destination pcd files was not set.\n";
    }
    if (vm.count("loggingLevel") != 0u) {
        std::cout << "The logging level was set to " << loggingLevel << ".\n";
    } else {
        std::cout << "The logging level was not set.\n";
    }

    //![Set_Logging_Level]
    setEnvironment(loggingLevel);

    //![Read_Files]
    fs::path srcDirPath(srcDir);
    auto srcFiles = io::getFilesInDirectory(srcDirPath);

    //![Process_And_Save_Files]
    auto elapsedStart = std::chrono::high_resolution_clock::now();
    tbb::parallel_for(static_cast<size_t>(0), srcFiles.size(), [&](size_t i) {
        INFO_STREAM("Process:[" << i << "/" << srcFiles.size() << "]");
        fs::path dstFile{dstDir + "/" + srcFiles[i].stem().generic_string() + "/.pcd"};
        auto cloud = kittiBin2Pcd(srcFiles[i]);
        io::savePointCloud<Point>(cloud, dstFile);
    });
    auto elapsedEnd = std::chrono::high_resolution_clock::now();
    INFO_STREAM("Elapsed time : " << std::chrono::duration_cast<std::chrono::seconds>(elapsedEnd - elapsedStart).count()
                                  << " seconds, "
                                  << std::chrono::duration_cast<std::chrono::milliseconds>(
                                         elapsedEnd - elapsedStart -
                                         std::chrono::duration_cast<std::chrono::seconds>(elapsedEnd - elapsedStart))
                                         .count()
                                  << " milliseconds.");

    return 1;
}
