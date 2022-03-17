#include <iostream>
#include <memory>
#include <thread>

#include "open3d/Open3D.h"

void PrintHelp() {
    using namespace open3d;

    PrintOpen3DVersion();
    // clang-format off
    utility::LogInfo("Usage:");
    utility::LogInfo("    > Visualizer [mesh|spin|slowspin|pointcloud|rainbow|image|depth|editing|editmesh] [filename]");
    utility::LogInfo("    > Visualizer [animation] [filename] [trajectoryfile]");
    utility::LogInfo("    > Visualizer [rgbd] [color] [depth] [--rgbd_type]");
    // clang-format on
    utility::LogInfo("");
}

int main(int argc, char *argv[]) {
    using namespace open3d;

    utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);

    if (argc < 3 ||
        utility::ProgramOptionExistsAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return 1;
    }

    std::string option(argv[1]);
     if (option == "pointcloud") {
        auto cloud_ptr = std::make_shared<geometry::PointCloud>();
        if (io::ReadPointCloud(argv[2], *cloud_ptr)) {
            utility::LogInfo("Successfully read {}", argv[2]);
        } else {
            utility::LogWarning("Failed to read {}", argv[2]);
            return 1;
        }
        cloud_ptr->NormalizeNormals();
        visualization::DrawGeometries({cloud_ptr}, "PointCloud", 1600, 900);
    }
    utility::LogInfo("End of the test.");

    return 0;
}
