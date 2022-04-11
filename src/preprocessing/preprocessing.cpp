#include <iostream>
#include <memory>
#include <thread>
#include <cmath>

#include "open3d/Open3D.h"

float degToRad(float degree);

struct cameraOrientation
{
    // In meter
    float X;
    float Y;
    float Z;

    // In radian
    float X_ANGLE;
    float Y_ANGLE;
    float Z_ANGLE;
};

const cameraOrientation LEFT_TOP = {
    X: -.5,
    Y: 1,
    Z: 1,

    X_ANGLE: degToRad(120),
    Y_ANGLE: degToRad(120),
    Z_ANGLE: degToRad(120),
};

const cameraOrientation RIGHT_TOP = {
    X: -.5,
    Y: 1,
    Z: 1,

    X_ANGLE: degToRad(120),
    Y_ANGLE: degToRad(120),
    Z_ANGLE: degToRad(120),
};

float degToRad(float degree) {
    return degree*(M_PI/180);
}

void transformPointcloud(open3d::geometry::PointCloud &pointcloud, cameraOrientation orientation) {
    const Eigen::Matrix3d rMat {
        {cos(orientation.Z_ANGLE)*cos(orientation.Y_ANGLE), 
        cos(orientation.Z_ANGLE)*sin(orientation.Y_ANGLE)*sin(orientation.X_ANGLE) - sin(orientation.Z_ANGLE)*cos(orientation.X_ANGLE), 
        cos(orientation.Z_ANGLE)*sin(orientation.Y_ANGLE)*cos(orientation.X_ANGLE) + sin(orientation.Z_ANGLE)*sin(orientation.X_ANGLE)}, 

        {sin(orientation.Z_ANGLE)*cos(orientation.Y_ANGLE), 
        sin(orientation.Z_ANGLE)*sin(orientation.Y_ANGLE)*sin(orientation.X_ANGLE) + cos(orientation.Z_ANGLE)*cos(orientation.X_ANGLE), 
        sin(orientation.Z_ANGLE)*sin(orientation.Y_ANGLE)*cos(orientation.X_ANGLE) - cos(orientation.Z_ANGLE)*sin(orientation.X_ANGLE)},
        
        {-sin(orientation.Y_ANGLE), cos(orientation.Y_ANGLE)*sin(orientation.X_ANGLE), cos(orientation.Y_ANGLE)*cos(orientation.X_ANGLE)}
        };
    
    const Eigen::Vector3d center {0, 0, 0};

    pointcloud.Rotate(rMat, center);
}

// void filterObject(open3d::geometry::PointCloud *poinCloud) {

// }

int main(int argc, char *argv[]) {
    using namespace open3d;

    utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);

    if (argc < 3 ||
        utility::ProgramOptionExistsAny(argc, argv, {"-h", "--help"})) {
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

        transformPointcloud(*cloud_ptr, RIGHT_TOP);

        visualization::DrawGeometries({cloud_ptr}, "PointCloud", 1600, 900);
    }
    utility::LogInfo("End of the test.");

    return 0;
}
