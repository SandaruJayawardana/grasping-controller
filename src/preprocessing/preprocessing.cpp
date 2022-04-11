#include <iostream>
#include <memory>
#include <thread>
#include <cmath>

#include "open3d/Open3D.h"

float degToRad(float);

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

    X_ANGLE: degToRad(60),
    Y_ANGLE: degToRad(60),
    Z_ANGLE: degToRad(60),
};

const cameraOrientation CROP_BOX = {
    X: -.5,
    Y: 1,
    Z: 1,

    X_ANGLE: degToRad(0),
    Y_ANGLE: degToRad(0),
    Z_ANGLE: degToRad(0),
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
    const Eigen::Vector3d translate {0, 0, 0};

    pointcloud.Rotate(rMat, center);
    pointcloud.Translate(translate);
}

void filterObject(open3d::geometry::PointCloud *poinCloud) {
    const Eigen::Vector3d center {CROP_BOX.X, CROP_BOX.Y, CROP_BOX.Z};
    const Eigen::Vector3d extent {30, 30, 30};
    const Eigen::Matrix3d rMat {
        {cos(CROP_BOX.Z_ANGLE)*cos(CROP_BOX.Y_ANGLE), 
        cos(CROP_BOX.Z_ANGLE)*sin(CROP_BOX.Y_ANGLE)*sin(CROP_BOX.X_ANGLE) - sin(CROP_BOX.Z_ANGLE)*cos(CROP_BOX.X_ANGLE), 
        cos(CROP_BOX.Z_ANGLE)*sin(CROP_BOX.Y_ANGLE)*cos(CROP_BOX.X_ANGLE) + sin(CROP_BOX.Z_ANGLE)*sin(CROP_BOX.X_ANGLE)}, 

        {sin(CROP_BOX.Z_ANGLE)*cos(CROP_BOX.Y_ANGLE), 
        sin(CROP_BOX.Z_ANGLE)*sin(CROP_BOX.Y_ANGLE)*sin(CROP_BOX.X_ANGLE) + cos(CROP_BOX.Z_ANGLE)*cos(CROP_BOX.X_ANGLE), 
        sin(CROP_BOX.Z_ANGLE)*sin(CROP_BOX.Y_ANGLE)*cos(CROP_BOX.X_ANGLE) - cos(CROP_BOX.Z_ANGLE)*sin(CROP_BOX.X_ANGLE)},
        
        {-sin(CROP_BOX.Y_ANGLE), cos(CROP_BOX.Y_ANGLE)*sin(CROP_BOX.X_ANGLE), cos(CROP_BOX.Y_ANGLE)*cos(CROP_BOX.X_ANGLE)}
        };
    const auto cropBox = open3d::geometry::OrientedBoundingBox(center, rMat, extent);

    poinCloud->Crop(cropBox);
}

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

        cloud_ptr = cloud_ptr->VoxelDownSample(0.005);

        visualization::DrawGeometries({cloud_ptr}, "PointCloud", 1600, 900);
    }
    utility::LogInfo("End of the test.");

    return 0;
}
