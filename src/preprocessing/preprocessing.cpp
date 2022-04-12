#include <cmath>
#include <iostream>
#include <memory>
#include <thread>

#include "open3d/Open3D.h"

/* Parameters */

// Tunning parameters
#define VOXEL_DOWN_SAMPLE 0.005

// Working area
// In meters
#define BOX_X 0.30
#define BOX_Y 0.30
#define BOX_Z 0.30
// In radian
#define BOX_X_ANGLE 0.30
#define BOX_Y_ANGLE 0.30
#define BOX_Z_ANGLE 0.30

// Working area
// In meters
#define BOX_X 0.30
#define BOX_Y 0.30
#define BOX_Z 0.30
// In radian
#define BOX_X_ANGLE 0.30
#define BOX_Y_ANGLE 0.30
#define BOX_Z_ANGLE 0.30

// Camera Right Top
// In meters
#define CAMERA_RIGHT_TOP_X 0.30
#define CAMERA_RIGHT_TOP_Y 0.30
#define CAMERA_RIGHT_TOP_Z 0.30
// In radian
#define CAMERA_RIGHT_TOP_X_ANGLE 0.30
#define CAMERA_RIGHT_TOP_Y_ANGLE 0.30
#define CAMERA_RIGHT_TOP_Z_ANGLE 0.30

// Camera Left Top
// In meters
#define CAMERA_LEFT_TOP_X 0.30
#define CAMERA_LEFT_TOP_X 0.30
#define CAMERA_LEFT_TOP_X 0.30
// In radian
#define CAMERA_LEFT_TOP_X_ANGLE 0.30
#define CAMERA_LEFT_TOP_Y_ANGLE 0.30
#define CAMERA_LEFT_TOP_Z_ANGLE 0.30

// Function signatures
float degToRad(float);

// Data structures
struct cameraOrientation {
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
    X : -.5,
    Y : 1,
    Z : 1,

    X_ANGLE : degToRad(120),
    Y_ANGLE : degToRad(120),
    Z_ANGLE : degToRad(120),
};

const cameraOrientation RIGHT_TOP = {
    X : -.5,
    Y : 1,
    Z : 1,

    X_ANGLE : degToRad(60),
    Y_ANGLE : degToRad(60),
    Z_ANGLE : degToRad(60),
};

const cameraOrientation CROP_BOX = {
    X : -.5,
    Y : 1,
    Z : 1,

    X_ANGLE : degToRad(0),
    Y_ANGLE : degToRad(0),
    Z_ANGLE : degToRad(0),
};


float degToRad(float degree) { return degree * (M_PI / 180); }

Eigen::Matrix3d getRotationalMatrix(cameraOrientation orientation) {
    Eigen::Matrix3d rMat{{cos(orientation.Z_ANGLE) * cos(orientation.Y_ANGLE),
                                cos(orientation.Z_ANGLE) * sin(orientation.Y_ANGLE) * sin(orientation.X_ANGLE) -
                                        sin(orientation.Z_ANGLE) * cos(orientation.X_ANGLE),
                                cos(orientation.Z_ANGLE) * sin(orientation.Y_ANGLE) * cos(orientation.X_ANGLE) +
                                        sin(orientation.Z_ANGLE) * sin(orientation.X_ANGLE)},

                               {sin(orientation.Z_ANGLE) * cos(orientation.Y_ANGLE),
                                sin(orientation.Z_ANGLE) * sin(orientation.Y_ANGLE) * sin(orientation.X_ANGLE) +
                                        cos(orientation.Z_ANGLE) * cos(orientation.X_ANGLE),
                                sin(orientation.Z_ANGLE) * sin(orientation.Y_ANGLE) * cos(orientation.X_ANGLE) -
                                        cos(orientation.Z_ANGLE) * sin(orientation.X_ANGLE)},

                               {-sin(orientation.Y_ANGLE), cos(orientation.Y_ANGLE) * sin(orientation.X_ANGLE),
                                cos(orientation.Y_ANGLE) * cos(orientation.X_ANGLE)}};
    
    return rMat;
}

void transformPointcloud(open3d::geometry::PointCloud &pointcloud, cameraOrientation orientation) {
    const Eigen::Matrix3d rMat = getRotationalMatrix(orientation);
    const Eigen::Vector3d center{0, 0, 0};
    const Eigen::Vector3d translate{0, 0, 0};

    pointcloud.Rotate(rMat, center);
    pointcloud.Translate(translate);
}

void filterObject(open3d::geometry::PointCloud *poinCloud) {
    const Eigen::Vector3d center{CROP_BOX.X, CROP_BOX.Y, CROP_BOX.Z};
    const Eigen::Vector3d extent{30, 30, 30};
    const Eigen::Matrix3d rMat = getRotationalMatrix(CROP_BOX);
    const auto cropBox = open3d::geometry::OrientedBoundingBox(center, rMat, extent);

    poinCloud->Crop(cropBox);
}

int main(int argc, char *argv[]) {
    using namespace open3d;

    utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);

    if (argc < 3 || utility::ProgramOptionExistsAny(argc, argv, {"-h", "--help"})) {
        return 1;
    }

    auto pointcloudRightTop = std::make_shared<geometry::PointCloud>();
    auto pointcloudLeftTop = std::make_shared<geometry::PointCloud>();

    if (io::ReadPointCloud(argv[1], *pointcloudRightTop)) {
        utility::LogInfo("Successfully read {}", argv[1]);
    } else {
        utility::LogWarning("Failed to read {}", argv[1]);
        return 1;
    }

    if (io::ReadPointCloud(argv[2], *pointcloudLeftTop)) {
        utility::LogInfo("Successfully read {}", argv[2]);
    } else {
        utility::LogWarning("Failed to read {}", argv[2]);
        return 1;
    }

    pointcloudRightTop->NormalizeNormals();

    transformPointcloud(*pointcloudRightTop, RIGHT_TOP);

    pointcloudRightTop = pointcloudRightTop->VoxelDownSample(0.005);
    pointcloudLeftTop = pointcloudLeftTop->VoxelDownSample(0.005);

    visualization::DrawGeometries({pointcloudRightTop}, "PointCloud", 1600, 900);

    utility::LogInfo("End of the test.");

    return 0;
}
