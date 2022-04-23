#include <cmath>
#include <iostream>
#include <memory>
#include <thread>

#include "open3d/Open3D.h"

/* Parameters */

// Tunning parameters
#define VOXEL_DOWN_SAMPLE 0.006

// Working area
// In meters
#define BOX_SIZE_X 0.25
#define BOX_SIZE_Y 0.25
#define BOX_SIZE_Z 0.25
// In meters
#define BOX_X 0
#define BOX_Y 0.13
#define BOX_Z 0
// In degree
#define BOX_X_ANGLE 0
#define BOX_Y_ANGLE 0
#define BOX_Z_ANGLE 0

// Camera Right Top
// In meters
#define CAMERA_RIGHT_TOP_X 0
#define CAMERA_RIGHT_TOP_Y 0
#define CAMERA_RIGHT_TOP_Z 0
// In degree
#define CAMERA_RIGHT_TOP_X_ANGLE -20
#define CAMERA_RIGHT_TOP_Y_ANGLE 60
#define CAMERA_RIGHT_TOP_Z_ANGLE 0
// In meters
#define CAMERA_RIGHT_TOP_CENTER_CORRECTION_X 0.55
#define CAMERA_RIGHT_TOP_CENTER_CORRECTION_Y 0.42
#define CAMERA_RIGHT_TOP_CENTER_CORRECTION_Z 0.37


// Camera Left Top
// In meters
#define CAMERA_LEFT_TOP_X 0
#define CAMERA_LEFT_TOP_Y 0
#define CAMERA_LEFT_TOP_Z 0
// In degree
#define CAMERA_LEFT_TOP_X_ANGLE -20.5
#define CAMERA_LEFT_TOP_Y_ANGLE -60
#define CAMERA_LEFT_TOP_Z_ANGLE 0
// In meters
#define CAMERA_LEFT_TOP_CENTER_CORRECTION_X -0.63
#define CAMERA_LEFT_TOP_CENTER_CORRECTION_Y 0.42
#define CAMERA_LEFT_TOP_CENTER_CORRECTION_Z 0.26

// Properties of Organized Points
#define GRID_SIZE 5 // In milimeter
#define GRID_SIZE_HALF 2.5

// Function signatures
float degToRad(float);

// Data structures
struct CameraOrientation {
    // In meter
    float X;
    float Y;
    float Z;

    // In radian
    float X_ANGLE;
    float Y_ANGLE;
    float Z_ANGLE;
};

struct OrganizedPointCloud {
    int pointCloudNo;
    Eigen::Vector3d points_;
    Eigen::Vector3d normals_;
    Eigen::Vector3d colors_;
    // Eigen::Matrix3d covariances_;
};

const CameraOrientation LEFT_TOP = {
    X : CAMERA_LEFT_TOP_X + CAMERA_LEFT_TOP_CENTER_CORRECTION_X,
    Y : CAMERA_LEFT_TOP_Y + CAMERA_LEFT_TOP_CENTER_CORRECTION_Y,
    Z : CAMERA_LEFT_TOP_Z + CAMERA_LEFT_TOP_CENTER_CORRECTION_Z,

    X_ANGLE : degToRad(CAMERA_LEFT_TOP_X_ANGLE),
    Y_ANGLE : degToRad(CAMERA_LEFT_TOP_Y_ANGLE),
    Z_ANGLE : degToRad(CAMERA_LEFT_TOP_Z_ANGLE),
};

const CameraOrientation RIGHT_TOP = {
    X : CAMERA_RIGHT_TOP_X + CAMERA_RIGHT_TOP_CENTER_CORRECTION_X,
    Y : CAMERA_RIGHT_TOP_Y + CAMERA_RIGHT_TOP_CENTER_CORRECTION_Y,
    Z : CAMERA_RIGHT_TOP_Z + CAMERA_RIGHT_TOP_CENTER_CORRECTION_Z,

    X_ANGLE : degToRad(CAMERA_RIGHT_TOP_X_ANGLE),
    Y_ANGLE : degToRad(CAMERA_RIGHT_TOP_Y_ANGLE),
    Z_ANGLE : degToRad(CAMERA_RIGHT_TOP_Z_ANGLE),
};

const CameraOrientation CROP_BOX = {
    X : BOX_X,
    Y : BOX_Y,
    Z : BOX_Z,

    X_ANGLE : degToRad(BOX_X_ANGLE),
    Y_ANGLE : degToRad(BOX_Y_ANGLE),
    Z_ANGLE : degToRad(BOX_Z_ANGLE),
};


float degToRad(float degree) { return degree * (M_PI / 180); }

Eigen::Matrix3d getRotationalMatrix(CameraOrientation orientation) {
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

void transformPointcloud(open3d::geometry::PointCloud &pointcloud, CameraOrientation orientation) {
    const Eigen::Matrix3d rMat = getRotationalMatrix(orientation);
    const Eigen::Vector3d center{0, 0, 0};
    const Eigen::Vector3d translate{orientation.X, orientation.Y, orientation.Z};

    pointcloud.Rotate(rMat, center);
    pointcloud.Translate(translate);
}

std::shared_ptr<open3d::geometry::PointCloud> cropWorkspace(open3d::geometry::PointCloud &poinCloud) {
    const Eigen::Vector3d center{CROP_BOX.X, CROP_BOX.Y, CROP_BOX.Z};
    const Eigen::Vector3d extent{BOX_SIZE_X, BOX_SIZE_Y, BOX_SIZE_Z};
    const Eigen::Matrix3d rMat = getRotationalMatrix(CROP_BOX);
    const auto cropBox = open3d::geometry::OrientedBoundingBox(center, rMat, extent);

    return poinCloud.Crop(cropBox);
}

std::tuple<std::shared_ptr<open3d::geometry::PointCloud>, std::vector<size_t>> 
filterObject(open3d::geometry::PointCloud &poinCloud) {
    std::tuple<std::shared_ptr<open3d::geometry::PointCloud>, std::vector<size_t>> preProcessedData = 
        poinCloud.VoxelDownSample(VOXEL_DOWN_SAMPLE)->RemoveRadiusOutliers(20, 0.03, true);
    
    return preProcessedData;
}

void reorderPointCloud(std::map<Eigen::Vector3d, OrganizedPointCloud> *organizedPointMap, open3d::geometry::PointCloud &poinCloud) {
    for(int i; i < poinCloud.points_.size(); i++) {
        Eigen::Vector3d coordinate = poinCloud.points_[i];
        coordinate[0] = (((int) (coordinate[0] * 1000)) / GRID_SIZE + GRID_SIZE_HALF) / 1000.0;
        coordinate[1] = (((int) (coordinate[1] * 1000)) / GRID_SIZE + GRID_SIZE_HALF) / 1000.0;
        coordinate[2] = (((int) (coordinate[2] * 1000)) / GRID_SIZE + GRID_SIZE_HALF) / 1000.0;

        auto iterator = organizedPointMap->find(coordinate);
        if (iterator != organizedPointMap->end()) {
            // not present
            OrganizedPointCloud newOrganizedPointCloud;
            newOrganizedPointCloud.points_ = coordinate;
            newOrganizedPointCloud.colors_ = poinCloud.colors_[i];
            newOrganizedPointCloud.normals_ = poinCloud.normals_[i];
            organizedPointMap->insert({coordinate, newOrganizedPointCloud});
        }
        OrganizedPointCloud newOrganizedPointCloud;

    }
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

    

    transformPointcloud(*pointcloudRightTop, RIGHT_TOP);
    transformPointcloud(*pointcloudLeftTop, LEFT_TOP);

    *pointcloudRightTop.get() = cropWorkspace(*pointcloudRightTop)->PaintUniformColor({1, 0, 0});
    *pointcloudLeftTop.get() = cropWorkspace(*pointcloudLeftTop)->PaintUniformColor({0, 0, 1});

    std::tuple<std::shared_ptr<open3d::geometry::PointCloud>, std::vector<size_t>> preProcessedRightTop =
        filterObject(*pointcloudRightTop);
    pointcloudRightTop = std::get<0>(preProcessedRightTop);

    std::tuple<std::shared_ptr<open3d::geometry::PointCloud>, std::vector<size_t>> preProcessedLeftTop =
        filterObject(*pointcloudLeftTop);
    pointcloudLeftTop = std::get<0>(preProcessedLeftTop);

    auto sphere = open3d::geometry::TriangleMesh::CreateCone(.05, 0.05);
    sphere->ComputeVertexNormals();
    sphere->PaintUniformColor({0.0, 1.0, 0.0});

    *pointcloudLeftTop.get() += *pointcloudRightTop.get();
    std::vector<double> radii = {0.02};

    auto mesh = std::make_shared<geometry::TriangleMesh>();
    // mesh = open3d::geometry::TriangleMesh::CreateFromPointCloudBallPivoting(*pointcloudLeftTop.get(), radii);
    mesh->PaintUniformColor({1.0, 0.0, 0.0});
    // visualization::DrawGeometries({pointcloudLeftTop, mesh, sphere}, "PointCloud", 1600, 900);

    pointcloudRightTop->EstimateNormals();
    pointcloudRightTop->NormalizeNormals();
    pointcloudLeftTop->EstimateNormals();
    pointcloudLeftTop->NormalizeNormals();

    visualization::DrawGeometries({pointcloudRightTop, pointcloudLeftTop}, "PointCloud", 1600, 900, 50, 50, true);

    std::map<Eigen::Vector3d, OrganizedPointCloud> organizedPointMap;
    // <coordinate, OrganizedPointCloud>

    reorderPointCloud(&organizedPointMap, *pointcloudLeftTop);

    return 0;
}
