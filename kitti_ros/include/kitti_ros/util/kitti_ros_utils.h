#ifndef kittirosutil_H
#define kittirosutil_H
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Eigen>
#include <cmath>
#include <opencv/cv.hpp>

namespace kitti_ros_util {

void EulerAngleToQuaternion(double ang, double *x, double *y, double *z,
                            double *w);

// This method Claculates 8 korners of box
// input ; Dimensions, positions of 3D BOX
// Output; 3D coordinates of each korner in camera frame
// Camera frame is defined as , x ; right , y ; down , z; forward
Eigen::MatrixXd ComputeCorners(std::vector<float> dimensions,
                               std::vector<float> positions, float ry);

Eigen::MatrixXf ComputeCornersfromBBX(std::vector<float> dimensions,
                                      std::vector<float> positions, float ry);

void SetMarkerData(visualization_msgs::Marker *marker, double px, double py,
                   double pz, double ox, double oy, double oz, double ow,
                   double sx, double sy, double sz, double r, double g,
                   double b, double a);

Eigen::MatrixXf KornersWorldtoKornersImage(std::vector<float> dimensions,
                                           std::vector<float> positions,
                                           float ry);

void Construct3DBoxOnImage(Eigen::MatrixXf *corners, cv::Mat *image);

std::vector<geometry_msgs::Point> Eigen2GeometryMsgs(Eigen::MatrixXf corners);

double scale_to_255(double a, int min, int max);

cv::Mat point_cloud_to_panorama(pcl::PointCloud<pcl::PointXYZI>::Ptr points,
                                double v_res, double h_res,
                                std::vector<double> v_fov,
                                std::vector<double> d_range, int y_fudge);

};  // namespace kitti_ros_util

#endif