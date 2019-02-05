#ifndef kittirosutil_H
#define kittirosutil_H
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

void Construct3DBoxOnImage(Eigen::MatrixXf *corners, cv::Mat *image);

};  // namespace kitti_ros_util

#endif