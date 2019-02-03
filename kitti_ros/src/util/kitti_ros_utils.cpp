#include <kitti_ros/util/kitti_ros_utils.h>

using namespace std;

namespace kitti_ros_util {

void EulerAngleToQuaternion(double ang, double *x, double *y, double *z,
                            double *w) {
    double rad = M_PI * (-ang) / 180;
    *x = 0;
    *y = 0;
    *z = std::sin(rad / 2);
    *w = std::cos(rad / 2);
}

// This method Claculates 8 korners of box
// input ; Dimensions, positions of 3D BOX
// Output; 3D coordinates of each korner in camera frame
// Camera frame is defined as , x ; right , y ; down , z; forward
Eigen::MatrixXd ComputeCorners(std::vector<float> dimensions,
                               std::vector<float> positions, float ry) {
    float h, w, l;
    h = dimensions[0];
    w = dimensions[1];
    l = dimensions[2];

    Eigen::MatrixXd corners(3, 8);
    Eigen::Matrix3d rot;

    rot << +cos(ry), 0, +sin(ry), 0, 1, 0, -sin(ry), 0, +cos(ry);

    Eigen::MatrixXd x(1, 8);
    x << -l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2;

    Eigen::MatrixXd z(1, 8);
    z << w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2;

    Eigen::MatrixXd y(1, 8);
    y << -h, -h, -h, -h, 0, 0, 0, 0;

    corners.row(0) = x;
    corners.row(1) = y;
    corners.row(2) = z;

    corners = rot * corners;
    // std::cout << x.size();
    for (int k = 0; k < x.size(); k++) {
        corners(0, k) += positions[0];
        corners(1, k) += positions[1];
        corners(2, k) += positions[2];
    }
    return corners;
}

Eigen::MatrixXd ComputeCornersfromBBX(std::vector<float> dimensions,
                                      std::vector<float> positions, float ry) {
    float h, w, l;
    h = dimensions[0];
    w = dimensions[1];
    l = dimensions[2];

    Eigen::MatrixXd corners(3, 8);
    Eigen::Matrix3d rot;

    rot << +cos(ry), 0, +sin(ry), 0, 1, 0, -sin(ry), 0, +cos(ry);

    Eigen::MatrixXd x(1, 8);
    x << -l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2;

    Eigen::MatrixXd y(1, 8);
    y << w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2;

    Eigen::MatrixXd z(1, 8);
    z << h, h, h, h, 0, 0, 0, 0;

    corners.row(0) = x;
    corners.row(1) = y;
    corners.row(2) = z;

    corners = rot * corners;
    // std::cout << x.size();
    for (int k = 0; k < x.size(); k++) {
        corners(0, k) += positions[0];
        corners(1, k) += positions[1];
        corners(2, k) += positions[2];
    }
    return corners;
}

void SetMarkerData(visualization_msgs::Marker *marker, double px, double py,
                   double pz, double ox, double oy, double oz, double ow,
                   double sx, double sy, double sz, double r, double g,
                   double b, double a) {
    marker->pose.position.x = px;
    marker->pose.position.y = py;
    marker->pose.position.z = pz;

    marker->pose.orientation.x = ox;
    marker->pose.orientation.y = oy;
    marker->pose.orientation.z = oz;
    marker->pose.orientation.w = ow;

    marker->scale.x = sx;
    marker->scale.y = sy;
    marker->scale.z = sz;

    marker->color.r = r;
    marker->color.g = g;
    marker->color.b = b;
    marker->color.a = a;
}

};  // namespace kitti_ros_util
