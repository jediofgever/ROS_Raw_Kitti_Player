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

Eigen::MatrixXf ComputeCornersfromBBX(std::vector<float> dimensions,
                                      std::vector<float> positions, float ry) {
    float h, w, l;
    h = dimensions[0];
    w = dimensions[1];
    l = dimensions[2];

    Eigen::MatrixXf corners(3, 8);
    Eigen::Matrix3f rot;

    rot << +cos(ry), 0, +sin(ry), 0, 1, 0, -sin(ry), 0, +cos(ry);

    Eigen::MatrixXf x(1, 8);
    x << -l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2;

    Eigen::MatrixXf y(1, 8);
    y << w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2;

    Eigen::MatrixXf z(1, 8);
    z << -h, -h, -h, -h, 0, 0, 0, 0;

    corners.row(0) = x;
    corners.row(1) = y;
    corners.row(2) = z;

    // corners = rot * corners;
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

Eigen::MatrixXf KornersWorldtoKornersImage(std::vector<float> dimensions,
                                           std::vector<float> positions,
                                           float ry) {
    float h, w, l;
    h = dimensions[0];
    w = dimensions[1];
    l = dimensions[2];

    Eigen::MatrixXf corners(3, 8);
    Eigen::Matrix3f rot;

    rot << +cos(ry), 0, +sin(ry), 0, 1, 0, -sin(ry), 0, +cos(ry);

    Eigen::MatrixXf x(1, 8);
    x << -l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2;

    Eigen::MatrixXf y(1, 8);
    y << w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2;

    Eigen::MatrixXf z(1, 8);
    z << h, h, h, h, 0, 0, 0, 0;

    corners.row(0) = x;
    corners.row(1) = y;
    corners.row(2) = z;

    // corners = rot * corners;
    // std::cout << x.size();
    for (int k = 0; k < x.size(); k++) {
        corners(0, k) += positions[0];
        corners(1, k) += positions[1];
        corners(2, k) += positions[2];
    }
    return corners;
}

void Construct3DBoxOnImage(Eigen::MatrixXf *corners, cv::Mat *image) {
    cv::Scalar clr = cv::Scalar(0, 0, 255);
    cv::Scalar clr_b = cv::Scalar(255, 0, 0);
    cv::Scalar clr_ta = cv::Scalar(0, 255, 255);

    // Declare cv Point to keep pixel coordinatres
    // Declare image_points vector to keep image coordinates
    // returned by CameraReproh->Project3Dpoint
    std::vector<cv::Point> image_points;

    for (int i = 0; i < corners->cols(); i++) {
        cv::Point image_point;
        image_point.x = (*corners)(0, i);
        image_point.y = (*corners)(1, i);
        image_points.push_back(image_point);
    }

    // Draw 12 lines that costructs box

    if (image_points.size() > 7) {
        cv::line(*image, image_points[0], image_points[1], clr_b, 2, 8);
        cv::line(*image, image_points[0], image_points[3], clr, 2, 8);
        cv::line(*image, image_points[0], image_points[4], clr_ta, 2, 8);
        cv::line(*image, image_points[1], image_points[2], clr, 2, 8);
        cv::line(*image, image_points[1], image_points[5], clr_ta, 2, 8);
        cv::line(*image, image_points[2], image_points[6], clr_ta, 2, 8);
        cv::line(*image, image_points[2], image_points[3], clr_b, 2, 8);
        cv::line(*image, image_points[3], image_points[7], clr_ta, 2, 8);
        cv::line(*image, image_points[7], image_points[4], clr, 2, 8);
        cv::line(*image, image_points[7], image_points[6], clr_b, 2, 8);
        cv::line(*image, image_points[4], image_points[5], clr_b, 2, 8);
        cv::line(*image, image_points[5], image_points[6], clr, 2, 8);
    }
}

std::vector<geometry_msgs::Point> Eigen2GeometryMsgs(Eigen::MatrixXf corners) {
    std::vector<geometry_msgs::Point> corners_geometry_msgs;
    for (int i = 0; i < corners.cols(); i++) {
        geometry_msgs::Point korner_point;

        korner_point.x = corners(0, i);
        korner_point.y = corners(1, i);
        korner_point.z = corners(2, i);
        corners_geometry_msgs.push_back(korner_point);
    }
    return corners_geometry_msgs;
}

double scale_to_255(double a, int min, int max) {
    /* Scales an array of values from specified min, max range to 0-255
        Optionally specify the data type of the output (default is uint8)
    */
    return (((a - min) / (max - min)) * 100);
}

cv::Mat point_cloud_to_panorama(pcl::PointCloud<pcl::PointXYZI>::Ptr points,
                                double v_res, double h_res,
                                std::vector<double> v_fov,
                                std::vector<double> d_range, int y_fudge) {
    /* Takes point cloud data as input and creates a 360 degree panoramic
        image, returned as a numpy array.

    Args:
        points: (np array)
            The numpy array containing the point cloud. .
            The shape should be at least Nx3 (allowing for more columns)
            - Where N is the number of points, and
            - each point is specified by at least 3 values (x, y, z)
        v_res: (float)
            vertical angular resolution in degrees. This will influence the
            height of the output image.
        h_res: (float)
            horizontal angular resolution in degrees. This will influence
            the width of the output image.
        v_fov: (tuple of two floats)
            Field of view in degrees (-min_negative_angle, max_positive_angle)
        d_range: (tuple of two floats) (default = (0,100))
            Used for clipping distance values to be within a min and max range.
        y_fudge: (float)
            A hacky fudge factor to use if the theoretical calculations of
            vertical image height do not match the actual data.
    Returns:
        a 360 degree panoramic image of the point
        cloud.
    */
    int x_max = 360.0 / h_res;
    double v_fov_total = -v_fov[0] + v_fov[1];

    // THEORETICAL MAX HEIGHT FOR IMAGE
    double d_plane = (v_fov_total / v_res) / (v_fov_total * (3.14 / 180));
    double h_below = d_plane * std::tan(-v_fov[0] * (3.14 / 180));
    double h_above = d_plane * std::tan(v_fov[1] * (3.14 / 180));
    int y_max = (h_below + h_above + y_fudge);

    cv::Mat img1(y_max + 2, x_max + 2, CV_8UC3, cv::Scalar(0, 0, 0));

    // Projecting to 2D
    for (int i = 0; i < points->points.size(); i++) {
        double x_points = points->points.at(i).x;
        double y_points = points->points.at(i).y;
        double z_points = points->points.at(i).z;
        double r_points = points->points.at(i).intensity;
        // map distance relative to origin

        double d_points = std::sqrt(x_points * x_points + y_points * y_points);
        // abs distance

        // We use map distance, because otherwise it would
        // not project onto a cylinder, instead, it would
        // map onto a segment of slice of a sphere.

        // RESOLUTION AND FIELD OF VIEW SETTINGS

        // CONVERT TO RADIANS
        double v_res_rad = v_res * (3.14 / 180);
        double h_res_rad = h_res * (3.14 / 180);

        // MAPPING TO CYLINDER
        double x_img = std::atan2(y_points, x_points) / h_res_rad;
        double y_img = -(std::atan2(z_points, d_points) / v_res_rad);

        // SHIFT COORDINATES TO MAKE 0, 0 THE MINIMUM
        int x_min = -360.0 / h_res / 2;
        x_img = (-x_img - x_min);

        int y_min = -((v_fov[1] / v_res) + y_fudge);
        y_img = y_img - y_min;

        // CLIP DISTANCES
        // d_points = np.clip(d_points, a_min = d_range[0], a_max =
        // d_range[1])

        // CONVERT TO IMAGE ARRAY
        double color_value = scale_to_255(r_points, d_range[0], d_range[1]);

        cv::Vec3b color;
        color.val[0] = color_value;
        color.val[1] = color_value;
        color.val[2] = color_value;

        img1.at<cv::Vec3b>(cv::Point(x_img, y_img)) = color;
    }

    return img1;
}

};  // namespace kitti_ros_util
