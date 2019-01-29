#include <kitti_ros/kitti_data_operator.h>

KITTIDataOperator::KITTIDataOperator() {
    nh_ = ros::NodeHandlePtr(new ros::NodeHandle());
};

KITTIDataOperator::~KITTIDataOperator(){};

// Get the recorded Lidar scan pointcloud.
// return: Pointcloud ROS type.
const sensor_msgs::PointCloud2 KITTIDataOperator::GetLidarScan() const {
    return lidar_scan_;
}

// Set the recorded Lidar scan pointcloud.
// param [in] value: Pointcloud ROS type.
void KITTIDataOperator::SetLidarScan(sensor_msgs::PointCloud2 value) {
    lidar_scan_ = value;
}

// Get the recorded camera image.
// return: Image matrix.
const cv::Mat KITTIDataOperator::GetCameraImage() const {
    return camera_image_;
}

// Set the recorded camera image.
// param [in] value: Image matrix.
void KITTIDataOperator::SetCameraImage(cv::Mat value) { camera_image_ = value; }

void KITTIDataOperator::ReadPcdFiles(std::string pcd_filename) {
    // load point cloud
    fstream input(pcd_filename.c_str(), ios::in | ios::binary);
    if (!input.good()) {
        cerr << "Could not read  POINT CLOUD file: " << pcd_filename << endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, ios::beg);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);

    for (int i = 0; input.good() && !input.eof(); i++) {
        pcl::PointXYZRGB point;
        input.read((char *)&point.x, 3 * sizeof(float));
        input.read((char *)&point.r, sizeof(float));
        cloud->push_back(point);
    }
    input.close();

    // Define matrix to write velodyne points into

    MatrixXf matrix_velodyne_points = MatrixXf::Zero(4, cloud->size());
    for (int i = 0; i < cloud->size(); ++i) {
        matrix_velodyne_points(0, i) = cloud->points[i].x;
        matrix_velodyne_points(1, i) = cloud->points[i].y;
        matrix_velodyne_points(2, i) = cloud->points[i].z;
        matrix_velodyne_points(3, i) = 1;
    }

    // Transform Velodyne Points to Camerta Frame
    matrix_velodyne_points = tools_.transformVeloToCam(matrix_velodyne_points);

    for (int i = 0; i < matrix_velodyne_points.cols(); i++) {
        cloud->points[i].x = matrix_velodyne_points(0, i);
        cloud->points[i].y = matrix_velodyne_points(1, i);
        cloud->points[i].z = matrix_velodyne_points(2, i);
    }

    // declare ROS type Point cloud to puvlish pointcloud which is in camera
    // frame now
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);

    KITTIDataOperator::SetLidarScan(cloud_msg);
}

void KITTIDataOperator::ReadImageFiles(std::string image_filename) {
    cv::Mat kitti_image = cv::imread(image_filename, CV_LOAD_IMAGE_COLOR);

    // Sanity check if image is loaded correctly
    if (kitti_image.cols == 0 || kitti_image.rows == 0) {
        ROS_WARN("image not read properly!");
        return;
    }

    KITTIDataOperator::SetCameraImage(kitti_image);
}
