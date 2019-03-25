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

// Set the recorded imu.
// param [in] value: Image matrix.
void KITTIDataOperator::SetImu(sensor_msgs::Imu value) { imu_ = value; }

// Get the recorded imu.
// return: Image matrix.
const sensor_msgs::Imu KITTIDataOperator::GetImu() const { return imu_; }

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

void KITTIDataOperator::ReadIMU(std::string full_filename_oxts) {
    sensor_msgs::Imu ros_msgImu;

    if (!getIMU(full_filename_oxts, &ros_msgImu)) {
        ROS_ERROR_STREAM("Fail to open " << full_filename_oxts);
        return ;
    }
    KITTIDataOperator::SetImu(ros_msgImu);
}

int KITTIDataOperator::getIMU(string filename, sensor_msgs::Imu *ros_msgImu) {
    ifstream file_oxts(filename.c_str());
    if (!file_oxts.is_open()) {
        ROS_ERROR_STREAM("Fail to open " << filename);
        return 0;
    }

    ROS_DEBUG_STREAM("Reading IMU data from oxts file: " << filename);

    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sep{" "};

    string line = "";

    getline(file_oxts, line);
    tokenizer tok(line, sep);
    vector<string> s(tok.begin(), tok.end());

    ros_msgImu->header.frame_id = ros::this_node::getName();
    // ros_msgImu->header.stamp = header->stamp;

    //    - ax:      acceleration in x, i.e. in direction of vehicle front
    //    (m/s^2)
    //    - ay:      acceleration in y, i.e. in direction of vehicle left
    //    (m/s^2)
    //    - az:      acceleration in z, i.e. in direction of vehicle top (m/s^2)
    ros_msgImu->linear_acceleration.x = boost::lexical_cast<double>(s[11]);
    ros_msgImu->linear_acceleration.y = boost::lexical_cast<double>(s[12]);
    ros_msgImu->linear_acceleration.z = boost::lexical_cast<double>(s[13]);

    //    - vf:      forward velocity, i.e. parallel to earth-surface (m/s)
    //    - vl:      leftward velocity, i.e. parallel to earth-surface (m/s)
    //    - vu:      upward velocity, i.e. perpendicular to earth-surface (m/s)
    ros_msgImu->angular_velocity.x = boost::lexical_cast<double>(s[8]);
    ros_msgImu->angular_velocity.y = boost::lexical_cast<double>(s[9]);
    ros_msgImu->angular_velocity.z = boost::lexical_cast<double>(s[10]);

    //    - roll:    roll angle (rad),  0 = level, positive = left side up
    //    (-pi..pi)
    //    - pitch:   pitch angle (rad), 0 = level, positive = front down
    //    (-pi/2..pi/2)
    //    - yaw:     heading (rad),     0 = east,  positive = counter clockwise
    //    (-pi..pi)
    tf::Quaternion q = tf::createQuaternionFromRPY(
        boost::lexical_cast<double>(s[3]), boost::lexical_cast<double>(s[4]),
        boost::lexical_cast<double>(s[5]));
    ros_msgImu->orientation.x = q.getX();
    ros_msgImu->orientation.y = q.getY();
    ros_msgImu->orientation.z = q.getZ();
    ros_msgImu->orientation.w = q.getW();

    return 1;
}