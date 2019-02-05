#include <kitti_ros/kitti_ros_node.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "kitti_node");

    ros::NodeHandle n;
    int node_loop_rate;
    n.param<int>("execution_frequency", node_loop_rate, 50);
    ros::Rate loop_rate(node_loop_rate);
    KittiRosNode kitti_node;

    while (ros::ok()) {
        kitti_node.ProcessNode();
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}

KittiRosNode::KittiRosNode() {
    nh_ = ros::NodeHandlePtr(new ros::NodeHandle());

    nh_->param<string>(
        "base_dir", base_dir,
        "/home/atas/kitti_data/2011_09_26/2011_09_26_drive_0001_sync");
    nh_->param<string>("pcd_file_dir", pcd_file_dir, "velodyne_points/data/");
    nh_->param<string>("image_dir", image_dir, "image_02/data/");
    nh_->param<string>("maskrcnn_detection_image_dir",
                       maskrcnn_detection_image_dir,
                       "maskrcnn_detections/detection_image_02/");
    nh_->param<string>("maskrcnn_detection_label_dir",
                       maskrcnn_detection_label_dir,
                       "maskrcnn_detections/detection_label_02/");
    nh_->param<string>("pcd_file_extension", pcd_file_extension, ".bin");
    nh_->param<string>("image_file_extension", image_file_extension, ".png");
    nh_->param<int>("number_of_pcd_files", number_of_pcd_files, 108);

    ROS_INFO_STREAM("base_dir: " << base_dir);
    ROS_INFO_STREAM("pcd_file_dir: " << pcd_file_dir);
    ROS_INFO_STREAM("pcd_file_extension: " << pcd_file_extension);
    ROS_INFO_STREAM("label_file_extension: " << label_file_extension);
    ROS_INFO_STREAM("number_of_pcd_files: " << number_of_pcd_files);

    sensor_fusion_.SetKITTIDataOperator(&kitti_data_operator_);
    sensor_fusion_.SetKittiObjectOperator(&kitti_object_operator_);
    sensor_fusion_.SetTools(&tools_);
    grid_cell_costmap_segmented_pcl_.SetTools(&tools_);

    sleep(1);
}

// Deconstruct
KittiRosNode::~KittiRosNode(){};

// Process all node do sensor fusion , obstacle detection , local costmap
// finding, Publish all data
void KittiRosNode::ProcessNode() {
    // for Number of files in this scenerios
    for (int i = 0; i < number_of_pcd_files; i++) {
        std::stringstream buffer;

        // a buffer to walk through each files
        // for KITTI scnerios setw(10)
        buffer << setfill('0') << setw(10) << i;

        // define path to pcd file to read point cloud
        std::string pcd_file =
            base_dir + pcd_file_dir + buffer.str() + pcd_file_extension;

        KittiRosNode::ProcessKittiPointCloud(pcd_file);

        // define path to image file
        std::string image_file =
            base_dir + image_dir + buffer.str() + image_file_extension;
        KittiRosNode::ProcessKittiImage(image_file);

        // Get lidar scan and Camera Image for Fusion
        sensor_fusion_.FillKittiData4Fusion();

        std::string training_image_name =
            base_dir + "rgb_pcl/" + buffer.str() + image_file_extension;
        // Process Fusion Publish Results and Raw Data
        sensor_fusion_.ProcessFusion(training_image_name);

        std::string maskrcnn_detection_image_path =
            base_dir + maskrcnn_detection_image_dir + buffer.str() +
            image_file_extension;

        cv::Mat maskrcnn_image = cv::imread(maskrcnn_detection_image_path, 1);
        sensor_fusion_.SegmentedPointCloudFromMaskRCNN(&maskrcnn_image);

        /*std::ifstream label_infile(label_file.c_str());
        kitti_objects_ =
            kitti_object_operator_.GetAllKittiObjectsFrame(label_infile);

        sensor_fusion_.ProcessLabelofBEVImage(label_file, training_image_name);
        KittiRosNode::ProcessKittiGroundTruthLabel(label_file,
                                                   training_image_name);*/

        // find Local costmap and Obstacles based on local costmap
        //
        KittiRosNode::ObstacleDetectionandGridCellCostmap();
        KittiRosNode::ObstacleDetectionSegmentedPCL();

        // sleep(5);
    }
}

void KittiRosNode::ProcessKittiPointCloud(std::string &pcd_file) {
    kitti_data_operator_.ReadPcdFiles(pcd_file);
}

void KittiRosNode::ProcessKittiGroundTruthLabel(
    std::string &label_infile_string, std::string image_file_path) {
    std::ifstream label_infile(label_infile_string.c_str());
    kitti_objects_ =
        kitti_object_operator_.GetAllKittiObjectsFrame(label_infile);
    kitti_object_operator_.VisualizeGTMarkers(kitti_objects_, image_file_path);
}

void KittiRosNode::ProcessKittiImage(std::string &image_file) {
    kitti_data_operator_.ReadImageFiles(image_file);
}

void KittiRosNode::ObstacleDetectionandGridCellCostmap() {
    sensor_msgs::PointCloud2 in_cloud = kitti_data_operator_.GetLidarScan();
    in_cloud.header.stamp = ros::Time::now();
    in_cloud.header.frame_id = "camera_link";
    sensor_msgs::PointCloud2::ConstPtr cld_ptr(
        new sensor_msgs::PointCloud2(in_cloud));

    // grid_cell_costmap_.DetectObstacles(cld_ptr);
}

void KittiRosNode::ObstacleDetectionSegmentedPCL() {
    const cv::Mat kitti_raw_image = kitti_data_operator_.GetCameraImage();
    sensor_msgs::PointCloud2 in_cloud = sensor_fusion_.GetSegmentedLidarScan();
    in_cloud.header.stamp = ros::Time::now();
    in_cloud.header.frame_id = "camera_link";
    sensor_msgs::PointCloud2::ConstPtr cld_ptr(
        new sensor_msgs::PointCloud2(in_cloud));

    // grid_cell_costmap_segmented_pcl_.DetectObstacles(cld_ptr,
    // kitti_raw_image);
}