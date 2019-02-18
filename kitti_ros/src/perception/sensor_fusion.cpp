#include <kitti_ros/perception/sensor_fusion.h>

SensorFusion::SensorFusion() {
    nh_ = ros::NodeHandlePtr(new ros::NodeHandle());

    // Publish rgb colored pointcloud
    rgb_pointcloud_pub_ =
        nh_->advertise<sensor_msgs::PointCloud2>("kitti_rgb_pointcloud", 1);

    // Publish segmented pointcloud from maskrcnn detection image;

    segmented_pointcloud_from_maskrcnn_pub_ =
        nh_->advertise<sensor_msgs::PointCloud2>(
            "segmented_pointcloud_from_maskrcnn", 1);

    // publish point cloud projected IMage
    pointcloud_projected_image_pub_ =
        nh_->advertise<sensor_msgs::Image>("pointcloud_projected_image", 1);

    // publish Kitti raw point cloud
    kitti_pcl_pub_ =
        nh_->advertise<sensor_msgs::PointCloud2>("kitti_raw_pointcloud", 1);
    // publish kitti raw image
    kitti_image_pub_ = nh_->advertise<sensor_msgs::Image>("kitti_raw_image", 1);

    // publish birdview pointcloud Image
    birdview_pointcloud_image_pub_ =
        nh_->advertise<sensor_msgs::Image>("image_from_colorful_PCL", 1);

    // vis jsk Bounding box detected by object builder
    jsk_box_array_pub_ = nh_->advertise<jsk_recognition_msgs::BoundingBoxArray>(
        "jsk_box_array", 1);

    // Publish Deteceted Obstacles
    detected_obstacles_publisher_ =
        nh_->advertise<visualization_msgs::MarkerArray>(
            "detected_obstacles_from_local_costmap_segmented_pcl", 1);

    // Publish 3D boundging box projected Image
    box_projetcted_image_pub_ =
        nh_->advertise<sensor_msgs::Image>("box_projetcted_image", 1);

    // Publishers for segmenters lib
    ground_pub_ = nh_->advertise<sensor_msgs::PointCloud2>("ground_cloud", 1);
    nonground_pub_ =
        nh_->advertise<sensor_msgs::PointCloud2>("nonground_cloud", 1);
    clusters_pub_ =
        nh_->advertise<sensor_msgs::PointCloud2>("cluster_cloud", 1);

    // paths to segmenters lib configs
    const std::string param_ns_prefix_ = "/detect";
    std::string ground_remover_type, non_ground_segmenter_type;
    private_nh.param<std::string>(param_ns_prefix_ + "/ground_remover_type",
                                  ground_remover_type,
                                  "GroundPlaneFittingSegmenter");

    private_nh.param<std::string>(
        param_ns_prefix_ + "/non_ground_segmenter_type",
        non_ground_segmenter_type, "EuclideanSegmenter");

    private_nh = ros::NodeHandle("~");
    SegmenterParams param =
        common::getSegmenterParams(private_nh, param_ns_prefix_);
    param.segmenter_type = ground_remover_type;
    ground_remover_ = segmenter::createGroundSegmenter(param);
    param.segmenter_type = non_ground_segmenter_type;
    segmenter_ = segmenter::createNonGroundSegmenter(param);
}

SensorFusion::~SensorFusion() {}

void SensorFusion::SetKITTIDataOperator(KITTIDataOperator* value) {
    kitti_data_operator_ = value;
}

const KITTIDataOperator* SensorFusion::GetKITTIDataOperator() {
    return kitti_data_operator_;
}

void SensorFusion::SetKittiObjectOperator(KittiObjectOperator* value) {
    kitti_object_operator_ = value;
}

const KittiObjectOperator* SensorFusion::GetKittiObjectOperator() {
    return kitti_object_operator_;
}

void SensorFusion::SetSegmentedLidarScan(sensor_msgs::PointCloud2 value) {
    segmented_lidar_scan_ = value;
}

sensor_msgs::PointCloud2 SensorFusion::GetSegmentedLidarScan() {
    return segmented_lidar_scan_;
}

void SensorFusion::SetTools(Tools* value) { tools_ = value; }

const Tools* SensorFusion::GetTools() { return tools_; }

void SensorFusion::FillKittiData4Fusion() {
    lidar_scan_ = kitti_data_operator_->GetLidarScan();
    lidar_scan_.header.stamp = ros::Time::now();
    lidar_scan_.header.frame_id = "camera_link";

    kitti_left_cam_img_ = kitti_data_operator_->GetCameraImage();
}

void SensorFusion::ProcessFusion(std::string training_image_name) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_out_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);

    cv::Mat cv_pointcloud_projected_image = kitti_left_cam_img_.clone();

    pcl::fromROSMsg(lidar_scan_, *in_cloud);

    Eigen::MatrixXf matrix_velodyne_points =
        MatrixXf::Zero(4, in_cloud->size());

    for (int i = 0; i < in_cloud->size(); ++i) {
        matrix_velodyne_points(0, i) = in_cloud->points[i].x;
        matrix_velodyne_points(1, i) = in_cloud->points[i].y;
        matrix_velodyne_points(2, i) = in_cloud->points[i].z;
        matrix_velodyne_points(3, i) = 1;
    }

    Eigen::MatrixXf matrix_image_points =
        tools_->transformCamToRectCam(matrix_velodyne_points);
    matrix_image_points = tools_->transformRectCamToImage(matrix_image_points);

    for (int m = 0; m < matrix_image_points.cols(); m++) {
        cv::Point point;
        point.x = matrix_image_points(0, m);
        point.y = matrix_image_points(1, m);

        // Store korners in pixels only of they are on image plane
        if (point.x >= 0 && point.x <= 1242) {
            if (point.y >= 0 && point.y <= 375) {
                pcl::PointXYZRGB colored_3d_point;

                cv::Vec3b rgb_pixel =
                    kitti_left_cam_img_.at<cv::Vec3b>(point.y, point.x);

                colored_3d_point.x = matrix_velodyne_points(0, m);
                colored_3d_point.y = matrix_velodyne_points(1, m);
                colored_3d_point.z = matrix_velodyne_points(2, m);

                colored_3d_point.r = rgb_pixel[2];
                colored_3d_point.g = rgb_pixel[1];
                colored_3d_point.b = rgb_pixel[0];

                if (colored_3d_point.z > 0) {
                    float distance_to_point =
                        SensorFusion::EuclidianDistofPoint(&colored_3d_point);

                    cv::circle(cv_pointcloud_projected_image, point, 1,
                               cv::Scalar(120, 0, distance_to_point * 15), 1);

                    rgb_out_cloud->points.push_back(colored_3d_point);
                }
            }
        }
    }

    // Create birdeyeview Lidar Image , with rgb values taken from corresponding
    // pixel
    rgb_out_cloud->width = 1;
    rgb_out_cloud->height = rgb_out_cloud->points.size();

    // prepare and publish RGB colored Lidar scan
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*rgb_out_cloud, cloud_msg);
    cloud_msg.header = lidar_scan_.header;
    rgb_pointcloud_pub_.publish(cloud_msg);

    // Prepare and publish Point cloud projected kitti image
    cv_bridge::CvImage pointcloud_projected_image;
    pointcloud_projected_image.image = cv_pointcloud_projected_image;
    pointcloud_projected_image.encoding = "bgr8";
    pointcloud_projected_image.header.stamp = ros::Time::now();
    pointcloud_projected_image_pub_.publish(
        pointcloud_projected_image.toImageMsg());
}

void SensorFusion::PublishRawData() {
    // Prepare and publish KITTI raw image
    cv_bridge::CvImage cv_kitti_image;
    cv_kitti_image.image = kitti_left_cam_img_;
    cv_kitti_image.encoding = "bgr8";
    cv_kitti_image.header.stamp = ros::Time::now();
    kitti_image_pub_.publish(cv_kitti_image.toImageMsg());

    // PUBLISH Raw Lidar scan
    kitti_pcl_pub_.publish(lidar_scan_);
}

float SensorFusion::EuclidianDistofPoint(pcl::PointXYZRGB* colored_3d_point) {
    float distance = std::sqrt(std::pow(colored_3d_point->x, 2) +
                               std::pow(colored_3d_point->y, 2) +
                               std::pow(colored_3d_point->z, 2));
    return distance;
}

void SensorFusion::SegmentedPointCloudFromMaskRCNN(
    cv::Mat* maskrcnn_segmented_image, std::string image_file_path) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_out_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_obj_builder(
        new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(lidar_scan_, *in_cloud);

    Eigen::MatrixXf matrix_velodyne_points =
        MatrixXf::Zero(4, in_cloud->size());

    for (int i = 0; i < in_cloud->size(); ++i) {
        matrix_velodyne_points(0, i) = in_cloud->points[i].x;
        matrix_velodyne_points(1, i) = in_cloud->points[i].y;
        matrix_velodyne_points(2, i) = in_cloud->points[i].z;
        matrix_velodyne_points(3, i) = 1;
    }
    int kDilationType = cv::MORPH_RECT;

    cv::Mat element = cv::getStructuringElement(
        kDilationType, cv::Size(2 * 7 + 1, 2 * 7 + 1), cv::Point(7, 7));

    cv::dilate(*maskrcnn_segmented_image, *maskrcnn_segmented_image, element);

    Eigen::MatrixXf matrix_image_points =
        tools_->transformCamToRectCam(matrix_velodyne_points);
    matrix_image_points = tools_->transformRectCamToImage(matrix_image_points);

    for (int m = 0; m < matrix_image_points.cols(); m++) {
        cv::Point point;
        point.x = matrix_image_points(0, m);
        point.y = matrix_image_points(1, m);

        // Store korners in pixels only of they are on image plane
        if (point.x >= 0 && point.x <= 1242) {
            if (point.y >= 0 && point.y <= 375) {
                pcl::PointXYZRGB colored_3d_point;

                pcl::PointXYZI out_cloud_point_obj_builder;

                cv::Vec3b rgb_pixel =
                    maskrcnn_segmented_image->at<cv::Vec3b>(point.y, point.x);

                colored_3d_point.x = matrix_velodyne_points(0, m);
                colored_3d_point.y = matrix_velodyne_points(1, m);
                colored_3d_point.z = matrix_velodyne_points(2, m);

                colored_3d_point.r = rgb_pixel[2];
                colored_3d_point.g = rgb_pixel[1];
                colored_3d_point.b = rgb_pixel[0];

                out_cloud_point_obj_builder.x = matrix_velodyne_points(0, m);
                out_cloud_point_obj_builder.y = matrix_velodyne_points(1, m);
                out_cloud_point_obj_builder.z = matrix_velodyne_points(2, m);

                if (rgb_pixel[2] != 255 && rgb_pixel[1] != 255 &&
                    rgb_pixel[0] != 255 && colored_3d_point.z > 0 &&
                    colored_3d_point.y < 1.65) {
                    rgb_out_cloud->points.push_back(colored_3d_point);
                    out_cloud_obj_builder->points.push_back(
                        out_cloud_point_obj_builder);
                }
            }
        }
    }

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(rgb_out_cloud);
    sor.setMeanK(12);
    sor.setStddevMulThresh(0.2);
    sor.filter(*rgb_out_cloud);

    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
    // build the filter
    outrem.setInputCloud(rgb_out_cloud);
    outrem.setRadiusSearch(0.3);
    outrem.setMinNeighborsInRadius(4);
    // apply filter
    outrem.filter(*rgb_out_cloud);

    // prepare and publish RGB colored Lidar scan
    sensor_msgs::PointCloud2 maskrcnn_cloud_msg;
    pcl::toROSMsg(*rgb_out_cloud, maskrcnn_cloud_msg);
    maskrcnn_cloud_msg.header = lidar_scan_.header;
    segmented_pointcloud_from_maskrcnn_pub_.publish(maskrcnn_cloud_msg);

    SensorFusion::SetSegmentedLidarScan(maskrcnn_cloud_msg);
    SensorFusion::ProcessObjectBuilder(out_cloud_obj_builder, image_file_path,
                                       maskrcnn_segmented_image);
}

void SensorFusion::ProcessObjectBuilder(
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_obj_builder,
    std::string image_file_path, cv::Mat* maskrcnn_segmented_image) {
    std_msgs::Header header = lidar_scan_.header;
    header.frame_id = "camera_link";
    header.stamp = ros::Time::now();

    std::vector<PointICloudPtr> cloud_clusters;
    PointICloudPtr cloud_ground(new PointICloud);
    PointICloudPtr cloud_nonground(new PointICloud);

    ground_remover_->segment(*out_cloud_obj_builder, &cloud_clusters);
    *cloud_ground = *cloud_clusters[0];
    *cloud_nonground = *cloud_clusters[1];

    // reset clusters
    cloud_clusters.clear();

    segmenter_->segment(*out_cloud_obj_builder, &cloud_clusters);
    common::publishClustersCloud<PointI>(clusters_pub_, header, cloud_clusters);

    common::publishCloud<PointI>(ground_pub_, header, *cloud_ground);
    common::publishCloud<PointI>(nonground_pub_, header, *cloud_nonground);

    // 2.define object builder
    boost::shared_ptr<object_builder::BaseObjectBuilder> object_builder_;

    // 3.create object builder by manager
    object_builder_ = object_builder::createObjectBuilder();

    // 4.build 3D orientation bounding box for clustering point cloud
    std::vector<autosense::ObjectPtr> objects;
    object_builder_->build(cloud_clusters, &objects);

    jsk_recognition_msgs::BoundingBoxArray box_array;
    box_array.header.frame_id = "camera_link";
    visualization_msgs::MarkerArray Dbox_array;

    for (int k = 0; k < objects.size(); k++) {
        jsk_recognition_msgs::BoundingBox box;
        box.header.frame_id = "camera_link";
        autosense::ObjectPtr obj_ptr = objects.at(k);

        if (obj_ptr->height < 22.5) {
            box.dimensions.x = obj_ptr->length;
            box.dimensions.y = obj_ptr->width;
            box.dimensions.z = 2.0;  // obj_ptr->height;

            box.pose.position.x = obj_ptr->ground_center[0];
            box.pose.position.y = obj_ptr->ground_center[1];
            box.pose.position.z = obj_ptr->ground_center[2] + obj_ptr->length;

            double yaw_rad = obj_ptr->yaw_rad;
            double x, y, z, w;

            kitti_ros_util::EulerAngleToQuaternion(yaw_rad, &x, &y, &z, &w);
            box.pose.orientation.x = x;
            box.pose.orientation.y = y;
            box.pose.orientation.z = z;
            box.pose.orientation.w = w;

            std::vector<float> dimensions, position;

            dimensions.push_back(obj_ptr->height);
            dimensions.push_back(1.6);
            dimensions.push_back(obj_ptr->length);

            position.push_back(obj_ptr->ground_center[0]);
            position.push_back(obj_ptr->ground_center[1]);
            position.push_back(obj_ptr->ground_center[2] + obj_ptr->height);

            Eigen::MatrixXf corners;
            corners = kitti_ros_util::KornersWorldtoKornersImage(
                dimensions, position, yaw_rad);

            Eigen::RowVectorXf vec(8);
            vec << 1, 1, 1, 1, 1, 1, 1, 1;

            corners.conservativeResize(corners.rows() + 1, corners.cols());
            corners.row(corners.rows() - 1) = vec;

            Eigen::MatrixXf corners_on_image =
                tools_->transformRectCamToImage(corners);

            kitti_ros_util::Construct3DBoxOnImage(&corners_on_image,
                                                  &kitti_left_cam_img_);

            cv::imwrite(image_file_path, kitti_left_cam_img_);

            // Prepare and publish 3D box projected image
            cv_bridge::CvImage cv_kitti_3D_BOX_image;
            cv_kitti_3D_BOX_image.image = kitti_left_cam_img_;
            cv_kitti_3D_BOX_image.encoding = "bgr8";
            cv_kitti_3D_BOX_image.header.stamp = ros::Time::now();
            box_projetcted_image_pub_.publish(
                cv_kitti_3D_BOX_image.toImageMsg());

            box_array.boxes.push_back(box);

            visualization_msgs::Marker visualization_marker_;

            visualization_marker_.type = visualization_msgs::Marker::LINE_STRIP;
            visualization_marker_.header.frame_id = "camera_link";
            visualization_marker_.header.stamp = ros::Time::now();
            visualization_marker_.ns = "DetectionBox";
            visualization_marker_.id = k;
            visualization_marker_.action = visualization_msgs::Marker::ADD;
            visualization_marker_.lifetime = ros::Duration(1.2);

            kitti_ros_util::SetMarkerData(&visualization_marker_, 0, 0, 0, x, y,
                                          z, w, 0.1, 0, 0, 0, 0, 1, 1);

            std::vector<geometry_msgs::Point> corners_geometry_msgs =
                kitti_ros_util::Eigen2GeometryMsgs(corners);
            // Construct 3D box with entering point in a sequence
            // yeah I know it looks ugly
            visualization_marker_.points.push_back(corners_geometry_msgs.at(0));
            visualization_marker_.points.push_back(corners_geometry_msgs.at(1));
            visualization_marker_.points.push_back(corners_geometry_msgs.at(0));
            visualization_marker_.points.push_back(corners_geometry_msgs.at(3));
            visualization_marker_.points.push_back(corners_geometry_msgs.at(0));
            visualization_marker_.points.push_back(corners_geometry_msgs.at(4));
            visualization_marker_.points.push_back(corners_geometry_msgs.at(1));
            visualization_marker_.points.push_back(corners_geometry_msgs.at(2));
            visualization_marker_.points.push_back(corners_geometry_msgs.at(1));
            visualization_marker_.points.push_back(corners_geometry_msgs.at(5));
            visualization_marker_.points.push_back(corners_geometry_msgs.at(1));
            visualization_marker_.points.push_back(corners_geometry_msgs.at(2));
            visualization_marker_.points.push_back(corners_geometry_msgs.at(6));
            visualization_marker_.points.push_back(corners_geometry_msgs.at(2));
            visualization_marker_.points.push_back(corners_geometry_msgs.at(3));
            visualization_marker_.points.push_back(corners_geometry_msgs.at(3));
            visualization_marker_.points.push_back(corners_geometry_msgs.at(7));
            visualization_marker_.points.push_back(corners_geometry_msgs.at(7));
            visualization_marker_.points.push_back(corners_geometry_msgs.at(4));
            visualization_marker_.points.push_back(corners_geometry_msgs.at(7));
            visualization_marker_.points.push_back(corners_geometry_msgs.at(6));
            visualization_marker_.points.push_back(corners_geometry_msgs.at(4));
            visualization_marker_.points.push_back(corners_geometry_msgs.at(5));
            visualization_marker_.points.push_back(corners_geometry_msgs.at(5));
            visualization_marker_.points.push_back(corners_geometry_msgs.at(6));

            Dbox_array.markers.push_back(visualization_marker_);
        }
    }

    jsk_box_array_pub_.publish(box_array);
    detected_obstacles_publisher_.publish(Dbox_array);
}
