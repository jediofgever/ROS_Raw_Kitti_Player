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

    ground_pub_ = nh_->advertise<sensor_msgs::PointCloud2>("ground_cloud", 1);
    nonground_pub_ =
        nh_->advertise<sensor_msgs::PointCloud2>("nonground_cloud", 1);
    clusters_pub_ =
        nh_->advertise<sensor_msgs::PointCloud2>("cluster_cloud", 1);

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
    SensorFusion::CreateBirdviewPointcloudImage(rgb_out_cloud,
                                                training_image_name);

    // Publish raw point cloud and raw rgb image
    SensorFusion::PublishRawData();

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

void SensorFusion::CreateBirdviewPointcloudImage(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud, std::string image_file) {
    cv::Mat bird_view_image(1200, 1200, CV_8UC3);
    bird_view_image.setTo(cv::Scalar(255, 255, 255));

    for (int r = 0; r < out_cloud->points.size(); r++) {
        pcl::PointXYZRGB rgb_point = out_cloud->points[r];

        if (rgb_point.z < 60 && rgb_point.x > -30 && rgb_point.x < 30) {
            cv::Point point;

            rgb_point.z -= 60;
            rgb_point.x += 30;

            point.x = rgb_point.x * 20;
            point.y = -rgb_point.z * 20;

            cv::Vec3b rgb_pixel, dense;

            rgb_pixel[2] = rgb_point.r;
            rgb_pixel[1] = rgb_point.g;
            rgb_pixel[0] = rgb_point.b;

            dense[2] = 255;
            dense[1] = 0;
            dense[0] = 0;

            if (point.x > 0 && point.x < 1200) {
                if (point.y > 0 && point.y < 1200) {
                    bird_view_image.at<cv::Vec3b>(point.y, point.x) = rgb_pixel;
                }
            }
        }
    }

    // Prepare and publish colorful image from PCL
    cv_bridge::CvImage cv_colorful_image_from_PCL;
    cv_colorful_image_from_PCL.image = bird_view_image;
    cv_colorful_image_from_PCL.encoding = "bgr8";
    cv_colorful_image_from_PCL.header.stamp = ros::Time::now();
    birdview_pointcloud_image_pub_.publish(
        cv_colorful_image_from_PCL.toImageMsg());

    cv::imwrite(image_file, bird_view_image);

    // pcl::io::savePCDFileASCII("test_pcd.pcd", *out_cloud);
}

void SensorFusion::ProcessLabelofBEVImage(std::string& label_infile_string,
                                          std::string image_file_path) {
    std::ifstream label_infile(label_infile_string.c_str());

    KittiObjectOperator::KittiObjectsThisFrame kitti_objects =
        kitti_object_operator_->GetAllKittiObjectsFrame(label_infile);

    cv::Mat BEV_image = cv::imread(image_file_path, cv::IMREAD_COLOR);

    for (int k = 0; k < kitti_objects.size(); k++) {
        if (kitti_objects[k].type == "Car") {
            std::vector<float> dimensions, position;

            dimensions =
                kitti_object_operator_->dimensionsVector(kitti_objects[k]);
            position = kitti_object_operator_->positionVector(kitti_objects[k]);

            Eigen::MatrixXd corners(3, 8);

            std::cout << "KORNER FROM LABEL " << corners << std::endl;

            corners = kitti_ros_util::ComputeCorners(
                dimensions, position, kitti_objects[k].rotation_y);

            cv::Point pt1_on2D, pt2_on2D, pt3_on2D, pt4_on2D, center;

            pt1_on2D.x = corners(0, 0);
            pt1_on2D.y = corners(2, 0);

            pt2_on2D.x = corners(0, 1);
            pt2_on2D.y = corners(2, 1);

            pt3_on2D.x = corners(0, 2);
            pt3_on2D.y = corners(2, 2);

            pt4_on2D.x = corners(0, 3);
            pt4_on2D.y = corners(2, 3);

            pt1_on2D.x += 30;
            pt1_on2D.y -= 60;

            pt2_on2D.x += 30;
            pt2_on2D.y -= 60;

            pt3_on2D.x += 30;
            pt3_on2D.y -= 60;

            pt4_on2D.x += 30;
            pt4_on2D.y -= 60;

            // scale up to image dimensions 1200 x 900

            pt1_on2D.x *= 20;
            pt2_on2D.x *= 20;
            pt3_on2D.x *= 20;
            pt4_on2D.x *= 20;

            pt1_on2D.y *= -20;
            pt2_on2D.y *= -20;
            pt3_on2D.y *= -20;
            pt4_on2D.y *= -20;

            center.x = position[0];
            center.y = position[2];
            center.x += 30;
            center.y -= 60;
            center.x *= 20;
            center.y *= -20;

            // cv::rectangle(frame, rect_on2D, c, 2, 2, 0);
            cv::Scalar clr = cv::Scalar(0, 0, 255);

            cv::circle(BEV_image, center, 4, clr, 1, 1, 0);
            cv::line(BEV_image, pt1_on2D, pt2_on2D, clr, 1, 8);
            cv::line(BEV_image, pt2_on2D, pt3_on2D, clr, 1, 8);
            cv::line(BEV_image, pt3_on2D, pt4_on2D, clr, 1, 8);
            cv::line(BEV_image, pt4_on2D, pt1_on2D, clr, 1, 8);
        }
    }
    cv::imwrite(image_file_path, BEV_image);
}

void SensorFusion::SegmentedPointCloudFromMaskRCNN(
    cv::Mat* maskrcnn_segmented_image) {
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
    SensorFusion::ProcessObjectBuilder(out_cloud_obj_builder);
}

void SensorFusion::ProcessObjectBuilder(
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_obj_builder) {
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
    for (int k = 0; k < objects.size(); k++) {
        jsk_recognition_msgs::BoundingBox box;
        box.header.frame_id = "camera_link";
        autosense::ObjectPtr obj_ptr = objects.at(k);
        double sx, double sy, d box.dimensions.x = obj_ptr->length;
        box.dimensions.y = obj_ptr->width;
        box.dimensions.z = obj_ptr->height;

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
        dimensions.push_back(obj_ptr->width);
        dimensions.push_back(obj_ptr->length);

        position.push_back(obj_ptr->ground_center[0]);
        position.push_back(obj_ptr->ground_center[1]);
        position.push_back(obj_ptr->ground_center[2] + 2 * obj_ptr->length);

        Eigen::MatrixXf corners;
        corners = kitti_ros_util::KornersWorldtoKornersImage(dimensions,
                                                             position, yaw_rad);

        Eigen::RowVectorXf vec(8);
        vec << 1, 1, 1, 1, 1, 1, 1, 1;

        corners.conservativeResize(corners.rows() + 1, corners.cols());
        corners.row(corners.rows() - 1) = vec;

        Eigen::MatrixXf corners_on_image =
            tools_.transformRectCamToImage(corners);

        kitti_ros_util::Construct3DBoxOnImage(&corners_on_image,
                                              &kitti_left_cam_img_);
        cv::imwrite(
            "/home/atas/kitti_data/2011_09_26/"
            "2011_09_26_drive_0001_sync/box_img.png",
            kitti_left_cam_img_);

        box_array.boxes.push_back(box);
    }

    jsk_box_array_pub_.publish(box_array);
}
