#ifndef sensorfusion_H
#define sensorfusion_H

#include <kitti_ros/kitti_data_operator.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

class SensorFusion {
   public:
    SensorFusion();
    ~SensorFusion();

    void FillKittiData4Fusion();

    void ProcessFusion(std::string training_image_name);

    void ProcessLabelofBEVImage(std::string& label_infile_string,
                                std::string image_file_path);

    void SetKITTIDataOperator(KITTIDataOperator* value);

    const KITTIDataOperator* GetKITTIDataOperator();

    void SetKittiObjectOperator(KittiObjectOperator* value);

    const KittiObjectOperator* GetKittiObjectOperator();

    void SetTools(Tools* value);

    const Tools* GetTools();

    void SegmentedPointCloudFromMaskRCNN(cv::Mat* maskrcnn_segmented_image);

    void SetSegmentedLidarScan(sensor_msgs::PointCloud2 value);

    sensor_msgs::PointCloud2 GetSegmentedLidarScan();

   private:
    float EuclidianDistofPoint(pcl::PointXYZRGB* colored_3d_point);
    void PublishRawData();

    void CreateBirdviewPointcloudImage(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud,
        std::string image_file);

    ros::NodeHandlePtr nh_;

    KITTIDataOperator* kitti_data_operator_;
    KittiObjectOperator* kitti_object_operator_;

    Tools* tools_;
    sensor_msgs::PointCloud2 lidar_scan_;
    sensor_msgs::PointCloud2 segmented_lidar_scan_;
    cv::Mat kitti_left_cam_img_;

    ros::Publisher rgb_pointcloud_pub_;

    ros::Publisher segmented_pointcloud_from_maskrcnn_pub_;

    ros::Publisher pointcloud_projected_image_pub_;

    ros::Publisher kitti_pcl_pub_;

    ros::Publisher kitti_image_pub_;

    ros::Publisher birdview_pointcloud_image_pub_;

    Eigen::MatrixXf TRANS_VELO_TO_CAM;
};
#endif