#include <pcl/PCLPointCloud2.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <sensor_msgs/PointCloud2.h>
#include <string>
#include "kitti_object_operator.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class KITTIDataOperator {
   public:
    KITTIDataOperator();
    ~KITTIDataOperator();

    void ReadPcdFiles(std::string pcd_filename);

    void ReadImageFiles(std::string image_filename);

    // Get the recorded Lidar scan pointcloud.
    // return: Pointcloud ROS type.
    const sensor_msgs::PointCloud2 GetLidarScan() const;

    // Set the recorded Lidar scan pointcloud.
    // param [in] value: Pointcloud ROS type.
    void SetLidarScan(sensor_msgs::PointCloud2 value);

    // Get the recorded camera image.
    // return: Image matrix.
    const cv::Mat GetCameraImage() const;

    // Set the recorded camera image.
    // param [in] value: Image matrix.
    void SetCameraImage(cv::Mat value);

   private:
    sensor_msgs::PointCloud2 lidar_scan_;

    sensor_msgs::PointCloud2 lidar_scan_in_velo_cordinates_;

    // Captured camera image.
    cv::Mat camera_image_;

    ros::NodeHandlePtr nh_;

    Tools tools_;
};