#ifndef kittiobjop_H
#define kittiobjop_H

#include <cv_bridge/cv_bridge.h>
#include <helper/tools.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include "util/kitti_ros_utils.h"

using namespace helper;
using namespace std;
using namespace kitti_ros_util;

class KittiObjectOperator {
   public:
    KittiObjectOperator();

    ~KittiObjectOperator();
    struct kitti_object {
        string type;
        float truncated;
        int occluded;
        float alpha;

        float bbox2D_tlx;
        float bbox2D_tly;
        float bbox2D_brx;
        float bbox2D_bry;

        float dimensions_h;
        float dimensions_w;
        float dimensions_l;

        float x;
        float y;
        float z;

        float rotation_y;
        float score;
    };

    typedef std::vector<kitti_object> KittiObjectsThisFrame;

    KittiObjectOperator::KittiObjectsThisFrame GetAllKittiObjectsFrame(
        std::ifstream& infile);

    void VisualizeGTMarkers(
        KittiObjectOperator::KittiObjectsThisFrame kitti_objects_,
        std::string image_file_path);

    const KittiObjectOperator::KittiObjectsThisFrame GetObjects() const;

    void SetObjects(KittiObjectOperator::KittiObjectsThisFrame value);

    std::vector<float> dimensionsVector(
        KittiObjectOperator::kitti_object kitti_object_);
    std::vector<float> positionVector(
        KittiObjectOperator::kitti_object kitti_object_);
    std::vector<float> BbbxVector(
        KittiObjectOperator::kitti_object kitti_object_);

   private:
    ros::NodeHandlePtr nh_;

    ros::Publisher kitt_3d_box_pub_;

    KittiObjectOperator::KittiObjectsThisFrame kitti_objects_;
};

#endif