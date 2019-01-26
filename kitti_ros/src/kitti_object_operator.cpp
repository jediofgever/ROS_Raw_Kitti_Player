#include <kitti_ros/kitti_object_operator.h>

KittiObjectOperator::KittiObjectOperator() {
    nh_ = ros::NodeHandlePtr(new ros::NodeHandle());

    kitt_3d_box_pub_ = nh_->advertise<visualization_msgs::MarkerArray>(
        "kitti_groundtruth3D_box", 1);
}

KittiObjectOperator::~KittiObjectOperator(){};

const KittiObjectOperator::KittiObjectsThisFrame
KittiObjectOperator::GetObjects() const {
    return kitti_objects_;
}

void KittiObjectOperator::SetObjects(
    KittiObjectOperator::KittiObjectsThisFrame value) {
    kitti_objects_ = value;
}

KittiObjectOperator::KittiObjectsThisFrame
KittiObjectOperator::GetAllKittiObjectsFrame(std::ifstream& infile) {
    std::string line;
    std::string type;

    KittiObjectOperator::KittiObjectsThisFrame kitti_objects;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);

        KittiObjectOperator::kitti_object kitti_object_;

        iss >> kitti_object_.type >> kitti_object_.truncated >>
            kitti_object_.occluded >> kitti_object_.alpha >>
            kitti_object_.bbox2D_tlx >> kitti_object_.bbox2D_tly >>
            kitti_object_.bbox2D_brx >> kitti_object_.bbox2D_bry >>
            kitti_object_.dimensions_h >> kitti_object_.dimensions_w >>
            kitti_object_.dimensions_l >> kitti_object_.x >> kitti_object_.y >>
            kitti_object_.z >> kitti_object_.rotation_y >> kitti_object_.score;

        kitti_objects.push_back(kitti_object_);
    }
    KittiObjectOperator::SetObjects(kitti_objects);
    return kitti_objects;
}

void KittiObjectOperator::VisualizeGTMarkers(
    KittiObjectOperator::KittiObjectsThisFrame kitti_objects_,
    std::string image_file_path) {
    cv::Mat BEV_image = cv::imread(image_file_path, cv::IMREAD_COLOR);

    // push boxes to this marker array
    visualization_msgs::MarkerArray gt3Dbox_array;
    for (int k = 0; k < kitti_objects_.size(); k++) {
        if (kitti_objects_[k].type == "Car") {
            visualization_msgs::Marker visualization_marker_;

            visualization_marker_.type = visualization_msgs::Marker::LINE_STRIP;
            visualization_marker_.header.frame_id = "camera_link";
            visualization_marker_.header.stamp = ros::Time::now();
            visualization_marker_.ns = "GT_3DBox";
            visualization_marker_.id = k;
            visualization_marker_.action = visualization_msgs::Marker::ADD;
            visualization_marker_.lifetime = ros::Duration(20.2);

            double ox, oy, oz, ow;

            kitti_ros_util::EulerAngleToQuaternion(kitti_objects_[k].rotation_y,
                                                   &ox, &oy, &oz, &ow);

            std::vector<float> dimensions, position;

            dimensions =
                KittiObjectOperator::dimensionsVector(kitti_objects_[k]);
            position = KittiObjectOperator::positionVector(kitti_objects_[k]);

            Eigen::MatrixXd corners;

            corners = kitti_ros_util::ComputeCorners(
                dimensions, position, kitti_objects_[k].rotation_y);

            // std::cout << "KORNER FROM Ground BOX " << corners << std::endl;

            kitti_ros_util::SetMarkerData(&visualization_marker_, 0, 0, 0, ox,
                                          oy, oz, ow, 0.1, 0, 0, 0, 0, 1, 1);

            geometry_msgs::Point bottom_first_korner_point;

            for (int c = 0; c < 4; c++) {
                geometry_msgs::Point korner_point, prev_korner_point;
                korner_point.x = corners(0, c);
                korner_point.y = corners(1, c);
                korner_point.z = corners(2, c);
                if (c == 0) {
                    bottom_first_korner_point = korner_point;
                }

                visualization_marker_.points.push_back(korner_point);
            }

            visualization_marker_.points.push_back(bottom_first_korner_point);

            geometry_msgs::Point up_first_korner_point;

            for (int c = 4; c < 8; c++) {
                geometry_msgs::Point korner_point, prev_korner_point;
                korner_point.x = corners(0, c);
                korner_point.y = corners(1, c);
                korner_point.z = corners(2, c);
                if (c == 4) {
                    up_first_korner_point = korner_point;
                }

                visualization_marker_.points.push_back(korner_point);
            }

            visualization_marker_.points.push_back(up_first_korner_point);

            gt3Dbox_array.markers.push_back(visualization_marker_);

            visualization_marker_.type = visualization_msgs::Marker::SPHERE;
            visualization_marker_.header.frame_id = "camera_link";
            visualization_marker_.header.stamp = ros::Time::now();
            visualization_marker_.ns = "sphere";
            visualization_marker_.id = k * k;
            visualization_marker_.action = visualization_msgs::Marker::ADD;

            kitti_ros_util::SetMarkerData(&visualization_marker_, position[0],
                                          position[1], position[2], ox, oy, oz,
                                          ow, 0.6, 0.6, 1, 1, 0, 1, 1);

            geometry_msgs::Point center_point;

            center_point.x = position[0];
            center_point.y = position[1];
            center_point.z = position[2];
            gt3Dbox_array.markers.push_back(visualization_marker_);
            //////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////
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
            cv::Scalar clr = cv::Scalar(255, 0, 0);

            cv::circle(BEV_image, center, 4, clr, 1, 1, 0);
            cv::line(BEV_image, pt1_on2D, pt2_on2D, clr, 1, 8);
            cv::line(BEV_image, pt2_on2D, pt3_on2D, clr, 1, 8);
            cv::line(BEV_image, pt3_on2D, pt4_on2D, clr, 1, 8);
            cv::line(BEV_image, pt4_on2D, pt1_on2D, clr, 1, 8);
        }
    }
    kitt_3d_box_pub_.publish(gt3Dbox_array);
    cv::imwrite(image_file_path, BEV_image);
}

std::vector<float> KittiObjectOperator::dimensionsVector(
    KittiObjectOperator::kitti_object kitti_object_) {
    std::vector<float> dimensions;
    dimensions.push_back(kitti_object_.dimensions_h);
    dimensions.push_back(kitti_object_.dimensions_w);
    dimensions.push_back(kitti_object_.dimensions_l);

    return dimensions;
}
std::vector<float> KittiObjectOperator::positionVector(
    KittiObjectOperator::kitti_object kitti_object_) {
    std::vector<float> position;

    position.push_back(kitti_object_.x);
    position.push_back(kitti_object_.y);
    position.push_back(kitti_object_.z);

    return position;
}
std::vector<float> KittiObjectOperator::BbbxVector(
    KittiObjectOperator::kitti_object kitti_object_) {
    std::vector<float> bbx;
    bbx.push_back(kitti_object_.bbox2D_tlx);
    bbx.push_back(kitti_object_.bbox2D_tly);
    bbx.push_back(kitti_object_.bbox2D_brx);
    bbx.push_back(kitti_object_.bbox2D_bry);

    return bbx;
}
