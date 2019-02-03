/**
 * @author Fetullah Atas
 * @email [fetulahatas1@gmail.com]
 * @create date 2018-11-06 17:36:10
 * @modify date 2018-11-06 17:36:10
 * @desc [description]
 */
#include <kitti_ros/perception/grid_cell_costmap_segmented_pcl.h>
#include <boost/foreach.hpp>

// Construct Copilot Costmap
GridCellCostmapSegmentedPCL::GridCellCostmapSegmentedPCL() {
    // Init NodeHandler Ptr
    nh_ = ros::NodeHandlePtr(new ros::NodeHandle());

    // Read Params from params.yaml file if avaliable
    nh_->param<double>("costmap_width", grid_width_, 400.0);
    nh_->param<double>("costmap_height", grid_height_, 400.0);
    nh_->param<double>("costmap_resolution", grid_resolution_, 0.1);
    nh_->param<double>("costmap_max_obstacle_height", max_obstacle_height_,
                       1.0);
    nh_->param<double>("costmap_min_obstacle_height", min_obstacle_height_,
                       -1.2);
    nh_->param<int>("costmap_obstacleValue", obstacle_value_, 99);
    nh_->param<int>("costmap_dilate_radius", costmap_dilate_radius_, 3);
    nh_->param<int>("costmap_smoothness_factor", costmap_smoothness_factor_, 1);

    // Prepare Occupancy Grid , setup size, resolution,obstacle_height
    InitLocalMapGrid();

    // a flag to chechk wheter costmap was updated or not
    updated_costmap_ = false;

    // Publish Dteceted Obstacles
    detected_obstacles_publisher_ =
        nh_->advertise<visualization_msgs::MarkerArray>(
            "detected_obstacles_from_local_costmap_segmented_pcl", 1);
    // Publish LOcal Costmap , can be used for path planing, but must be
    // combines with global map
    local_costmap_publisher_ = nh_->advertise<nav_msgs::OccupancyGrid>(
        "local_costmap_segmented_pcl", 1);
}

// Deconstruct  Costmap
GridCellCostmapSegmentedPCL::~GridCellCostmapSegmentedPCL(){};

// Get local occupancy grid map. Includes the map topology and the road
// obstacles.
// return: Occupancy grid ROS type.
const nav_msgs::OccupancyGrid GridCellCostmapSegmentedPCL::GetOccupancyGrid() {
    return obstacle_grid_;
}

// Get local occupancy grid map pointer. Includes the map topology and the
// road obstacles.
// return: Occupancy grid ROS type pointer.
// Returns pointer to costmap
nav_msgs::OccupancyGrid::ConstPtr
GridCellCostmapSegmentedPCL::GetOccupancyGridPtr() {
    nav_msgs::OccupancyGrid::ConstPtr grid_ptr(
        new nav_msgs::OccupancyGrid(obstacle_grid_));
    return grid_ptr;
}

// Set local occupancy grid map.
// param [in] value: Occupancy grid ROS information.
void GridCellCostmapSegmentedPCL::SetOccupancyGrid(
    const nav_msgs::OccupancyGrid value) {
    obstacle_grid_ = value;
}

// Gets the flag, returns true if costmap was Updated
const bool GridCellCostmapSegmentedPCL::GetUpdatedCostMap() {
    return updated_costmap_;
}

// Sets the flag of costmap update
void GridCellCostmapSegmentedPCL::SetUpdatedCostMap(bool value) {
    updated_costmap_ = value;
}

// Costmap Parameter Initializer
void GridCellCostmapSegmentedPCL::InitLocalMapGrid() {
    obstacle_grid_.info.height = grid_height_;
    obstacle_grid_.info.width = grid_width_;
    obstacle_grid_.info.resolution = grid_resolution_;
    obstacle_grid_.header.frame_id = "base_link";
    obstacle_grid_.info.origin.position.x = 0;
    obstacle_grid_.info.origin.position.y =
        -grid_resolution_ * grid_width_ / 2.0;
    obstacle_grid_.info.origin.position.z = 0.0;

    obstacle_grid_.info.origin.orientation.x = 0.0;
    obstacle_grid_.info.origin.orientation.y = 0.0;
    obstacle_grid_.info.origin.orientation.z = 0.0;
    obstacle_grid_.info.origin.orientation.w = 1.0;

    obstacle_grid_.header.stamp = ros::Time::now();
    obstacle_grid_.data.reserve(obstacle_grid_.info.width *
                                obstacle_grid_.info.height);
    for (int i = 0; i < obstacle_grid_.info.width * obstacle_grid_.info.height;
         i++) {
        obstacle_grid_.data.push_back(0);
    }
}

void GridCellCostmapSegmentedPCL::ModifyCostmapPointValue(
    nav_msgs::OccupancyGrid& map, const int value,
    const geometry_msgs::Point32 point) {
    int aux_x = (point.x) / map.info.resolution;
    int aux_y = (point.y) / map.info.resolution;

    aux_x = (map.info.height / 2) - aux_x;
    aux_y = (map.info.width) - aux_y;

    std::size_t og_index = aux_x * map.info.width + aux_y;

    if (og_index > 0 && og_index < map.data.size()) {
        map.data[og_index] = value;
    }
}

// Reads Lidar Points , Iterates through all the points, Once the point x,y,z
// values found to be in defined bounds, overrides the correspoinding array
// index to occupied value(99)
void GridCellCostmapSegmentedPCL::ProcessGridMap(
    sensor_msgs::PointCloud2::ConstPtr& input_pointcloud) {
    nav_msgs::OccupancyGrid grid = GetOccupancyGrid();
    std::fill(grid.data.begin(), grid.data.end(), 0);
    sensor_msgs::PointCloud2 in_cld = *input_pointcloud.get();

    if (in_cld.height == 0 || in_cld.width == 0) {
        return;
    }

    // loop through points
    for (sensor_msgs::PointCloud2Iterator<float> iter_x(in_cld, "x");
         iter_x != iter_x.end(); ++iter_x) {
        // assign a current point to Point32 type for simplicity
        // chechk z axis bounds
        if (iter_x[1] > min_obstacle_height_ &&
            iter_x[1] < max_obstacle_height_) {
            // Check x , y axes bounds
            if ((iter_x[2] > -grid_height_ / 2 &&
                 iter_x[2] < grid_height_ / 2) &&
                (iter_x[0] > -grid_width_ / 2 && iter_x[0] < grid_width_ / 2)) {
                // This point is on some obstacle , so we change the
                // corresponding value in local_grid_map_.data to
                // kOccupiedCellValue(99)
                geometry_msgs::Point32 aux_point;
                aux_point.x = iter_x[0];
                aux_point.y = -iter_x[2];
                ModifyCostmapPointValue(grid, obstacle_value_, aux_point);
            }
        }
    }

    int kDilationType = cv::MORPH_RECT;
    cv::Mat image =
        cv::Mat(grid.info.width, grid.info.height, CV_8U, &(grid.data[0]));

    // Mimics Inflation Layer
    if (costmap_dilate_radius_ > 0) {
        cv::Mat element = cv::getStructuringElement(
            kDilationType,
            cv::Size(2 * costmap_dilate_radius_ + 1,
                     2 * costmap_dilate_radius_ + 1),
            cv::Point(costmap_dilate_radius_, costmap_dilate_radius_));

        cv::dilate(image, image, element);

        element = cv::getStructuringElement(
            kDilationType,
            cv::Size(4 * costmap_dilate_radius_ + 1,
                     4 * costmap_dilate_radius_ + 1),
            cv::Point(costmap_dilate_radius_, costmap_dilate_radius_));
        // cv::erode(image, image, element);
    }

    if (costmap_smoothness_factor_ > 0) {
        cv::GaussianBlur(
            image, image,
            cv::Size(costmap_smoothness_factor_, costmap_smoothness_factor_), 0,
            0);
    }

    // Put updates Csotmap flag to true
    SetUpdatedCostMap(true);
    SetOccupancyGrid(grid);

    local_costmap_publisher_.publish(grid);
}
// Detects Obstacles , Converts Grid Cell costmap to cv Image , finds contours
// and Minimum Rectangle that surroounds this countours
// Finally Publish Obstacles as Marker Array
ros::Time previous_time;
void GridCellCostmapSegmentedPCL::DetectObstacles(
    sensor_msgs::PointCloud2::ConstPtr& input_pointcloud) {
    int kMaxValue = 255;
    int kMinObstacleSize = 8;
    int kMaxObstacleSize = 100;

    float kRad2Deg = 57.32;
    int kObstaclesThreshold = 99;
    float kDefaultObstacleHeight = 2.0;

    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

    // Process the Costmap
    if (previous_time != input_pointcloud->header.stamp) {
        previous_time = input_pointcloud->header.stamp;
        ProcessGridMap(input_pointcloud);
    }

    // Get the Occupancy grid.
    nav_msgs::OccupancyGrid occupancy_map = GetOccupancyGrid();

    std_msgs::Header header = occupancy_map.header;
    nav_msgs::MapMetaData info = occupancy_map.info;

    detected_obstacles_.markers.clear();
    if (GetUpdatedCostMap()) {
        SetUpdatedCostMap(false);

        if (info.width > 0 && info.height > 0) {
            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;

            // Costmap to image.
            cv::Mat image = cv::Mat(info.width, info.height, CV_8U,
                                    &(occupancy_map.data[0]));

            // Finde obstacles contour.
            cv::findContours(image, contours, hierarchy, cv::RETR_TREE,
                             cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

            for (int i = 0; i < contours.size(); i++) {
                float ang;
                cv::RotatedRect minRect;

                // Fit the contour to the minimum rectangle.
                minRect = cv::minAreaRect(cv::Mat(contours[i]));

                if (minRect.size.width > kMinObstacleSize &&
                    minRect.size.width < kMaxObstacleSize &&
                    minRect.size.height > kMinObstacleSize &&
                    minRect.size.height < kMaxObstacleSize) {
                    float aux_x, aux_y;

                    aux_x = (minRect.center.x * info.resolution);
                    //-(info.width * info.resolution / 2);
                    aux_y = (minRect.center.y * info.resolution) -
                            (info.height * info.resolution / 2);

                    // Doing this because of  MinRect logic
                    if (minRect.size.height < minRect.size.width) {
                        std::swap(minRect.size.width, minRect.size.height);
                    }

                    // create A marker to represent The MinRect AROUND This
                    // contour in RVIZ
                    visualization_msgs::Marker marker;

                    //  x' = x*cos q - y*sin q
                    // y' = x*sin q + y*cos q

                    // Fill Marker Information

                    // Note , Countour and MinRect are on cv Image so the metric
                    // is pixel , however we need to convert this to units that
                    // RVIZ accepts which is meters in this case

                    marker.header.frame_id = "base_link";

                    marker.header.stamp = ros::Time::now();
                    marker.ns = "detected_obstacles";
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.lifetime = ros::Duration(0.2);
                    marker.id = i;
                    marker.type = visualization_msgs::Marker::CUBE;
                    marker.pose.position.x = aux_x;
                    marker.pose.position.y = aux_y;
                    marker.pose.position.z = 1;
                    marker.pose.orientation.x = 0.0;
                    marker.pose.orientation.y = 0.0;
                    marker.pose.orientation.z = 0.0;
                    marker.pose.orientation.w = 1.0;

                    marker.scale.y = minRect.size.width * info.resolution;
                    marker.scale.x = minRect.size.height * info.resolution;
                    marker.scale.z = 2.0;
                    marker.color.a = 1.0;
                    // give random color to Box
                    marker.color.r = i % 3;
                    marker.color.g = 0.5;
                    marker.color.b = i % 2;
                    // Store this marker in array
                    detected_obstacles_.markers.push_back(marker);
                }
            }
        }
    }

    // finally Publish all markers
    detected_obstacles_publisher_.publish(detected_obstacles_);
}