/**
 * @author Fetullah Atas
 * @email [fetulahatas1@gmail.com]
 * @create date 2018-11-06 17:35:20
 * @modify date 2018-11-06 17:35:20
 * @desc [description]
 */

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/MarkerArray.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// input: LIDAR point clouds, sensor_msgs/PointCloud2
// output: Costmap , nav_msgs/OccupancyGrid
// output: Obstacle Detection visualization_msgs/MarkerArray
// Subscribes to laser_scan topic provided by simulated LIDAR, Produces a Local
// Costmap, Regresses a minial Rectangle around each obstacle

namespace gridcellcostmap {

class GridCellCostmap {
    // Public Methods
   public:
    // Constructs the Instance
    GridCellCostmap();
    // Densonsctructs The instance
    ~GridCellCostmap();

    // input: LIDAR point clouds, sensor_msgs/PointCloud2
    // output: Costmap , nav_msgs/OccupancyGrid
    void ProcessGridMap(sensor_msgs::PointCloud2::ConstPtr& input_pointcloud);

    // Get local occupancy grid map. Includes the map topology and the road
    // obstacles.
    // return:nav_msgs/OccupancyGrid ROS type.
    const nav_msgs::OccupancyGrid GetOccupancyGrid();

    // Get local occupancy grid map pointer. Includes the map topology and the
    // road obstacles.
    // return: nav_msgs/OccupancyGridROS type pointer.
    nav_msgs::OccupancyGrid::ConstPtr GetOccupancyGridPtr();

    // Set local occupancy grid map.
    // param [in] value: Occupancy grid ROS information.
    void SetOccupancyGrid(const nav_msgs::OccupancyGrid value);

    // returns bool value , true if the costmap was updated
    const bool GetUpdatedCostMap();

    // Setter of costmap , utilized inside ProcessGridMap , after done process
    void SetUpdatedCostMap(bool value);

    // input: Costmap , nav_msgs/OccupancyGrid
    // input: Detected Obstacles published as , visuazlization_msgs/MarkerArray
    void DetectObstacles(sensor_msgs::PointCloud2::ConstPtr& input_pointcloud);

   private:
    // data is stored in nav_msgs/OccupancyGrid as 1D array, so we need to find
    // index of occupied cell in this 1D array and modify the value
    void ModifyCostmapPointValue(nav_msgs::OccupancyGrid& map, const int value,
                                 const geometry_msgs::Point32 point);

    // setups GridCell Map variables such as width , height, resolution etc ..
    void InitLocalMapGrid();

    // BEGIN  ######## Grid Cell Varibales Declaration
    double grid_width_;
    double grid_height_;
    double grid_resolution_;
    double max_obstacle_height_;
    double min_obstacle_height_;
    double map_update_rate_;
    int obstacle_value_;
    // END ######## Grid Cell Variables Declaration

    // Grid Cell
    nav_msgs::OccupancyGrid obstacle_grid_;

    // Costmap Update Flag
    bool updated_costmap_;

    // Costmap dilate radius for safety marging.
    int costmap_dilate_radius_;

    // Costmap smoothnes factor.
    int costmap_smoothness_factor_;

    // Ros Node Handleer Pointer
    ros::NodeHandlePtr nh_;

    // Marker Array for visualzing detected Obstacles
    visualization_msgs::MarkerArray detected_obstacles_;

    // Publish Detected Obstacles
    ros::Publisher detected_obstacles_publisher_;

    // Publish Local Costmap(Occupancy Grid )
    ros::Publisher local_costmap_publisher_;
};

}  // namespace gridcellcostmap