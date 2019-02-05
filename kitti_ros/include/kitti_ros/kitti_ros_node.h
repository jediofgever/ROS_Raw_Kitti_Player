#include <kitti_ros/perception/grid_cell_costmap.h>
#include <kitti_ros/perception/grid_cell_costmap_segmented_pcl.h>
#include <kitti_ros/perception/sensor_fusion.h>
#include <iostream>
#include <sstream>

using namespace std;
using namespace gridcellcostmap;

class KittiRosNode {
   public:
    KittiRosNode();
    ~KittiRosNode();

    void ProcessNode();

    void ProcessKittiPointCloud(std::string &pcd_infile_string);
    void ProcessKittiImage(std::string &pcd_infile_string);

    void ProcessKittiGroundTruthLabel(std::string &label_infile_string,
                                      std::string image_file_path);

    void ObstacleDetectionandGridCellCostmap();

    void ObstacleDetectionSegmentedPCL();

   private:
    ros::NodeHandlePtr nh_;

    KittiObjectOperator kitti_object_operator_;

    KITTIDataOperator kitti_data_operator_;

    KittiObjectOperator::KittiObjectsThisFrame kitti_objects_;

    SensorFusion sensor_fusion_;

    GridCellCostmap grid_cell_costmap_;

    GridCellCostmapSegmentedPCL grid_cell_costmap_segmented_pcl_;

    Tools tools_;

    std::string base_dir;
    std::string pcd_file_dir;
    std::string image_dir;
    std::string label_dir;
    std::string maskrcnn_detection_image_dir;

    std::string maskrcnn_detection_label_dir;
    std::string pcd_file_extension;
    std::string label_file_extension;
    std::string image_file_extension;

    int number_of_pcd_files;
    int node_loop_rate;
};
