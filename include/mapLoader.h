#ifndef _MAP_LOADER_H_
#define _MAP_LOADER_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <pcl_ros/transforms.h>

class MapLoader{
public:

    float tranversal_dist=0;
    float map_switch_thres=10.;

    ros::Publisher pc_map_pub_;
    ros::Subscriber ndt_pose_sub_;
    ros::Subscriber initial_pose_sub_;
    std::vector<std::string> file_list_;

    MapLoader(ros::NodeHandle &nh);

private:

    float tf_x_, tf_y_, tf_z_, tf_roll_, tf_pitch_, tf_yaw_; 
    geometry_msgs::Pose curr_pose_, pre_pose_;
    float submap_size_xy_, submap_size_z_;
    float traversal_dist_=0.;
    float map_switch_thres_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_ptr_;

    void init_tf_params(ros::NodeHandle &nh);
    void createPcd();
    void transformMap();
    void SaveMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr map_pc_ptr);
    void callbackRobotPose(const nav_msgs::Odometry::ConstPtr &ndt_odom_msg);
    void callbackInitPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &initial_pose_msg_ptr);
    sensor_msgs::PointCloud2 switchSubmap(geometry_msgs::Pose pose);

}; //MapLoader

#endif