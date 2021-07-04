#include "mapLoader.h"

MapLoader::MapLoader(ros::NodeHandle &nh){
    std::string pcd_file_path, map_topic, robot_pose_topic, init_pose_topic;

    nh.param<std::string>("pcd_path", pcd_file_path, "");
    nh.param<std::string>("map_topic", map_topic, "point_map");
    nh.param<std::string>("init_pose_topic", init_pose_topic, "initialpose");

    std::cout << "init_pose_topic: " << init_pose_topic << std::endl;

    nh.param<std::string>("robot_pose_topic", robot_pose_topic, "ndt_pose");
    nh.param<float>("submap_size_xy", submap_size_xy_, 100);
    nh.param<float>("submap_size_z", submap_size_z_, 50);
    nh.param<float>("map_switch_thres", map_switch_thres_, 10);

    init_tf_params(nh);

    pc_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>(map_topic, 10, true);
    initial_pose_sub_ = nh.subscribe(init_pose_topic, 1, &MapLoader::callbackInitPose, this);
    ndt_pose_sub_ = nh.subscribe(robot_pose_topic, 10, &MapLoader::callbackRobotPose, this);

    file_list_.push_back(pcd_file_path);

    createPcd();
    transformMap();
}

void MapLoader::init_tf_params(ros::NodeHandle &nh){
    nh.param<float>("x", tf_x_, 0.0);
    nh.param<float>("y", tf_y_, 0.0);
    nh.param<float>("z", tf_z_, 0.0);
    nh.param<float>("roll", tf_roll_, 0.0);
    nh.param<float>("pitch", tf_pitch_, 0.0);
    nh.param<float>("yaw", tf_yaw_, 0.0);
    ROS_INFO_STREAM("x" << tf_x_ <<"y: "<<tf_y_<<"z: "<<tf_z_<<"roll: "
                        <<tf_roll_<<" pitch: "<< tf_pitch_<<"yaw: "<<tf_yaw_);
}

void MapLoader::transformMap(){

    Eigen::Translation3f tl_m2w(tf_x_, tf_y_, tf_z_);                 // tl: translation
    Eigen::AngleAxisf rot_x_m2w(tf_roll_, Eigen::Vector3f::UnitX());  // rot: rotation
    Eigen::AngleAxisf rot_y_m2w(tf_pitch_, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_m2w(tf_yaw_, Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f tf_m2w = (tl_m2w * rot_z_m2w * rot_y_m2w * rot_x_m2w).matrix();

    pcl::transformPointCloud(*global_map_ptr_, *global_map_ptr_, tf_m2w);

    SaveMap(global_map_ptr_);
}

sensor_msgs::PointCloud2 MapLoader::switchSubmap(geometry_msgs::Pose pose) {

    pcl::CropBox<pcl::PointXYZ> box_filter;
    box_filter.setMin(Eigen::Vector4f(pose.position.x-submap_size_xy_, pose.position.y-submap_size_xy_, pose.position.z-submap_size_z_, 1.0));
    box_filter.setMax(Eigen::Vector4f(pose.position.x+submap_size_xy_, pose.position.y+submap_size_xy_, pose.position.z+submap_size_z_, 1.0));

    std::cout << "min: " << pose.position.x-submap_size_xy_ << ", " << pose.position.y-submap_size_xy_ << ", " << pose.position.z-submap_size_z_ << std::endl;
    std::cout << "max: " << pose.position.x+submap_size_xy_ << ", " << pose.position.y+submap_size_xy_ << ", " << pose.position.z+submap_size_z_ << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr submap_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    box_filter.setInputCloud(global_map_ptr_);
    box_filter.filter(*submap_ptr);

    sensor_msgs::PointCloud2 submap_msg;
    pcl::toROSMsg(*submap_ptr, submap_msg);
    return submap_msg;
}

void MapLoader::callbackInitPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &initial_pose_msg_ptr)
{
    curr_pose_ = initial_pose_msg_ptr->pose.pose;
    auto sub_map_msg = switchSubmap(curr_pose_);

    if (sub_map_msg.width != 0) {
		sub_map_msg.header.frame_id = "map";
		pc_map_pub_.publish(sub_map_msg);
        ROS_INFO("submap is published!");
	}
    pre_pose_ = curr_pose_;
}

void MapLoader::callbackRobotPose(const nav_msgs::Odometry::ConstPtr &ndt_odom_msg)
{
  curr_pose_ = ndt_odom_msg->pose.pose;

  traversal_dist_ += sqrt(pow(curr_pose_.position.x-pre_pose_.position.x,2) + pow(curr_pose_.position.y-pre_pose_.position.y,2));

  if(traversal_dist_>= map_switch_thres_) {

    auto sub_map_msg = switchSubmap(curr_pose_);

    if (sub_map_msg.width != 0) {
	    sub_map_msg.header.frame_id = "map";
	    pc_map_pub_.publish(sub_map_msg);
        ROS_INFO("new submap is published!");
	}
    traversal_dist_ = 0;
  }
  pre_pose_ = curr_pose_;
}

void MapLoader::SaveMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr map_pc_ptr){
    pcl::io::savePCDFile("/tmp/transformed_map.pcd", *map_pc_ptr);
}

void MapLoader::createPcd()
{
	sensor_msgs::PointCloud2 pcd, part;
	for (const std::string& path : file_list_) {
		// Following outputs are used for progress bar of Runtime Manager.
		if (pcd.width == 0) {
			if (pcl::io::loadPCDFile(path.c_str(), pcd) == -1) {
				std::cerr << "load failed " << path << std::endl;
			}
		} else {
			if (pcl::io::loadPCDFile(path.c_str(), part) == -1) {
				std::cerr << "load failed " << path << std::endl;
			}
			pcd.width += part.width;
			pcd.row_step += part.row_step;
			pcd.data.insert(pcd.data.end(), part.data.begin(), part.data.end());
		}
		std::cerr << "load " << path << std::endl;
		if (!ros::ok()) break;
	}
    global_map_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(pcd, *global_map_ptr_);

    // Publish map for easy visual initialization
    pcd.header.frame_id = "map";
    pc_map_pub_.publish(pcd);

    ROS_INFO_STREAM("done!");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_loader");

    ROS_INFO("\033[1;32m---->\033[0m Map Loader Started.");

    ros::NodeHandle nh("~");

    MapLoader map_loader(nh);

    ros::spin();

    return 0;
}
