#include "ndt.h"

NdtLocalizer::NdtLocalizer(ros::NodeHandle &nh, ros::NodeHandle &private_nh) : nh_(nh), private_nh_(private_nh), tf2_listener_(tf2_buffer_)
{

  key_value_stdmap_["state"] = "Initializing";
  init_params();

  // Publishers
  sensor_aligned_pose_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("points_aligned", 10);
  ndt_pose_pub_ = nh_.advertise<nav_msgs::Odometry>("ndt_pose", 10);
  exe_time_pub_ = nh_.advertise<std_msgs::Float32>("exe_time_ms", 10);
  transform_probability_pub_ = nh_.advertise<std_msgs::Float32>("transform_probability", 10);
  iteration_num_pub_ = nh_.advertise<std_msgs::Float32>("iteration_num", 10);
  diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 10);
  poly_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("mapping_path", 1);

  // Subscribers
  initial_pose_sub_ = nh_.subscribe("initialpose", 100, &NdtLocalizer::callback_init_pose, this);
  map_points_sub_ = nh_.subscribe("points_map", 1, &NdtLocalizer::callback_pointsmap, this);
  sensor_points_sub_ = nh_.subscribe("filtered_points", 1, &NdtLocalizer::callback_pointcloud, this);
  odom_sub_ = nh_.subscribe("odom", 10, &NdtLocalizer::callback_odom, this);

  diagnostic_thread_ = std::thread(&NdtLocalizer::timer_diagnostic, this);
  diagnostic_thread_.detach();
}

NdtLocalizer::~NdtLocalizer() {}

void NdtLocalizer::getXYZRPYfromMat(const Eigen::Matrix4f mat, Pose &p)
{
  Eigen::Vector3f translation = mat.block<3, 1>(0, 3);

  Eigen::Matrix3f rotation_matrix = mat.block<3, 3>(0, 0);
  Eigen::Vector3f euler = rotation_matrix.eulerAngles(0, 1, 2);

  p.x = translation(0);
  p.y = translation(1);
  p.z = translation(2);
  p.roll = euler(0);
  p.pitch = euler(1);
  p.yaw = euler(2);
}

void NdtLocalizer::timer_diagnostic()
{
  ros::Rate rate(100);
  while (ros::ok())
  {
    diagnostic_msgs::DiagnosticStatus diag_status_msg;
    diag_status_msg.name = "ndt_scan_matcher";
    diag_status_msg.hardware_id = "";

    for (const auto &key_value : key_value_stdmap_)
    {
      diagnostic_msgs::KeyValue key_value_msg;
      key_value_msg.key = key_value.first;
      key_value_msg.value = key_value.second;
      diag_status_msg.values.push_back(key_value_msg);
    }

    diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::OK;
    diag_status_msg.message = "";
    if (key_value_stdmap_.count("state") && key_value_stdmap_["state"] == "Initializing")
    {
      diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::WARN;
      diag_status_msg.message += "Initializing State. ";
    }
    if (
        key_value_stdmap_.count("skipping_publish_num") &&
        std::stoi(key_value_stdmap_["skipping_publish_num"]) > 1)
    {
      diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::WARN;
      diag_status_msg.message += "skipping_publish_num > 1. ";
    }
    if (
        key_value_stdmap_.count("skipping_publish_num") &&
        std::stoi(key_value_stdmap_["skipping_publish_num"]) >= 5)
    {
      diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      diag_status_msg.message += "skipping_publish_num exceed limit. ";
    }

    diagnostic_msgs::DiagnosticArray diag_msg;
    diag_msg.header.stamp = ros::Time::now();
    diag_msg.status.push_back(diag_status_msg);

    diagnostics_pub_.publish(diag_msg);

    rate.sleep();
  }
}

void NdtLocalizer::callback_init_pose(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &initial_pose_msg_ptr)
{
  if (initial_pose_msg_ptr->header.frame_id == map_frame_)
  {
    initial_pose_cov_msg_ = *initial_pose_msg_ptr;

    if (path_file!="")
    {
      loadPath(path_file);
      initial_pose_cov_msg_.pose.pose.position.z = getNearestHeight(initial_pose_cov_msg_.pose.pose);
      //debug_pose_marker(initial_pose_cov_msg_.pose.pose);
      poly_pub_.publish(poly);
    }

    ROS_INFO("Manually initialised");
  }
  else
  {
    // get TF from pose_frame to map_frame
    geometry_msgs::TransformStamped::Ptr TF_pose_to_map_ptr(new geometry_msgs::TransformStamped);
    get_transform(map_frame_, initial_pose_msg_ptr->header.frame_id, TF_pose_to_map_ptr);

    std::cout << "initial pose frame if" << initial_pose_msg_ptr->header.frame_id << std::endl;

    // transform pose_frame to map_frame
    geometry_msgs::PoseWithCovarianceStamped::Ptr mapTF_initial_pose_msg_ptr(
        new geometry_msgs::PoseWithCovarianceStamped);
    tf2::doTransform(*initial_pose_msg_ptr, *mapTF_initial_pose_msg_ptr, *TF_pose_to_map_ptr);
    // mapTF_initial_pose_msg_ptr->header.stamp = initial_pose_msg_ptr->header.stamp;
    initial_pose_cov_msg_ = *mapTF_initial_pose_msg_ptr;
  }

  nav_msgs::Odometry init_odom_msg;
  init_odom_msg.header.stamp = initial_pose_cov_msg_.header.stamp;
  init_odom_msg.header.frame_id = map_frame_;
  init_odom_msg.child_frame_id = "base_link";
  init_odom_msg.pose.pose = initial_pose_cov_msg_.pose.pose;

  ndt_pose_pub_.publish(init_odom_msg);

  // if click the initpose again, re init！
  init_pose = false;
}

void NdtLocalizer::callback_pointsmap(
    const sensor_msgs::PointCloud2::ConstPtr &map_points_msg_ptr)
{
  const auto trans_epsilon = ndt_->getTransformationEpsilon();
  const auto step_size = ndt_->getStepSize();
  const auto resolution = ndt_->getResolution();
  const auto max_iterations = ndt_->getMaximumIterations();

  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>* ndt_new(new pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>);
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>* ndt_old = ndt_;

  ndt_new->setTransformationEpsilon(trans_epsilon);
  ndt_new->setStepSize(step_size);
  ndt_new->setResolution(resolution);
  ndt_new->setMaximumIterations(max_iterations);

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_points_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*map_points_msg_ptr, *map_points_ptr);
  ndt_new->setInputTarget(map_points_ptr);
  // create Thread
  // detach
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  ndt_new->align(*output_cloud, Eigen::Matrix4f::Identity());

  // swap
  //const auto swap_time = std::chrono::system_clock::now();
  ndt_map_mtx_.lock();
  ndt_ = ndt_new;
  ndt_map_mtx_.unlock();
  //const auto swap_end_time = std::chrono::system_clock::now();
  //const auto swap_use_time = std::chrono::duration_cast<std::chrono::microseconds>(swap_end_time - swap_time).count() / 1000.0;
  //std::cout << "swap map time: " << swap_use_time << std::endl;

  delete ndt_old;
}

void NdtLocalizer::callback_odom(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
  if(is_ndt_published) {
    geometry_msgs::Pose odom_pose_msg = odom_msg->pose.pose;

    Eigen::Vector3f t(odom_pose_msg.position.x, odom_pose_msg.position.y, odom_pose_msg.position.z);
    odom_trans.block<3, 1>(0, 3) = t;
    Eigen::Quaternionf q(odom_pose_msg.orientation.w, odom_pose_msg.orientation.x, odom_pose_msg.orientation.y, odom_pose_msg.orientation.z);
    odom_trans.block<3, 3>(0, 0) = q.toRotationMatrix();
  }
}

void NdtLocalizer::callback_pointcloud(
    const sensor_msgs::PointCloud2::ConstPtr &sensor_points_sensorTF_msg_ptr)
{
  const auto exe_start_time = std::chrono::system_clock::now();
  // mutex Map
  std::lock_guard<std::mutex> lock(ndt_map_mtx_);

  const std::string sensor_frame = sensor_points_sensorTF_msg_ptr->header.frame_id;
  const auto sensor_ros_time = sensor_points_sensorTF_msg_ptr->header.stamp;

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensor_points_sensorTF_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(*sensor_points_sensorTF_msg_ptr, *sensor_points_sensorTF_ptr);

  // get TF odom to base
  geometry_msgs::TransformStamped::Ptr TF_odom_to_base_ptr(new geometry_msgs::TransformStamped);
  get_transform(odom_frame_, base_frame_, TF_odom_to_base_ptr, sensor_ros_time);

  const Eigen::Affine3d odom_to_base_affine = tf2::transformToEigen(*TF_odom_to_base_ptr);
  const Eigen::Matrix4f odom_to_base_matrix = odom_to_base_affine.matrix().cast<float>();

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensor_points_baselinkTF_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);

  // get the sensor offset to the base_link
  if (!init_pose)
  {
    // get TF base to sensor
    geometry_msgs::TransformStamped::Ptr TF_base_to_sensor_ptr(new geometry_msgs::TransformStamped);
    get_transform(base_frame_, sensor_frame, TF_base_to_sensor_ptr);

    const Eigen::Affine3d base_to_sensor_affine = tf2::transformToEigen(*TF_base_to_sensor_ptr);
    base_to_sensor_matrix_ = base_to_sensor_affine.matrix().cast<float>();
  }

  pcl::transformPointCloud(
      *sensor_points_sensorTF_ptr, *sensor_points_baselinkTF_ptr, base_to_sensor_matrix_);

  // set input point cloud
  ndt_->setInputSource(sensor_points_baselinkTF_ptr);

  if (ndt_->getInputTarget() == nullptr || !is_ndt_published)
  {
    ROS_WARN_STREAM_THROTTLE(1, "No MAP!");
    Eigen::Affine3d initial_pose_affine;
    tf2::fromMsg(initial_pose_cov_msg_.pose.pose, initial_pose_affine);
    initial_pose_matrix = initial_pose_affine.matrix().cast<float>();

    Pose tmp;
    getXYZRPYfromMat(initial_pose_matrix, tmp);
    std::cout << "manual initial pose: " << "x: " << tmp.x << " y: " << tmp.y << " z: " << tmp.z << " r: " << tmp.roll << " p: " << tmp.pitch << " y: " << tmp.yaw << std::endl;

    // for the first time, we don't know the pre_trans, so just use the init_trans,
    // which means, the delta trans for the second time is 0
    pre_trans = initial_pose_matrix;
    odom_trans = initial_pose_matrix;
    pre_odom_trans = odom_trans;
    map_to_odom_matrix.setIdentity(); // // local odom mode

    std::cout << "not initialised!" << std::endl;

    init_pose = true;
  }
  else
  {
    // use predicted pose as init guess
    // initial_pose_matrix = pre_trans * delta_trans;  // linear prediction mode
    //initial_pose_matrix = map_to_odom_matrix * odom_trans;  // local odom mode
    initial_pose_matrix = odom_trans; // global odom mode
  }
 

  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  const auto align_start_time = std::chrono::system_clock::now();
  key_value_stdmap_["state"] = "Aligning";
  ndt_->align(*output_cloud, initial_pose_matrix);
  key_value_stdmap_["state"] = "Sleeping";
  const auto align_end_time = std::chrono::system_clock::now();
  const double align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end_time - align_start_time).count() / 1000.0;

  const Eigen::Matrix4f result_pose_matrix = ndt_->getFinalTransformation();
  Eigen::Affine3d result_pose_affine;
  result_pose_affine.matrix() = result_pose_matrix.cast<double>();
  const geometry_msgs::Pose result_pose_msg = tf2::toMsg(result_pose_affine);

  const auto exe_end_time = std::chrono::system_clock::now();
  const double exe_time = std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count() / 1000.0;

  const float transform_probability = ndt_->getTransformationProbability();
  const int iteration_num = ndt_->getFinalNumIteration();

  //Pose tmp;
  //getXYZRPYfromMat(result_pose_matrix, tmp);
  //std::cout << "ndt pose: " << "x: " << tmp.x << " y: " << tmp.y << " z: " << tmp.z << " r: " << tmp.roll << " p: " << tmp.pitch << " y: " << tmp.yaw << std::endl;

  bool is_converged = true;
  static size_t skipping_publish_num = 0;
  if (is_ndt_published &&
      (iteration_num >= ndt_->getMaximumIterations() + 2 ||
      transform_probability < converged_param_transform_probability_))
  {
    is_converged = false;
    ++skipping_publish_num;
    ROS_ERROR("Not Converged");
  }
  else
  {
    skipping_publish_num = 0;
  }
  // calculate the delta tf from pre_trans to current_trans, delta_trans is mostly used in linear prediction model
  delta_trans = pre_trans.inverse() * result_pose_matrix;

  Eigen::Vector3f delta_translation = delta_trans.block<3, 1>(0, 3);
  std::cout << "delta x: " << delta_translation(0) << " y: " << delta_translation(1) << " z: " << delta_translation(2) << std::endl;

  Eigen::Matrix3f delta_rotation_matrix = delta_trans.block<3, 3>(0, 0);
  Eigen::Vector3f delta_euler = delta_rotation_matrix.eulerAngles(2, 1, 0);
  std::cout << "delta yaw: " << delta_euler(0) << " pitch: " << delta_euler(1) << " roll: " << delta_euler(2) << std::endl;

  // define the variance of ndt localsiation
  double deviation_t, deviation_r;

  if(!is_converged)
  {
    deviation_t = deviation_r = 1000.; 
    delta_trans.setIdentity();
  }
  else
  {
    pre_trans = result_pose_matrix;
    deviation_t = delta_translation.norm()*10;
    deviation_r = std::min(std::abs(float(M_PI)-delta_euler(0)), std::abs(delta_euler(0)))*10;
  }

  // publish pose message
  nav_msgs::Odometry result_pose_stamped_msg;
  result_pose_stamped_msg.header.stamp = sensor_ros_time;
  result_pose_stamped_msg.header.frame_id = map_frame_;
  result_pose_stamped_msg.child_frame_id = "base_link";
  result_pose_stamped_msg.pose.pose = result_pose_msg;
  result_pose_stamped_msg.pose.covariance[0] = deviation_t;
  result_pose_stamped_msg.pose.covariance[7] = deviation_t;
  result_pose_stamped_msg.pose.covariance[14] = deviation_t;
  result_pose_stamped_msg.pose.covariance[21] = deviation_r;
  result_pose_stamped_msg.pose.covariance[28] = deviation_r;
  result_pose_stamped_msg.pose.covariance[35] = deviation_r;

  if (is_converged || !is_ndt_published)
  {
    ndt_pose_pub_.publish(result_pose_stamped_msg);
    is_ndt_published = true;
  }

  // publish tf
  map_to_odom_matrix = result_pose_matrix * (odom_to_base_matrix.inverse());

  if (!is_converged)
  {
    map_to_odom_matrix = pre_corr_trans;
  }

  Eigen::Affine3d map_to_odom_affine;
  map_to_odom_affine.matrix() = map_to_odom_matrix.cast<double>();
  const geometry_msgs::Pose result_pose_msg2 = tf2::toMsg(map_to_odom_affine);

  // publish tf (map frame to odom frame)
  bool is_publish_tf = false;

  if(is_publish_tf)
  {
    geometry_msgs::PoseStamped result_pose_stamped_msg2;
    result_pose_stamped_msg2.header.stamp = sensor_ros_time;
    result_pose_stamped_msg2.header.frame_id = map_frame_;
    result_pose_stamped_msg2.pose = result_pose_msg2;

    publish_tf(map_frame_, odom_frame_, result_pose_stamped_msg2);
  }

  pre_corr_trans = map_to_odom_matrix;

  // publish aligned point cloud
  if (is_converged)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_points_mapTF_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(
        *sensor_points_baselinkTF_ptr, *sensor_points_mapTF_ptr, result_pose_matrix);
    sensor_msgs::PointCloud2 sensor_points_mapTF_msg;
    pcl::toROSMsg(*sensor_points_mapTF_ptr, sensor_points_mapTF_msg);
    sensor_points_mapTF_msg.header.stamp = sensor_ros_time;
    sensor_points_mapTF_msg.header.frame_id = map_frame_;
    sensor_aligned_pose_pub_.publish(sensor_points_mapTF_msg);
  }

  std_msgs::Float32 exe_time_msg;
  exe_time_msg.data = exe_time;
  exe_time_pub_.publish(exe_time_msg);

  std_msgs::Float32 transform_probability_msg;
  transform_probability_msg.data = transform_probability;
  transform_probability_pub_.publish(transform_probability_msg);

  std_msgs::Float32 iteration_num_msg;
  iteration_num_msg.data = iteration_num;
  iteration_num_pub_.publish(iteration_num_msg);

  key_value_stdmap_["seq"] = std::to_string(sensor_points_sensorTF_msg_ptr->header.seq);
  key_value_stdmap_["transform_probability"] = std::to_string(transform_probability);
  key_value_stdmap_["iteration_num"] = std::to_string(iteration_num);
  key_value_stdmap_["skipping_publish_num"] = std::to_string(skipping_publish_num);

  std::cout << "------------------------------------------------" << std::endl;
  std::cout << "align_time: " << align_time << "ms" << std::endl;
  std::cout << "exe_time: " << exe_time << "ms" << std::endl;
  std::cout << "trans_prob: " << transform_probability << std::endl;
  std::cout << "iter_num: " << iteration_num << std::endl;
  std::cout << "skipping_publish_num: " << skipping_publish_num << std::endl;
}

void NdtLocalizer::init_params()
{

  private_nh_.getParam("base_frame", base_frame_);
  ROS_INFO("base_frame_id: %s", base_frame_.c_str());

  ndt_ = new pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;

  double trans_epsilon = ndt_->getTransformationEpsilon();
  double step_size = ndt_->getStepSize();
  double resolution = ndt_->getResolution();
  int max_iterations = ndt_->getMaximumIterations();

  private_nh_.getParam("trans_epsilon", trans_epsilon);
  private_nh_.getParam("step_size", step_size);
  private_nh_.getParam("resolution", resolution);
  private_nh_.getParam("max_iterations", max_iterations);
  private_nh_.getParam("path_file", path_file);

  map_frame_ = "map";
  odom_frame_ = "odom";

  ndt_->setTransformationEpsilon(trans_epsilon);
  ndt_->setStepSize(step_size);
  ndt_->setResolution(resolution);
  ndt_->setMaximumIterations(max_iterations);

  ROS_INFO(
      "trans_epsilon: %lf, step_size: %lf, resolution: %lf, max_iterations: %d", trans_epsilon,
      step_size, resolution, max_iterations);

  private_nh_.getParam(
      "converged_param_transform_probability", converged_param_transform_probability_);
}

bool NdtLocalizer::get_transform(
    const std::string &target_frame, const std::string &source_frame,
    const geometry_msgs::TransformStamped::Ptr &transform_stamped_ptr, const ros::Time &time_stamp)
{
  if (target_frame == source_frame)
  {
    transform_stamped_ptr->header.stamp = time_stamp;
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return true;
  }

  try
  {
    *transform_stamped_ptr =
        tf2_buffer_.lookupTransform(target_frame, source_frame, time_stamp, ros::Duration(1.0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    ROS_ERROR("Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    transform_stamped_ptr->header.stamp = time_stamp;
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return false;
  }
  return true;
}

bool NdtLocalizer::get_transform(
    const std::string &target_frame, const std::string &source_frame,
    const geometry_msgs::TransformStamped::Ptr &transform_stamped_ptr)
{
  if (target_frame == source_frame)
  {
    transform_stamped_ptr->header.stamp = ros::Time::now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return true;
  }

  try
  {
    *transform_stamped_ptr =
        tf2_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    ROS_ERROR("Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    transform_stamped_ptr->header.stamp = ros::Time::now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return false;
  }
  return true;
}

void NdtLocalizer::publish_tf(
    const std::string &frame_id, const std::string &child_frame_id,
    const geometry_msgs::PoseStamped &pose_msg)
{
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.header.stamp = pose_msg.header.stamp;

  transform_stamped.transform.translation.x = pose_msg.pose.position.x;
  transform_stamped.transform.translation.y = pose_msg.pose.position.y;
  transform_stamped.transform.translation.z = pose_msg.pose.position.z;

  tf2::Quaternion tf_quaternion;
  tf2::fromMsg(pose_msg.pose.orientation, tf_quaternion);
  transform_stamped.transform.rotation.x = tf_quaternion.x();
  transform_stamped.transform.rotation.y = tf_quaternion.y();
  transform_stamped.transform.rotation.z = tf_quaternion.z();
  transform_stamped.transform.rotation.w = tf_quaternion.w();

  tf2_broadcaster_.sendTransform(transform_stamped);
}

bool NdtLocalizer::loadPath(std::string path_file)
  {
    std::cout << "Opening path at: " << path_file << std::endl;
    std::ifstream csv;
    csv.open(path_file);
    if (csv.eof() || !csv)
    {
      ROS_ERROR("The file at path %s doesn't exist", path_file.c_str());
      exit(1);
    }

    std::string line;
    std::getline(csv, line);
    std::string cell;

    geometry_msgs::PoseStamped odom;
    while (std::getline(csv, line))
    {
      std::stringstream lineStream(line);
      std::vector<std::string> str_odom;

      while (std::getline(lineStream, cell, ','))
      {
        str_odom.push_back(cell);
        //std::cout << cell << ",";
      }

      geometry_msgs::Point32 p;
      p.x = std::stof(str_odom[1]);
      p.y = std::stof(str_odom[2]);
      p.z = std::stof(str_odom[3]);

      poly.polygon.points.push_back(p);
    }

    poly.header.frame_id = "map";
    poly.header.seq = ros::Time::now().nsec;
    return poly.polygon.points.empty();
  }

  double NdtLocalizer::getNearestHeight(const geometry_msgs::Pose input_init)
  {
    double z = 0, dist = std::numeric_limits<double>::max();
    for (auto pose : poly.polygon.points)
    {
      double dx = pose.x - input_init.position.x;
      double dy = pose.y - input_init.position.y;
      double ddist = sqrt(dx * dx + dy * dy);

      if (ddist < dist)
      {
        z = pose.z;
        dist = ddist;
      }
    }
    return z;
  }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ndt_localizer");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  NdtLocalizer ndt_localizer(nh, private_nh);

  ros::spin();

  return 0;
}