  #include<ndt.h>
  
  void debugPoseMarker(geometry_msgs::Pose p)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = p.position.x;
    marker.pose.position.y = p.position.y;
    marker.pose.position.z = p.position.z;
    marker.pose.orientation.x = p.orientation.x;
    marker.pose.orientation.y = p.orientation.y;
    marker.pose.orientation.z = p.orientation.z;
    marker.pose.orientation.w = p.orientation.w;
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    debug_init_pose_pub.publish(marker);
  }