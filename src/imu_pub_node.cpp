#include <Eigen/Core>
#include <fstream>
#include <iostream>

#include "imu.h"
#include "nav_msgs/Path.h"
#include "param.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "utilities.h"
#include "lio/CameraMeasurement.h"
#include "lio/FeatureMeasurement.h"
#include "lio/TrackingInfo.h"
#include "feature_generator.h"
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include "visualizer.h"

using namespace std;
using namespace lio;

main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_pub_node");

  ros::NodeHandle nh("~");

  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu_sim", 1000);
  ros::Publisher gt_path_pub = nh.advertise<nav_msgs::Path>("gt", 1000);
  ros::Publisher gps_pub = nh.advertise<sensor_msgs::NavSatFix>("/gps", 1000);
  ros::Publisher gps_path_pub = nh.advertise<nav_msgs::Path>("gps_path", 1000);
  ros::Publisher gt_pose_pub = nh.advertise<CameraMeasurement>("/features", 100);
  //ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  Visualizer visualizer(nh);

  ros::Rate loop_rate(200);
  Param params;
  IMU imuGen(params);

  string model_path;
  nh.getParam("model_path", model_path);
  FeatureGenerator feature_generator(model_path);
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> features;

  sensor_msgs::Imu imu_msg;
  nav_msgs::Path path_gt_msg;
  nav_msgs::Path path_gps_msg;
  sensor_msgs::NavSatFix gps;
  gps.position_covariance = {0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0001};
  int pub_feature_step = 0;

  //当按Ctrl+C时，ros::ok()会返回0，退出该while循环，。
  float t = params.t_start;

  double begin = ros::Time::now().toSec();
  std::ofstream save_points;
  std::string str = "published_points.txt";
  save_points.open(str.c_str());

  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> points = feature_generator.getPoints();
  std::vector<std::pair<Eigen::Vector4d, Eigen::Vector4d>, Eigen::aligned_allocator<std::pair<Eigen::Vector4d, Eigen::Vector4d>>>
      lines = feature_generator.getLines();
  visualizer.AddPoints(points);
  visualizer.AddLines(lines);
  // save_points.open(str.c_str());

  while (ros::ok())
  {
    ros::Time time_now(begin + t);
    MotionData data = imuGen.MotionModel(t);
    Eigen::Quaterniond q(data.Rwb);

    // publish marker
    visualizer.PublishMarker(time_now);
    // publish cam frame & body frame
    Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d Twb = Eigen::Matrix4d::Identity();
    Twc.block<3, 3>(0, 0) = data.Rwb * params.R_bc;
    Twc.block<3, 1>(0, 3) = data.twb + data.Rwb * params.t_bc;
    Twb.block<3, 3>(0, 0) = data.Rwb;
    Twb.block<3, 1>(0, 3) = data.twb;
    visualizer.PublishBodyFrame(time_now, Twb);

    // publish ground truth msg
    geometry_msgs::Pose pose;
    pose.position.x = data.twb(0);
    pose.position.y = data.twb(1);
    pose.position.z = data.twb(2);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = time_now;
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose = pose;
    path_gt_msg.header.stamp = time_now;
    path_gt_msg.header.frame_id = "world";
    path_gt_msg.poses.push_back(pose_stamped);
    gt_path_pub.publish(path_gt_msg);

    //imuGen.addIMUnoise(data);

    // publish noisy imu data
    imu_msg.header.stamp = time_now;
    imu_msg.header.frame_id = "world";
    imu_msg.orientation.x = t; // q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();
    imu_msg.orientation.w = q.w();
    //线加速度
    imu_msg.linear_acceleration.x = data.imu_acc(0);
    imu_msg.linear_acceleration.y = data.imu_acc(1);
    imu_msg.linear_acceleration.z = data.imu_acc(2);
    //角速度
    imu_msg.angular_velocity.x = data.imu_gyro(0);
    imu_msg.angular_velocity.y = data.imu_gyro(1);
    imu_msg.angular_velocity.z = data.imu_gyro(2);
    imu_pub.publish(imu_msg);
    ROS_INFO("pub imu_msg time : %f", imu_msg.header.stamp.toSec());

    pub_feature_step += 20;
    if (pub_feature_step == params.imu_frequency)
    {
      // publish cam features
      features = feature_generator.featureObservation(Twc);
      visualizer.PublishFeaturePoints(time_now, features);
      visualizer.PublishCamFrame(time_now, Twc);
      visualizer.PublishFeatureLines(time_now, Twc);

      // publish gps data
      gps.header.stamp = time_now;
      gps.header.frame_id = "world";
      gps.latitude = data.twb(0);
      gps.longitude = data.twb(1);
      gps.altitude = data.twb(2);

      pose.position.x = data.twb(0);
      pose.position.y = data.twb(1);
      pose.position.z = data.twb(2);
      pose_stamped.pose = pose;
      path_gps_msg.header.stamp = time_now;
      path_gps_msg.header.frame_id = "world";
      path_gps_msg.poses.push_back(pose_stamped);
      gps_pub.publish(gps);
      gps_path_pub.publish(path_gps_msg);

      pub_feature_step = 0;
    }

    save_points << t << " " << q.w() << " " << q.x() << " " << q.y() << " "
                << q.z() << " " << data.twb(0) << " " << data.twb(1) << " "
                << data.twb(2) << " " << data.imu_gyro(0) << " "
                << data.imu_gyro(1) << " " << data.imu_gyro(2) << " "
                << data.imu_acc(0) << " " << data.imu_acc(1) << " "
                << data.imu_acc(2) << " " << 0 << " " << 0 << " " << 0 << " "
                << 0 << " " << 0 << " " << 0 << " " << std::endl;
    //读取和更新ROS topics，如果没有spinonce()或spin()，节点不会发布消息。
    ros::spinOnce();

    t += 1.0 / params.imu_frequency;
    loop_rate.sleep();
  }
  return 0;
  ros::spin();

  return 0;
}