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

using namespace std;

main(int argc, char** argv) {
  ros::init(argc, argv, "imu_pub_node");

  ros::NodeHandle nh("~");

  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu_sim", 1000);
  ros::Publisher gt_pub = nh.advertise<nav_msgs::Path>("gt", 1000);
  ros::Publisher gps_pub = nh.advertise<sensor_msgs::NavSatFix>("/gps", 1000);
  ros::Publisher gps_path_pub = nh.advertise<nav_msgs::Path>("gps_path", 1000);
  ros::Publisher feature_pub = nh.advertise<lio::CameraMeasurement>("/features", 100);
  // ros::Subscriber imu_sub = nh.subscribe("/imu0", 1000, imuCallback);

  ros::Rate loop_rate(200);
  int number_count = 0;
  Param params;
  IMU imuGen(params);
  FeatureGenerator feature_generator("/home/rankin/vio_ws/src/lio/house_model/house.txt");
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > features;

  sensor_msgs::Imu msg;
  nav_msgs::Path path_gt_msg;
  nav_msgs::Path path_gps_msg;
  sensor_msgs::NavSatFix gps;
  gps.position_covariance = {0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0001};
  int gps_count = 0;

  //当按Ctrl+C时，ros::ok()会返回0，退出该while循环，。
  float t = params.t_start;

  double begin = ros::Time::now().toSec();
  std::ofstream save_points;
  std::string str = "published_points.txt";
  save_points.open(str.c_str());

  // save_points.open(str.c_str());
  while (ros::ok()) {
    ros::Time time_now(begin + t);

    msg.header.stamp = time_now;
    msg.header.frame_id = "world";
    gps.header.stamp = time_now;
    gps.header.frame_id = "world";

    MotionData data = imuGen.MotionModel(t);
    Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
    Twc.block<3, 3>(0, 0) = data.Rwb * params.R_bc;
    Twc.block<3, 1>(0, 3) = data.twb + data.Rwb * params.t_bc;
    
    Eigen::Quaterniond q(data.Rwb);
    geometry_msgs::Pose pose;
    pose.position.x = data.twb(0);
    pose.position.y = data.twb(1);
    pose.position.z = data.twb(2);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = msg.header;
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose = pose;
    path_gt_msg.header.stamp = time_now;
    path_gt_msg.header.frame_id = "world";
    path_gt_msg.poses.push_back(pose_stamped);

    imuGen.addIMUnoise(data);

    // Eigen::Quaterniond q(data.Rwb);
    //四元数位姿
    msg.orientation.x = t;  // q.x();
    msg.orientation.y = q.y();
    msg.orientation.z = q.z();
    msg.orientation.w = q.w();
    //线加速度
    msg.linear_acceleration.x = data.imu_acc(0);
    msg.linear_acceleration.y = data.imu_acc(1);
    msg.linear_acceleration.z = data.imu_acc(2);
    //角速度
    msg.angular_velocity.x = data.imu_gyro(0);
    msg.angular_velocity.y = data.imu_gyro(1);
    msg.angular_velocity.z = data.imu_gyro(2);

    ROS_INFO("pub msg time : %f", msg.header.stamp.toSec());
    save_points << t << " " << q.w() << " " << q.x() << " " << q.y() << " "
                << q.z() << " " << data.twb(0) << " " << data.twb(1) << " "
                << data.twb(2) << " " << data.imu_gyro(0) << " "
                << data.imu_gyro(1) << " " << data.imu_gyro(2) << " "
                << data.imu_acc(0) << " " << data.imu_acc(1) << " "
                << data.imu_acc(2) << " " << 0 << " " << 0 << " " << 0 << " "
                << 0 << " " << 0 << " " << 0 << " " << std::endl;

    imu_pub.publish(msg);
    gt_pub.publish(path_gt_msg);

    gps_count+= 10;
    if (gps_count == params.imu_frequency) {
      features = feature_generator.featureObservation(Twc);
      lio::CameraMeasurementPtr feature_msg_ptr(new lio::CameraMeasurement);
      feature_msg_ptr->header.stamp = time_now;
      //std::cout << t << " Twc " << Twc << std::endl;
      for (int i = 0; i < features.size(); ++i) {
        feature_msg_ptr->features.push_back(lio::FeatureMeasurement());
        feature_msg_ptr->features[i].id = i;
        feature_msg_ptr->features[i].u0 = features[i](0);
        feature_msg_ptr->features[i].v0 = features[i](1);
        feature_msg_ptr->features[i].u1 = features[i](0);
        feature_msg_ptr->features[i].v1 = features[i](1);
        //std::cout << "features " << features[i].transpose() << std::endl;
      }
      feature_pub.publish(feature_msg_ptr);
      // gps data
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
      gps_count = 0;
    }

    //读取和更新ROS topics，如果没有spinonce()或spin()，节点不会发布消息。
    ros::spinOnce();

    loop_rate.sleep();
    t += 1.0 / params.imu_frequency;
  }
  return 0;
  ros::spin();

  return 0;
}