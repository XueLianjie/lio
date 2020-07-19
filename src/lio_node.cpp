#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Path.h>
#include <tf_conversions/tf_eigen.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/QR>
#include <Eigen/SPQRSupport>
#include <Eigen/SVD>
#include <Eigen/SparseCore>
#include <fstream>
#include <iostream>
#include <queue>
#include <vector>

#include "imu.h"
#include "imu_state.h"
#include "nav_msgs/Odometry.h"
#include "param.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"

using namespace std;

#define MID_INTEGRATION

bool initialize_flag = false;
vector<sensor_msgs::Imu> imuBuffer;
const int CheckInitializeNum = 20;

// state variables
Eigen::Quaterniond q_wi;
Eigen::Vector3d p_0, v_0, ba_0, bg_0, acc_0, gyro_0;
Eigen::Quaterniond q_0;
Eigen::Vector3d gravity_w(0, 0, -9.81);
// IMUState imu_state;

StateIDType IMUState::next_id = 0;
double IMUState::gyro_noise = 0.001;
double IMUState::acc_noise = 0.01;
double IMUState::gyro_bias_noise = 0.001;
double IMUState::acc_bias_noise = 0.01;
Eigen::Vector3d IMUState::gravity =
    Eigen::Vector3d(0, 0, -GRAVITY_ACCELERATION);
Eigen::Isometry3d IMUState::T_imu_body = Eigen::Isometry3d::Identity();

ros::Publisher odom_pub;
ros::Publisher path_pub;
nav_msgs::Path path;

ofstream ofile;

bool staticInitialize() {
  Eigen::Vector3d sum_acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d sum_gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d m_acc, m_gyro;

  for (auto imu_msg : imuBuffer) {
    tf::vectorMsgToEigen(imu_msg.linear_acceleration, m_acc);
    tf::vectorMsgToEigen(imu_msg.angular_velocity, m_gyro);
    sum_acc += m_acc;
    sum_gyro += m_gyro;
  }
  bg_0 = sum_gyro / imuBuffer.size();
  cout << "bg_0 " << bg_0 << endl;
  Eigen::Vector3d gravity_imu = sum_acc / imuBuffer.size();

  double gravity_norm = gravity_imu.norm();
  gravity_w = Eigen::Vector3d(0.0, 0.0,
                              -gravity_norm);  // gravity or acc in world fram ?

  q_0 = Eigen::Quaterniond::FromTwoVectors(gravity_imu, -gravity_w);
  ba_0 = Eigen::Vector3d::Zero();
  p_0 = Eigen::Vector3d::Zero();
  v_0 = Eigen::Vector3d::Zero();
  acc_0 = m_acc;
  gyro_0 = m_gyro;
  return true;
}

void imuCallback(const sensor_msgs::ImuConstPtr& msg) {
  double imu_time_s = msg->header.stamp.toSec();
  ROS_INFO("imu time stamp: %f ", imu_time_s);
  // cout << "imu time stamp: %f " << imu_time_s << endl;
  cout << "acc: \n " << msg->linear_acceleration << endl;
  cout << "gyro: \n " << msg->angular_velocity << endl;

  // if (!initialize_flag) {
  //   imuBuffer.push_back(*msg);
  //   if (imuBuffer.size() == CheckInitializeNum) {
  //     if (!staticInitialize()) {
  //       // imuBuffer.pop_front();
  //       imuBuffer.erase(imuBuffer.begin());
  //       return;
  //     } else {
  //       initialize_flag = true;
  //     }
  //   }
  // }

  ofile << imu_time_s << " " << msg->orientation.w << " " << msg->orientation.x
        << " " << msg->orientation.y << " " << msg->orientation.z << " " << 0
        << " " << 0 << " " << 0 << " " << msg->angular_velocity.x << " "
        << msg->angular_velocity.y << " " << msg->angular_velocity.z << " "
        << msg->linear_acceleration.x << " " << msg->linear_acceleration.y
        << " " << msg->linear_acceleration.z << " " << 0 << " " << 0 << " " << 0
        << " " << 0 << " " << 0 << " " << 0 << " " << std::endl;

  if (!initialize_flag) {
    Param params;
    IMU imuGen(params);
    MotionData data = imuGen.MotionModel(msg->orientation.x);
    q_0 = Eigen::Quaterniond(data.Rwb);
    // q_0.x() = q.x();
    // q_0.y() = msg->orientation.y;
    // q_0.z() = msg->orientation.z;
    // q_0.w() = msg->orientation.w;
    bg_0 = Eigen::Vector3d::Zero();

    ba_0 = Eigen::Vector3d::Zero();
    p_0 = data.twb;
    v_0 = data.imu_velocity;
    tf::vectorMsgToEigen(msg->linear_acceleration, acc_0);
    tf::vectorMsgToEigen(msg->angular_velocity, gyro_0);

    // q_0.x() = msg->orientation.x;
    // q_0.y() = msg->orientation.y;
    // q_0.z() = msg->orientation.z;
    // q_0.w() = msg->orientation.w;
    // bg_0 = Eigen::Vector3d::Zero();

    // ba_0 = Eigen::Vector3d::Zero();
    // p_0 = Eigen::Vector3d(20, 5, 5);
    // v_0 = Eigen::Vector3d(0, 6.28319, 3.14159);
    // tf::vectorMsgToEigen(msg->linear_acceleration, acc_0);
    // tf::vectorMsgToEigen(msg->angular_velocity, gyro_0);

    // acc_0 = Eigen::Vector3d::Zero();
    // gyro_0 = Eigen::Vector3d::Zero();
    initialize_flag = true;
#ifdef MID_INTEGRATION
    return;
#endif
  }
  double dt = 1.0 / 200.0;
  Eigen::Vector3d m_acc, m_gyro;
  tf::vectorMsgToEigen(msg->linear_acceleration, m_acc);
  tf::vectorMsgToEigen(msg->angular_velocity, m_gyro);

#ifdef MID_INTEGRATION

  Eigen::Vector3d un_acc_0 = q_0 * (acc_0 - ba_0) + gravity_w;
  Eigen::Vector3d un_gyro =
      0.5 * (gyro_0 + m_gyro) - bg_0;  // calculate average gyro
  Eigen::Quaterniond dq_tmp;
  dq_tmp.x() =
      un_gyro.x() * dt / 2.0;  // calculate theta / 2 = un_gyro * dt / 2
  dq_tmp.y() = un_gyro.y() * dt / 2.0;
  dq_tmp.z() = un_gyro.z() * dt / 2.0;
  dq_tmp.w() = 1.0;
  dq_tmp.normalize();
  q_0 = q_0 * dq_tmp;
  Eigen::Vector3d un_acc_1 = q_0 * (m_acc - ba_0) + gravity_w;
  Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
  p_0 = p_0 + dt * v_0 + 0.5 * un_acc * dt * dt;
  v_0 = v_0 + dt * un_acc;  // this need to be calculated after p0
  // ba_0 = ba_0;
  // bg_0 = bg_0;
  acc_0 = m_acc;
  gyro_0 = m_gyro;

#else

  // delta_q = [1 , 1/2 * thetax , 1/2 * theta_y, 1/2 * theta_z]
  Eigen::Quaterniond dq;
  Eigen::Vector3d dtheta_half = m_gyro * dt / 2.0;
  dq.w() = 1;
  dq.x() = dtheta_half.x();
  dq.y() = dtheta_half.y();
  dq.z() = dtheta_half.z();
  dq.normalize();
  //　imu 动力学模型　参考svo预积分论文
  Eigen::Vector3d acc_w =
      q_0 * (m_acc) + gravity_w;  // aw = Rwb * ( acc_body - acc_bias ) + gw
  q_0 = q_0 * dq;
  p_0 = p_0 + v_0 * dt + 0.5 * dt * dt * acc_w;
  v_0 = v_0 + acc_w * dt;

#endif
  nav_msgs::Odometry odometry;
  odometry.header = msg->header;
  odometry.header.frame_id = "world";
  odometry.pose.pose.position.x = p_0.x();
  odometry.pose.pose.position.y = p_0.y();
  odometry.pose.pose.position.z = p_0.z();
  odometry.pose.pose.orientation.x = q_0.x();
  odometry.pose.pose.orientation.y = q_0.y();
  odometry.pose.pose.orientation.z = q_0.z();
  odometry.pose.pose.orientation.w = q_0.w();
  odometry.twist.twist.linear.x = v_0.x();
  odometry.twist.twist.linear.y = v_0.y();
  odometry.twist.twist.linear.z = v_0.z();
  odom_pub.publish(odometry);

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = msg->header;
  pose_stamped.header.frame_id = "world";
  pose_stamped.pose = odometry.pose.pose;

  path.header = msg->header;
  path.header.frame_id = "world";
  path.poses.push_back(pose_stamped);
  path_pub.publish(path);

  return;
}

void gpsCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg) {
  ROS_INFO("gps data received %f, %f, %f", gps_msg->latitude,
           gps_msg->longitude, gps_msg->altitude);
}

main(int argc, char** argv) {
  ros::init(argc, argv, "lio_node");
  ofile.open("received_point.txt");

  ros::NodeHandle nh("~");

  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1000);
  path_pub = nh.advertise<nav_msgs::Path>("path", 1000);

  ros::Subscriber imu_sub = nh.subscribe("/imu_sim", 1000, imuCallback);
  ros::Subscriber gps_sub = nh.subscribe("/gps", 100, gpsCallback);

  // ros::Rate rate(10);

  ros::spin();

  return 0;
}