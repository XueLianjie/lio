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
//#include <feature.hpp>
#include <fstream>
#include <iostream>
#include <queue>
#include <vector>

#include "imu.h"
#include "imu_state.h"
//#include "lio.h"
#include "lio/CameraMeasurement.h"
#include "lio/FeatureMeasurement.h"
#include "lio/TrackingInfo.h"
#include "nav_msgs/Odometry.h"
#include "param.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"

using namespace std;
using namespace Eigen;

#define MID_INTEGRATION

bool initialize_flag = false;
vector<sensor_msgs::Imu> imuBuffer;
const int CheckInitializeNum = 20;

// state variables
Eigen::Quaterniond q_wi;
Eigen::Vector3d p_0, v_0, ba_0, bg_0, acc_0, gyro_0;
Eigen::Quaterniond q_0;
Eigen::Vector3d gravity_w(0, 0, -9.81);
Eigen::Matrix<double, 15, 15> CovX = Eigen::Matrix<double, 15, 15>::Identity();
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

Eigen::Matrix3d toSkewMatrix(const Eigen::Vector3d &vec)
{
  Eigen::Matrix3d skew_matrix = Eigen::Matrix3d::Zero();
  skew_matrix << 0.0, -vec(2), vec(1), vec(2), 0.0, -vec(0), -vec(1), vec(0), 0;
  return skew_matrix;
}

bool staticInitialize()
{
  Eigen::Vector3d sum_acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d sum_gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d m_acc, m_gyro;

  for (auto imu_msg : imuBuffer)
  {
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
                              -gravity_norm); // gravity or acc in world fram ?

  q_0 = Eigen::Quaterniond::FromTwoVectors(gravity_imu, -gravity_w);
  ba_0 = Eigen::Vector3d::Zero();
  p_0 = Eigen::Vector3d::Zero();
  v_0 = Eigen::Vector3d::Zero();
  acc_0 = m_acc;
  gyro_0 = m_gyro;
  return true;
}

void imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
  double imu_time_s = msg->header.stamp.toSec();
  //ROS_INFO("imu time stamp: %f ", imu_time_s);
  // cout << "imu time stamp: %f " << imu_time_s << endl;
  //cout << "acc: \n " << msg->linear_acceleration << endl;
  //cout << "gyro: \n " << msg->angular_velocity << endl;

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

  if (!initialize_flag)
  {
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
    CovX = Eigen::Matrix<double, 15, 15>::Identity();
    CovX.block<3, 3>(0, 0) = 100. * Eigen::Matrix3d::Identity();
    CovX.block<3, 3>(3, 3) = 100. * Eigen::Matrix3d::Identity();
    CovX.block<3, 3>(6, 6) = 1.0 * Eigen::Matrix3d::Identity();
    CovX.block<3, 3>(9, 9) = 0.0001 * Eigen::Matrix3d::Identity();
    CovX.block<3, 3>(12, 12) = 0.0001 * Eigen::Matrix3d::Identity();

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
      0.5 * (gyro_0 + m_gyro) - bg_0; // calculate average gyro
  Eigen::Quaterniond dq_tmp;
  dq_tmp.x() =
      un_gyro.x() * dt / 2.0; // calculate theta / 2 = un_gyro * dt / 2
  dq_tmp.y() = un_gyro.y() * dt / 2.0;
  dq_tmp.z() = un_gyro.z() * dt / 2.0;
  dq_tmp.w() = 1.0;
  dq_tmp.normalize();
  q_0 = q_0 * dq_tmp;
  Eigen::Vector3d un_acc_1 = q_0 * (m_acc - ba_0) + gravity_w;
  Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
  p_0 = p_0 + dt * v_0 + 0.5 * un_acc * dt * dt;
  v_0 = v_0 + dt * un_acc; // this need to be calculated after p0
  // ba_0 = ba_0;
  // bg_0 = bg_0;
  acc_0 = m_acc;
  gyro_0 = m_gyro;

  // Covariance Matrix
  Eigen::Matrix3d R(q_0);
  Eigen::Matrix<double, 15, 15> Fx = Eigen::Matrix<double, 15, 15>::Identity();
  Fx.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
  Fx.block<3, 3>(3, 6) = -R * toSkewMatrix(un_acc - gravity_w) * dt;
  Fx.block<3, 3>(3, 9) = -R * dt;
  Eigen::Vector3d delta_theta = un_gyro * dt;
  if (delta_theta.norm() > 1e-12)
  {
    Fx.block<3, 3>(6, 6) =
        Eigen::AngleAxisd(delta_theta.norm(), delta_theta.normalized())
            .toRotationMatrix()
            .transpose();
  }
  else
  {
    Fx.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();
  }

  // Fx.block<3, 3>(6, 6) = Eigen::Matrix3d(un_gyro * dt).transpose();
  Fx.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

  Eigen::Matrix<double, 15, 12> Fi = Eigen::Matrix<double, 15, 12>::Zero();
  Fi.block<12, 12>(3, 0) = Eigen::Matrix<double, 12, 12>::Identity();

  Eigen::Matrix<double, 12, 12> Qi = Eigen::Matrix<double, 12, 12>::Zero();
  Qi.block<3, 3>(0, 0) =
      dt * dt * IMUState::acc_noise * Eigen::Matrix3d::Identity();
  Qi.block<3, 3>(3, 3) =
      dt * dt * IMUState::gyro_noise * Eigen::Matrix3d::Identity();
  Qi.block<3, 3>(6, 6) =
      dt * IMUState::acc_bias_noise * Eigen::Matrix3d::Identity();
  Qi.block<3, 3>(9, 9) =
      dt * IMUState::gyro_bias_noise * Eigen::Matrix3d::Identity();

  CovX = Fx * CovX * Fx.transpose() + Fi * Qi * Fi.transpose();

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
      q_0 * (m_acc) + gravity_w; // aw = Rwb * ( acc_body - acc_bias ) + gw
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

void gpsCallback(const sensor_msgs::NavSatFixConstPtr &gps_msg)
{
  ROS_INFO("gps timestamp %f,  %f, %f, %f", gps_msg->header.stamp.toSec(),
           gps_msg->latitude, gps_msg->longitude, gps_msg->altitude);

  Eigen::Vector3d gps(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude);
  Eigen::Matrix<double, 3, 15> Hx = Eigen::Matrix<double, 3, 15>::Zero();
  Hx.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  Eigen::Vector3d tig = Eigen::Vector3d::Zero();
  Hx.block<3, 3>(0, 6) = -Eigen::Matrix3d(q_0) * toSkewMatrix(tig);

  double residual[3] = {0.0, 0.0, 0.0};
  Eigen::Map<Eigen::Vector3d> residual_vec(residual);
  residual_vec = gps - p_0;
  Eigen::Map<const Eigen::Matrix3d> V(gps_msg->position_covariance.data());
  cout << "obser cov " << V << endl;
  const Eigen::MatrixXd K =
      CovX * Hx.transpose() * (Hx * CovX * Hx.transpose() + V).inverse();
  Eigen::VectorXd delta_x = K * residual_vec;
  p_0 += delta_x.block<3, 1>(0, 0);
  v_0 += delta_x.block<3, 1>(3, 0);
  ba_0 += delta_x.block<3, 1>(9, 0);
  bg_0 += delta_x.block<3, 1>(12, 0);

  if (delta_x.block<3, 1>(6, 0).norm() > 1e-12)
  {
    q_0 *= Eigen::Quaterniond(
        Eigen::AngleAxisd(delta_x.block<3, 1>(6, 0).norm(),
                          delta_x.block<3, 1>(6, 0).normalized())
            .toRotationMatrix());
  }

  //这种方法是错误的，原因是dtheta的更新量是弧度量，而下面的表示是
  // Eigen::Quaterniond delta_q;

  // delta_q.x() = delta_x(6);  // delta_x.block<3, 1>(6,0);
  // delta_q.y() = delta_x(7);
  // delta_q.z() = delta_x(8);
  // delta_x.w() = 1.0;
  // delta_q =  delta_q.normalized();  //  normaliz
  // q_0 = q_0 * delta_q;

  Eigen::MatrixXd I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * Hx;
  CovX = I_KH * CovX * I_KH.transpose() + K * V * K.transpose();
}
/*
void stateAugmentation(const double& time) {

  const Matrix3d& R_i_c = state_server.imu_state.R_imu_cam0;
  const Vector3d& t_c_i = state_server.imu_state.t_cam0_imu;

  // Add a new camera state to the state server.
  Matrix3d R_w_i = quaternionToRotation(
      state_server.imu_state.orientation);
  Matrix3d R_w_c = R_i_c * R_w_i;
  Vector3d t_c_w = state_server.imu_state.position +
    R_w_i.transpose()*t_c_i;

  state_server.cam_states[state_server.imu_state.id] =
    CAMState(state_server.imu_state.id);
  CAMState& cam_state = state_server.cam_states[
    state_server.imu_state.id];

  cam_state.time = time;
  cam_state.orientation = rotationToQuaternion(R_w_c);
  cam_state.position = t_c_w;

  cam_state.orientation_null = cam_state.orientation;
  cam_state.position_null = cam_state.position;

  // Update the covariance matrix of the state.
  // To simplify computation, the matrix J below is the nontrivial block
  // in Equation (16) in "A Multi-State Constraint Kalman Filter for Vision
  // -aided Inertial Navigation".
  Matrix<double, 6, 21> J = Matrix<double, 6, 21>::Zero();
  J.block<3, 3>(0, 0) = R_i_c;
  J.block<3, 3>(0, 15) = Matrix3d::Identity();
  J.block<3, 3>(3, 0) = skewSymmetric(R_w_i.transpose()*t_c_i);
  //J.block<3, 3>(3, 0) = -R_w_i.transpose()*skewSymmetric(t_c_i);
  J.block<3, 3>(3, 12) = Matrix3d::Identity();
  J.block<3, 3>(3, 18) = Matrix3d::Identity();

  // Resize the state covariance matrix.
  size_t old_rows = state_server.state_cov.rows();
  size_t old_cols = state_server.state_cov.cols();
  state_server.state_cov.conservativeResize(old_rows+6, old_cols+6);

  // Rename some matrix blocks for convenience.
  const Matrix<double, 21, 21>& P11 =
    state_server.state_cov.block<21, 21>(0, 0);
  const MatrixXd& P12 =
    state_server.state_cov.block(0, 21, 21, old_cols-21);

  // Fill in the augmented state covariance.
  state_server.state_cov.block(old_rows, 0, 6, old_cols) << J*P11, J*P12;
  state_server.state_cov.block(0, old_cols, old_rows, 6) =
    state_server.state_cov.block(old_rows, 0, 6, old_cols).transpose();
  state_server.state_cov.block<6, 6>(old_rows, old_cols) =
    J * P11 * J.transpose();

  // Fix the covariance to be symmetric
  MatrixXd state_cov_fixed = (state_server.state_cov +
      state_server.state_cov.transpose()) / 2.0;
  state_server.state_cov = state_cov_fixed;

  return;
}
*/
void featureCallback(const lio::CameraMeasurementConstPtr &msg)
{
  ROS_INFO("received feature msg: %f ", msg->header.stamp.toSec());
  if (!initialize_flag)
  {
    return;
  }

  ros::Time start_time = ros::Time::now();
  // Augment the state vector.
  start_time = ros::Time::now();
  //stateAugmentation(msg->header.stamp.toSec());
  double state_augmentation_time = (ros::Time::now() - start_time).toSec();

  // Add new observations for existing features or new
  // features in the map server.
  // 将特征点放到对应的map server里面，如果已经有了则增加观测项
  // start_time = ros::Time::now();
  // addFeatureObservations(msg);
  // double add_observations_time = (
  // ros::Time::now()-start_time).toSec();

  // // Perform measurement update if necessary.
  // start_time = ros::Time::now();
  // removeLostFeatures();
  // double remove_lost_features_time = (
  // ros::Time::now()-start_time).toSec();
}

main(int argc, char **argv)
{
  ros::init(argc, argv, "lio_node");
  ofile.open("received_point.txt");

  ros::NodeHandle nh("~");

  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1000);
  path_pub = nh.advertise<nav_msgs::Path>("path", 1000);

  ros::Subscriber imu_sub = nh.subscribe("/imu_sim", 1000, imuCallback);
  ros::Subscriber gps_sub = nh.subscribe("/gps", 100, gpsCallback);
  ros::Subscriber feature_sub = nh.subscribe("/features", 100, featureCallback);

  // ros::Rate rate(10);

  ros::spin();

  return 0;
}