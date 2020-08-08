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
#include <feature.hpp>
#include <fstream>
#include <iostream>
#include <queue>
#include <vector>

#include "imu.h"
#include "imu_state.h"
#include "lio.h"
#include "lio/CameraMeasurement.h"
#include "lio/FeatureMeasurement.h"
#include "lio/TrackingInfo.h"
#include "nav_msgs/Odometry.h"
#include "param.h"
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"

using namespace std;
using namespace Eigen;

#define MID_INTEGRATION

bool initialize_flag = false;
const int CheckInitializeNum = 20;

// state variables
Eigen::Quaterniond q_wi;
Eigen::Vector3d& position = msckf_vio::state_server.imu_state.position;
Eigen::Vector3d& velocity = msckf_vio::state_server.imu_state.velocity;
Eigen::Vector3d& acc_bias = msckf_vio::state_server.imu_state.acc_bias;
Eigen::Vector3d& gyro_bias = msckf_vio::state_server.imu_state.gyro_bias;
Eigen::Quaterniond& orientation = msckf_vio::state_server.imu_state.orientation;
Eigen::Vector3d acc_0, gyro_0;

Eigen::Vector3d gravity_w(0, 0, -9.81);
Eigen::Matrix<double, 15, 15> CovX = Eigen::Matrix<double, 15, 15>::Identity();
// msckf_vio::IMUState imu_state;

msckf_vio::StateIDType msckf_vio::IMUState::next_id = 0;
double msckf_vio::IMUState::gyro_noise = 0.001;
double msckf_vio::IMUState::acc_noise = 0.01;
double msckf_vio::IMUState::gyro_bias_noise = 0.001;
double msckf_vio::IMUState::acc_bias_noise = 0.01;
Eigen::Vector3d msckf_vio::IMUState::gravity =
    Eigen::Vector3d(0, 0, -GRAVITY_ACCELERATION);
Eigen::Isometry3d msckf_vio::IMUState::T_imu_body =
    Eigen::Isometry3d::Identity();

ros::Publisher odom_pub;
ros::Publisher path_pub;
nav_msgs::Path path;

ofstream ofile;

Eigen::Matrix3d toSkewMatrix(const Eigen::Vector3d& vec) {
  Eigen::Matrix3d skew_matrix = Eigen::Matrix3d::Zero();
  skew_matrix << 0.0, -vec(2), vec(1), vec(2), 0.0, -vec(0), -vec(1), vec(0), 0;
  return skew_matrix;
}

void publish(const sensor_msgs::NavSatFixConstPtr& msg) {
  nav_msgs::Odometry odometry;
  odometry.header = msg->header;
  odometry.header.frame_id = "world";
  odometry.pose.pose.position.x = position.x();
  odometry.pose.pose.position.y = position.y();
  odometry.pose.pose.position.z = position.z();
  odometry.pose.pose.orientation.x = orientation.x();
  odometry.pose.pose.orientation.y = orientation.y();
  odometry.pose.pose.orientation.z = orientation.z();
  odometry.pose.pose.orientation.w = orientation.w();
  odometry.twist.twist.linear.x = velocity.x();
  odometry.twist.twist.linear.y = velocity.y();
  odometry.twist.twist.linear.z = velocity.z();
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

bool staticInitialize() {
  Eigen::Vector3d sum_acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d sum_gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d m_acc, m_gyro;

  for (auto imu_msg : msckf_vio::imu_msg_buffer) {
    tf::vectorMsgToEigen(imu_msg.linear_acceleration, m_acc);
    tf::vectorMsgToEigen(imu_msg.angular_velocity, m_gyro);
    sum_acc += m_acc;
    sum_gyro += m_gyro;
  }
  gyro_bias = sum_gyro / msckf_vio::imu_msg_buffer.size();
  cout << "gyro_bias " << gyro_bias << endl;
  Eigen::Vector3d gravity_imu = sum_acc / msckf_vio::imu_msg_buffer.size();

  double gravity_norm = gravity_imu.norm();
  gravity_w = Eigen::Vector3d(0.0, 0.0,
                              -gravity_norm);  // gravity or acc in world fram ?

  orientation = Eigen::Quaterniond::FromTwoVectors(gravity_imu, -gravity_w);
  acc_bias = Eigen::Vector3d::Zero();
  position = Eigen::Vector3d::Zero();
  velocity = Eigen::Vector3d::Zero();
  acc_0 = m_acc;
  gyro_0 = m_gyro;
  return true;
}

void imuCallback(const sensor_msgs::ImuConstPtr& msg) {
  double imu_time_s = msg->header.stamp.toSec();
  ROS_INFO("imu time stamp: %f ", imu_time_s);
  // cout << "imu time stamp: %f " << imu_time_s << endl;
  // cout << "acc: \n " << msg->linear_acceleration << endl;
  // cout << "gyro: \n " << msg->angular_velocity << endl;

  // if (!initialize_flag) {
  //   msckf_vio::imu_msg_buffer.push_back(*msg);
  //   if (msckf_vio::imu_msg_buffer.size() == CheckInitializeNum) {
  //     if (!staticInitialize()) {
  //       // msckf_vio::imu_msg_buffer.pop_front();
  //       msckf_vio::imu_msg_buffer.erase(msckf_vio::imu_msg_buffer.begin());
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
    orientation = Eigen::Quaterniond(data.Rwb);
    // orientation.x() = q.x();
    // orientation.y() = msg->orientation.y;
    // orientation.z() = msg->orientation.z;
    // orientation.w() = msg->orientation.w;
    gyro_bias = Eigen::Vector3d::Zero();

    acc_bias = Eigen::Vector3d::Zero();
    position = data.twb;
    velocity = data.imu_velocity;
    tf::vectorMsgToEigen(msg->linear_acceleration, acc_0);
    tf::vectorMsgToEigen(msg->angular_velocity, gyro_0);

    // orientation.x() = msg->orientation.x;
    // orientation.y() = msg->orientation.y;
    // orientation.z() = msg->orientation.z;
    // orientation.w() = msg->orientation.w;
    // gyro_bias = Eigen::Vector3d::Zero();

    // acc_bias = Eigen::Vector3d::Zero();
    // position = Eigen::Vector3d(20, 5, 5);
    // velocity = Eigen::Vector3d(0, 6.28319, 3.14159);
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
    return;
  }

  // // delta_q = [1 , 1/2 * thetax , 1/2 * theta_y, 1/2 * theta_z]
  // Eigen::Quaterniond dq;
  // Eigen::Vector3d dtheta_half = m_gyro * dt / 2.0;
  // dq.w() = 1;
  // dq.x() = dtheta_half.x();
  // dq.y() = dtheta_half.y();
  // dq.z() = dtheta_half.z();
  // dq.normalize();
  // //　imu 动力学模型　参考svo预积分论文
  // Eigen::Vector3d acc_w = orientation * (m_acc) +
  //                         gravity_w;  // aw = Rwb * ( acc_body - acc_bias ) +
  //                         gw
  // orientation = orientation * dq;
  // position = position + velocity * dt + 0.5 * dt * dt * acc_w;
  // velocity = velocity + acc_w * dt;

  return;
}

void predictNewState(const double& dt, const Vector3d& m_gyro,
                     const Vector3d& m_acc) {
  Eigen::Vector3d un_acc_0 = orientation * (acc_0 - acc_bias) + gravity_w;
  Eigen::Vector3d un_gyro = 0.5 * (gyro_0 + m_gyro) - gyro_bias;
  Eigen::Quaterniond dq_tmp;
  dq_tmp.x() =
      un_gyro.x() * dt / 2.0;  // calculate theta / 2 = un_gyro * dt / 2
  dq_tmp.y() = un_gyro.y() * dt / 2.0;
  dq_tmp.z() = un_gyro.z() * dt / 2.0;
  dq_tmp.w() = 1.0;
  dq_tmp.normalize();
  orientation = orientation * dq_tmp;
  Eigen::Vector3d un_acc_1 = orientation * (m_acc - acc_bias) + gravity_w;
  Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
  position = position + dt * velocity + 0.5 * un_acc * dt * dt;
  velocity = velocity + dt * un_acc;  // this need to be calculated after p0
  // acc_bias = acc_bias;
  // gyro_bias = gyro_bias;
  acc_0 = m_acc;
  gyro_0 = m_gyro;
  return;
}

void processModel(const double& time, const Vector3d& m_gyro,
                  const Vector3d& m_acc) {
  // double dt = 1.0 / 200.0;
  // Eigen::Vector3d m_acc, m_gyro;
  // tf::vectorMsgToEigen(msg->linear_acceleration, m_acc);
  // tf::vectorMsgToEigen(msg->angular_velocity, m_gyro);
  double dt = 1.0 / 200.0;//time - msckf_vio::state_server.imu_state.time;

  predictNewState(dt, m_gyro, m_acc);
  // Covariance Matrix
  Eigen::Matrix3d R(orientation);
  Eigen::Matrix<double, 15, 15> Fx = Eigen::Matrix<double, 15, 15>::Identity();
  Fx.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
  Fx.block<3, 3>(3, 6) = -R * toSkewMatrix(m_acc - gravity_w) * dt;
  Fx.block<3, 3>(3, 9) = -R * dt;
  Eigen::Vector3d delta_theta = m_gyro * dt;
  if (delta_theta.norm() > 1e-12) {
    Fx.block<3, 3>(6, 6) =
        Eigen::AngleAxisd(delta_theta.norm(), delta_theta.normalized())
            .toRotationMatrix()
            .transpose();
  } else {
    Fx.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();
  }

  // Fx.block<3, 3>(6, 6) = Eigen::Matrix3d(un_gyro * dt).transpose();
  Fx.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

  Eigen::Matrix<double, 15, 12> Fi = Eigen::Matrix<double, 15, 12>::Zero();
  Fi.block<12, 12>(3, 0) = Eigen::Matrix<double, 12, 12>::Identity();

  Eigen::Matrix<double, 12, 12> Qi = Eigen::Matrix<double, 12, 12>::Zero();
  Qi.block<3, 3>(0, 0) =
      dt * dt * msckf_vio::IMUState::acc_noise * Eigen::Matrix3d::Identity();
  Qi.block<3, 3>(3, 3) =
      dt * dt * msckf_vio::IMUState::gyro_noise * Eigen::Matrix3d::Identity();
  Qi.block<3, 3>(6, 6) =
      dt * msckf_vio::IMUState::acc_bias_noise * Eigen::Matrix3d::Identity();
  Qi.block<3, 3>(9, 9) =
      dt * msckf_vio::IMUState::gyro_bias_noise * Eigen::Matrix3d::Identity();

  CovX = Fx * CovX * Fx.transpose() + Fi * Qi * Fi.transpose();

  msckf_vio::state_server.imu_state.time = time;
  return;
}

void batchImuProcessing(const double& time_bound) {
  // Counter how many IMU msgs in the buffer are used.
  int used_imu_msg_cntr = 0;

  for (const auto& imu_msg : msckf_vio::imu_msg_buffer) {
    double imu_time = imu_msg.header.stamp.toSec();
    if (imu_time < msckf_vio::state_server.imu_state.time) {
      ++used_imu_msg_cntr;
      continue;
    }
    if (imu_time > time_bound) break;
    // Convert the msgs.
    Vector3d m_gyro, m_acc;
    tf::vectorMsgToEigen(imu_msg.angular_velocity, m_gyro);
    tf::vectorMsgToEigen(imu_msg.linear_acceleration, m_acc);

    // Execute process model.
    processModel(imu_time, m_gyro, m_acc);
    ++used_imu_msg_cntr;
  }
  // Set the state ID for the new IMU state.
  msckf_vio::state_server.imu_state.id = msckf_vio::IMUState::next_id++;

  // Remove all used IMU msgs.
  msckf_vio::imu_msg_buffer.erase(
      msckf_vio::imu_msg_buffer.begin(),
      msckf_vio::imu_msg_buffer.begin() + used_imu_msg_cntr);

  return;
}

void gpsCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg) {
  ROS_INFO("gps timestamp %f,  %f, %f, %f", gps_msg->header.stamp.toSec(),
           gps_msg->latitude, gps_msg->longitude, gps_msg->altitude);
  if (!initialize_flag) {
    return;
  }
  batchImuProcessing(gps_msg->header.stamp.toSec());

  Eigen::Vector3d gps(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude);
  Eigen::Matrix<double, 3, 15> Hx = Eigen::Matrix<double, 3, 15>::Zero();
  Hx.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  Eigen::Vector3d tig = Eigen::Vector3d::Zero();
  Hx.block<3, 3>(0, 6) = -Eigen::Matrix3d(orientation) * toSkewMatrix(tig);

  double residual[3] = {0.0, 0.0, 0.0};
  Eigen::Map<Eigen::Vector3d> residual_vec(residual);
  residual_vec = gps - position;
  Eigen::Map<const Eigen::Matrix3d> V(gps_msg->position_covariance.data());
  cout << "obser cov " << V << endl;
  const Eigen::MatrixXd K =
      CovX * Hx.transpose() * (Hx * CovX * Hx.transpose() + V).inverse();
  Eigen::VectorXd delta_x = K * residual_vec;
  position += delta_x.block<3, 1>(0, 0);
  velocity += delta_x.block<3, 1>(3, 0);
  acc_bias += delta_x.block<3, 1>(9, 0);
  gyro_bias += delta_x.block<3, 1>(12, 0);

  if (delta_x.block<3, 1>(6, 0).norm() > 1e-12) {
    orientation *= Eigen::Quaterniond(
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
  // orientation = orientation * delta_q;

  Eigen::MatrixXd I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * Hx;
  CovX = I_KH * CovX * I_KH.transpose() + K * V * K.transpose();

  publish(gps_msg);
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
void featureCallback(const lio::CameraMeasurementConstPtr& msg) {
  ROS_INFO("received feature msg: %f ", msg->header.stamp.toSec());
  if (!initialize_flag) {
    return;
  }
  if (msckf_vio::is_first_img) {
    msckf_vio::is_first_img = false;
    msckf_vio::state_server.imu_state.time = msg->header.stamp.toSec();
  }
  ros::Time start_time = ros::Time::now();
  // Augment the state vector.
  start_time = ros::Time::now();
  // stateAugmentation(msg->header.stamp.toSec());
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

main(int argc, char** argv) {
  ros::init(argc, argv, "lio_node");
  ofile.open("received_point.txt");

  ros::NodeHandle nh("~");

  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1000);
  path_pub = nh.advertise<nav_msgs::Path>("path", 1000);

  ros::Subscriber imu_sub = nh.subscribe("/imu_sim", 1000, imuCallback);
  ros::Subscriber gps_sub = nh.subscribe("/gps", 100, gpsCallback);
  ros::Subscriber feature_sub =
      nh.subscribe("/features0", 100, featureCallback);

  // ros::Rate rate(10);

  ros::spin();

  return 0;
}