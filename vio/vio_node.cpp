#include "estimator.h"
#include <Eigen/Eigen>
#include <condition_variable>
#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <iostream>
#include <map>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <queue>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf_conversions/tf_eigen.h>
#include <thread>
#include <vector>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ctime>

using namespace std;
mutex m_buf;
queue<ImuData> imu_que;
queue<sensor_msgs::ImageConstPtr> image_que;
queue<sensor_msgs::PointCloud2ConstPtr> velodyne_que;
map<double, nav_msgs::Odometry> odometry_que;
ros::Publisher pub_image;
ros::Publisher pub_cloud;

cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg) {
  cv_bridge::CvImageConstPtr ptr;
  ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat img = ptr->image.clone();
  return img;
}

void image_callback(const sensor_msgs::ImageConstPtr &image_msg) {
  // ROS_INFO("received image_msg time: %f ", image_msg->header.stamp.toSec());
  m_buf.lock();
  image_que.push(image_msg);
  printf("image_que size: %d \n", image_que.size());
  if (image_que.size() > 1000) {
    image_que.pop();
  }
  m_buf.unlock();
}

void velodyne_callback(const sensor_msgs::PointCloud2ConstPtr &velodyne_msg) {
  // ROS_INFO("received velodyne_msg time: %f ",
  //          velodyne_msg->header.stamp.toSec());
  m_buf.lock();
  velodyne_que.push(velodyne_msg);
  printf("velodyne_que size: %d \n", velodyne_que.size());
  if (velodyne_que.size() > 1000) {
    velodyne_que.pop();
  }
  m_buf.unlock();
}

void odometry_callback(const nav_msgs::Odometry::ConstPtr &odometry_msg) {
  m_buf.lock();
  ROS_INFO("received odometry_msg time: %f ",
           odometry_msg->header.stamp.toSec());

  odometry_que.insert(std::make_pair(odometry_msg->header.stamp.toSec() ,*odometry_msg));
  printf("odometry_que size: %d \n", odometry_que.size());
  // if (odometry_que.size() > 5000) {
  //   odometry_que.pop();
  // }
  m_buf.unlock();

}

void ProjectVelodynePointsToImage(
    const sensor_msgs::ImageConstPtr &image_msg,
    const sensor_msgs::PointCloud2ConstPtr &velodyne_msg) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr velodyne_points(
      new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromROSMsg(*velodyne_msg, *velodyne_points);
  cv::Mat projected_image = getImageFromMsg(image_msg);

  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
  K(0, 0) = 554.3836;
  K(1, 1) = 554.3826;
  K(0, 2) = 320.;
  K(1, 2) = 240.;
  std::cout << "K \n" << K << std::endl;

  // Eigen::Matrix4d Tbl = Eigen::Matrix4d::Identity();
  // Tbl(2, 3) = 0.4;
  // std::cout << "Tbl \n" << Tbl << std::endl;

  Eigen::Matrix4d Tbc = Eigen::Matrix4d::Identity();
  Tbc.block<3, 3>(0, 0) << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  Tbc(0, 3) = 1.121;
  Tbc(1, 3) = 0.167;
  Tbc(2, 3) = -0.018;
  std::cout << "Tbc \n" << Tbc << std::endl;

  Eigen::Matrix4d Tcl = Tbc.inverse();
  std::cout << "Tcl \n" << Tcl << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>());

  for (const auto &point : velodyne_points->points) {
    Eigen::Vector4d point_in_lidar(point.x, point.y, point.z, 1.0);
    Eigen::Vector4d point_in_cam = Tcl * point_in_lidar;
    Eigen::Vector3d point_in_pixel = K * point_in_cam.head(3) / point_in_cam(2);
    int u = static_cast<int>(point_in_pixel(0));
    int v = static_cast<int>(point_in_pixel(1));
    if (point_in_cam(2) < 0 || u < 0 || u >= projected_image.cols || v < 0 ||
        v >= projected_image.rows) {
      continue;
    }
    pcl::PointXYZRGB p;
    p.x = point.x;
    p.y = point.y;
    p.z = point.z;
    p.b = projected_image
              .data[v * projected_image.step + u * projected_image.channels()];
    p.g = projected_image.data[v * projected_image.step +
                               u * projected_image.channels() + 1];
    p.r = projected_image.data[v * projected_image.step +
                               u * projected_image.channels() + 2];
    color_cloud->points.push_back(p);
  }
  color_cloud->is_dense = false;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  m_buf.lock();
  while (odometry_que.upper_bound(velodyne_msg->header.stamp.toSec()) == odometry_que.end()) {
    m_buf.unlock();
    usleep(300);
  }

  if (!odometry_que.empty() && odometry_que.upper_bound(velodyne_msg->header.stamp.toSec()) != odometry_que.end()) {
    nav_msgs::Odometry odom = (odometry_que.upper_bound(velodyne_msg->header.stamp.toSec())->second);
    Eigen::Matrix3d rot  = 
    Eigen::Quaterniond(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z).toRotationMatrix();
    Eigen::Vector3d trans(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
    pose.block<3, 3>(0, 0) = rot;
    pose.block<3, 1>(0, 3) = trans;
    pcl::transformPointCloud(*color_cloud, *transformed_cloud, pose);
      // *map_ = *map_ + *transformed_cloud;

    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*transformed_cloud, tempCloud);
    tempCloud.header.stamp = ros::Time::now();
    tempCloud.header.frame_id = "velodyne";
    pub_cloud.publish(tempCloud);
  }
  m_buf.unlock();


}

void process() {
  while (true) {
    while (!image_que.empty() && !velodyne_que.empty()) {
      m_buf.lock();
      while (!velodyne_que.empty() &&
             !(image_que.front()->header.stamp.toSec() <
               velodyne_que.front()->header.stamp.toSec())) {
        velodyne_que.pop();
      }
      if (velodyne_que.empty()) {
        m_buf.unlock();
        break;
      }

      if (image_que.back()->header.stamp.toSec() <
          velodyne_que.front()->header.stamp.toSec()) {
        m_buf.unlock();
        break;
      }
      auto pre_image_ptr = image_que.front();
      while (image_que.front()->header.stamp.toSec() <
             velodyne_que.front()->header.stamp.toSec()) {
        pre_image_ptr = image_que.front();
        printf("pre_image_ptr time stamp: %f \n",
               pre_image_ptr->header.stamp.toSec());
        image_que.pop();
      }
      auto next_image_ptr = image_que.front();
      printf("next_image_ptr time stamp: %f \n",
             next_image_ptr->header.stamp.toSec());

      auto current_velodyne_points = velodyne_que.front();
      printf("curr_velodyne_points time stamp: %f \n",
             current_velodyne_points->header.stamp.toSec());
      velodyne_que.pop();
      m_buf.unlock();
      auto current_image = next_image_ptr;
      if (next_image_ptr->header.stamp.toSec() -
              current_velodyne_points->header.stamp.toSec() >
          current_velodyne_points->header.stamp.toSec() -
              pre_image_ptr->header.stamp.toSec()) {
        current_image = pre_image_ptr;
      }
      ProjectVelodynePointsToImage(current_image, current_velodyne_points);

    }
    ros::Duration(0.01).sleep();
  }

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_projection_node"); // node name
  ros::NodeHandle n("~"); // 指定子命名空间， launch中通过 ns =
  ros::Subscriber image_subscriber =
      n.subscribe("/d435/camera/color/image_raw", 2000, image_callback);
  ros::Subscriber velodyne_subscriber =
      n.subscribe("/velodyne_points", 2000, velodyne_callback);
  ros::Subscriber odometry_subscriber =
      n.subscribe("/lio_sam/mapping/odometry", 2000, odometry_callback);

  std::thread measurement_process{process};
  pub_image = n.advertise<sensor_msgs::Image>("/image_track", 1000);
  pub_cloud = n.advertise<sensor_msgs::PointCloud2>("/color_cloud", 10);
  ros::spin();
  return 0;
}
