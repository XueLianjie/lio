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
#include <tf_conversions/tf_eigen.h>
#include <thread>
#include <vector>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;
mutex m_buf;
queue<ImuData> imu_que;
queue<sensor_msgs::ImageConstPtr> image_que;
queue<sensor_msgs::PointCloud2ConstPtr> velodyne_que;
ros::Publisher pub_image;
ros::Publisher pub_cloud;
static int i = 0;
std::string dir = "/mnt/e/dataset/wangqiang/images";
cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg) {
  cv_bridge::CvImageConstPtr ptr;
  ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat img = ptr->image.clone();
  return img;
}

void image_callback(const sensor_msgs::ImageConstPtr &image_msg) {
  if (i % 5 != 0) {
    i++;
    return;
  }
  ROS_INFO("received image_msg time: %f ", image_msg->header.stamp.toSec());
  cv::Mat projected_image = getImageFromMsg(image_msg);
  cv::imwrite(dir + "/" + std::to_string(i) + ".png", projected_image);
  ++i;
  return;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "image_projection_node"); // node name
  ros::NodeHandle n("~"); // 指定子命名空间， launch中通过 ns =
  ros::Subscriber image_subscriber =
      n.subscribe("/camera/color/image_raw", 2000, image_callback);
  ros::spin();
  return 0;
}
