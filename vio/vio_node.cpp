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

// double last_imu_t = 0;
// Estimator vio_estimator;

// void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
// {
//     ROS_INFO("received imu_msg time: %f ", imu_msg->header.stamp.toSec());
//     // if (imu_msg->header.stamp.toSec() <= last_imu_t)
//     // {
//     //     ROS_WARN("imu message in disorder!");
//     //     return;
//     // }
//     // last_imu_t = imu_msg->header.stamp.toSec();
//     m_buf.lock();
//     Eigen::Vector3d acc, gyro;
//     tf::vectorMsgToEigen(imu_msg->linear_acceleration, acc);
//     tf::vectorMsgToEigen(imu_msg->angular_velocity, gyro);
//     imu_que.push(ImuData(imu_msg->header.stamp.toSec(), acc, gyro));
//     printf("imu_que size: %d \n", imu_que.size());
//     if (imu_que.size() > 200 && !vio_estimator.GetInitializeFlag())
//     {
//         vector<ImuData> imu_vec;
//         while (!imu_que.empty())
//         {
//             imu_vec.emplace_back(imu_que.front());
//             imu_que.pop();
//         }
//         m_buf.unlock();
//         vio_estimator.StaticInitialize(imu_vec);
//         return;
//     }

//     m_buf.unlock();

//     // con.notify_one();

//     // last_imu_t = imu_msg->header.stamp.toSec();
//     // {
//     //     std::lock_guard<std::mutex> lg(m_state);
//     //     predict(imu_msg);
//     //     std_msgs::Header header = imu_msg->header;
//     //     header.frame_id = "world";
//     //     if(estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
//     //         pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);

//     // }
// }

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

// std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>,
// sensor_msgs::PointCloudConstPtr>> getMeasurements()
// {
//     std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>,
//     sensor_msgs::PointCloudConstPtr>> measurements;

//     while (true)
//     {
//         if (imu_que.empty() || feature_que.empty())
//             return measurements;

//         if (!(imu_que.back()->header.stamp.toSec() >
//         feature_que.front()->header.stamp.toSec() + estimator.td))
//         {
//             //ROS_WARN("wait for imu, only should happen at the beginning");
//             sum_of_wait++;
//             return measurements;
//         }

//         if (!(imu_que.front()->header.stamp.toSec() <
//         feature_que.front()->header.stamp.toSec() + estimator.td))
//         {
//             ROS_WARN("throw img, only should happen at the beginning");
//             feature_que.pop();
//             continue;
//         }
//         sensor_msgs::PointCloudConstPtr feature_ptr = feature_que.front();
//         feature_que.pop();

//         std::vector<sensor_msgs::ImuConstPtr> IMUs;
//         while (imu_que.front()->header.stamp.toSec() <
//         feature_ptr->header.stamp.toSec() + estimator.td)
//         {
//             IMUs.emplace_back(imu_que.front());
//             imu_que.pop();
//         }
//         IMUs.emplace_back(imu_que.front());
//         if (IMUs.empty())
//             ROS_WARN("no imu between two image");
//         measurements.emplace_back(IMUs, feature_ptr);
//     }
//     return measurements;
// }

void ProjectVelodynePointsToImage(
    const sensor_msgs::ImageConstPtr &image_msg,
    const sensor_msgs::PointCloud2ConstPtr &velodyne_msg) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr velodyne_points(
      new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromROSMsg(*velodyne_msg, *velodyne_points);
  cv::Mat projected_image = getImageFromMsg(image_msg);

  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
  K(0, 0) = 554.2547;
  K(1, 1) = 554.2547;
  K(0, 2) = 320.5;
  K(1, 2) = 240.5;
  std::cout << "K \n" << K << std::endl;

  Eigen::Matrix4d Tbl = Eigen::Matrix4d::Identity();
  Tbl(2, 3) = 0.4;
  std::cout << "Tbl \n" << Tbl << std::endl;

  Eigen::Matrix4d Tbc = Eigen::Matrix4d::Identity();
  Tbc.block<3, 3>(0, 0) << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  Tbc(0, 3) = -0.087;
  Tbc(1, 3) = 0.0205;
  Tbc(2, 3) = 0.2870;
  std::cout << "Tbc \n" << Tbc << std::endl;

  Eigen::Matrix4d Tcl = Tbc.inverse() * Tbl;

  for (const auto &point : velodyne_points->points) {
    Eigen::Vector4d point_in_lidar(point.x, point.y, point.z, 1.0);
    Eigen::Vector4d point_in_cam = Tcl * point_in_lidar;
    Eigen::Vector3d point_in_pixel = K * point_in_cam.head(3);
  }

  // cv::Mat projected_image = getImageFromMsg(image_msg);
  // printf("cols %d rows %d ", projected_image.cols, projected_image.rows);
  // std_msgs::Header header;
  // header.frame_id = "velodyne";
  // header.stamp = ros::Time::now();
  // sensor_msgs::ImagePtr projected_imageMsg =
  //     cv_bridge::CvImage(header, "mono8", projected_image).toImageMsg();
  // pub_image.publish(projected_imageMsg);
  // for (int i = 0; i < 5; i++) {
  //   cout << "转换图像中: " << i + 1 << endl;
  //   cv::Mat color = colorImgs[i];
  //   cv::Mat depth = depthImgs[i];
  //   Eigen::Isometry3d T = poses[i];
  //   for (int v = 0; v < color.rows; v++)
  //     for (int u = 0; u < color.cols; u++) {
  //       unsigned int d = depth.ptr<unsigned short>(v)[u]; // 深度值
  //       if (d == 0)
  //         continue; // 为0表示没有测量到
  //       Eigen::Vector3d point;
  //       point[2] = double(d) / depthScale;
  //       point[0] = (u - cx) * point[2] / fx;
  //       point[1] = (v - cy) * point[2] / fy;
  //       Eigen::Vector3d pointWorld = T * point;

  //       PointT p;
  //       p.x = pointWorld[0];
  //       p.y = pointWorld[1];
  //       p.z = pointWorld[2];
  //       p.b = color.data[v * color.step + u * color.channels()];
  //       p.g = color.data[v * color.step + u * color.channels() + 1];
  //       p.r = color.data[v * color.step + u * color.channels() + 2];
  //       pointCloud->points.push_back(p);
  //     }
  // }

  // pointCloud->is_dense = false;
  // cout << "点云共有" << pointCloud->size() << "个点." << endl;
  // pcl::io::savePCDFileBinary("map.pcd", *pointCloud);
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
      auto pre_image_ptr =image_que.front();
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

      auto curr_velodyne_points = velodyne_que.front();
      printf("curr_velodyne_points time stamp: %f \n",
             curr_velodyne_points->header.stamp.toSec());
      velodyne_que.pop();
      m_buf.unlock();

      

  // cv::Mat projected_image = getImageFromMsg(image_msg);
  // printf("cols %d rows %d ", projected_image.cols, projected_image.rows);
  // std_msgs::Header header;
  // header.frame_id = "velodyne";
  // header.stamp = ros::Time::now();
  // sensor_msgs::ImagePtr projected_imageMsg =
  //     cv_bridge::CvImage(header, "bgr8", projected_image).toImageMsg();
  // pub_image.publish(projected_imageMsg);

    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_projection_node"); // node name
  ros::NodeHandle n("~"); // 指定子命名空间， launch中通过 ns =
  ros::Subscriber image_subscriber =
      n.subscribe("/camera/rgb/image_raw", 2000, image_callback);
  ros::Subscriber velodyne_subscriber =
      n.subscribe("/velodyne_points", 2000, velodyne_callback);
    std::thread measurement_process{process};
  pub_image = n.advertise<sensor_msgs::Image>("/image_track", 1000);
  ros::spin();
  return 0;
}
