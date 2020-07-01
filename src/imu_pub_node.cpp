#include <Eigen/Core>
#include <fstream>
#include <iostream>

#include "imu.h"
#include "param.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "utilities.h"

using namespace std;

main(int argc, char** argv) {
  ros::init(argc, argv, "imu_pub_node");

  ros::NodeHandle nh("~");

  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu_sim", 1000);

  // ros::Subscriber imu_sub = nh.subscribe("/imu0", 1000, imuCallback);

  ros::Rate loop_rate(200);
  int number_count = 0;
  Param params;
  IMU imuGen(params);
  sensor_msgs::Imu msg;

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
    MotionData data = imuGen.MotionModel(t);
    if (t == 0) {
      cout << "Rwb " << data.Rwb << endl;
      cout << "p " << data.twb << endl;
      cout << "v " << data.imu_velocity << endl;
      cout << "bg " << data.imu_gyro_bias << endl;
      cout << "ba " << data.imu_acc_bias << endl;
    }

    Eigen::Quaterniond q(data.Rwb);
    //四元数位姿
    msg.orientation.x = t;//q.x();
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
        save_points << t << " "
                    << q.w() << " "
                    << q.x() << " "
                    << q.y() << " "
                    << q.z() << " "
                    << data.twb(0) << " "
                    << data.twb(1) << " "
                    << data.twb(2) << " "
                    << data.imu_gyro(0) << " "
                    << data.imu_gyro(1) << " "
                    << data.imu_gyro(2) << " "
                    << data.imu_acc(0) << " "
                    << data.imu_acc(1) << " "
                    << data.imu_acc(2) << " "
                    << 0 << " "
                    << 0 << " "
                    << 0 << " "
                    << 0 << " "
                    << 0 << " "
                    << 0 << " "
                    << std::endl;

    imu_pub.publish(msg);
    //读取和更新ROS topics，如果没有spinonce()或spin()，节点不会发布消息。
    ros::spinOnce();

    loop_rate.sleep();
    t += 1.0 / params.imu_frequency;
  }
  return 0;
  ros::spin();

  return 0;
}