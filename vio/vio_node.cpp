#include <iostream>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <thread>
#include <mutex>
#include <queue>
#include <vector>
#include <map>
#include <condition_variable>
#include "estimator.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <tf_conversions/tf_eigen.h>

using namespace std;

// #include <sensor_msgs/Image.h>

//std::condition_variable con;
mutex m_buf;

queue<ImuData, Eigen::aligned_allocator<ImuData>> imu_que;

queue<sensor_msgs::PointCloudConstPtr> feature_que;

// double last_imu_t = 0;
Estimator vio_estimator;

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    ROS_INFO("received imu_msg time: %f ", imu_msg->header.stamp.toSec());
    // if (imu_msg->header.stamp.toSec() <= last_imu_t)
    // {
    //     ROS_WARN("imu message in disorder!");
    //     return;
    // }
    // last_imu_t = imu_msg->header.stamp.toSec();
    m_buf.lock();
    Eigen::Vector3d acc, gyro;
    tf::vectorMsgToEigen(imu_msg->linear_acceleration, acc);
    tf::vectorMsgToEigen(imu_msg->angular_velocity, gyro);
    imu_que.push_back(ImuData(imu_msg->header.stamp.toSec(), acc, gyro));
    printf("imu_que size: %d \n", imu_que.size());
    if (imu_que.size() > 200 && !vio_estimator.GetInitializeFlag())
    {
        vector<ImuData, Eigen::aligned_allocator<ImuData>> imu_vec;
        while (!imu_que.empty())
        {
            imu_vec.emplace_back(imu_que.front());
            imu_que.pop();
        }
        m_buf.unlock();
        vio_estimator.StaticInitialize(imu_vec);
        return;
    }

    m_buf.unlock();

    // con.notify_one();

    // last_imu_t = imu_msg->header.stamp.toSec();
    // {
    //     std::lock_guard<std::mutex> lg(m_state);
    //     predict(imu_msg);
    //     std_msgs::Header header = imu_msg->header;
    //     header.frame_id = "world";
    //     if(estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    //         pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);

    // }
}

void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    ROS_INFO("received feature_msg time: %f ", feature_msg->header.stamp.toSec());
    if (!vio_estimator.GetInitializeFlag())
    {
        return; // not initialized do not receive features
    }
    // if(!init_feature)
    // {
    //     // 跳过第一帧图像特征消息，因为第一帧数据没有特征速度信息。
    //     init_feature = 1;
    //     return;
    // }
    m_buf.lock();
    feature_que.push_back(feature_msg);
    printf("imu_que size: %d \n", imu_que.size());

    m_buf.unlock();
    // con.notify_one();
}

// std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>>
// getMeasurements()
// {
//     std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;

//     while (true)
//     {
//         if (imu_que.empty() || feature_que.empty())
//             return measurements;

//         if (!(imu_que.back()->header.stamp.toSec() > feature_que.front()->header.stamp.toSec() + estimator.td))
//         {
//             //ROS_WARN("wait for imu, only should happen at the beginning");
//             sum_of_wait++;
//             return measurements;
//         }

//         if (!(imu_que.front()->header.stamp.toSec() < feature_que.front()->header.stamp.toSec() + estimator.td))
//         {
//             ROS_WARN("throw img, only should happen at the beginning");
//             feature_que.pop();
//             continue;
//         }
//         sensor_msgs::PointCloudConstPtr feature_ptr = feature_que.front();
//         feature_que.pop();

//         std::vector<sensor_msgs::ImuConstPtr> IMUs;
//         while (imu_que.front()->header.stamp.toSec() < feature_ptr->header.stamp.toSec() + estimator.td)
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

void process()
{
    while (true)
    {
        while (!imu_que.empty() && !feature_que.empty())
        {
            m_buf.lock();
            // i i i i  | image 为一帧
            while (!feature_que.empty() && !(imu_que.front().time_stamp_ < feature_que.front()->header.stamp.toSec()))
            {
                feature_que.pop();
            }
            if (feature_que.empty())
            {
                m_buf.unlock();
                break;
            }

            if (imu_que.back()->header.stamp.toSec() < feature_que.front()->header.stamp.toSec())
            {
                m_buf.unlock();
                break;
            }
            vector<ImuData, Eigen::aligned_allocator<ImuData>> imu_vec;
            while (imu_que.front().time_stamp_ < feature_que.front()->header.stamp.toSec())
            {
                imu_vec.emplace_back(imu_que.front());
                printf("imu time stamp: %f \n", imu_que.front().time_stamp_);
                imu_que.pop();
            }
            imu_vec.emplace_back(imu_que.front());
            printf("imu time stamp: %f \n", imu_que.front().time_stamp_);

            sensor_msgs::PointCloudConstPtr feature_ptr = feature_que.front();
            printf("imu time stamp: %f \n", feature_que.front()->header.stamp.toSec());
            feature_que.pop();
            m_buf.unlock();

            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
            for (unsigned int i = 0; i < feature_ptr->points.size(); i++)
            {
                int v = feature_ptr->channels[0].values[i] + 0.5;
                int feature_id = v / 1;
                int camera_id = v % 1;
                double x = feature_ptr->points[i].x;
                double y = feature_ptr->points[i].y;
                double z = feature_ptr->points[i].z;
                double p_u = feature_ptr->channels[1].values[i];
                double p_v = feature_ptr->channels[2].values[i];
                double velocity_x = feature_ptr->channels[3].values[i];
                double velocity_y = feature_ptr->channels[4].values[i];
                ROS_ASSERT(z == 1);
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                image[feature_id].emplace_back(camera_id, xyz_uv_velocity);
            }

            //            estimator.processImage(image, feature_ptr->header);
        }
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "vio_optimization_based_node"); // node name

    ros::NodeHandle n("~"); // 指定子命名空间， launch中通过 ns = "node_namespace"指定全局命名空间

    //ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    //readParameters(n);

    ros::Subscriber sub_imu = n.subscribe("/imu_sim", 2000, imu_callback); // , ros::TransportHints().tcpNoDelay()); // TODO: 探究下这里的tcpNoDelay是什么意思
    ros::Subscriber sub_image = n.subscribe("/imu_pub_node/img_feature", 2000, feature_callback);
    std::thread measurement_process{process};
    ros::spin();
    return 0;
}
