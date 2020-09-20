#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Eigen>
#include "lio/CameraMeasurement.h"
#include "lio/FeatureMeasurement.h"
#include <sensor_msgs/PointCloud.h>

using namespace lio;

class Visualizer
{
public:
    Visualizer(const ros::NodeHandle &nh) : nh_(nh)
    {
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);

        //初始化
        points_.header.frame_id = line_strip_.header.frame_id = line_list_.header.frame_id = "/world";
        points_.ns = line_strip_.ns = line_list_.ns = "points_and_lines";
        points_.action = line_strip_.action = line_list_.action = visualization_msgs::Marker::ADD;
        points_.pose.orientation.w = line_strip_.pose.orientation.w = line_list_.pose.orientation.w = 1.0;

        //分配3个id
        points_.id = 1;
        line_strip_.id = 0;
        line_list_.id = 2;

        //初始化形状
        points_.type = visualization_msgs::Marker::POINTS;
        line_strip_.type = visualization_msgs::Marker::LINE_STRIP;
        line_list_.type = visualization_msgs::Marker::LINE_LIST;

        //初始化大小
        // POINTS markers use x and y scale for width/height respectively
        points_.scale.x = 0.1;
        points_.scale.y = 0.1;

        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_strip_.scale.x = 0.1;
        line_strip_.scale.y = 0.1;

        line_list_.scale.x = 0.05;
        // line_list_.scale.y = 0.1;

        //初始化颜色
        // Points are green
        points_.color.g = 1.0;
        points_.color.a = 1.0;

        // Line strip is blue
        line_strip_.color.b = 1.0;
        line_strip_.color.a = 1.0;

        // Line list is red
        line_list_.color.r = 1.0;
        line_list_.color.a = 1.0;

        features_pub_ = nh_.advertise<CameraMeasurement>("/features", 100);

        //初始化
        feature_points_.header.frame_id = "cam_frame";
        feature_points_.ns = feature_lines_list_.ns = "features";
        feature_points_.action = feature_lines_list_.action = visualization_msgs::Marker::ADD;
        feature_points_.pose.orientation.w = feature_lines_list_.pose.orientation.w = 1.0;

        feature_lines_list_.header.frame_id = "/world";

        //分配3个id
        feature_points_.id = 0;
        feature_lines_list_.id = 1;

        //初始化形状
        feature_points_.type = visualization_msgs::Marker::POINTS;
        feature_lines_list_.type = visualization_msgs::Marker::LINE_LIST;

        feature_points_.scale.x = 0.1;
        feature_points_.scale.y = 0.1;

        feature_lines_list_.scale.x = 0.02;

        //初始化颜色
        feature_points_.color.g = 1.0;
        feature_points_.color.a = 1.0;

        feature_lines_list_.color.r = 1.0;
        feature_lines_list_.color.g = 1.0;
        feature_lines_list_.color.a = 1.0;

        feature_points_cloud_.reset(new sensor_msgs::PointCloud);
        feature_points_cloud_->header.frame_id = "/world";
        pub_img = nh_.advertise<sensor_msgs::PointCloud>("img_feature", 1000);
        velocity_flag = false;
        // for(int i = 0; i < 18; ++i)
        // last_features_.push_back(Eigen::Vector2d(0.0, 0.0));

    }

    Visualizer(const Visualizer &vis) = delete;
    Visualizer &operator=(const Visualizer &vis) = delete;

    ~Visualizer()
    {
    }

    void AddPoints(const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &points)
    {
        // Create the vertices for the points and lines
        for (uint32_t i = 0; i < points.size(); ++i)
        {
            geometry_msgs::Point p;
            p.x = points[i](0);
            p.y = points[i](1);
            p.z = points[i](2);

            points_.points.push_back(p);
            //line_strip_.points.push_back(p);

            // The line list needs two points for each line
            // line_list_.points.push_back(p);
            // p.z += 1.0;
            // line_list_.points.push_back(p);
        }
    }

    void AddLines(const std::vector<std::pair<Eigen::Vector4d, Eigen::Vector4d>, Eigen::aligned_allocator<std::pair<Eigen::Vector4d, Eigen::Vector4d>>> &lines)
    {
        // Create the vertices for the points and lines
        for (uint32_t i = 0; i < lines.size(); ++i)
        {
            geometry_msgs::Point p0, p1;
            p0.x = lines[i].first(0);
            p0.y = lines[i].first(1);
            p0.z = lines[i].first(2);

            p1.x = lines[i].second(0);
            p1.y = lines[i].second(1);
            p1.z = lines[i].second(2);

            // points_.points.push_back(p0);
            // line_strip_.points.push_back(p);

            // The line list needs two points for each line
            line_list_.points.push_back(p0);
            // p.z += 1.0;
            line_list_.points.push_back(p1);
        }
    }

    void PublishMarker(const ros::Time &stamp)
    {
        points_.header.stamp = line_strip_.header.stamp = line_list_.header.stamp = stamp;
        //marker_pub_.publish(line_strip_);
        marker_pub_.publish(line_list_);
        marker_pub_.publish(points_);
    }

    void PublishCamFrame(const ros::Time &stamp, const Eigen::Matrix4d &Twc)
    {
        Eigen::Vector3d twc = Twc.block<3, 1>(0, 3);
        Eigen::Quaterniond qwc(Twc.block<3, 3>(0, 0));
        transform_.setOrigin(tf::Vector3(twc(0), twc(1), twc(2)));
        tf::Quaternion tf_qwc(qwc.x(), qwc.y(), qwc.z(), qwc.w());
        transform_.setRotation(tf_qwc);

        br_.sendTransform(tf::StampedTransform(transform_, stamp, "world", "cam_frame"));
    }

    void PublishBodyFrame(const ros::Time &stamp, const Eigen::Matrix4d &Twb)
    {
        Eigen::Vector3d twb = Twb.block<3, 1>(0, 3);
        Eigen::Quaterniond qwb(Twb.block<3, 3>(0, 0));
        transform_.setOrigin(tf::Vector3(twb(0), twb(1), twb(2)));
        tf::Quaternion tf_qwb(qwb.x(), qwb.y(), qwb.z(), qwb.w());
        transform_.setRotation(tf_qwb);

        br_.sendTransform(tf::StampedTransform(transform_, stamp, "world", "body_frame"));
    }

    void PublishFeaturePoints(const ros::Time &stamp, const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> &features)
    {
        CameraMeasurementPtr feature_msg_ptr(new CameraMeasurement);
        feature_msg_ptr->header.stamp = stamp;
        feature_points_cloud_->header.stamp = stamp;

        feature_points_.header.stamp = stamp;
        feature_points_.points.clear();
        //std::cout << t << " Twc " << Twc << std::endl;
        std::cout << "observation size " << features.size() << std::endl;
        geometry_msgs::Point p;
        geometry_msgs::Point32 p32;

        sensor_msgs::ChannelFloat32 id_of_point;
        sensor_msgs::ChannelFloat32 u_of_point;
        sensor_msgs::ChannelFloat32 v_of_point;
        sensor_msgs::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::ChannelFloat32 velocity_y_of_point;
        feature_points_cloud_->points.clear();

        for (int i = 0; i < features.size(); ++i)
        {
            feature_msg_ptr->features.push_back(FeatureMeasurement());
            feature_msg_ptr->features[i].id = i;
            feature_msg_ptr->features[i].u0 = features[i](0);
            feature_msg_ptr->features[i].v0 = features[i](1);
            feature_msg_ptr->features[i].u1 = features[i](0);
            feature_msg_ptr->features[i].v1 = features[i](1);
            //std::cout << "features " << features[i].transpose() << std::endl;
            p.x = features[i](0);
            p.y = features[i](1);
            p.z = 1.0;
            feature_points_.points.push_back(p);

                    id_of_point.values.push_back(i);

                    p32.x = features[i](0);
                    p32.y = features[i](1);
                    p32.z = 1;
                    feature_points_cloud_->points.push_back(p32);

                    float ux = 460 * p32.x + 376;
                    float uy = 460 * p32.y + 240;
                    u_of_point.values.push_back(ux);
                    v_of_point.values.push_back(uy);

            if(!velocity_flag)
            {
                    velocity_x_of_point.values.push_back(0.0);
                    velocity_y_of_point.values.push_back(0.0);

            }
            else
            {
                    velocity_x_of_point.values.push_back(20.0 * (- last_features_[i][0] + features[i](0) ) );
                    velocity_y_of_point.values.push_back(20.0 * (- last_features_[i][1] + features[i](1) ) );

            }

        }
        velocity_flag = true;
        last_features_ = features;

        feature_points_cloud_->channels.push_back(id_of_point);
        feature_points_cloud_->channels.push_back(u_of_point);
        feature_points_cloud_->channels.push_back(v_of_point);
        feature_points_cloud_->channels.push_back(velocity_x_of_point);
        feature_points_cloud_->channels.push_back(velocity_y_of_point);
        ROS_INFO("publish %f ", feature_points_cloud_->header.stamp.toSec());

        pub_img.publish(feature_points_cloud_);

        features_pub_.publish(feature_msg_ptr);
        marker_pub_.publish(feature_points_);
    }

    void PublishFeatureLines(const ros::Time &stamp, const Eigen::Matrix4d &Twc)
    {
        geometry_msgs::Point p0, p1;

        Eigen::Vector3d twc = Twc.block<3, 1>(0, 3);
        p0.x = twc(0);
        p0.y = twc(1);
        p0.z = twc(2);
        feature_lines_list_.header.stamp = stamp;
        feature_lines_list_.points.clear();
        for (uint32_t i = 0; i < points_.points.size(); ++i)
        {
            p1 = points_.points[i];
            // The line list needs two points for each line
            feature_lines_list_.points.push_back(p0);
            // p.z += 1.0;
            feature_lines_list_.points.push_back(p1);
        }

        marker_pub_.publish(feature_lines_list_);
    }

private:
    visualization_msgs::Marker points_, line_strip_, line_list_;
    visualization_msgs::Marker feature_points_, feature_lines_list_;
    ros::Publisher marker_pub_;
    ros::Publisher features_pub_;
    ros::Publisher pub_img; // = n.advertise<sensor_msgs::PointCloud>("feature", 1000);

    sensor_msgs::PointCloudPtr feature_points_cloud_; //(new sensor_msgs::PointCloud);
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> last_features_;
    bool velocity_flag;
    ros::NodeHandle nh_;
    tf::TransformBroadcaster br_;
    tf::Transform transform_;
};

#endif
