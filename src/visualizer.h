#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Eigen>

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
        points_.scale.x = 1;
        points_.scale.y = 1;

        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_strip_.scale.x = 0.1;
        line_strip_.scale.y = 0.1;

        line_list_.scale.x = 0.05;
        // line_list_.scale.y = 0.1;

        //初始化颜色
        // Points are green
        points_.color.r = 1.0;
        points_.color.a = 1.0;

        // Line strip is blue
        line_strip_.color.b = 1.0;
        line_strip_.color.a = 1.0;

        // Line list is red
        line_list_.color.r = 1.0;
        line_list_.color.a = 1.0;
    }

    Visualizer(const Visualizer &vis) = delete;
    Visualizer &operator=(const Visualizer &vis) = delete;

    ~Visualizer()
    {
    }

    void AddPoints(std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &points)
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

    void AddLines(std::vector<std::pair<Eigen::Vector4d, Eigen::Vector4d>, Eigen::aligned_allocator<std::pair<Eigen::Vector4d, Eigen::Vector4d>>> &lines)
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

    void PublishMarker(ros::Time &stamp)
    {
        points_.header.stamp = line_strip_.header.stamp = line_list_.header.stamp = stamp;
        //marker_pub_.publish(line_strip_);
        marker_pub_.publish(line_list_);
        marker_pub_.publish(points_);
    }

    void PublishCamFrame(ros::Time &stamp, Eigen::Matrix4d &Twc)
    {
        Eigen::Vector3d twc = Twc.block<3, 1>(0, 3);
        Eigen::Quaterniond qwc(Twc.block<3, 3>(0, 0));
        transform_.setOrigin(tf::Vector3(twc(0), twc(1), twc(2)));
        tf::Quaternion tf_qwc(qwc.x(), qwc.y(), qwc.z(), qwc.w());
        transform_.setRotation(tf_qwc);

        br_.sendTransform(tf::StampedTransform(transform_, stamp, "world", "cam_frame"));
    }

    void PublishBodyFrame(ros::Time &stamp, Eigen::Matrix4d &Twb)
    {
        Eigen::Vector3d twb = Twb.block<3, 1>(0, 3);
        Eigen::Quaterniond qwb(Twb.block<3, 3>(0, 0));
        transform_.setOrigin(tf::Vector3(twb(0), twb(1), twb(2)));
        tf::Quaternion tf_qwb(qwb.x(), qwb.y(), qwb.z(), qwb.w());
        transform_.setRotation(tf_qwb);

        br_.sendTransform(tf::StampedTransform(transform_, stamp, "world", "body_frame"));
    }

private:
    visualization_msgs::Marker points_, line_strip_, line_list_;
    ros::Publisher marker_pub_;
    ros::NodeHandle nh_;
    tf::TransformBroadcaster br_;
    tf::Transform transform_;
};