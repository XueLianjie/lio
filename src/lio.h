#ifndef LIO_H
#define LIO_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/PointCloud2.h>
#include "imu_state.h"
#include "lidar_state.h"

//#include "feature.hpp"
//#include <msckf_vio/CameraMeasurement.h>

namespace lio {

class Lio {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Lio(ros::NodeHandle& pnh);
  Lio(const Lio& lio) = delete;
  Lio operator=(const Lio& lio) = delete;

  ~Lio() {}

  bool initialize();

  void reset();

  typedef boost::shared_ptr<Lio> Ptr;
  typedef boost::shared_ptr<const Lio> ConstPtr;

 private:
  struct StateServer {
    IMUState imu_state;
    LIDARState lidar_state;

    Eigen::MatrixXd state_cov;
    Eigen::Matrix<double, 12, 12> continuous_noise_cov;
  };

  bool loadParameters();

  bool createRosIO();

  void imuCallback(const sensor_msgs::ImuConstPtr& msg);

  void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg);

  /*
     * @brief publish Publish the results of VIO.
     * @param time The time stamp of output msgs.
     */
  void publish(const ros::Time& time);

  /*
     * @brief initializegravityAndBias
     *    Initialize the IMU bias and initial orientation
     *    based on the first few IMU readings.
     */
  void initializeGravityAndBias();

  /*
     * @brief checkIMUStatic
     *    Check if the IMU is static for intialize 
     */
  void checkIMUStatic();

  /*
     * @biref resetCallback
     *    Callback function for the reset service.
     *    Note that this is NOT anytime-reset. This function should
     *    only be called before the sensor suite starts moving.
     *    e.g. while the robot is still on the ground.
     */
  bool resetCallback(std_srvs::Trigger::Request& req,
                     std_srvs::Trigger::Response& res);

  // Filter related functions
  // Propogate the state
  void batchImuProcessing(
      const double& time_bound);
  void processModel(const double& time,
                    const Eigen::Vector3d& m_gyro,
                    const Eigen::Vector3d& m_acc);
  void predictNewState(const double& dt,
                       const Eigen::Vector3d& gyro,
                       const Eigen::Vector3d& acc);

  // Measurement update
  void stateAugmentation(const double& time);
  // void addFeatureObservations(const CameraMeasurementConstPtr& msg);
  // // This function is used to compute the measurement Jacobian
  // // for a single feature observed at a single camera frame.
  // void measurementJacobian(const StateIDType& cam_state_id,
  //     const FeatureIDType& feature_id,
  //     Eigen::Matrix<double, 4, 6>& H_x,
  //     Eigen::Matrix<double, 4, 3>& H_f,
  //     Eigen::Vector4d& r);
  // This function computes the Jacobian of all measurements viewed
  // in the given camera states of this feature.
  // void featureJacobian(const FeatureIDType& feature_id,
  //     const std::vector<StateIDType>& cam_state_ids,
  //     Eigen::MatrixXd& H_x, Eigen::VectorXd& r);
  void measurementUpdate(const Eigen::MatrixXd& H,
                         const Eigen::VectorXd& r);
  bool gatingTest(const Eigen::MatrixXd& H,
                  const Eigen::VectorXd& r, const int& dof);

  // void removeLostFeatures();

  // void findRedundantCamStates(
  //     std::vector<StateIDType>& rm_cam_state_ids);
  void pruneCamStateBuffer();
  // Reset the system online if the uncertainty is too large.
  void onlineReset();

  // Chi squared test table.
  static std::map<int, double> chi_squared_test_table;

  // State vector
  StateServer state_server;
  // Maximum number of camera states
  int max_cam_state_size;

  // Features used
  //MapServer map_server;

  // IMU data buffer
  // This is buffer is used to handle the unsynchronization or
  // transfer delay between IMU and Image messages.
  std::vector<sensor_msgs::Imu> imu_msg_buffer;

  // Indicate if the gravity vector is set.
  bool is_gravity_set;

  // Indicate if the received image is the first one. The
  // system will start after receiving the first image.
  bool is_first_point_cloud;

  // The position uncertainty threshold is used to determine
  // when to reset the system online. Otherwise, the ever-
  // increaseing uncertainty will make the estimation unstable.
  // Note this online reset will be some dead-reckoning.
  // Set this threshold to nonpositive to disable online reset.
  double position_std_threshold;

  // Tracking rate
  double tracking_rate;

  // Threshold for determine keyframes
  double translation_threshold;
  double rotation_threshold;
  double tracking_rate_threshold;

  // Ros node handle
  ros::NodeHandle nh;

  // Subscribers and publishers
  ros::Subscriber imu_sub;
  ros::Subscriber feature_sub;
  ros::Publisher odom_pub;
  ros::Publisher feature_pub;
  ros::Publisher path_pub;
  tf::TransformBroadcaster tf_pub;
  ros::ServiceServer reset_srv;
  nav_msgs::Path path;

  // Frame id
  std::string fixed_frame_id;
  std::string child_frame_id;

  // Whether to publish tf or not.
  bool publish_tf;

  // Framte rate of the stereo images. This variable is
  // only used to determine the timing threshold of
  // each iteration of the filter.
  double frame_rate;

  // Debugging variables and functions
  void mocapOdomCallback(
      const nav_msgs::OdometryConstPtr& msg);

  ros::Subscriber mocap_odom_sub;
  ros::Publisher mocap_odom_pub;
  geometry_msgs::TransformStamped raw_mocap_odom_msg;
  Eigen::Isometry3d mocap_initial_frame;
};

typedef Lio::Ptr LioVioPtr;
typedef Lio::ConstPtr LioConstPtr;

}  // namespace lio

#endif