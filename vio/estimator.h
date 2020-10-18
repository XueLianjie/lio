#ifndef __ESTIMATOR__
#define __ESTIMATOR__

#include <Eigen/Eigen>
#include <vector>

struct ImuData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ImuData(double time_stamp, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro)
    {
        time_stamp_ = time_stamp;
        acc_ = acc;
        gyro_ = gyro;
    }
    double time_stamp_;
    Eigen::Vector3d acc_;
    Eigen::Vector3d gyro_;
};

/*
 * @brief IMUState State for IMU
 */
struct IMUState
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef long long int StateIDType;

    // An unique identifier for the IMU state.
    StateIDType id;

    // id for next IMU state
    static StateIDType next_id; //static变量

    // Time when the state is recorded
    double time;

    // Orientation
    // Take a vector from the world frame to
    // the IMU (body) frame.
    Eigen::Vector4d orientation; //只有orientation 是 world -> IMU

    // Position of the IMU (body) frame
    // in the world frame.
    Eigen::Vector3d position;

    // Velocity of the IMU (body) frame
    // in the world frame.
    Eigen::Vector3d velocity;

    // Bias for measured angular velocity
    // and acceleration.
    Eigen::Vector3d gyro_bias;
    Eigen::Vector3d acc_bias;

    // Transformation between the IMU and the
    // left camera (cam0)
    Eigen::Matrix3d R_imu_cam0;
    Eigen::Vector3d t_cam0_imu;

    // These three variables should have the same physical
    // interpretation with `orientation`, `position`, and
    // `velocity`. These three variables are used to modify
    // the transition matrices to make the observability matrix
    // have proper null space.
    Eigen::Vector4d orientation_null;
    Eigen::Vector3d position_null;
    Eigen::Vector3d velocity_null;

    // Process noise 这些也都是static变量，需要进行类外初始化
    static double gyro_noise;
    static double acc_noise;
    static double gyro_bias_noise;
    static double acc_bias_noise;

    // Gravity vector in the world frame
    static Eigen::Vector3d gravity;

    // Transformation offset from the IMU frame to
    // the body frame. The transformation takes a
    // vector from the IMU frame to the body frame.
    // The z axis of the body frame should point upwards.
    // Normally, this transform should be identity.
    static Eigen::Isometry3d T_imu_body; // Identity

    // 默认构造函数，初值列采用的是拷贝构造
    IMUState() : id(0), time(0),
                 orientation(Eigen::Vector4d(0, 0, 0, 1)),
                 position(Eigen::Vector3d::Zero()),
                 velocity(Eigen::Vector3d::Zero()),
                 gyro_bias(Eigen::Vector3d::Zero()),
                 acc_bias(Eigen::Vector3d::Zero()),
                 orientation_null(Eigen::Vector4d(0, 0, 0, 1)),
                 position_null(Eigen::Vector3d::Zero()),
                 velocity_null(Eigen::Vector3d::Zero()) {}

    // 构造函数重载
    IMUState(const StateIDType &new_id) : id(new_id), time(0),
                                          orientation(Eigen::Vector4d(0, 0, 0, 1)),
                                          position(Eigen::Vector3d::Zero()),
                                          velocity(Eigen::Vector3d::Zero()),
                                          gyro_bias(Eigen::Vector3d::Zero()),
                                          acc_bias(Eigen::Vector3d::Zero()),
                                          orientation_null(Eigen::Vector4d(0, 0, 0, 1)),
                                          position_null(Eigen::Vector3d::Zero()),
                                          velocity_null(Eigen::Vector3d::Zero()) {}
};

typedef IMUState::StateIDType StateIDType;

class Estimator
{
private:
    /* data */
    bool initialize_flag_;
    IMUState imu_state_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Estimator(/* args */);
    ~Estimator();
    bool GetInitializeFlag() const { return initialize_flag_; }
    bool StaticInitialize(const std::vector<ImuData> &imu_vec);
    void Predict();
    void Update();
};

#endif