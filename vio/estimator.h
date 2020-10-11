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

class Estimator
{
private:
    /* data */
    bool initialize_flag_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Estimator(/* args */);
    ~Estimator();
    bool GetInitializeFlag() const { return initialize_flag_; }
    bool StaticInitialize(const std::vector<ImuData, Eigen::aligned_allocator<ImuData>> &imu_vec);
    void Predict();
    void Update();
};

#endif