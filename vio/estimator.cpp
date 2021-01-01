#include "estimator.h"

Estimator::Estimator(/* args */) : initialize_flag_(false), imu_state_(IMUState())
{
}

Estimator::~Estimator()
{
}

bool Estimator::StaticInitialize(const std::vector<ImuData>& imu_vec)
{
    Eigen::Vector3d sum_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d sum_gyro = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_acc, m_gyro;

    for (auto imu : imu_vec)
    {
        sum_acc += imu.acc_;
        sum_gyro += imu.gyro_;
    }
    printf("finish static initializer! \n");

    //std::cout << "imu_vec acc " << imu_vec[imu_vec.size() - 1].acc_ << " gyro " << imu_vec[imu_vec.size() - 1].gyro_ << std::endl;
    // bg_0 = sum_gyro / imu_vec.size();
    // Eigen::Vector3d gravity_imu = sum_acc / imu_vec.size();

    // double gravity_norm = gravity_imu.norm();
    // gravity_w = Eigen::Vector3d(0.0, 0.0,
    //                             -gravity_norm); // gravity or acc in world fram ?

    // q_0 = Eigen::Quaterniond::FromTwoVectors(gravity_imu, -gravity_w);
    // ba_0 = Eigen::Vector3d::Zero();
    // p_0 = Eigen::Vector3d::Zero();
    // v_0 = Eigen::Vector3d::Zero();
    // acc_0 = m_acc;
    // gyro_0 = m_gyro;

    initialize_flag_ = true;
    return true;
}
