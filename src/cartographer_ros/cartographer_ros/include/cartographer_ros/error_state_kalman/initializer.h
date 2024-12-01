#pragma once

#include <deque>

#include "base_type.h"

namespace error_state_kalman {

constexpr int kImuDataBufferLength = 100;
constexpr int kAccStdLimit         = 3.;

class Initializer {
public:
    Initializer(const Eigen::Vector3d& init_I_p_Gps);
    
    void AddImuData(const ImuDataPtr imu_data_ptr);

    bool AddLaserData(const LaserPoseDataPtr laser_data_ptr, State* state);

private:
    bool ComputeG_R_IFromImuData(Eigen::Matrix3d* G_R_I);

    bool InitOrientationFromLases(Eigen::Matrix3d* G_R_I, const LaserPoseDataPtr laser_pose);

    Eigen::Vector3d init_I_p_Gps_;
    std::deque<ImuDataPtr> imu_buffer_;
};

}  // namespace ImuGpsLocalization