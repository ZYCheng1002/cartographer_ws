#pragma once

#include <Eigen/Core>

#include "base_type.h"
#include "imu_processor.h"
#include "initializer.h"
#include "laser_process.h"

namespace error_state_kalman {
    class ErrorStateKalman {
    public:
        ErrorStateKalman(const double acc_noise, const double gyro_noise,
                         const double acc_bias_noise, const double gyro_bias_noise,
                         const Eigen::Vector3d &I_p_Gps);

        bool ProcessImuData(const ImuDataPtr imu_data_ptr, State *fused_state);


        bool ProcessLaserPositionData(const LaserPoseDataPtr laser_data_ptr);

    private:
        std::unique_ptr<Initializer> initializer_;
        std::unique_ptr<ImuProcessor> imu_processor_;
        std::unique_ptr<LaserProcessor> laser_processor_;

        bool initialized_;
        Eigen::Vector3d init_lla_; // 初始gps位姿
        State state_;
    };
} // namespace ImuGpsLocalization
