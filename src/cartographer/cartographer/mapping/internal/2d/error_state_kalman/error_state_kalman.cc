#include "error_state_kalman.h"

#include <glog/logging.h>


namespace error_state_kalman {
    ErrorStateKalman::ErrorStateKalman(const double acc_noise, const double gyro_noise,
                                       const double acc_bias_noise, const double gyro_bias_noise,
                                       const Eigen::Vector3d &I_p_Gps)
        : initialized_(false) {
        initializer_ = std::make_unique<Initializer>(I_p_Gps);
        imu_processor_ = std::make_unique<ImuProcessor>(acc_noise, gyro_noise,
                                                        acc_bias_noise, gyro_bias_noise,
                                                        Eigen::Vector3d(0., 0., -9.81007));
        laser_processor_ = std::make_unique<LaserProcessor>();
    }

    bool ErrorStateKalman::ProcessImuData(const ImuDataPtr imu_data_ptr, State *fused_state) {
        if (!initialized_) {
            initializer_->AddImuData(imu_data_ptr);
            return false;
        }

        // imu 预测
        imu_processor_->Predict(state_.imu_data_ptr, imu_data_ptr, &state_);

        // Convert ENU state to lla.
        // ConvertENUToLLA(init_lla_, state_.G_p_I, &(state_.lla));
        *fused_state = state_;

        return true;
    }


    bool ErrorStateKalman::ProcessLaserPositionData(const LaserPoseDataPtr laser_data_ptr) {
        if (!initialized_) {
            if (!initializer_->AddLaserData(laser_data_ptr, &state_)) {
                return false;
            }

            initialized_ = true;

            LOG(INFO) << "[ProcessGpsPositionData]: System initialized!";
            return true;
        }

        // 进行观测时, 传入初始gps位置和此时的gps位置, 计算相对位置变化
        laser_processor_->UpdateStateByLaserPosition(laser_data_ptr, &state_);
        return true;
    }
} // namespace ImuGpsLocalization
