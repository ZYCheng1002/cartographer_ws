//
// Created by zycheng on 24-11-21.
//

#ifndef IMU_GPS_LOCALIZATION_LASER_PROCESS_H
#define IMU_GPS_LOCALIZATION_LASER_PROCESS_H

#include <Eigen/Dense>

#include "base_type.h"

namespace error_state_kalman {
    class LaserProcessor {
    public:
        LaserProcessor();

        bool
        UpdateStateByLaserPosition(const LaserPoseDataPtr gps_data_ptr, State *state);

    private:
        void ComputeJacobianAndResidual(
                const LaserPoseDataPtr gps_data,
                const State &state,
                Eigen::Matrix<double, 6, 15> *jacobian,
                Eigen::Vector<double, 6> *residual);

        void AddDeltaToState(const Eigen::Matrix<double, 15, 1> &delta_x, State *state);

        template<typename T>
        Eigen::Matrix<T, 3, 1> Log(const Eigen::Matrix<T, 3, 3> &R) {
            T theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
            Eigen::Matrix<T, 3, 1> K(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
            return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
        }
    };


}  // namespace ImuGpsLocalization
#endif //IMU_GPS_LOCALIZATION_LASER_PROCESS_H
