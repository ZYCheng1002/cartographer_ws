#pragma once

#include <memory>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace error_state_kalman {
    struct ImuData {
        double timestamp; // In second.

        Eigen::Vector3d acc; // Acceleration in m/s^2
        Eigen::Vector3d gyro; // Angular velocity in radian/s.
    };

    using ImuDataPtr = std::shared_ptr<ImuData>;

    struct LaserPoseData {
        double timestamp;
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
        Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Identity() * 0.0001; // Covariance in m^2.
    };

    using LaserPoseDataPtr = std::shared_ptr<LaserPoseData>;

    struct State {
        double timestamp;

        Eigen::Vector3d lla; // WGS84 position.
        Eigen::Vector3d G_p_I; // The original point of the IMU frame in the Global frame.
        Eigen::Vector3d G_v_I; // The velocity original point of the IMU frame in the Global frame.
        Eigen::Matrix3d G_R_I; // The rotation from the IMU frame to the Global frame.
        Eigen::Vector3d acc_bias; // The bias of the acceleration sensor.
        Eigen::Vector3d gyro_bias; // The bias of the gyroscope sensor.

        // Covariance.
        Eigen::Matrix<double, 15, 15> cov;

        // The imu data.
        ImuDataPtr imu_data_ptr;
    };

    inline Eigen::Matrix3d GetSkewMatrix(const Eigen::Vector3d &v) {
        Eigen::Matrix3d w;
        w << 0., -v(2), v(1),
                v(2), 0., -v(0),
                -v(1), v(0), 0.;

        return w;
    }

    constexpr double kDegreeToRadian = M_PI / 180.;
    constexpr double kRadianToDegree = 180. / M_PI;
} // ImuGpsLocalization
