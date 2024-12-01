//
// Created by zycheng on 24-11-21.
//
#include "cartographer_ros/error_state_kalman/laser_process.h"


namespace error_state_kalman {

    LaserProcessor::LaserProcessor() {}

    bool LaserProcessor::UpdateStateByLaserPosition(const LaserPoseDataPtr laser_data_ptr,
                                                    State *state) {
        Eigen::Matrix<double, 6, 15> H;
        Eigen::Vector<double, 6> residual;
        // 计算残差和观测的雅可比矩阵
        ComputeJacobianAndResidual(laser_data_ptr, *state, &H, &residual);
        // 获取观测的协方差矩阵
        const Eigen::Matrix<double, 6, 6> &V = laser_data_ptr->cov;

        // 计算卡尔曼增益
        const Eigen::MatrixXd &P = state->cov;
        const Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + V).inverse();
        // 计算误差变量的后验
        const Eigen::VectorXd delta_x = K * residual;

        // 更新状态量
        AddDeltaToState(delta_x, state);

        // 更新协方差
        const Eigen::MatrixXd I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * H;
        state->cov = I_KH * P * I_KH.transpose() + K * V * K.transpose();
        return true;
    }

    void LaserProcessor::ComputeJacobianAndResidual(
            const LaserPoseDataPtr laser_data,
            const State &state,
            Eigen::Matrix<double, 6, 15> *jacobian,
            Eigen::Vector<double, 6> *residual) {

        // 计算误差
        residual->block<3, 1>(0, 0) = laser_data->position - state.G_p_I;
        residual->block<3, 1>(3, 0) = Log(state.G_R_I.inverse() * laser_data->orientation);

        // 观测的雅可比矩阵
        jacobian->setZero();
        jacobian->block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        jacobian->block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();
    }

    void LaserProcessor::AddDeltaToState(const Eigen::Matrix<double, 15, 1> &delta_x, State *state) {
        state->G_p_I += delta_x.block<3, 1>(0, 0);
        state->G_v_I += delta_x.block<3, 1>(3, 0);
        state->acc_bias += delta_x.block<3, 1>(9, 0);
        state->gyro_bias += delta_x.block<3, 1>(12, 0);

        if (delta_x.block<3, 1>(6, 0).norm() > 1e-12) {
            state->G_R_I *= Eigen::AngleAxisd(delta_x.block<3, 1>(6, 0).norm(),
                                              delta_x.block<3, 1>(6, 0).normalized()).toRotationMatrix();
        }
    }

}  // namespace ImuGpsLocalization