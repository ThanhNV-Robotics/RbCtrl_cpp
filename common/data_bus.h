/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024-2025 Humanoid Robot (Shanghai) Co., Ltd.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#pragma once

#include "Eigen/Dense"

struct DataBus
{
    const int model_nv; // number of dq
    
    Eigen::VectorXd q, dq, ddq, tau; // q=[base_pos(3),base_quat(4),joint_pos]; dq/ddq=[base_lin_vel(3),base_ang_vel(3),joint_vel/acc]
    Eigen::VectorXd qOld;       // q from the previous timestep, cached in updateQ()
    Eigen::VectorXd qCmd, dqCmd, tauff;            // command q, dq and feedforward torque
    Eigen::VectorXd tauCmd;            // reserved, unused: commanded joint torque   

    DataBus(int model_nvIn) : model_nv(model_nvIn)
    {
        q = Eigen::VectorXd::Zero(model_nv + 1);
        dq = Eigen::VectorXd::Zero(model_nv);
        ddq = Eigen::VectorXd::Zero(model_nv);
        tau = Eigen::VectorXd::Zero(model_nv);
        qCmd = Eigen::VectorXd::Zero(model_nv + 1);
        dqCmd = Eigen::VectorXd::Zero(model_nv);
        tauCmd = Eigen::VectorXd::Zero(model_nv);
    };
};
