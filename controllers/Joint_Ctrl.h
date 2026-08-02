#pragma once

#include <Eigen/Dense>
#include <map>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <fstream>
#include <iostream>
#include <string>
#include "json/json.h"
#include <vector>
#include "data_bus.h"

class JointController
{
public:
    int jointNum;
    

    // Joint order matching joint_pos_des/joint_vel_des/joint_tor_des and every
    // other per-joint vector below -- callers must map onto this order by
    // name, not assume it matches any other model's/file's joint order (this
    // one comes from jsoncpp's getMemberNames(), which sorts alphabetically).
    const std::vector<std::string> &getJointname() const { return jointName_; }
    void printInfo();
    int getModel_nv () {return this->pin_model_.nv;}

    void getDataBus (DataBus& databus);

    JointController(double dt, const char *jsonPath, const char *urdfPath, bool verbose ); // constructor

    Eigen::VectorXd computeGq ();

private:
    std::vector<std::string> jointName_;
    Eigen::VectorXd q_, dq_, ddq_, qCmd_, dqCmd_, tauff_; // joint pos, vel, accel feedback

    std::map<std::string, pinocchio::JointIndex> JointMap_;

    Eigen::VectorXd maxTor_;
    Eigen::VectorXd maxVel_;
    Eigen::VectorXd maxPos_;
    Eigen::VectorXd minPos_;
    Eigen::VectorXd gear_;

    Eigen::VectorXd Kp_;
    Eigen::VectorXd Kd_;

    // pin model and data
    pinocchio::Model pin_model_;
    pinocchio::Data pin_data_;

    // Dynamic model terms
    Eigen::VectorXd dyn_G_;
};