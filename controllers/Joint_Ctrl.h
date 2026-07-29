#pragma once
#include <fstream>
#include "json/json.h"
#include <vector>
#include <cmath>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/aba.hpp"

class JointController
{
public:
    int jointNum;
    std::vector<double> motor_pos_cur;
    std::vector<double> motor_pos_des_old;
    std::vector<double> motor_vel;
    std::vector<double> motor_tor_out_link;  // final tau output
    std::vector<double> motor_tor_out_motor; // final tau output

    // Joint order matching motor_pos_des/motor_vel_des/motor_tor_des and every
    // other per-joint vector below -- callers must map onto this order by
    // name, not assume it matches any other model's/file's joint order (this
    // one comes from jsoncpp's getMemberNames(), which sorts alphabetically).
    const std::vector<std::string> &getMotorNames() const { return motorName; }

    std::vector<double> motor_pos_des; // P des
    std::vector<double> motor_vel_des; // V des
    std::vector<double> motor_tor_des; // T des

    std::vector<double> pvt_Kp;
    std::vector<double> pvt_Kd;
    std::vector<double> maxTor;
    std::vector<double> maxVel;
    std::vector<double> maxPos;
    std::vector<double> minPos;
    std::vector<double> gear;
    void printInfo();
    JointController(double dt, const char *jsonPath);

private:
    std::vector<std::string> jointName;
};