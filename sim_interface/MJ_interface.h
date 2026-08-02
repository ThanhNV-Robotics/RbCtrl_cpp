#pragma once

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <mujoco/mujoco.h>
#include <string>
#include <vector>
#include "json/json.h"
#include "data_bus.h"

// Struct to hold per-joint position, velocity, acceleration, and torque
struct JointState
{
    Eigen::VectorXd q;   // joint position
    Eigen::VectorXd dq;  // joint velocity
    Eigen::VectorXd ddq; // joint acceleration
    Eigen::VectorXd tau; // joint torque
};

struct JointInfo
{
    int mjJointId;
    std::string name;
    int qposAdr;
    int qvelAdr;
    int actuatorId;
};

class MJ_Interface
{
public:
    int jointNum{0};
    JointState jointState; // joint state feedback container

    std::vector<std::string> JointName = {}; // parsed from JSON config file

    MJ_Interface(mjModel *mj_modelIn, mjData *mj_dataIn, const char *jsonPath);
    
    void updateJointState(); // read joint states from simulator into jointState (q, dq, ddq, tau)
    void writeDataBus (DataBus& dataIn);
    void updateDataBus (DataBus& dataIn);
    // Torque command setters
    void setMotorsTorque(const Eigen::VectorXd &tauIn);


    // Getters returning Eigen::VectorXd references
    const Eigen::VectorXd &getJointPos() const { return jointState.q; }
    const Eigen::VectorXd &getJointVel() const { return jointState.dq; }
    const Eigen::VectorXd &getJointAccel() const { return jointState.ddq; }
    const Eigen::VectorXd &getJointTorque() const { return jointState.tau; }

    void printInfo();
    void printJointPos();
    void printJointInfo (const JointInfo& jointInfoIn);

private:
    mjModel *mj_model; // pointer to mjModel
    mjData *mj_data;   // pointer to mjData
    std::vector<JointInfo> jointInfo_;
};