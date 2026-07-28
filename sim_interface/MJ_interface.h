#pragma once

#include <mujoco/mujoco.h>
#include <fstream>
#include <string>
#include <vector>
#include "json/json.h" // to process json config file
#include <iostream>

// This class use to read robot states (sensors, motor states) or write control force from/to mujoco simulator

class MJ_Interface
{

public:
    int jointNum{0};
    std::vector<double> joint_pos;
    std::vector<double> joint_pos_Old;
    std::vector<double> joint_vel;
    std::vector<double> joint_accel;
    std::vector<double> joint_torque;

    std::vector<std::string> JointName = {}; // this will be initialized with constructor by parsing the json config file

    MJ_Interface(mjModel *mj_modelIn, mjData *mj_dataIn, const char *jsonPath); // constructor
    void updateSensorValues(); // get motor joint states 
    void setMotorsTorque(std::vector<double> &tauIn); // set joint torque
    void printInfo ();
    void printJointPos ();

    std::vector<double> getJointPos ();
    std::vector<double> getJointVel ();
    std::vector<double> getJointAccel ();
    std::vector<double> getJointTorque ();


private:
    mjModel *mj_model; // pointer to mjModel struct to read/write the data
    mjData *mj_data;   // pointer to mjData struct to read/write the data
    std::vector<int> jntId_qpos, jntId_qvel, jntId_dctl; // joint id to read position, vel, acceleration, torque
};