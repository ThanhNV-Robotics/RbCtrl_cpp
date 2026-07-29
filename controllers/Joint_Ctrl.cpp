#include "Joint_Ctrl.h"

void JointController::printInfo()
{
    std::printf("Joint Controller Info:");
}

JointController::JointController(double dt, const char* jsonPath)
{
    // read joint pvt parameters
    Json::Reader reader;
    Json::Value root_read;
    std::ifstream in(jsonPath,std::ios::binary);

    reader.parse(in,root_read);

    jointName = root_read.getMemberNames(); // joint names come from the config file's top-level keys
    jointNum=jointName.size();

    tau_out_lpf.assign(jointNum,LPF_Fst());
    traj_pos_lpf.assign(jointNum,LPF_Fst());
    traj_vel_lpf.assign(jointNum,LPF_Fst());
    motor_vel.assign(jointNum,0);
    motor_pos_cur.assign(jointNum,0);
    motor_pos_des_old.assign(jointNum,0);
    motor_tor_out_link.assign(jointNum,0);
    motor_tor_out_motor.assign(jointNum,0);
    pvt_Kp.assign(jointNum,0);
    pvt_Kd.assign(jointNum,0);
    maxTor.assign(jointNum,400);
    maxVel.assign(jointNum,50);
    maxPos.assign(jointNum,3.14);
    minPos.assign(jointNum,-3.14);
    PV_enable.assign(jointNum,1);
    gear.assign(jointNum,1.0);

    // update controller parameter from json file
    for (int i=0;i<jointNum;i++){
        pvt_Kp[i]=root_read[motorName[i]]["kp"].asDouble();
        pvt_Kd[i]=root_read[motorName[i]]["kd"].asDouble();
        maxTor[i]=root_read[motorName[i]]["maxTorque"].asDouble();
        maxVel[i]=root_read[motorName[i]]["maxSpeed"].asDouble();
        maxPos[i]=root_read[motorName[i]]["maxPos"].asDouble();
        minPos[i]=root_read[motorName[i]]["minPos"].asDouble();
        double fc=root_read[motorName[i]]["PVT_LPF_Fc"].asDouble();
        gear[i] = root_read[motorName[i]]["gear"].asDouble();
        tau_out_lpf[i].setPara(fc, timeStepIn);
        tau_out_lpf[i].ftOut(0);

        const double trajLPF_Fc = 1.0; // cutoff (Hz) for smoothing genTestTrajectory's reference
        traj_pos_lpf[i].setPara(trajLPF_Fc, timeStepIn);
        traj_vel_lpf[i].setPara(trajLPF_Fc, timeStepIn);
        traj_pos_lpf[i].ftOut(0);
        traj_vel_lpf[i].ftOut(0);
    }
}