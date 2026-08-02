#include "MJ_interface.h"
#include <cstdio>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "data_bus.h"

// constructor
MJ_Interface::MJ_Interface(mjModel *mj_modelIn, mjData *mj_dataIn, const char* jsonPath)
{
    this->mj_model = mj_modelIn;
    this->mj_data = mj_dataIn;

    // Read json file
    Json::Reader reader;
    Json::Value root_read;
    std::ifstream in(jsonPath, std::ios::binary);

    reader.parse(in, root_read);

    JointName = root_read.getMemberNames(); // joint names come from the config file's top-level keys
    this->jointNum = JointName.size();

    // Initialize Eigen vectors in JointState
    jointState.q = Eigen::VectorXd::Zero(jointNum);
    jointState.dq = Eigen::VectorXd::Zero(jointNum);
    jointState.ddq = Eigen::VectorXd::Zero(jointNum);
    jointState.tau = Eigen::VectorXd::Zero(jointNum);

    // build jointInfo_, ordered to match JointName (from the JSON config) so
    // that jointInfo_[i] lines up with jointState.q[i]/dq[i]/etc.
    for (int i = 0; i < this->jointNum; i++)
    {
        JointInfo info;

        int jid = mj_name2id(this->mj_model, mjOBJ_JOINT, JointName[i].c_str());
        if (jid < 0) {
            std::printf("MJ_Interface: joint '%s' not found in mujoco model\n", JointName[i].c_str());
            continue;
        }

        info.mjJointId = jid;
        info.name = JointName[i];
        info.qposAdr = this->mj_model->jnt_qposadr[jid];
        info.qvelAdr = this->mj_model->jnt_dofadr[jid];

        // find the actuator that drives this joint
        info.actuatorId = -1;
        for (int a = 0; a < this->mj_model->nu; a++)
        {
            if (this->mj_model->actuator_trntype[a] == mjTRN_JOINT &&
                this->mj_model->actuator_trnid[2 * a] == jid)
            {
                info.actuatorId = a;
                break;
            }
        }
        if (info.actuatorId < 0) {
            std::printf("MJ_Interface: no actuator found for joint '%s'\n", JointName[i].c_str());
        }

        this->jointInfo_.push_back(info);
    }
}

void MJ_Interface::updateJointState()
{
    // update joint state
    for (int i = 0; i < this->jointNum; i++)
    {
        const auto& joint = this->jointInfo_[i];
        
        this->jointState.q[i] = mj_data->qpos[joint.qposAdr];
        this->jointState.dq[i] = mj_data->qvel[joint.qvelAdr];
        this->jointState.ddq[i] = mj_data->qacc[joint.qvelAdr];
        this->jointState.tau[i] = mj_data->qfrc_actuator[joint.qvelAdr];
    }
}


void MJ_Interface::setMotorsTorque(const Eigen::VectorXd &tauIn)
{
    if(tauIn.size() != this->jointNum)
    {
        std::printf("MjInterface::setMotorsTorque-> size not match");
        return;
    }
    // matching joint torque using jointMap
    for (int i = 0; i < this->jointNum; i++)
    {
        const auto& joint = this->jointInfo_[i];
        //send torque cmd to mujoco
        this->mj_data->ctrl[joint.actuatorId] = tauIn[i];
    }
}

void MJ_Interface::writeDataBus (DataBus& dataIn)
{
    dataIn.q = this->getJointPos();
    dataIn.dq = this->getJointVel();
    dataIn.ddq = this->getJointAccel();
    dataIn.tau = this->getJointTorque();
}

void MJ_Interface::updateDataBus (DataBus& dataIn)
{
    this->updateJointState();
    this->writeDataBus(dataIn);
}

// Printing out function
void MJ_Interface::printInfo()
{
    std::printf("Number of joint: %d\n", this->jointNum);
    std::printf("List of joints:\n");

    for (int i = 0; i < this->jointNum; i++)
    {
        const auto& joint = this->jointInfo_[i];
        printJointInfo(joint);
    }
}

void MJ_Interface::printJointPos()
{
    for (int i = 0; i < this->jointNum; i++)
    {
        std::printf("Joint %-25s position: %.3f\n", this->JointName[i].c_str(), jointState.q(i));
    }
}

void MJ_Interface::printJointInfo (const JointInfo& jointInfoIn)
{
    std::cout<<"Joint ID: "<<jointInfoIn.mjJointId<<"\n";
    std::cout<<"Joint name: "<<jointInfoIn.name<<"\n";
    std::cout<<"qpos address: "<<jointInfoIn.qposAdr<<"\n";
    std::cout<<"qvel address: "<<jointInfoIn.qvelAdr<<"\n";
}
