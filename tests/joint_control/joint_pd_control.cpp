// Use pinocchio to calculate rigid body dynamics
#include <Eigen/Dense>
#include <mujoco/mujoco.h>
#include <cstdio>
#include <iostream>
#include <string>
#include "GLFW_callbacks.h"
#include "Joint_Ctrl.h"
#include "MJ_interface.h"
#include "data_bus.h"

bool loadXmlModel(const std::string &model_path, mjModel* &model, mjData* &data)
{
    model = nullptr;
    data = nullptr;

    std::cout << "Loading MuJoCo XML model: " << model_path << "\n";
    char loadError[1024] = "";

    model = mj_loadXML(model_path.c_str(), nullptr, loadError, sizeof(loadError));
    if (!model) {
        std::cerr << "Failed to load XML model from " << model_path << ": " << loadError << "\n";
        return false;
    }

    data = mj_makeData(model);
    if (!data) {
        std::cerr << "Failed to create mjData for model: " << model_path << "\n";
        mj_deleteModel(model);
        model = nullptr;
        return false;
    }

    std::cout << "Successfully loaded XML model!\n";
    return true;
}

const std::string MODEL_DIR = "models/mjcf"; // path to mujoco xml model
const std::string URDF_FILE_PATH = "models/urdf/ur10e.urdf";
const std::string JSON_FILE_PATH = "controllers/config/ur10_joint_pdgain.json";

int main (int argc, char** argv)
{
    std::cout << "Program Starts, Loading Mujoco xml model\n";

    //-------------------------------------------------------------------
    // Compile mujoco xml model
    //-------------------------------------------------------------------
    const std::string model_path = MODEL_DIR + "/scene_ur10.xml";

    mjModel* mj_model = nullptr;
    mjData* mj_data = nullptr;

    if (!loadXmlModel(model_path, mj_model, mj_data)) {
        return 1;
    }

    const double dt = mj_model->opt.timestep; // time step in simulator

    //-------------------------------------------------------------------
    // Compile urdf model
    //-------------------------------------------------------------------
    JointController rb_jointController (dt,JSON_FILE_PATH.c_str(), URDF_FILE_PATH.c_str(), true);
    Eigen::VectorXd Gq = Eigen::VectorXd::Zero(rb_jointController.jointNum);

    //-------------------------------------------------------------------
    // MuJoCo Interface
    //-------------------------------------------------------------------
    MJ_Interface mj_interface (mj_model, mj_data, JSON_FILE_PATH.c_str());
    mj_interface.printInfo();

    // Data bus
    DataBus RobotState(rb_jointController.getModel_nv());
    
    // Init mujoco UI from GLFW_callbacks
    UIctr ui(mj_model, mj_data);
    ui.iniGLFW();
    ui.disableTracking();
    ui.createWindow(model_path.c_str(), /*saveVideo=*/false);
    

    // Sim loop
    double simStart = mj_data->time;
    double simTime = mj_data->time;
    const double sim_duration = 10;
    // const int printingFreq = 100;
    int count = 0;

    while (!glfwWindowShouldClose(ui.window) )
    {
        simStart = mj_data->time;
        while (mj_data->time - simStart < 1.0 / 60.0 && ui.runSim) // ensure the rendering in real-time
        {
            ui.applyPerturbation();
            mj_step(mj_model, mj_data);

            mj_interface.updateDataBus(RobotState); // update robot state from mujoco simulator data

            rb_jointController.getDataBus(RobotState); // joint controller updates data from data bus

            RobotState.tauCmd = rb_jointController.computeGq();
            // compute gravity vector
            // Gq = rb_jointController.computeGq();

            // apply to joint torque
            mj_interface.setMotorsTorque(RobotState.tauCmd);

            simTime = mj_data->time;
        }
        ui.updateScene(); //
    }

    ui.Close(); // close and delete variables

    // Free MuJoCo model and data
    mj_deleteData(mj_data);
    mj_deleteModel(mj_model);

    return 0;
}