// Use pinocchio to calculate rigid body dynamics
#include <mujoco/mujoco.h>
#include <cstdio>
#include <iostream>
#include <string>
#include "GLFW_callbacks.h"


const std::string MODEL_DIR = "models/mjcf"; // path to mujoco xml model

int main (int argc, char** argv)
{
    std::cout<< "Program Starts, Loading Mujoco xml model\n";

    //-------------------------------------------------------------------
    // Compile mujoco xml model
    //-------------------------------------------------------------------

    const std::string model_path = MODEL_DIR + "/right_leg_scene.xml";
    //const std::string model_path = (argc > 1) ? argv[1] : model_path; // if input model path in the arg then use that path
    std::cout<< "Input model path: " + model_path + "\n";
    char loadError[1024] = ""; // character array, size 1024

    // load/compile xml model
    // model_path.c_str() return a read-only pointer to const std::string model_path
    mjModel* mj_model = mj_loadXML(model_path.c_str(), nullptr, loadError, sizeof(loadError));

    // check if loading xml is ok or not
    if (!mj_model){ // if mj_model is a nullptr
        std::fprintf(stderr, "failed to load %s: %s\n", model_path.c_str(), loadError);
    return 1; // stop program
    }

    std::printf("Load xml successful \n");

    mjData* mj_data = mj_makeData(mj_model); // pointer to mjData struct

    //-------------------------------------------------------------------
    // Init Joint Controller
    //-------------------------------------------------------------------
    
    //-------------------------------------------------------------------
    // Init classes
    //-------------------------------------------------------------------

    const std::string joint_ctrl_config_path = "config/right_joint_ctrl_config.json";


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

    while (!glfwWindowShouldClose(ui.window) && simTime <= sim_duration)
    {
        simStart = mj_data->time;
        while (mj_data->time - simStart < 1.0 / 60.0 && ui.runSim) // ensure the rendering in real-time
        {
            ui.applyPerturbation();
            mj_step(mj_model, mj_data);

            simTime = mj_data->time;
        }
        ui.updateScene(); //
    }

    ui.Close(); // close and delete variables

    return 0;
}