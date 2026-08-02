#include "Joint_Ctrl.h"



JointController::JointController(double dt, const char *jsonPath,
                                 const char *urdfPath, bool verbose) {
  // read joint pvt parameters
  Json::Reader reader;
  Json::Value root_read;
  std::ifstream in(jsonPath, std::ios::binary);

  reader.parse(in, root_read);

  jointName_ = root_read.getMemberNames(); // joint names come from the config
                                           // file's top-level keys
  jointNum = jointName_.size();

  Kp_.setZero(jointNum);
  Kd_.setZero(jointNum);
  maxTor_.setZero(jointNum);
  maxVel_.setZero(jointNum);
  maxPos_.setZero(jointNum);
  minPos_.setZero(jointNum);

  gear_.setZero(jointNum);

  // update controller parameter from json file
  for (int i = 0; i < jointNum; i++) {
    Kp_[i] = root_read[jointName_[i]]["kp"].asDouble();
    Kd_[i] = root_read[jointName_[i]]["kd"].asDouble();
    maxTor_[i] = root_read[jointName_[i]]["maxTorque"].asDouble();
    maxVel_[i] = root_read[jointName_[i]]["maxSpeed"].asDouble();
    maxPos_[i] = root_read[jointName_[i]]["maxPos"].asDouble();
    minPos_[i] = root_read[jointName_[i]]["minPos"].asDouble();
    double fc = root_read[jointName_[i]]["PVT_LPF_Fc"].asDouble();
    gear_[i] = root_read[jointName_[i]]["gear"].asDouble();
  }

  // process urdf file
  pinocchio::urdf::buildModel(urdfPath, this->pin_model_);
  this->pin_data_ = pinocchio::Data(this->pin_model_);

  // q_ must be nq-sized (Pinocchio config vector), initialized to neutral pose
  q_  = pinocchio::neutral(pin_model_);
  dq_.setZero(pin_model_.nv);
  ddq_.setZero(pin_model_.nv);

  // build JointMap
  for (pinocchio::JointIndex id = 0; id < pin_model_.njoints; ++id) {
    this->JointMap_[pin_model_.names[id]] = id;
  }

  if (verbose) {
    // printout pinocchio model info
    std::cout << "Robot pinocchio model infor\n";
    std::cout << "Model nv: " << pin_model_.nv << "\n";
    std::cout << "Robot Joint Map:\n";
    for (const auto &joint : this->JointMap_) {
      std::cout << "Joint name: " << joint.first << " | ID: " << joint.second
                << '\n';
    }
  }

  // Init dynamic terms value
  this->dyn_G_ = Eigen::VectorXd::Zero(pin_model_.nv); // gravity vector
}

void JointController::getDataBus(DataBus &databus)
{
    // Map each named joint's position/velocity from the DataBus
    // into the correct Pinocchio configuration/velocity slot via JointMap_.
    // jointName_[i] is sorted alphabetically (from JSON); DataBus.q is
    // also indexed in that same order, so we iterate by name.
    for (int i = 0; i < jointNum; i++)
    {
        const std::string &name = jointName_[i];
        auto it = JointMap_.find(name);
        if (it == JointMap_.end()) continue;

        pinocchio::JointIndex jid = it->second;
        // Pinocchio: joint config starts at pin_model_.joints[jid].idx_q()
        //            joint vel/acc starts at pin_model_.joints[jid].idx_v()
        int iq = pin_model_.idx_qs[jid];
        int iv = pin_model_.idx_vs[jid];

        if (i < (int)databus.q.size())  q_(iq)   = databus.q(i);
        if (i < (int)databus.dq.size()) dq_(iv)  = databus.dq(i);
        if (i < (int)databus.ddq.size()) ddq_(iv) = databus.ddq(i);
    }

    this->qCmd_  = databus.qCmd;
    this->dqCmd_ = databus.dqCmd;
    this->tauff_ = databus.tauff;
}

Eigen::VectorXd JointController::computeGq() {
  // dyn_G_ comes back in Pinocchio/URDF joint order; callers (e.g.
  // MJ_Interface::setMotorsTorque) index by jointName_ order (alphabetical,
  // from the JSON config), so remap through JointMap_ before returning.
  this->dyn_G_ = pinocchio::computeGeneralizedGravity(pin_model_, pin_data_, q_);

  Eigen::VectorXd Gq = Eigen::VectorXd::Zero(jointNum);
  for (int i = 0; i < jointNum; i++) {
    auto it = JointMap_.find(jointName_[i]);
    if (it == JointMap_.end()) continue;
    int iv = pin_model_.idx_vs[it->second];
    Gq[i] = this->dyn_G_(iv);
  }
  return Gq;
}

void JointController::printInfo() { std::printf("Joint Controller Info:"); }