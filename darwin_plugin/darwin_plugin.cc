#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/PID.hh"
// Author : Bharadwaj-Ramesh
// Date   : 8th March 2013
// Email  : br375@drexel.edu
 
// basic controll that inillizes all the joints to the initial position
// rename the plugin to darwin_plugin.cc and then build it 
namespace gazebo
{
  class PID1Joints : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      this->model_ = _model;
      // initialize a PID class
      this->target_position_ = 0.0;
// leg joints
      this->pid_j_ankle2_r.Init(100, 0, 0, 0, 0, 25, -25);
      this->pid_j_ankle2_r.SetCmd(this->target_position_);
      this->ankle2_r_joint_ = this->model_->GetJoint("j_ankle2_r");

      this->pid_j_ankle1_r.Init(100, 0, 0, 0, 0, 25, -25);
      this->pid_j_ankle1_r.SetCmd(this->target_position_);
      this->ankle1_r_joint_ = this->model_->GetJoint("j_ankle1_r");

      this->pid_j_tibia_r.Init(100, 0, 0, 0, 0, 25, -25);
      this->pid_j_tibia_r.SetCmd(this->target_position_);
      this->tibia_r_joint_ = this->model_->GetJoint("j_tibia_r");

      this->pid_j_thigh2_r.Init(100, 0, 0, 0, 0, 25, -25);
      this->pid_j_thigh2_r.SetCmd(this->target_position_);
      this->thigh2_r_joint_ = this->model_->GetJoint("j_thigh2_r");

      this->pid_j_thigh1_r.Init(100, 0, 0, 0, 0, 25, -25);
      this->pid_j_thigh1_r.SetCmd(this->target_position_);
      this->thigh1_r_joint_ = this->model_->GetJoint("j_thigh1_r");
// left leg
      this->pid_j_ankle2_l.Init(100, 0, 0, 0, 0, 25, -25);
      this->pid_j_ankle2_l.SetCmd(this->target_position_);
      this->ankle2_l_joint_ = this->model_->GetJoint("j_ankle2_l");

      this->pid_j_ankle1_l.Init(100, 0, 0, 0, 0, 25, -25);
      this->pid_j_ankle1_l.SetCmd(this->target_position_);
      this->ankle1_l_joint_ = this->model_->GetJoint("j_ankle1_l");

      this->pid_j_tibia_l.Init(100, 0, 0, 0, 0, 25, -25);
      this->pid_j_tibia_l.SetCmd(this->target_position_);
      this->tibia_l_joint_ = this->model_->GetJoint("j_tibia_l");

      this->pid_j_thigh2_l.Init(100, 0, 0, 0, 0, 25, -25);
      this->pid_j_thigh2_l.SetCmd(this->target_position_);
      this->thigh2_l_joint_ = this->model_->GetJoint("j_thigh2_l");

      this->pid_j_thigh1_l.Init(100, 0, 0, 0, 0, 25, -25);
      this->pid_j_thigh1_l.SetCmd(this->target_position_);
      this->thigh1_l_joint_ = this->model_->GetJoint("j_thigh1_l");
// left shoulder and upper limb
      this->pid_j_shoulder_l.Init(100, 0, 0, 0, 0, 25, -25);
      this->pid_j_shoulder_l.SetCmd(this->target_position_);
      this->shoulder_l_joint_ = this->model_->GetJoint("j_shoulder_l");

      this->pid_j_high_arm_l.Init(100, 0, 0, 0, 0, 25, -25);
      this->pid_j_high_arm_l.SetCmd(this->target_position_);
      this->high_arm_l_joint_ = this->model_->GetJoint("j_high_arm_l");

      this->pid_j_low_arm_l.Init(100, 0, 0, 0, 0, 25, -25);
      this->pid_j_low_arm_l.SetCmd(this->target_position_);
      this->low_arm_l_joint_ = this->model_->GetJoint("j_low_arm_l");

      this->pid_j_wrist_l.Init(100, 0, 0, 0, 0, 25, -25);
      this->pid_j_wrist_l.SetCmd(this->target_position_);
      this->wrist_l_joint_ = this->model_->GetJoint("j_wrist_l");

      this->pid_j_gripper_l.Init(100, 0, 0, 0, 0, 25, -25);
      this->pid_j_gripper_l.SetCmd(this->target_position_);
      this->gripper_l_joint_ = this->model_->GetJoint("j_gripper_l");

// right hand

      this->pid_j_shoulder_r.Init(100, 0, 0, 0, 0, 25, -25);
      this->pid_j_shoulder_r.SetCmd(this->target_position_);
      this->shoulder_r_joint_ = this->model_->GetJoint("j_shoulder_r");

      this->pid_j_high_arm_r.Init(100, 0, 0, 0, 0, 25, -25);
      this->pid_j_high_arm_r.SetCmd(this->target_position_);
      this->high_arm_r_joint_ = this->model_->GetJoint("j_high_arm_r");

      this->pid_j_low_arm_r.Init(100, 0, 0, 0, 0, 25, -25);
      this->pid_j_low_arm_r.SetCmd(this->target_position_);
      this->low_arm_r_joint_ = this->model_->GetJoint("j_low_arm_r");

      this->pid_j_wrist_r.Init(100, 0, 0, 0, 0, 25, -25);
      this->pid_j_wrist_r.SetCmd(this->target_position_);
      this->wrist_r_joint_ = this->model_->GetJoint("j_wrist_r");

      this->pid_j_gripper_r.Init(100, 0, 0, 0, 0, 25, -25);
      this->pid_j_gripper_r.SetCmd(this->target_position_);
      this->gripper_r_joint_ = this->model_->GetJoint("j_gripper_r");

      this->pid_j_pan.Init(100, 0, 0, 0, 0, 25, -25);
      this->pid_j_pan.SetCmd(this->target_position_);
      this->pan_joint_ = this->model_->GetJoint("j_pan");

      this->pid_j_tilt.Init(100, 0, 0, 0, 0, 25, -25);
      this->pid_j_tilt.SetCmd(this->target_position_);
      this->tilt_joint_ = this->model_->GetJoint("j_tilt");

      this->pid_j_pelvis_l.Init(100, 0, 0, 0, 0, 25, -25);
      this->pid_j_pelvis_l.SetCmd(this->target_position_);
      this->pelvis_l_joint_ = this->model_->GetJoint("j_pelvis_l");

      this->pid_j_pelvis_r.Init(100, 0, 0, 0, 0, 25, -25);
      this->pid_j_pelvis_r.SetCmd(this->target_position_);
      this->pelvis_r_joint_ = this->model_->GetJoint("j_pelvis_r");

      this->last_update_time_ = this->model_->GetWorld()->GetSimTime();
      this->update_connection_ = event::Events::ConnectWorldUpdateStart(
        boost::bind(&PID1Joints::UpdatePID, this));
    }
    void UpdatePID()
    {
      common::Time current_time = this->model_->GetWorld()->GetSimTime();
      double error = this->ankle2_r_joint_->GetAngle(0).Radian()
                   - target_position_;
      double dt    = current_time.Double()
                   - this->last_update_time_.Double();
      this->pid_j_ankle2_r.Update(error, dt);
      this->ankle2_r_joint_->SetForce(0, this->pid_j_ankle2_r.GetCmd());
      gzdbg << "error [" << error
            << "] cmd [" << this->pid_j_ankle2_r.GetCmd() << "]\n";
      error = this->ankle1_r_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_j_ankle1_r.Update(error, dt);
      this->ankle1_r_joint_->SetForce(0, this->pid_j_ankle1_r.GetCmd());

      error = this->tibia_r_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_j_tibia_r.Update(error, dt);
      this->tibia_r_joint_->SetForce(0, this->pid_j_tibia_r.GetCmd());

      error = this->thigh2_r_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_j_thigh2_r.Update(error, dt);
      this->thigh2_r_joint_->SetForce(0, this->pid_j_thigh2_r.GetCmd());

      error = this->thigh1_r_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_j_thigh1_r.Update(error, dt);
      this->thigh1_r_joint_->SetForce(0, this->pid_j_thigh1_r.GetCmd());

// left leg
      error = this->ankle2_l_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_j_ankle2_l.Update(error, dt);
      this->ankle2_l_joint_->SetForce(0, this->pid_j_ankle2_l.GetCmd());

      error = this->ankle1_l_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_j_ankle1_l.Update(error, dt);
      this->ankle1_l_joint_->SetForce(0, this->pid_j_ankle1_l.GetCmd());

      error = this->tibia_l_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_j_tibia_l.Update(error, dt);
      this->tibia_l_joint_->SetForce(0, this->pid_j_tibia_l.GetCmd());

      error = this->thigh2_l_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_j_thigh2_l.Update(error, dt);
      this->thigh2_l_joint_->SetForce(0, this->pid_j_thigh2_l.GetCmd());

      error = this->thigh1_l_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_j_thigh1_l.Update(error, dt);
      this->thigh1_l_joint_->SetForce(0, this->pid_j_thigh1_l.GetCmd());

// left arm
      error = this->shoulder_l_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_j_shoulder_l.Update(error, dt);
      this->shoulder_l_joint_->SetForce(0, this->pid_j_shoulder_l.GetCmd());


      error = this->high_arm_l_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_j_high_arm_l.Update(error, dt);
      this->high_arm_l_joint_->SetForce(0, this->pid_j_high_arm_l.GetCmd());

      error = this->low_arm_l_joint_->GetAngle(0).Radian()
                   - target_position_ ;
      this->pid_j_low_arm_l.Update(error, dt);
      this->low_arm_l_joint_->SetForce(0, this->pid_j_low_arm_l.GetCmd());


      error = this->wrist_l_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_j_wrist_l.Update(error, dt);
      this->wrist_l_joint_->SetForce(0, this->pid_j_wrist_l.GetCmd());
      

      
      error = this->gripper_l_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_j_gripper_l.Update(error, dt);
      this->gripper_l_joint_->SetForce(0, this->pid_j_gripper_l.GetCmd());


// right arm 
      error = this->shoulder_r_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_j_shoulder_r.Update(error, dt);
      this->shoulder_r_joint_->SetForce(0, this->pid_j_shoulder_r.GetCmd());


      error = this->high_arm_r_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_j_high_arm_r.Update(error, dt);
      this->high_arm_r_joint_->SetForce(0, this->pid_j_high_arm_r.GetCmd());

      error = this->low_arm_r_joint_->GetAngle(0).Radian()
                   - target_position_ ;
      this->pid_j_low_arm_r.Update(error, dt);
      this->low_arm_r_joint_->SetForce(0, this->pid_j_low_arm_r.GetCmd());


      error = this->wrist_r_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_j_wrist_r.Update(error, dt);
      this->wrist_r_joint_->SetForce(0, this->pid_j_wrist_r.GetCmd());
      

      
      error = this->gripper_r_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_j_gripper_r.Update(error, dt);
      this->gripper_r_joint_->SetForce(0, this->pid_j_gripper_r.GetCmd());
// head and pelvis 

      error = this->pan_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_j_pan.Update(error, dt);
      this->pan_joint_->SetForce(0, this->pid_j_pan.GetCmd());

      error = this->tilt_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_j_tilt.Update(error, dt);
      this->tilt_joint_->SetForce(0, this->pid_j_tilt.GetCmd());

      error = this->pelvis_l_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_j_pelvis_l.Update(error, dt);
      this->pelvis_l_joint_->SetForce(0, this->pid_j_pelvis_l.GetCmd());

      error = this->pelvis_r_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_j_pelvis_r.Update(error, dt);
      this->pelvis_r_joint_->SetForce(0, this->pid_j_pelvis_r.GetCmd());


      this->last_update_time_ = current_time;

    }


// PID controllers for each joint :

// head

    common::PID pid_j_pan;
    common::PID pid_j_tilt;

// pelvis

    common::PID pid_j_pelvis_l;
    common::PID pid_j_pelvis_r;


// leg joints
    common::PID pid_j_ankle2_r;
    common::PID pid_j_ankle1_r;
    common::PID pid_j_tibia_r;
    common::PID pid_j_thigh2_r;
    common::PID pid_j_thigh1_r;

    common::PID pid_j_ankle2_l;
    common::PID pid_j_ankle1_l;
    common::PID pid_j_tibia_l;
    common::PID pid_j_thigh2_l;
    common::PID pid_j_thigh1_l;

// left arm 
    common::PID pid_j_shoulder_l;
    common::PID pid_j_high_arm_l;
    common::PID pid_j_low_arm_l;
    common::PID pid_j_wrist_l;
    common::PID pid_j_gripper_l;

    common::PID pid_j_shoulder_r;
    common::PID pid_j_high_arm_r;
    common::PID pid_j_low_arm_r;
    common::PID pid_j_wrist_r;
    common::PID pid_j_gripper_r;

// pid controllers for the leg joints

    double target_position_;
    double k_;

// ** Pointers for each joints

// head
    physics::JointPtr pan_joint_;
    physics::JointPtr tilt_joint_;

// pelvis
    physics::JointPtr pelvis_l_joint_;
    physics::JointPtr pelvis_r_joint_;


// legs
    physics::JointPtr ankle2_r_joint_;
    physics::JointPtr ankle1_r_joint_;
    physics::JointPtr tibia_r_joint_;
    physics::JointPtr thigh2_r_joint_;
    physics::JointPtr thigh1_r_joint_;

    physics::JointPtr ankle2_l_joint_;
    physics::JointPtr ankle1_l_joint_;
    physics::JointPtr tibia_l_joint_;
    physics::JointPtr thigh2_l_joint_;
    physics::JointPtr thigh1_l_joint_;

// arm 

    physics::JointPtr shoulder_l_joint_;
    physics::JointPtr high_arm_l_joint_;
    physics::JointPtr low_arm_l_joint_;
    physics::JointPtr wrist_l_joint_;
    physics::JointPtr gripper_l_joint_;

    physics::JointPtr shoulder_r_joint_;
    physics::JointPtr high_arm_r_joint_;
    physics::JointPtr low_arm_r_joint_;
    physics::JointPtr wrist_r_joint_;
    physics::JointPtr gripper_r_joint_;

    physics::ModelPtr model_;
    event::ConnectionPtr update_connection_;
    common::Time last_update_time_;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(PID1Joints)
}
