#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <man_controller/Traj.h>
#include <boost/bind.hpp>
#include "ff_torque.h"
#include "gp.h"

namespace gazebo
{
  class ManipulatorPlugin : public ModelPlugin
  {

    private: 
      InvDynController InvDyn;
      int count = 0;
      physics::ModelPtr model;
      event::ConnectionPtr updateConnection;
      ros::NodeHandle* rosNode;
      ros::Subscriber torque_sub;
      ros::Subscriber ref_pos_sub;
      ros::Subscriber ref_vel_sub;
      ros::Subscriber ref_acc_sub;
      ros::Publisher joint_pos_publisher;
      ros::Publisher joint_vel_publisher;
      ros::Publisher joint_acc_publisher;
      physics::JointPtr joint1;
      physics::JointPtr joint2;
      physics::JointPtr joint3;
      physics::JointPtr joint4;
      man_controller::Traj joint_pos;
      man_controller::Traj joint_vel;
      man_controller::Traj joint_acc;
      float dt;
      Eigen::VectorXf torque;
      gazebo::common::PID pid = gazebo::common::PID(1000000, 10, 10);

    public: ManipulatorPlugin() {
      if(!ros::isInitialized()){
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
      }

      this->rosNode = new ros::NodeHandle();
      // this->InvDyn = InvDynController::InvDynController();

    }

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      this->model = _model;


      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ManipulatorPlugin::OnUpdate, this)
      );

      // this->torque_sub = this->rosNode->subscribe<man_controller::Traj>("/torque_values", 1, boost::bind(&ManipulatorPlugin::callback, this, _1), ros::VoidPtr(), ros::TransportHints());
      this->ref_pos_sub = this->rosNode->subscribe<man_controller::Traj>("/position_reference", 1, boost::bind(&ManipulatorPlugin::posCallback, this, _1), ros::VoidPtr(), ros::TransportHints());
      this->ref_vel_sub = this->rosNode->subscribe<man_controller::Traj>("/velocity_reference", 1, boost::bind(&ManipulatorPlugin::velCallback, this, _1), ros::VoidPtr(), ros::TransportHints());
      this->ref_acc_sub = this->rosNode->subscribe<man_controller::Traj>("/acceleration_reference", 1, boost::bind(&ManipulatorPlugin::accCallback, this, _1), ros::VoidPtr(), ros::TransportHints());
      
      this->joint_pos_publisher = this->rosNode->advertise<man_controller::Traj>("/joint_pos_publisher", 5);
      this->joint_vel_publisher = this->rosNode->advertise<man_controller::Traj>("/joint_vel_publisher", 5);
      this->joint_acc_publisher = this->rosNode->advertise<man_controller::Traj>("/joint_acc_publisher", 5);

      


      joint1 = this->model->GetJoint("base_link_link_01");
      joint2 = this->model->GetJoint("link_01_link_02");
      joint3 = this->model->GetJoint("link_02_link_03");
      joint4 = this->model->GetJoint("link_03_link_04");

      this->dt = 0.1;
      std::cout<<"Works 1"<<std::endl;
      // this->sub = this->rosNode->subscribe('/reference_trajectory', 1, &ManipulatorPlugin::callback, this);
    }

    public: void OnUpdate(){
      // std::cout<<"Print if working"<<std::endl;
      this->joint_pos = man_controller::Traj();

      float pos1 = this->joint1->Position(0);
      float pos2 = this->joint2->Position(0);
      float pos3 = this->joint3->Position(0);
      float pos4 = this->joint4->Position(0);

      this->joint_pos.num1 = pos1;
      this->joint_pos.num2 = pos2;
      this->joint_pos.num3 = pos3;
      this->joint_pos.num4 = pos4;
      this->joint_pos_publisher.publish(this->joint_pos);

      this->InvDyn.joint_pos << pos1, pos2, pos3;

      man_controller::Traj vel = man_controller::Traj();

      vel.num1 = this->joint1->GetVelocity(0);
      vel.num2 = this->joint2->GetVelocity(0);
      vel.num3 = this->joint3->GetVelocity(0);
      vel.num4 = this->joint4->GetVelocity(0);

      this->InvDyn.joint_vel << vel.num1, vel.num2, vel.num3;
      
      man_controller::Traj acc = man_controller::Traj();

      acc.num1 = (vel.num1 - this->joint_vel.num1)/dt;
      acc.num2 = (vel.num2 - this->joint_vel.num2)/dt;
      acc.num3 = (vel.num3 - this->joint_vel.num3)/dt;
      acc.num4 = (vel.num4 - this->joint_vel.num4)/dt;

      this->InvDyn.joint_acc << acc.num1, acc.num2, acc.num3;

      this->joint_vel.num1 = this->joint1->GetVelocity(0);
      this->joint_vel.num2 = this->joint2->GetVelocity(0);
      this->joint_vel.num3 = this->joint3->GetVelocity(0);
      this->joint_vel.num4 = this->joint4->GetVelocity(0);
      this->joint_vel_publisher.publish(this->joint_vel);    

      this->joint_acc_publisher.publish(acc);
      // joint1->SetForce(0, torque[0]);
      // joint2->SetForce(0, torque[1]);
      // joint3->SetForce(0, torque[2]);

      // Sampling and fitting at 0.1 Hz
      if(count%10 == 0){
        
      }

      std::cout<<"Current position: "<<this->InvDyn.joint_pos<<std::endl;
      this->torque = this->InvDyn.get_total_torque();
      std::cout<<"Desired Position"<<this->InvDyn.joint_pos_ref<< std::endl;
      std::cout<<"Torque applied"<<this->torque<<std::endl;
      joint1->SetForce(0, torque[0]);
      joint2->SetForce(0, torque[1]);
      joint3->SetForce(0, torque[2]);
      
      count++;

      ros::spinOnce();
    }

    // public: ignition::math::Vector3d format(data){
    //   return torque;
    // }

    public: void posCallback(const man_controller::Traj::ConstPtr& msg){
      
      float joint1_pos = msg->num1;
      float joint2_pos = msg->num2;
      float joint3_pos = msg->num3;

      // PD Controller over the position
      double dt = 0.01;

      this->InvDyn.joint_pos_ref << joint1_pos, joint2_pos, joint3_pos;


      // double error1 = joint1_pos - current_pos1;
      // double error2 = joint2_pos - current_pos2;
      // double error3 = joint3_pos - current_pos3;

      // double cmd1 = pid.Update(error1, dt);
      // double cmd2 = pid.Update(error2, dt);
      // double cmd3 = pid.Update(error3, dt);

      // joint1->SetForce(0, cmd1);
      // joint2->SetForce(0, cmd2);
      // joint3->SetForce(0, cmd3);

    }


    public: void velCallback(const man_controller::Traj::ConstPtr& msg){
      
      float joint1_vel = msg->num1;
      float joint2_vel = msg->num2;
      float joint3_vel = msg->num3;

      this->InvDyn.joint_vel_ref << joint1_vel, joint2_vel, joint3_vel;

    }

    public: void accCallback(const man_controller::Traj::ConstPtr& msg){
      
      float joint1_acc = msg->num1;
      float joint2_acc = msg->num2;
      float joint3_acc = msg->num3;

      this->InvDyn.joint_acc_ref << joint1_acc, joint2_acc, joint3_acc;
    }

  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(ManipulatorPlugin)
}