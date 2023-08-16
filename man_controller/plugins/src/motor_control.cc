#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <man_controller/Traj.h>
#include <man_controller/FloatValue.h>
#include <std_msgs/Float32.h>
#include <boost/bind.hpp>
#include "ff_torque.h"
#include "gp.h"

namespace gazebo
{
  class ManipulatorPlugin : public ModelPlugin
  {

    private: 
      InvDynController InvDyn;
      GP_fit gp1;
      GP_fit gp2;
      GP_fit gp3;
      int flag = 0;
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
      ros::Publisher error_publisher;

      physics::JointPtr joint1;
      physics::JointPtr joint2;
      physics::JointPtr joint3;
      // physics::JointPtr joint4;
      man_controller::Traj joint_pos;
      man_controller::Traj joint_vel;
      man_controller::Traj joint_acc;
      float dt;
      Eigen::VectorXf torque;
      Eigen::VectorXf state;
      Eigen::VectorXf errors;
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

      this->error_publisher = this->rosNode->advertise<man_controller::FloatValue>("/error", 1);

      // gp1 = GP_fit::GP_fit();
      // gp2 = GP_fit::GP_fit();
      // gp3 = GP_fit::GP_fit();

      joint1 = this->model->GetJoint("base_link_link_01");
      joint2 = this->model->GetJoint("link_01_link_02");
      joint3 = this->model->GetJoint("link_02_link_03");
      // joint4 = this->model->GetJoint("link_03_link_04");

      this->dt = 0.1;
      std::cout<<"Works 1"<<std::endl;

      state = Eigen::VectorXf::Zero(9);
      errors = Eigen::VectorXf::Zero(0);
      // this->sub = this->rosNode->subscribe('/reference_trajectory', 1, &ManipulatorPlugin::callback, this);
    }

    public: void OnUpdate(){
      // std::cout<<"Print if working"<<std::endl;
      this->joint_pos = man_controller::Traj();

      float pos1 = this->joint1->Position(0);
      float pos2 = this->joint2->Position(0);
      float pos3 = this->joint3->Position(0);
      // float pos4 = this->joint4->Position(0);

      this->joint_pos.num1 = pos1;
      this->joint_pos.num2 = pos2;
      this->joint_pos.num3 = pos3;
      this->joint_pos.num4 = 0;
      this->joint_pos_publisher.publish(this->joint_pos);

      this->InvDyn.joint_pos << pos1, pos2, pos3;

      man_controller::Traj vel = man_controller::Traj();

      vel.num1 = this->joint1->GetVelocity(0);
      vel.num2 = this->joint2->GetVelocity(0);
      vel.num3 = this->joint3->GetVelocity(0);
      vel.num4 = 0;

      this->InvDyn.joint_vel << vel.num1, vel.num2, vel.num3;
      
      man_controller::Traj acc = man_controller::Traj();

      acc.num1 = (vel.num1 - this->joint_vel.num1)/dt;
      acc.num2 = (vel.num2 - this->joint_vel.num2)/dt;
      acc.num3 = (vel.num3 - this->joint_vel.num3)/dt;
      // acc.num4 = (vel.num4 - this->joint_vel.num4)/dt;
      acc.num4 = 0;

      this->InvDyn.joint_acc << acc.num1, acc.num2, acc.num3;

      this->joint_vel.num1 = vel.num1;
      this->joint_vel.num2 = vel.num2;
      this->joint_vel.num3 = vel.num3;
      this->joint_vel.num4 = 0;
      this->joint_vel_publisher.publish(this->joint_vel);    

      this->joint_acc_publisher.publish(acc);
      // joint1->SetForce(0, torque[0]);
      // joint2->SetForce(0, torque[1]);
      // joint3->SetForce(0, torque[2]);

      // Sampling and fitting at 0.1 Hz


      // std::cout<<"Current position: "<<std::endl<<this->InvDyn.joint_pos.transpose()<<std::endl;
      // this->torque = this->InvDyn.get_total_torque();
      // std::cout<<"Desired Position"<<std::endl<<this->InvDyn.joint_pos_ref.transpose()<< std::endl;
      // // std::cout<<"Torque applied"<<this->torque<<std::endl;
      // joint1->SetForce(0, torque[0]);
      // joint2->SetForce(0, torque[1]);
      // joint3->SetForce(0, torque[2]);
      
      // std::cout<<"Checkpoint 1"<<std::endl;
      state(0) = this-> InvDyn.joint_pos(0);
      state(1) = this-> InvDyn.joint_pos(1);
      state(2) = this-> InvDyn.joint_pos(2);
      state(3) = this-> InvDyn.joint_vel(0);
      state(4) = this-> InvDyn.joint_vel(1);
      state(5) = this-> InvDyn.joint_vel(2);
      // std::cout<<"Checkpoint 2"<<std::endl;
      Eigen::VectorXf eta_acc = this-> InvDyn.joint_acc_ref + this-> InvDyn.KP * (this-> InvDyn.joint_pos_ref - this-> InvDyn.joint_pos)  + this-> InvDyn.KD * (this-> InvDyn.joint_vel_ref - this-> InvDyn.joint_vel);
      state(6) = eta_acc(0); state(7) = eta_acc(1); state(8) = eta_acc(2);

      if(count%10 == 0 && count>0){
        std::cout<<"SAMPLING state is: "<<state<<std::endl;
        // Eigen::VectorXf eta_acc = this-> InvDyn.joint_acc_ref + this-> InvDyn.KP * (this-> InvDyn.joint_pos_ref - this-> InvDyn.joint_pos)  + this-> InvDyn.KD * (this-> InvDyn.joint_vel_ref - this-> InvDyn.joint_vel);
        // state(6) = eta_acc(0); state(7) = eta_acc(1); state(8) = eta_acc(2);

        Eigen::VectorXf obs = this-> InvDyn.joint_acc - eta_acc;
        // std::cout<<"Checkpoint 4"<<std::endl;
        gp1.add_data(state, obs(0));
        gp2.add_data(state, obs(1));
        gp3.add_data(state, obs(2));
      }

      // std::cout<<"Checkpoint 3"<<std::endl;

      Eigen::Vector3f mean;
      mean = Eigen::Vector3f::Zero();

      if(gp1.flag && gp2.flag && gp3.flag & !(count%10==0)){
        std::cout<<"State is: "<<state.transpose()<<std::endl;
        std::cout<<"K matrix"<<std::endl<<gp1.K<<std::endl;
        Eigen::VectorXf result1 = gp1.get_prediction(state);
        this->gp1.mean = result1(0);
        this->gp1.std_dev = result1(1);

        Eigen::VectorXf result2 = gp2.get_prediction(state);
        this->gp2.mean = result2(0);
        this->gp2.std_dev = result2(1);

        Eigen::VectorXf result3 = gp3.get_prediction(state);
        this->gp3.mean = result3(0);
        this->gp3.std_dev = result3(1);

        std::cout<<"Estimate for 1: "<<this->gp1.mean<<std::endl;
        std::cout<<"Estimate for 2: "<<this->gp2.mean<<std::endl;
        std::cout<<"Estimate for 3: "<<this->gp3.mean<<std::endl;
        
        mean(0) = gp1.mean;
        mean(1) = gp2.mean;
        mean(2) = gp3.mean;
        
      }

      std::cout<<"Current position: "<<std::endl<<this->InvDyn.joint_pos.transpose()<<std::endl;
      this->torque = this->InvDyn.get_total_torque(mean);
      std::cout<<"Desired Position"<<std::endl<<this->InvDyn.joint_pos_ref.transpose()<< std::endl;

      man_controller::FloatValue err = man_controller::FloatValue();

      Eigen::VectorXf error;
      error = this->InvDyn.joint_pos - this->InvDyn.joint_pos_ref;
      float err_norm = error.squaredNorm();

      err.value = err_norm;
      // float size = errors.size();
      // errors.conservativeResize(size+1);
      // errors(size) = err_norm;

      this->error_publisher.publish(err);

      // std::cout<<"Torque applied"<<this->torque<<std::endl;
      joint1->SetForce(0, torque[0]);
      joint2->SetForce(0, torque[1]);
      joint3->SetForce(0, torque[2]);

      // std::cout<<"Checkpoint 5"<<std::endl;

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