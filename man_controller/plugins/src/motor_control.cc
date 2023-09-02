#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <man_controller/Traj.h>
#include <man_controller/FloatValue.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <man_controller/FloatArray.h>
#include <boost/bind.hpp>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
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
      ros::Subscriber gp_pred;
      ros::Subscriber gp_corr;
      ros::Time savedTimestamp;

      ros::Publisher joint_pos_publisher;
      ros::Publisher joint_vel_publisher;
      ros::Publisher joint_acc_publisher;
      ros::Publisher error_publisher;
      ros::Publisher gp_state_publisher;
      ros::Publisher error_state_publisher;
      ros::Publisher pred_error_publisher;
      ros::Publisher training_data_publisher;

      ros::Publisher gp_observations_publisher;

      physics::JointPtr joint1;
      physics::JointPtr joint2;
      physics::JointPtr joint3;

      physics::LinkPtr link1;
      physics::LinkPtr link2;
      physics::LinkPtr link3;
      // physics::JointPtr joint4;
      man_controller::Traj joint_pos;
      man_controller::Traj joint_vel;
      man_controller::Traj joint_acc;
      float dt;
      Eigen::VectorXf torque;
      Eigen::VectorXf state;
      Eigen::VectorXf errors;
      Eigen::VectorXf gp_mean;

      Eigen::VectorXf corr;

      // std::vector<float> gp_mean(3);
      // std::vector<float> gp_stddev(3);

      float accumulated_error = 0;
      int count_rms = 0;

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
      this->gp_pred = this->rosNode->subscribe<man_controller::FloatArray>("/gp_predictions", 1, boost::bind(&ManipulatorPlugin::predCallback, this, _1), ros::VoidPtr(), ros::TransportHints());
      this->gp_corr = this->rosNode->subscribe<man_controller::FloatArray>("/gp_corrections", 1, boost::bind(&ManipulatorPlugin::corrCallback, this, _1), ros::VoidPtr(), ros::TransportHints());


      this->error_state_publisher = this->rosNode->advertise<std_msgs::Float64MultiArray>("/error_states", 1);
      this->joint_pos_publisher = this->rosNode->advertise<man_controller::Traj>("/joint_pos_publisher", 5);
      this->joint_vel_publisher = this->rosNode->advertise<man_controller::Traj>("/joint_vel_publisher", 5);
      this->joint_acc_publisher = this->rosNode->advertise<man_controller::Traj>("/joint_acc_publisher", 5);
      this->gp_state_publisher = this->rosNode->advertise<man_controller::FloatArray>("/states_topic", 1);
      this->pred_error_publisher = this->rosNode->advertise<man_controller::FloatValue>("/pred_error", 1);
      this->training_data_publisher = this->rosNode->advertise<man_controller::FloatArray>("/train_data", 1); 

      // this->gp_observations_publisher = this->rosNode->advertise<std_msgs::Float64MultiArray>("/observations_topic", 1);
      this->gp_observations_publisher = this->rosNode->advertise<man_controller::FloatArray>("/observations_topic", 1);

      this->error_publisher = this->rosNode->advertise<man_controller::FloatValue>("/error", 1);

      // gp1 = GP_fit::GP_fit();
      // gp2 = GP_fit::GP_fit();
      // gp3 = GP_fit::GP_fit();

      joint1 = this->model->GetJoint("base_link_link_01");
      joint2 = this->model->GetJoint("link_01_link_02");
      joint3 = this->model->GetJoint("link_02_link_03");

      link3 = this->model->GetLink("link_03");
      link2 = this->model->GetLink("link_02");
      link1 = this->model->GetLink("link_01");

      // joint4 = this->model->GetJoint("link_03_link_04");

      this->dt = 0.1;
      // std::cout<<"Works 1"<<std::endl;

      state = Eigen::VectorXf::Zero(9);
      errors = Eigen::VectorXf::Zero(0);
      gp_mean = Eigen::VectorXf::Zero(3);

      corr = Eigen::VectorXf::Zero(3);
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
      state(0) = this-> InvDyn.joint_pos(0);
      state(1) = this-> InvDyn.joint_pos(1);
      state(2) = this-> InvDyn.joint_pos(2);
      state(3) = this-> InvDyn.joint_vel(0);
      state(4) = this-> InvDyn.joint_vel(1);
      state(5) = this-> InvDyn.joint_vel(2);
      // std::cout<<"Checkpoint 2"<<std::endl;
      Eigen::VectorXf eta_acc = this-> InvDyn.joint_acc_ref + this-> InvDyn.KP * (this-> InvDyn.joint_pos_ref - this-> InvDyn.joint_pos)  + this-> InvDyn.KD * (this-> InvDyn.joint_vel_ref - this-> InvDyn.joint_vel);
      // std::cout<<"Commanded acc is: "<<eta_acc.transpose()<<std::endl;
      state(6) = eta_acc(0); state(7) = eta_acc(1); state(8) = eta_acc(2);

      // std::cout<<"Sim Time: "<<ros::Time::now()<<std::endl;

      man_controller::FloatArray gp_state;
      gp_state.header.stamp = ros::Time::now();
      gp_state.header.frame_id = "Please be right";

      for(int i=0; i<9;i++){
        gp_state.data.push_back(state(i));
      }

      man_controller::FloatArray training_data;
      training_data.header.stamp = ros::Time::now();
      training_data.header.frame_id = "Please be right";

      for(int i=0; i<9;i++){
        training_data.data.push_back(state(i));
      }  

      Eigen::VectorXf error_state_gp = this->InvDyn.joint_pos - this->InvDyn.joint_pos_ref;
      Eigen::VectorXf error_dot_state_gp = this->InvDyn.joint_vel - this->InvDyn.joint_vel_ref;
      

      std_msgs::Float64MultiArray error_state;  
      for(int i=0; i<6;i++){
        if(i<3)
          error_state.data.push_back(error_state_gp(i));
        else
          error_state.data.push_back(error_dot_state_gp(i-3));
      }

      error_state_publisher.publish(error_state);
      gp_state_publisher.publish(gp_state);


      // std::cout<<"Actual acc is: "<<this->InvDyn.joint_acc.transpose()<<std::endl;

      Eigen::VectorXf obs = this->InvDyn.joint_acc - eta_acc;
      man_controller::FloatArray gp_obs;
      gp_obs.header.stamp = ros::Time::now();
      // std::cout<<"Eta is: "<<obs.transpose()<<std::endl;
      for(int i=0; i<3;i++){
        gp_obs.data.push_back(obs(i));
        training_data.data.push_back(obs(i));
      }

      gp_observations_publisher.publish(gp_obs);

      Eigen::Vector3f mean;
      // mean = Eigen::Vector3f::Zero();

      // std::cout<<"Observation to learn is: "<<obs.transpose()<<std::endl;
      // std::cout<<"Prediction from GP: "<<gp_mean.transpose()<<std::endl;

      man_controller::FloatValue pred_er = man_controller::FloatValue();

      Eigen::VectorXf pred_err = gp_mean - obs;
      pred_er.value = pred_err.norm();
      this->pred_error_publisher.publish(pred_er);

      // std::cout<<"States being published are: "<<state.transpose()<<std::endl;
      // std::cout<<"Torque correction is: "<<-1 * this->InvDyn.M * gp_mean<<std::endl;
      mean = Eigen::Vector3f::Zero();
      mean = gp_mean;

      gazebo::common::Time currentGzTime = this->model->GetWorld()->SimTime();
      // std::cout << "Gazebo Simulation Time: " << currentGzTime.Double() << " seconds" << std::endl;

      ros::Time currentROSTime = ros::Time::now();
      // std::cout << "ROS Time: " << currentROSTime.toSec() << " seconds" << std::endl;

      // std::cout << "Prediction ROS Time:  "<<this->savedTimestamp.toSec() << " seconds "<<std::endl;

      // std::cout<<"Current position: "<<std::endl<<this->InvDyn.joint_pos.transpose()<<std::endl;
      // corr = Eigen::Vector3f::Zero();

      bool hasNan = false;
      
      for (int i = 0; i < corr.size(); ++i) {
        if (std::isnan(corr[i])) {
            hasNan = true;
            break; // Exit the loop if a NaN is found
        }
      }

      if(hasNan)
      corr = Eigen::Vector3f::Zero();

      this->torque = this->InvDyn.get_total_torque(corr);
      // std::cout<<"State input: "<<
      // std::cout<<"Corr is: "<<corr.transpose()<<std::endl;
      // std::cout<<"Torque being applied: "<<std::endl<<this->torque<< std::endl;
      // std::cout<<"Desired Position"<<std::endl<<this->InvDyn.joint_pos_ref.transpose()<< std::endl;

      man_controller::FloatValue err = man_controller::FloatValue();

      Eigen::VectorXf error;
      error = this->InvDyn.joint_pos - this->InvDyn.joint_pos_ref;
      float err_norm = error.squaredNorm();
      accumulated_error += err_norm;
      count_rms += 1;
      this->training_data_publisher.publish(training_data);

      err.value = err_norm;

      this->error_publisher.publish(err);

      // std::cout<<"Torque applied"<<this->torque<<std::endl;
      joint1->SetForce(0, torque[0]);
      joint2->SetForce(0, torque[1]);
      joint3->SetForce(0, torque[2]);

      // std::cout<<"Checkpoint 5"<<std::endl;

      count++;

      ros::spinOnce();
    }

    public: void predCallback(const man_controller::FloatArray::ConstPtr &msg){
      float mean1 = msg->data[0];
      float mean2 = msg->data[1];
      float mean3 = msg->data[2];
      
      // this->savedTimestamp = msg->header.stamp;

      gp_mean(0) = mean1;
      gp_mean(1) = mean2;
      gp_mean(2) = mean3;
    }

    public: void corrCallback(const man_controller::FloatArray::ConstPtr &msg){
      float corr1 = msg->data[0];
      float corr2 = msg->data[1];
      float corr3 = msg->data[2];
      
      this->savedTimestamp = msg->header.stamp;

      corr(0) = corr1;
      corr(1) = corr2;
      corr(2) = corr3;
    }

    public: void posCallback(const man_controller::Traj::ConstPtr& msg){
      
      float joint1_pos = msg->num1;
      float joint2_pos = msg->num2;
      float joint3_pos = msg->num3;

      // PD Controller over the position
      double dt = 0.01;

      this->InvDyn.joint_pos_ref << joint1_pos, joint2_pos, joint3_pos;

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