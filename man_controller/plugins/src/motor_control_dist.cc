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
#include <algorithm>
#include "drake/math/continuous_algebraic_riccati_equation.h"
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include "ff_torque.h"
#include "gp.h"
#include <cstdlib>

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
      ros::Subscriber gp_pred1;
      ros::Subscriber gp_corr1;
      ros::Subscriber gp_pred2;
      ros::Subscriber gp_corr2;
      ros::Subscriber gp_pred3;
      ros::Subscriber gp_corr3;
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
      Eigen::VectorXf error_state_gp;
      Eigen::VectorXf error_dot_state_gp;

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

      man_controller::Traj vel;
      man_controller::Traj acc;
      man_controller::FloatArray gp_state;
      man_controller::FloatArray training_data;
      man_controller::FloatArray gp_obs;
      man_controller::FloatValue pred_er;
      std_msgs::Float64MultiArray error_state;
      float dt;
      Eigen::VectorXf torque;
      Eigen::VectorXf state;
      Eigen::VectorXf errors;
      Eigen::VectorXf gp_mean;

      Eigen::VectorXf corr;
      float var1;
      float var2;
      float var3;

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
      this->gp_pred1 = this->rosNode->subscribe<man_controller::FloatArray>("/gp_mean1", 1, boost::bind(&ManipulatorPlugin::mean1Callback, this, _1), ros::VoidPtr(), ros::TransportHints());
      this->gp_corr1 = this->rosNode->subscribe<man_controller::FloatArray>("/gp_var1", 1, boost::bind(&ManipulatorPlugin::var1Callback, this, _1), ros::VoidPtr(), ros::TransportHints());
      this->gp_pred2 = this->rosNode->subscribe<man_controller::FloatArray>("/gp_mean2", 1, boost::bind(&ManipulatorPlugin::mean2Callback, this, _1), ros::VoidPtr(), ros::TransportHints());
      this->gp_corr2 = this->rosNode->subscribe<man_controller::FloatArray>("/gp_var2", 1, boost::bind(&ManipulatorPlugin::var2Callback, this, _1), ros::VoidPtr(), ros::TransportHints());
      this->gp_pred3 = this->rosNode->subscribe<man_controller::FloatArray>("/gp_mean3", 1, boost::bind(&ManipulatorPlugin::mean3Callback, this, _1), ros::VoidPtr(), ros::TransportHints());
      this->gp_corr3 = this->rosNode->subscribe<man_controller::FloatArray>("/gp_var3", 1, boost::bind(&ManipulatorPlugin::var3Callback, this, _1), ros::VoidPtr(), ros::TransportHints());


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

      this->joint_pos.num1 = this->joint1->Position(0);
      this->joint_pos.num2 = this->joint2->Position(0);
      this->joint_pos.num3 = this->joint3->Position(0);
      this->joint_pos.num4 = 0;
      this->joint_pos_publisher.publish(this->joint_pos);

      this->InvDyn.joint_pos << this->joint_pos.num1, this->joint_pos.num2, this->joint_pos.num3;

      vel = man_controller::Traj();

      vel.num1 = this->joint1->GetVelocity(0);
      vel.num2 = this->joint2->GetVelocity(0);
      vel.num3 = this->joint3->GetVelocity(0);
      vel.num4 = 0;

      this->InvDyn.joint_vel << vel.num1, vel.num2, vel.num3;
      
      acc = man_controller::Traj();

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

      gp_state.header.stamp = ros::Time::now();
      gp_state.header.frame_id = "Please be right";
      gp_state.data.clear();

      for(int i=0; i<9;i++){
        gp_state.data.push_back(state(i));
      }

      
      training_data.header.stamp = ros::Time::now();
      training_data.header.frame_id = "Please be right";
      training_data.data.clear();

      for(int i=0; i<9;i++){
        training_data.data.push_back(state(i));
      }  

      error_state_gp = this->InvDyn.joint_pos - this->InvDyn.joint_pos_ref;
      error_dot_state_gp = this->InvDyn.joint_vel - this->InvDyn.joint_vel_ref;
      Eigen::VectorXd error_calc = Eigen::VectorXd::Zero(6);
      for(int i=0;i<6;i++){
        if(i<3)
        error_calc(i) = error_state_gp(i);
        else
        error_calc(i) = error_dot_state_gp(i-3); 
      }

      error_state.data.clear();  
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

      // man_controller::FloatValue pred_er 

      Eigen::VectorXf pred_err = gp_mean - obs;
      pred_er.value = abs(pred_err.norm())/obs.norm();
      this->pred_error_publisher.publish(pred_er);

      // std::cout<<"States being published are: "<<state.transpose()<<std::endl;
      // std::cout<<"Torque correction is: "<<-1 * this->InvDyn.M * gp_mean<<std::endl;
      mean = Eigen::Vector3f::Zero();
      // mean = gp_mean;

      gazebo::common::Time currentGzTime = this->model->GetWorld()->SimTime();
      // std::cout << "Gazebo Simulation Time: " << currentGzTime.Double() << " seconds" << std::endl;

      ros::Time currentROSTime = ros::Time::now();
      // std::cout << "ROS Time: " << currentROSTime.toSec() << " seconds" << std::endl;

      // std::cout << "Prediction ROS Time:  "<<this->savedTimestamp.toSec() << " seconds "<<std::endl;

      // std::cout<<"Current position: "<<std::endl<<this->InvDyn.joint_pos.transpose()<<std::endl;
      // corr = Eigen::Vector3f::Zero();

      // bool hasNan = false;
      
      // for (int i = 0; i < corr.size(); ++i) {
      //   if (std::isnan(corr[i])) {
      //       hasNan = true;
      //       break; // Exit the loop if a NaN is found
      //   }
      // }

      // if(hasNan)
      // corr = Eigen::Vector3::Zero();

      corr = this->get_corr(gp_mean(0), gp_mean(1), gp_mean(2), var1, var2, var3, error_calc);
      std::cout<<"Correction is: "<<corr.transpose()<<std::endl;

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

    public: Eigen::VectorXf get_corr(float mean1, float mean2, float mean3, float var1, float var2, float var3, Eigen::VectorXd error){
      double rho1 = std::max(abs(mean1 - 2.5*var1), abs(mean1 + 2.5*var1));
      double rho2 = std::max(abs(mean2 - 2.5*var2), abs(mean2 + 2.5*var2));
      double rho3 = std::max(abs(mean3 - 2.5*var3), abs(mean3 + 2.5*var3));

      std::cout<<"Means are: "<< mean1<<" "<<mean2<<" "<<mean3<<std::endl;
      std::cout<<"Vars are: "<< var1<<" "<<var2<<" "<<var3<<std::endl;

      double rho = rho1*rho1 + rho2*rho2 + rho3*rho3;
      rho = sqrt(rho);

      Eigen::MatrixXd B = Eigen::MatrixXd::Zero(6, 3);
      B.block(3, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);

      Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6, 6);
      A.block(3, 0, 3, 3) = -1 * this->InvDyn.KP.cast<double>();
      A.block(3, 3, 3, 3) = -1 * this->InvDyn.KD.cast<double>();
      A.block(0, 3, 3, 3) = Eigen::MatrixXd::Identity(3, 3);

      double epsilon = 0.1;
      Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(6, 6);
      Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3);

      Eigen::MatrixXd P;
      Eigen::MatrixXd B_tmp = Eigen::MatrixXd::Zero(6, 3);
      P = drake::math::ContinuousAlgebraicRiccatiEquation(A, B_tmp, Q, R);
      Eigen::MatrixXd r;

      Eigen::MatrixXd w = B.transpose() * P * error;
      if(w.norm()>epsilon)
        r = -rho * w/w.norm();
      else{
        r = -rho * w/epsilon;
      }


      return r.cast<float>();

    }

    public: void mean1Callback(const man_controller::FloatArray::ConstPtr &msg){
      float mean1 = msg->data[0];
      // std::cout<<"GP1 prediction: "<<mean1<<std::endl;
      gp_mean(0) = mean1;
    }

    public: void mean2Callback(const man_controller::FloatArray::ConstPtr &msg){
      float mean2 = msg->data[0];
      // std::cout<<"GP2 prediction: "<<mean2<<std::endl;
      gp_mean(1) = mean2;
    }

    public: void mean3Callback(const man_controller::FloatArray::ConstPtr &msg){
      float mean3 = msg->data[0];
      // std::cout<<"GP3 prediction: "<<mean3<<std::endl;
      gp_mean(2) = mean3;
    }

    public: void var1Callback(const man_controller::FloatArray::ConstPtr &msg){
      var1 = msg->data[0];
    }

    public: void var2Callback(const man_controller::FloatArray::ConstPtr &msg){
      var2 = msg->data[0];
    }

    public: void var3Callback(const man_controller::FloatArray::ConstPtr &msg){
      var3 = msg->data[0];
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