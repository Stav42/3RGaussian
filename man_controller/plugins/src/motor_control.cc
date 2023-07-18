#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <man_controller/Traj.h>
#include <boost/bind.hpp>



namespace gazebo
{
  class ManipulatorPlugin : public ModelPlugin
  {

    private: physics::ModelPtr model;
    private: event::ConnectionPtr updateConnection;
    private: ros::NodeHandle* rosNode;
    private: ros::Subscriber torque_sub;
    private: ros::Subscriber ref_pos_sub;
    private: ros::Publisher joint_state_publisher;
    private: physics::JointPtr joint1;
    private: physics::JointPtr joint2;
    private: physics::JointPtr joint3;
    private: gazebo::common::PID pid = gazebo::common::PID(1000000, 10, 10);

    
    
    public: ManipulatorPlugin() {
      if(!ros::isInitialized()){
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
      }

      this->rosNode = new ros::NodeHandle();
    }

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      this->model = _model;

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ManipulatorPlugin::OnUpdate, this)
      );

      this->torque_sub = this->rosNode->subscribe<man_controller::Traj>("/torque_values", 1, boost::bind(&ManipulatorPlugin::callback, this, _1), ros::VoidPtr(), ros::TransportHints());
      this->ref_pos_sub = this->rosNode->subscribe<man_controller::Traj>("/position_reference", 1, boost::bind(&ManipulatorPlugin::posCallback, this, _1), ros::VoidPtr(), ros::TransportHints());
      // this->ref_vel_sub = this->rosNode->subscribe('/velocity_reference', 1, &ManipulatorPlugin::velCallback)
      // this->ref_acc_sub = this->rosNode->subscribe('/acceleration_reference', 1, &ManipulatorPlugin::accCallback)

      this->joint_state_publisher = this->rosNode->publish;

      joint1 = this->model->GetJoint("base_link_link_01");
      joint2 = this->model->GetJoint("link_01_link_02");
      joint3 = this->model->GetJoint("link_02_link_03");

      // this->sub = this->rosNode->subscribe('/reference_trajectory', 1, &ManipulatorPlugin::callback, this);
    }

    public: void OnUpdate(){
      // std::cout<<"Print if working"<<std::endl;
      ros::spinOnce();
    }

    // public: ignition::math::Vector3d format(data){
    //   return torque;
    // }

    public: void callback(const man_controller::Traj::ConstPtr& msg){
      
      // float tau_1 = format(msg.data.torque[0]);
      // tau_2 = format(msg.data.torque[1]);
      // tau_3 = format(msg.data.torque[2]);

      // const physics::LinkPtr link1 = model->GetLink('link01');
      // const physics::LinkPtr link2 = model->GetLink('link02');
      // const physics::LinkPtr link3 = model->GetLink('link03');
      
      // link1->SetTorque(tau_1);
      // link2->SetTorque(taU_2);
      // link3->SetTorque(tau_3);

    }

    public: void posCallback(const man_controller::Traj::ConstPtr& msg){
      
      float joint1_pos = msg->num1;
      float joint2_pos = msg->num2;
      float joint3_pos = msg->num3;

      // PD Controller over the position
      double dt = 0.01;

      // in your OnUpdate function or callback
      double current_pos1 = this->joint1->Position(0);
      double current_pos2 = this->joint2->Position(0);
      double current_pos3 = this->joint3->Position(0);

      std::cout<<"Current position of joint 1: "<<current_pos1<<std::endl;
      std::cout<<"Desired position of joint 1: "<<joint1_pos<<std::endl;
      std::cout<<"Current position of joint 2: "<<current_pos2<<std::endl;
      std::cout<<"Desired position of joint 2: "<<joint2_pos<<std::endl;
      std::cout<<"Current position of joint 3: "<<current_pos3<<std::endl;
      std::cout<<"Desired position of joint 3: "<<joint3_pos<<std::endl;


      double error1 = joint1_pos - current_pos1;
      double error2 = joint2_pos - current_pos2;
      double error3 = joint3_pos - current_pos3;

      double cmd1 = pid.Update(error1, dt);
      double cmd2 = pid.Update(error2, dt);
      double cmd3 = pid.Update(error3, dt);

      joint1->SetForce(0, cmd1);
      joint2->SetForce(0, cmd2);
      joint3->SetForce(0, cmd3);

    }

    public: void velCallback(const man_controller::Traj::ConstPtr& msg){
      
      std::cout<<"Mission Failed Successfully!"<<std::endl;

      float joint1_vel = msg->num1;
      float joint2_vel = msg->num2;
      float joint3_vel = msg->num3;

      this->joint1->SetVelocity(2, joint1_vel);
      this->joint2->SetVelocity(1, joint2_vel);
      this->joint3->SetVelocity(1, joint3_vel);

    }

  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(ManipulatorPlugin)
}