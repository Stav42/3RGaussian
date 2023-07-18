#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>


namespace gazebo
{
  class ManipulatorPlugin : public ModelPlugin
  {

    private: physics::ModelPtr model;
    private: event::ConnectionPtr updateConnection;
    private: ros::NodeHandle* rosNode;
    private: ros::Subscriber torque_sub;
    private: physics::JointPtr joint1;
    private: physics::JointPtr joint2;
    private: physics::JointPtr joint3;
    
    
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

      this->torque_sub = this->rosNode->subscribe('/torque_values', 1, &ManipulatorPlugin::callback)
      this->ref_pos_sub = this->rosNode->subscribe('/position_reference', 1, &ManipulatorPlugin::posCallback)
      this->ref_vel_sub = this->rosNode->subscribe('/velocity_reference', 1, &ManipulatorPlugin::velCallback)
      this->ref_acc_sub = this->rosNode->subscribe('/acceleration_reference', 1, &ManipulatorPlugin::accCallback)

      joint1 = model->GetJoint('base_link_link_01');
      joint2 = model->GetJoint('link_01_link_02');
      joint3 = model->GetJoint('link_02_link_03');

      // this->sub = this->rosNode->subscribe('/reference_trajectory', 1, &ManipulatorPlugin::callback, this);
    }

    public: void OnUpdate(){
      // std::cout<<"Print if working"<<std::endl;
      ros::spinOnce();
    }

    public: ignition::math::Vector3d format(data){
      return torque;
    }

    public: void callback(const std_msgs::Float64::ConstPtr& msg){
      
      tau_1 = format(msg.data.torque[0]);
      tau_2 = format(msg.data.torque[1]);
      tau_3 = format(msg.data.torque[2]);

      const physics::LinkPtr link1 = model->GetLink('link01');
      const physics::LinkPtr link2 = model->GetLink('link02');
      const physics::LinkPtr link3 = model->GetLink('link03');
      
      link1->SetTorque(tau_1);
      link2->SetTorque(taU_2);
      link3->SetTorque(tau_3);

    }

    public: void posCallback(const man_controller::Traj& msg){
      
      joint1_pos = msg.num1;
      joint2_pos = msg.num2;
      joint3_pos = msg.num3;

      joint1->SetJointPosition(joint1_pos);
      joint2->SetJointPosition(joint2_pos);
      joint3->SetJointPosition(joint3_pos);

    }

  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(ManipulatorPlugin)
}