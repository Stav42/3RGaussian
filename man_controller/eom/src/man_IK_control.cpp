// // This is only a working implementation. This has to be abstracted into a class.
// #include <ros/ros.h>
// #include <eigen3/Eigen/Dense>
// #include <vector>
// #include "std_msgs/Float32.h"
// #include "shvan_msgs/motor_command_data.h"
// #include "shvan_msgs/motor_gain_multiplier.h"
// #include "shvan_msgs/float_vec.h"

// #include "kinematics.h"

// #define OFFSET 0.00

// Eigen::VectorXd ee_state;
// Eigen::VectorXd ee_vel;
// Eigen::VectorXd js_pos;
// Eigen::VectorXd js_vel;
// Eigen::VectorXd t_ff;
// Eigen::VectorXd t_act;

// shvan_msgs::motor_command_data motor_cmd_data;

// // set this according to the hardware
// // std::vector<int> joint_dir = {-1, -1, 1, -1, -1, 1, -1, -1, 1, -1, -1, 1};
// std::vector<int> joint_dir = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

// // simulation joint directions
// std::vector<int> joint_dir_sim = {-1, -1, -1, -1, 1, 1, -1, 1, 1, -1, 1, -1};

// void get_ee_state_ref(const shvan_msgs::float_vec::ConstPtr& msg) {
//     for (int i = 0; i < ee_state.rows(); i++) {
//         ee_state(i) = msg -> at[i];
//     }
// }

// void get_ee_vel_ref(const shvan_msgs::float_vec::ConstPtr& msg) {
//     for (int i = 0; i < ee_vel.rows(); i++) {
//         ee_vel(i) = msg -> at[i];
//     }
// }

// void get_motor_ff(const shvan_msgs::float_vec::ConstPtr& msg) {
//     for (int i = 0; i < t_ff.rows(); i++) {
//         t_ff(i) = msg -> at[i];
//     }
// }

// void get_motor_cmd_act(const shvan_msgs::motor_command_data::ConstPtr& msg) {
//     for (int i = 0; i < msg -> pos_ref.size(); i++) {
//         js_pos(i) = msg -> pos_ref[i];
//         js_vel(i) = msg -> vel_ref[i];
// 	t_act(i) = msg -> tau_ff[i];
//     }
// }

// void joint_state_to_sim_motor_command(shvan_msgs::float_vec& joint_state, shvan_msgs::float_vec& joint_vel, shvan_msgs::motor_command_data& motor_command) {
//     // motor_command.kp_scale = 1;
//     // motor_command.kd_scale = 1;
//     motor_command.tau_ff.clear();

//     for (int i = 0; i < 6; i++) {
//         motor_command.pos_ref.push_back(joint_dir_sim[(i + 3) % 6] * joint_state.at[6 + ((i + 3) % 6)]);
//         motor_command.vel_ref.push_back(joint_dir_sim[(i + 3) % 6] * joint_vel.at[6 + ((i + 3) % 6)]);
//         motor_command.tau_ff.push_back(joint_dir[(i + 3) % 6] * t_ff((i + 3) % 6));
//     }
//     for (int i = 6; i < 12; i++) {
//         motor_command.pos_ref.push_back(joint_dir_sim[i] * joint_state.at[i + 6]);
//         motor_command.vel_ref.push_back(joint_dir_sim[i] * joint_vel.at[i + 6]);
//         motor_command.tau_ff.push_back(joint_dir[i] * t_ff(i));
//     }
// }

// void joint_state_to_motor_command(shvan_msgs::float_vec& joint_state, shvan_msgs::float_vec& joint_vel, shvan_msgs::motor_command_data& motor_command) {
//     // motor_command.kp_scale = 1;
//     // motor_command.kd_scale = 1;
//     motor_command.tau_ff.clear();

//     for (int i = 0; i < 12; i++) {
//         motor_command.pos_ref.push_back(joint_state.at[i + 6]);
//         motor_command.vel_ref.push_back(joint_vel.at[i + 6]);
//         motor_command.tau_ff.push_back(t_ff(i));
//     }
// }

// void joint_state_to_motor_cmd(shvan_msgs::float_vec& joint_state, shvan_msgs::float_vec& joint_vel, shvan_msgs::motor_command_data& motor_command) {
// 	motor_command.pos_ref.clear();
// 	motor_command.vel_ref.clear();
// 	motor_command.tau_ff.clear();
//     // motor_command.kp_scale = 1;
//     // motor_command.kd_scale = 1;
//     // change this when running on hardware that used FR-FL-RL-RR format (current shvan hardware)
//     // for (int i = 0; i < 6; i++) {
//     //     motor_command.pos_ref.push_back(joint_dir[(i + 3) % 6] * joint_state.at[6 + ((i + 3) % 6)]);
//     //     motor_command.vel_ref.push_back(0);
//     // }
//     for (int i = 0; i < 12; i++) {
//         if ((i + 1) % 3 == 0) {
//             motor_command.pos_ref.push_back(joint_dir[i] * joint_state.at[i + 6] - joint_dir[i - 1] * joint_state.at[i + 6 - 1] / 2);
//             motor_command.vel_ref.push_back(joint_dir[i] * joint_vel.at[i + 6] - joint_dir[i - 1] * joint_vel.at[i + 6 - 1] / 2);
//             motor_command.tau_ff.push_back(joint_dir[i] * t_ff(i));
//             // motor_command.vel_ref.push_back(0);
//         } else {
//             motor_command.pos_ref.push_back(joint_dir[i] * joint_state.at[i + 6]);
//             motor_command.vel_ref.push_back(joint_dir[i] * joint_vel.at[i + 6]);
//             motor_command.tau_ff.push_back(joint_dir[i] * t_ff(i));
//             // motor_command.vel_ref.push_back(0);
//         }
//     }
// }

// void motor_cmd_to_joint_state(shvan_msgs::motor_command_data& joint_state) {
//     joint_state.pos_ref.clear();
//     joint_state.vel_ref.clear();
//     joint_state.tau_ff.clear();
//     for (int i = 0; i < 12; i++) {
// 	if ((i + 1) % 3 == 0) {
// 	    joint_state.pos_ref.push_back(joint_dir[i] * js_pos(i) + 0.0 * joint_dir[i - 1] * js_pos(i - 1));
// 	    joint_state.vel_ref.push_back(joint_dir[i] * js_vel(i) + 0.0 * joint_dir[i - 1] * js_vel(i - 1));
// 	    joint_state.tau_ff.push_back(joint_dir[i] * t_act(i));
// 	} else {
// 	    joint_state.pos_ref.push_back(js_pos(i));
// 	    joint_state.vel_ref.push_back(js_vel(i));
// 	    joint_state.tau_ff.push_back(t_act(i));
// 	}
//     }
// }

// void motor_command_to_joint_state(shvan_msgs::motor_command_data& joint_state) {
//     joint_state.pos_ref.clear();
//     joint_state.vel_ref.clear();
//     joint_state.tau_ff.clear();
//     for (int i = 0; i < 12; i++) {
// 	    joint_state.pos_ref.push_back(js_pos(i));
// 	    joint_state.vel_ref.push_back(js_vel(i));
// 	    joint_state.tau_ff.push_back(t_act(i));
//     }
// }

// int main(int argc, char **argv) {
//     ee_state = Eigen::VectorXd::Zero(18);
//     ee_vel = Eigen::VectorXd::Zero(18);
//     js_pos = Eigen::VectorXd::Zero(12);
//     js_vel = Eigen::VectorXd::Zero(12);
//     t_ff = Eigen::VectorXd::Zero(12);
//     t_act = Eigen::VectorXd::Zero(12);
//     ee_state(2) = 0.28;
//     ee_state(6) = 0.243 - OFFSET;
//     ee_state(7) = -0.125;
//     ee_state(8) = 0.0;
//     ee_state(9) = 0.243 - OFFSET;
//     ee_state(10) = 0.12;
//     ee_state(11) = 0.0;
//     ee_state(12) = -0.243 - OFFSET;
//     ee_state(13) = 0.125;
//     ee_state(14) = 0.0;
//     ee_state(15) = -0.243 - OFFSET;
//     ee_state(16) = -0.125;
//     ee_state(17) = 0.0;


//     shvan_msgs::float_vec joint_state;
//     shvan_msgs::float_vec joint_vel;
//     joint_state.at.clear();
//     joint_vel.at.clear();

//     inverse_kinematics(ee_state, joint_state);
//     get_joint_velocities(ee_vel, joint_vel, joint_state);
    
//     ros::init(argc, argv, "svan_inverse_kinematics_publisher");
//     ros::NodeHandle nh;

//     ros::Subscriber ee_ref_data_sub = nh.subscribe("/shvan/ee_state_ref_data", 1, get_ee_state_ref);
//     ros::Subscriber ee_vel_data_sub = nh.subscribe("/shvan/ee_vel_ref_data", 1, get_ee_vel_ref);
//     ros::Subscriber motor_cmd_sub = nh.subscribe("/shvan/motor_command_act", 1, get_motor_cmd_act);
//     ros::Subscriber motor_ff_sub = nh.subscribe("/shvan/motor_ff", 1, get_motor_ff);
//     // ros::Publisher joint_ref_data_pub = nh.advertise<shvan_msgs::joint_ref_array>("/shvan/joint_ref_data", 1);

//     ros::Publisher motor_cmd_pub = nh.advertise<shvan_msgs::motor_command_data>("/shvan/motor_command", 1);
//     ros::Publisher motor_cmd_corrected = nh.advertise<shvan_msgs::motor_command_data>("/shvan/motor_command_links", 1);

//     ros::Rate loop_rate(5000);

//     shvan_msgs::motor_command_data motor_cmd;
//     shvan_msgs::motor_command_data joint_state_data;

//     bool gotnan = 0;

//     while (ros::ok()) {
//         motor_cmd.pos_ref.clear();
//         motor_cmd.vel_ref.clear();
//         motor_cmd.tau_ff.clear();
//         // std::cout << "ee_ref_state: " << ee_state.transpose() << std::endl;
        
//         gotnan = 0;

//         inverse_kinematics_svan(ee_state, joint_state);
//         get_joint_velocities(ee_vel, joint_vel, joint_state);

//         // std::cout << "ee_vel: \n";
//         // std::cout << ee_vel.transpose() << std::endl;

//         // std::cout << "js_vel: \n";
//         // for (auto &jv : joint_vel.at) {
//         //     std::cout << jv << " ";
//         // }
//         // std::cout << std::endl;
//         // std::cout << "J_inv * x_dot: \n" << ( ContactJacobian(joint_state) * ee_vel ).transpose() << std::endl;

//         for (auto &ja : joint_state.at) {
//             // std::cout << ja << " ";
//             if (std::isnan(ja)) {
//                 gotnan = 1;
//                 // std::cout << "Nan encountered/n";
//             }
//         }
//         // std::cout << std::endl;

//         // std::cout << "Manually marked: \n" << Jacobian(joint_state) << std::endl;
//         // std::cout << "Automatically generated: \n " << ContactJacobian(joint_state) << std::endl;
//         // std::cout << "Automatically generated 2: \n " << ContactJacobian2(joint_state) << std::endl;
//         // std::cout << "Manually marked Inverse: \n" << JacobianInverse_svan(joint_state) << std::endl;
//         // std::cout << "Inverse of Automatically generated: \n" << ( ContactJacobian(joint_state) ).inverse() << std::endl;

//         if (gotnan) {
//             ros::spinOnce();
//             loop_rate.sleep();
//             continue;
//         } 

// 	// converting joint angles to motor angles and publishing as reference values
//         joint_state_to_motor_command(joint_state, joint_vel, motor_cmd);
//         motor_cmd_pub.publish(motor_cmd);

// 	// converting motor angles to joint angles and publishing as actual values
//         motor_cmd_to_joint_state(joint_state_data);
// 	    motor_cmd_corrected.publish(joint_state_data);

//         ros::spinOnce();
//         loop_rate.sleep();
//     }
// }