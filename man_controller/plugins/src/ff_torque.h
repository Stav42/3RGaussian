#include <eigen3/Eigen/Dense>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <man_controller/Traj.h>
#include <math.h>

// #ifndef FF_TORQUE_H
// #define FF_TORQUE_H

class InvDynController {
    public:
        Eigen::Vector3f joint_pos;
        Eigen::Vector3f joint_vel;
        Eigen::Vector3f joint_acc;

        Eigen::Vector3f joint_pos_ref;
        Eigen::Vector3f joint_vel_ref;
        Eigen::Vector3f joint_acc_ref;
        
        Eigen::Matrix3f I_1;
        Eigen::Matrix3f I_2;
        Eigen::Matrix3f I_3;

        float L1, L2, L3;
        float m1, m2, m3;

        Eigen::Matrix3f KP, KD;
        
        Eigen::Vector3f ff_torque, fb_torque, total_torque;
        
    public:
        InvDynController();
        Eigen::Vector3f get_ff_torque();
        Eigen::Vector3f get_fb_torque();
        Eigen::Vector3f get_total_torque();

};

