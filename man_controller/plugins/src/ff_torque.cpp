#include <eigen3/Eigen/Dense>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <man_controller/Traj.h>
#include <cmath>
#include "ff_torque.h"
        
InvDynController::InvDynController(){

    joint_pos = Eigen::Vector3f::Zero();
    joint_vel = Eigen::Vector3f::Zero();
    joint_acc = Eigen::Vector3f::Zero();

    joint_pos_ref = Eigen::Vector3f::Zero();
    joint_vel_ref = Eigen::Vector3f::Zero();
    joint_acc_ref = Eigen::Vector3f::Zero();

    I_1 = Eigen::Matrix3f::Zero();
    I_2 = Eigen::Matrix3f::Zero();
    I_3 = Eigen::Matrix3f::Zero();

    KP = Eigen::Matrix3f::Zero();
    KD = Eigen::Matrix3f::Zero();

    M = Eigen::Matrix3f::Zero();

    Eigen::Matrix3f M;
    Eigen::Vector3f G, H;


    I_1 << 0.16447, 0, 0,
           0, 0.16447, 0,
           0, 0, 0.0957;

    I1 = 0.16447;
    I1_bar = 0.0957;

    I_2 << 0.0126, 0, 0,
           0, 0.06168, 0,
           0, 0, 0.06168;

    I2 = 0.06168;
    I2_bar = 0.0126;

    I_3 << 0.0126, 0, 0,
           0, 0.06168, 0,
           0, 0, 0.06168;

    I3 = 0.06168;
    I3_bar = 0.0126;

    L1 = 1.4;m1=1.5633;
    L2 = 0.8;m2=1.0986;
    L3 = 0.8;m3=1.0986;

    bool introduce_errors = 1;

    // Introducing the errors
    if(introduce_errors){
        m1 = 1.5633*0.8;
        m2 = 1.0986*0.8;
        m3 = 1.0986*0.8;
        L1 = 0.4*0.8;
        L2 = 0.8*0.8;
        L3 = 0.8*0.8;
        I1 *= 0.9; I2 *= 0.9; I3 *= 0.9;
        I1_bar *= 0.9; I2_bar *= 0.9; I3_bar *= 0.9;

    }
    


    KP << 26.5948*5*9, 0, 0,
          0, 26.59488*5*9, 0,
          0, 0, 26.5948*5*9;

    KD << 23.0629*3, 0, 0,
          0, 23.0629*3, 0,
          0, 0, 23.0629*3;

    ff_torque = Eigen::Vector3f::Zero();
    fb_torque = Eigen::Vector3f::Zero();
    total_torque = Eigen::Vector3f::Zero();

}

Eigen::Vector3f InvDynController::get_ff_upd(Eigen::Vector3f correction){
    float g = 9.81;

    Eigen::Matrix3f M_temp;
    M_temp<<1.0*I1_bar + 1.0*I2*sin(joint_pos(1))*sin(joint_pos(1)) + 1.0*I2_bar*cos(joint_pos(1))*cos(joint_pos(1)) + 1.0*I3*sin(joint_pos(1) + joint_pos(2))*sin(joint_pos(1) + joint_pos(2)) + 1.0*I3_bar*cos(joint_pos(1) + joint_pos(2))*cos(joint_pos(1) + joint_pos(2)) + 0.25*L2*L2*m2*sin(joint_pos(1))*sin(joint_pos(1)) + 0.25*m3*(2*L2*sin(joint_pos(1)) + L3*sin(joint_pos(1) + joint_pos(2)))*(2*L2*sin(joint_pos(1)) + L3*sin(joint_pos(1) + joint_pos(2))), 0, 0, 0, 1.0*I2 + 1.0*I3 + 0.25*L2*L2*m2 + 1.0*L2*L2*m3 + 1.0*L2*L3*m3*cos(joint_pos(2)) + 0.25*L3*L3*m3, 1.0*I3 + 0.25*L3*m3*(2*L2*cos(joint_pos(2)) + L3) , 0, 1.0*I3 + 0.25*L3*m3*(2*L2*cos(joint_pos(2)) + L3), 1.0*I3 + 0.25*L3*L3*m3;
    M = M_temp;

    Eigen::Vector3f G;
    G<<0, -g*(L2*m2*sin(joint_pos(1)) + 2*L2*m3*sin(joint_pos(1)) + L3*m3*sin(joint_pos(1) + joint_pos(2)))/2, -L3*g*m3*sin(joint_pos(1) + joint_pos(2))/2;

    Eigen::Vector3f H;
    H<<((2.0*I3*sin(joint_pos(1) + joint_pos(2)) - 2.0*I3_bar*sin(joint_pos(1) + joint_pos(2)) + 0.5*L3*m3*(2*L2*sin(joint_pos(1)) + L3*sin(joint_pos(1) + joint_pos(2))))*cos(joint_pos(1) + joint_pos(2))*joint_vel(2) + (I2*sin(2*joint_pos(1)) - 1.0*I2_bar*sin(2*joint_pos(1)) + I3*sin(2*joint_pos(1) + 2*joint_pos(2)) - 1.0*I3_bar*sin(2*joint_pos(1) + 2*joint_pos(2)) + 0.25*L2*L2*m2*sin(2*joint_pos(1)) + 0.25*m3*(4*L2*L2*sin(2*joint_pos(1)) + 4*L2*L3*sin(2*joint_pos(1) + joint_pos(2)) + L3*L3*sin(2*joint_pos(1) + 2*joint_pos(2))))*joint_vel(1))*joint_vel(0), -0.5*I2*sin(2*joint_pos(1))*joint_vel(0)*joint_vel(0) + 0.5*I2_bar*sin(2*joint_pos(1))*joint_vel(0)*joint_vel(0) - 0.5*I3*sin(2*joint_pos(1) + 2*joint_pos(2))*joint_vel(0)*joint_vel(0) + 0.5*I3_bar*sin(2*joint_pos(1) + 2*joint_pos(2))*joint_vel(0)*joint_vel(0) - 0.125*L2*L2*m2*sin(2*joint_pos(1))*joint_vel(0)*joint_vel(0) - L2*L3*m3*(1.0*joint_vel(1) + 0.5*joint_vel(2))*sin(joint_pos(2))*joint_vel(2) - 0.125*m3*(4*L2*L2*sin(2*joint_pos(1)) + 4*L2*L3*sin(2*joint_pos(1) + joint_pos(2)) + L3*L3*sin(2*joint_pos(1) + 2*joint_pos(2)))*joint_vel(0)*joint_vel(0), -0.5*I3*sin(2*joint_pos(1) + 2*joint_pos(2))*joint_vel(0)*joint_vel(0) + 0.5*I3_bar*sin(2*joint_pos(1) + 2*joint_pos(2))*joint_vel(0)*joint_vel(0) - 0.25*L2*L3*m3*sin(2*joint_pos(1) + joint_pos(2))*joint_vel(0)*joint_vel(0) + 0.25*L2*L3*m3*sin(joint_pos(2))*joint_vel(0)*joint_vel(0) + 0.5*L2*L3*m3*sin(joint_pos(2))*joint_vel(1)*joint_vel(1) - 0.125*L3*L3*m3*sin(2*joint_pos(1) + 2*joint_pos(2))*joint_vel(0)*joint_vel(0); 

    Eigen::Vector3f ff = M * (joint_acc_ref + KP * (joint_pos_ref - joint_pos) + KD * (joint_vel_ref - joint_vel) - correction) + G + H;
    
    return ff;
}

Eigen::Vector3f InvDynController::get_ff(Eigen::Vector3f correction){
    
    float g = 9.81;

    Eigen::Matrix3f M;

    M << I_2(0, 0)/2 + I_3(0, 0)/2 + I_1(2, 2) + I_2(2, 2)/2 + I_3(2, 2)/2 + (2*m2)/25 + (4*m3)/25 - (2*m2*cos(2*joint_pos(1)))/25 - (2*m3*cos(2*joint_pos(1)))/25 - (I_3(0, 0)*cos(2*joint_pos(1) + 2*joint_pos(2)))/2 + (I_3(2, 2)*cos(2*joint_pos(1) + 2*joint_pos(2)))/2 - (2*m3*cos(2*joint_pos(1) + 2*joint_pos(2)))/25 + (4*m3*cos(joint_pos(2)))/25 - (4*m3*cos(2*joint_pos(1) + joint_pos(2)))/25 - (I_2(0, 0)*cos(2*joint_pos(1)))/2 + (I_2(2, 2)*cos(2*joint_pos(1)))/2, 0, 0,
        0, I_2(1, 1) + I_3(1, 1) + (4*m2)/25 + (8*m3)/25 + (8*m3*cos(joint_pos(2)))/25, I_3(1, 1) + (4*m3)/25 + (4*m3*cos(joint_pos(2)))/25, 
        0, I_3(1, 1) + (4*m3)/25 + (4*m3*cos(joint_pos(2)))/25, I_3(1, 1) + (4*m3)/25;

    Eigen::Vector3f G;
    G << 0, ((2*m3*sin(joint_pos(1) + joint_pos(2)))/5 + (2*m2*sin(joint_pos(1)))/5 + (2*m3*sin(joint_pos(1)))/5)*g, ((2*m3*sin(joint_pos(1) + joint_pos(2)))/5)*g;

    Eigen::Vector3f H;
    H << (joint_vel(0)*(8*m3*joint_vel(1)*sin(2*joint_pos(1) + joint_pos(2)) - 4*m3*joint_vel(2)*sin(joint_pos(2)) + 4*m3*joint_vel(2)*sin(2*joint_pos(1) + joint_pos(2)) + 25*I_2(0, 0)*joint_vel(1)*sin(2*joint_pos(1)) - 25*I_2(2, 2)*joint_vel(1)*sin(2*joint_pos(1)) + 4*m2*joint_vel(1)*sin(2*joint_pos(1)) + 4*m3*joint_vel(1)*sin(2*joint_pos(1)) + 25*I_3(0, 0)*joint_vel(1)*sin(2*joint_pos(1) + 2*joint_pos(2)) + 25*I_3(0, 0)*joint_vel(2)*sin(2*joint_pos(1) + 2*joint_pos(2)) - 25*I_3(2, 2)*joint_vel(1)*sin(2*joint_pos(1) + 2*joint_pos(2)) - 25*I_3(2, 2)*joint_vel(2)*sin(2*joint_pos(1) + 2*joint_pos(2)) + 4*m3*joint_vel(1)*sin(2*joint_pos(1) + 2*joint_pos(2)) + 4*m3*joint_vel(2)*sin(2*joint_pos(1) + 2*joint_pos(2))))/25, (I_2(2, 2)*joint_vel(0)*joint_vel(0)*sin(2*joint_pos(1)))/2 - (4*m3*joint_vel(2)*joint_vel(2)*sin(joint_pos(2)))/25 - (4*m3*joint_vel(0)*joint_vel(0)*sin(2*joint_pos(1) + joint_pos(2)))/25 - (I_2(0, 0)*joint_vel(0)*joint_vel(0)*sin(2*joint_pos(1)))/2 - (2*m3*joint_vel(0)*joint_vel(0)*sin(2*joint_pos(1) + 2*joint_pos(2)))/25 - (2*m2*joint_vel(0)*joint_vel(0)*sin(2*joint_pos(1)))/25 - (2*m3*joint_vel(0)*joint_vel(0)*sin(2*joint_pos(1)))/25 - (I_3(0, 0)*joint_vel(0)*joint_vel(0)*sin(2*joint_pos(1) + 2*joint_pos(2)))/2 + (I_3(2, 2)*joint_vel(0)*joint_vel(0)*sin(2*joint_pos(1) + 2*joint_pos(2)))/2 - (8*m3*joint_vel(1)*joint_vel(2)*sin(joint_pos(2)))/25, (2*m3*joint_vel(0)*joint_vel(0)*sin(joint_pos(2)))/25 - (2*m3*joint_vel(0)*joint_vel(0)*sin(2*joint_pos(1) + 2*joint_pos(2)))/25 + (4*m3*joint_vel(1)*joint_vel(1)*sin(joint_pos(2)))/25 - (2*m3*joint_vel(0)*joint_vel(0)*sin(2*joint_pos(1) + joint_pos(2)))/25 - (I_3(0, 0)*joint_vel(0)*joint_vel(0)*sin(2*joint_pos(1) + 2*joint_pos(2)))/2 + (I_3(2, 2)*joint_vel(0)*joint_vel(0)*sin(2*joint_pos(1) + 2*joint_pos(2)))/2;

    Eigen::Vector3f ff = M * (joint_acc_ref + KP * (joint_pos_ref - joint_pos)  + KD * (joint_vel_ref - joint_vel) + correction) + G + H;
    return ff;

}

Eigen::Vector3f InvDynController::get_fb_torque(){
    
    fb_torque = KP * (joint_pos_ref - joint_pos) + KD * (joint_vel_ref - joint_vel);
    return fb_torque;

};
Eigen::Vector3f InvDynController::get_total_torque(Eigen::Vector3f correction){
    
    Eigen::Vector3f ff = get_ff_upd(correction);
    Eigen::Vector3f fb = get_fb_torque();
    return ff;

};


