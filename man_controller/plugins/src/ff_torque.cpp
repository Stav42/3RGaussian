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

    Eigen::Matrix3f M;
    Eigen::Vector3f G, H;


    I_1 << 0.16447, 0, 0,
           0, 0.16447, 0,
           0, 0, 0.0957;


    I_2 << 0.06168, 0, 0,
           0, 0.06168, 0,
           0, 0, 0.0126;

    I_3 << 0.06168, 0, 0,
           0, 0.06168, 0,
           0, 0, 0.0126;

    m1 = 1.5633;
    m2 = 1.0986;
    m3 = 1.0986;
    L1 = 0.4;
    L2 = 0.8;
    L3 = 0.8;

    KP << 100000, 0, 0,
          0, 80000, 0,
          0, 0, 10000;

    KD << 150, 0, 0,
          0, 150, 0,
          0, 0, 150;

    ff_torque = Eigen::Vector3f::Zero();
    fb_torque = Eigen::Vector3f::Zero();
    total_torque = Eigen::Vector3f::Zero();

}

Eigen::Vector3f InvDynController::get_ff(){
    
    float g = 9.81;

    Eigen::Matrix3f M;
    M << (I_1(2, 2) + I_2(0, 0) * sin(joint_pos(1))*sin(joint_pos(1)) + I_2(1, 1) * cos(joint_pos(1)) + I_3(0, 0) * sin(joint_pos(1) + joint_pos(2))*sin(joint_pos(1) + joint_pos(2)) + I_3(1, 1) * cos(joint_pos(1) + joint_pos(2))*cos(joint_pos(1) + joint_pos(2)) + 0.25 * L2 * L2 * m2 * cos(joint_pos(1)) * cos(joint_pos(1)) + 0.25*m3*(2*L2*cos(joint_pos(1)) + L3*cos(joint_pos(1) + joint_pos(2))) * (2*L2*cos(joint_pos(1)) + L3*cos(joint_pos(1) + joint_pos(2)))), 0, 0,
                        0, (I_2(2, 2) + I_3(2, 2) + 0.25 * L2 * L2 * m2 + L2*L2*m3*sin(joint_pos(2)) * sin(joint_pos(2)) + 0.5*m3 *(L2 * cos(joint_pos(2)+L3/2))*(2*L2*cos(joint_pos(2))+L3)), (I_3(2, 2) + 0.25*L3*m3*(2*L2*cos(joint_pos(2))+L3)), 
                        0, I_3(2, 2) + 0.25 *L3*m3 * (2*L2*cos(joint_pos(2))+L3), I_3(2, 2) + 0.25*L3*L3*m3;

    Eigen::Vector3f G;
    G << 0, L2*g*m2*cos(joint_pos(1))/2 + g*m3*(L2*cos(joint_pos(1))+L3*(-1 * sin(joint_pos(1))*sin(joint_pos(2)) + cos(joint_pos(1))*cos(joint_pos(2)))/2), L3*g*m3*(-sin(joint_pos(1)*sin(joint_pos(2))+cos(joint_pos(1))*cos(joint_pos(2))/2));

    Eigen::Vector3f H;
    H << (2.0*I_3(0, 0)*sin(joint_pos(1) + joint_pos(2))*cos(joint_pos(1) + joint_pos(2))*joint_vel(0) - 2.0*I_3(1, 1)*sin(joint_pos(1) + joint_pos(2))*cos(joint_pos(1) + joint_pos(2))*joint_vel(0) - 0.5*L3*m3*(2*L2*cos(joint_pos(1)) + L3*cos(joint_pos(1) + joint_pos(2)))*sin(joint_pos(1) + joint_pos(2))*joint_vel(0))*joint_vel(2) + (2.0*I_2(0, 0)*sin(joint_pos(1))*cos(joint_pos(1))*joint_vel(0) - 2.0*I_2(1, 1)*sin(joint_pos(1))*cos(joint_pos(1))*joint_vel(0) + 2.0*I_3(0, 0)*sin(joint_pos(1) + joint_pos(2))*cos(joint_pos(1) + joint_pos(2))*joint_vel(0) - 2.0*I_3(1, 1)*sin(joint_pos(1) + joint_pos(2))*cos(joint_pos(1) + joint_pos(2))*joint_vel(0) - 0.5*L2*L2*m2*sin(joint_pos(1))*cos(joint_pos(1))*joint_vel(0) + 0.25*m3*(-4*L2*sin(joint_pos(1)) - 2*L3*sin(joint_pos(1) + joint_pos(2)))*(2*L2*cos(joint_pos(1)) + L3*cos(joint_pos(1) + joint_pos(2)))*joint_vel(0))*joint_vel(1), 
                        -1.0*I_2(0, 0)*sin(joint_pos(1))*cos(joint_pos(1))*joint_vel(0)*joint_vel(0) + 1.0*I_2(1, 1)*sin(joint_pos(1))*cos(joint_pos(1))*joint_vel(0)*joint_vel(0) - 1.0*I_3(0, 0)*sin(joint_pos(1) + joint_pos(2))*cos(joint_pos(1) + joint_pos(2))*joint_vel(0)*joint_vel(0) + 1.0*I_3(1, 1)*sin(joint_pos(1) + joint_pos(2))*cos(joint_pos(1) + joint_pos(2))*joint_vel(0)*joint_vel(0) + 0.25*L2*L2*m2*sin(joint_pos(1))*cos(joint_pos(1))*joint_vel(0)*joint_vel(0) - 0.125*m3*(-4*L2*sin(joint_pos(1)) - 2*L3*sin(joint_pos(1) + joint_pos(2)))*(2*L2*cos(joint_pos(1)) + L3*cos(joint_pos(1) + joint_pos(2)))*joint_vel(0)*joint_vel(0) + (2.0*L2*L2*m3*sin(joint_pos(2))*cos(joint_pos(2))*joint_vel(1) - 0.5*L2*m3*(2*L2*cos(joint_pos(2)) + L3)*sin(joint_pos(2))*joint_vel(1) - 1.0*L2*m3*(L2*cos(joint_pos(2))*joint_vel(1) + L3*(joint_vel(1) + joint_vel(2))/2)*sin(joint_pos(2)))*joint_vel(2), 
                        -1.0*I_3(0, 0)*sin(joint_pos(1) + joint_pos(2))*cos(joint_pos(1) + joint_pos(2))*joint_vel(0)*joint_vel(0) + 1.0*I_3(1, 1)*sin(joint_pos(1) + joint_pos(2))*cos(joint_pos(1) + joint_pos(2))*joint_vel(0)*joint_vel(0) - 1.0*L2*L2*m3*sin(joint_pos(2))*cos(joint_pos(2))*joint_vel(1)*joint_vel(1) - 0.5*L2*L3*m3*sin(joint_pos(2))*joint_vel(1)*joint_vel(2) + 1.0*L2*m3*(L2*cos(joint_pos(2))*joint_vel(1) + L3*(joint_vel(1) + joint_vel(2))/2)*sin(joint_pos(2))*joint_vel(1) + 0.25*L3*m3*(2*L2*cos(joint_pos(1)) + L3*cos(joint_pos(1) + joint_pos(2)))*sin(joint_pos(1) + joint_pos(2))*joint_vel(0)*joint_vel(0);

    Eigen::Vector3f ff = M * joint_acc_ref + G + H;

    return ff;

}

Eigen::Vector3f InvDynController::get_ff_torque(){
    
    float g = 9.81;
    /// Joint_1
    float term1 = -0.5 * I_3(0, 0) * sin(2 * (joint_pos(1) + joint_pos(2))) * joint_vel(0) * joint_vel(0); 
    float term2 = 0.5 * I_3(1, 1) * sin(2 * (joint_pos(1) + joint_pos(2))) * joint_vel(0) * joint_vel(0);
    float term3 = I_3(2, 2) * (joint_acc_ref(1) + joint_acc_ref(2)) + 0.25 * L2 * L3 * m3 * sin(2 * joint_pos(1) + joint_pos(2)) * joint_acc_ref(0);
    float term4 = 0.25 * L2 * L3 * m3 * sin(joint_pos(2)) * joint_acc_ref(0);
    float term5 = 0.5 * L2 * L3 * m3 * (sin(joint_pos(2)) * joint_pos(1) *joint_pos(1));
    float term6 = 0.5 * L2 * L3 * m3 * (cos(joint_pos(2)) * joint_acc_ref(1));
    float term7 = 0.125 * L3 * L3 * m3 * sin(2* (joint_pos(1) + joint_pos(2))) * joint_vel(0);
    float term8 = 0.25 * L3 * L3 * m3 * (joint_acc_ref(1) + joint_acc_ref(2));
    float term9 = 0.5 * L3 * g * m3 * cos(joint_pos(1) + joint_pos(2));

    float ff_1 =  term1 + term2 + term3 + term4 + term5 + term6 + term7 + term8 + term9;

    /// Joint_2
    term1 = -0.5*(I_2(0, 0) + I_2(1, 1))*sin(2 * joint_pos(1)) * joint_vel(0) * joint_vel(0) + I_2(2, 2) * joint_acc_ref(2) - 0.5 * (I_3(1, 1)-I_3(0, 0))*sin(2*joint_pos(1)+2*joint_pos(2)) * joint_vel(0) * joint_vel(0);
    term2 = I_3(2, 2) * (joint_acc_ref(2) + joint_acc_ref(2)) + 0.125 * L2 * L2 * m2 * sin(2 * joint_pos(1)) * joint_vel(0) * joint_vel(0);
    term3 = 0.25 * L2 * L2 * m2 * joint_acc_ref(1) + 0.5 * L2 * L2 * m3 * sin(2 * joint_pos(1)) * joint_vel(0) * joint_vel(0);
    term4 = L2 * L2 * m3 * joint_acc_ref(2) + 0.5 * L2 * L3 * m3* sin(2*joint_pos(1) + joint_pos(2))*joint_vel(0) *joint_vel(0);
    term5 = -1 * L2 * L3 * m3 * sin(joint_pos(2)) * joint_vel(1) * joint_vel(2) - 0.5 * L2 * L3 *m3 * sin(joint_pos(2)) * joint_vel(2)*joint_vel(2);
    term6 = L2 * L3 *m3 * cos(joint_pos(2)) * joint_acc_ref(1) + 0.5 * L2 * L3 *m3 * cos(joint_pos(2)) * joint_acc_ref(2);
    term7 = 0.5 * L2 * g * m2 * cos(joint_pos(1)) + 1.0 * L2 * g * m3 * cos(joint_pos(1));
    term8 = 0.125 * L3 * L3 * m3 * sin(2 * (joint_pos(1) + joint_pos(2))) * joint_vel(0) * joint_vel(0);
    term9 = 0.25 * L3 * L3 * m3 * (joint_acc_ref(1) + joint_acc_ref(2)) + 0.5 * L3 * g * m3 * cos(joint_pos(1) + joint_pos(2));

    float ff_2 =  term1 + term2 + term3 + term4 + term5 + term6 + term7 + term8 + term9;

    // Joint_3
    term1 = I_1(2, 2) * joint_acc_ref(0) + I_2(0, 0) * sin(joint_pos(1)) * sin(joint_pos(1)) * joint_acc_ref(0) + (-1 * I_2(0, 0) + I_2(1, 1)) * sin(2 * joint_pos(1)) * joint_vel(0) * joint_vel(1);
    term2 = I_2(1, 1) * cos(joint_pos(1)) * cos(joint_pos(1)) * joint_acc_ref(0) + I_3(0, 0) * (joint_vel(0) + joint_vel(1)) * sin(2 * (joint_pos(1) + joint_pos(2))) * joint_vel(0);
    term3 = I_3(0, 0) * sin(joint_pos(1) + joint_pos(2)) * joint_acc_ref(0) - I_3(1, 1) * (joint_vel(1)+ joint_vel(2)) * sin(2 * (joint_pos(1) + joint_pos(2))) * joint_vel(0);
    term4 = I_3(1, 1) * cos(joint_pos(1) + joint_pos(2)) * cos(joint_pos(1) + joint_pos(2)) * joint_acc_ref(0) - 0.25 * L2 * L2 * m2 * sin(2*joint_pos(1))*joint_vel(0) * joint_vel(1);
    term5 = 0.25 * L2 * L2 * m2 * cos(joint_pos(1)) * cos(joint_pos(1)) * joint_acc_ref(0);
    term6 = 0.25 * m3 * (2 * L2 * cos(joint_pos(1)) + L3 * cos(joint_pos(2) + joint_pos(1)))* (2 * L2 * cos(joint_pos(1)) + L3 * cos(joint_pos(2) + joint_pos(1))) *joint_acc_ref(0);
    term7 = -0.5 * m3 * (2 * L2 * cos(joint_pos(1)) + L3 * cos(joint_pos(2) + joint_pos(1))) * (2 * L2 * sin(joint_pos(1)) * joint_vel(1) + L3 * (joint_vel(1) + joint_vel(2)) *sin(joint_pos(1) + joint_pos(2)) ) * joint_vel(0);

    float ff_3 = term1 + term2 + term3 + term4 + term5 + term6 + term7;

    ff_torque  << ff_1, ff_2, ff_3;
    // ff_torque = Eigen::Vector3f::Zero();
    return ff_torque;
}

Eigen::Vector3f InvDynController::get_fb_torque(){
    
    // fb_torque = Eigen::Vector3f::Zero();
    fb_torque = KP * (joint_pos_ref - joint_pos) + KD * (joint_vel_ref - joint_vel);
    // std::cout<<"joint_pos_ref is: "<<joint_pos_ref<<std::endl;
    // std::cout<<"joint_pos is: "<<joint_pos<<std::endl;
    // joint_pos_ref[1] = 0;
    // joint_pos_ref[0] = 0;
    // fb_torque = KP * (joint_pos_ref - joint_pos);
    // std::cout<<"fb_torque is: "<<fb_torque<<std::endl;
    
    return fb_torque;

};
Eigen::Vector3f InvDynController::get_total_torque(){
    
    // if(!joint_pos_ref.isZero(0.001) && !joint_vel_ref.isZero(0.001) && !joint_acc_ref.isZero(0.001)){
            // torque = get_ff_torque() + get_fb_torque();
    Eigen::Vector3f ff = get_ff();
    Eigen::Vector3f ff2 = get_ff_torque();
    Eigen::Vector3f fb = get_fb_torque();
    std::cout<<"Feedforward Torque from M"<<ff<<std::endl<<"Feedforward Torque from long equation"<<ff2<<std::endl<<"Feedback Torque"<<fb<<std::endl;
    return ff;
    // return get_ff_torque() + get_fb_torque();
    // return get_fb_torque();
    // }
    // else{
        // 
        // std::cout<<"Some values not initialized yet"<<std::endl;
        // return Eigen::Vector3f::Zero();
    // }


};


