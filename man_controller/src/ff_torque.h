#include <eigen3/Eigen/Dense>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <man_controller/Traj.h>
#include <math.h>



class InvDynController {
    private:
        Eigen::VectorXf joint_pos;
        Eigen::VectorXf joint_vel;
        Eigen::VectorXf joint_acc;

        Eigen::VectorXf joint_pos_ref;
        Eigen::VectorXf joint_vel_ref;
        Eigen::VectorXf joint_acc_ref;
        
        Eigen::MatrixXf I_1;
        Eigen::MatrixXf I_2;
        Eigen::MatrixXf I_3;

        float L1, L2, L3;
        float m1, m2, m3;

        Eigen::MatrixXf KP, KD;
        
        Eigen::VectorXf ff_torque, fb_torque, total_torque;

        InvDynController(){
            I_1 = {
                {13.235, 0, 0},
                {0, 13.235, 0},
                {0, 0, 9.655}
            };

            I_2 = {
                {12.679, 0, 0},
                {0, 12.679, 0},
                {0, 0, 0.651}
            };
            I_3 = {
                {12.679, 0, 0},
                {0, 12.679, 0},
                {0, 0, 0.651}
            };
            m1 = 157.633;
            m2 = 57.986;
            m3 = 57.986;
            L_1 = 0.4;
            L_2 = 0.8;
            L_3 = 0.8;

            KP = {
                {100, 0, 0}, 
                {0, 100, 0}, 
                {0, 0, 100}
            }

            KD = {
                {100, 0, 0}, 
                {0, 100, 0}, 
                {0, 0, 100}
            }

        }

        Eigen::VectorXf get_ff_torque(){
            

            /// Joint_1
            float term1 = -0.5 * I_3(0, 0) * math.sin(2 * (joint_pos_ref(1) + joint_pos_ref(2))) * joint_vel_ref(0) ** 2; 
            float term2 = 0.5 * I_3(1, 1) * math.sin(2 * (joint_pos_ref(1) + joint_pos_ref(2))) * joint_vel_ref(0) ** 2;
            float term3 = I_3(2, 2) * (joint_acc_ref(1) + joint_acc_ref(2)) + 0.25 * L2 * L3 * m3 * math.sin(2 * joint_pos_ref(1) + joint_pos_ref(2)) * joint_acc_ref(0);
            float term4 = 0.25 * L2 * L3 * m3 * math.sin(joint_pos_ref(2)) * joint_acc_ref(0);
            float term5 = 0.5 * L2 * L3 * m3 * (math.sin(joint_pos_ref(2)) * joint_pos_ref(1) ** 2);
            float term6 = 0.5 * L2 * L3 * m3 * (math.cos(joint_pos_ref(2)) * joint_acc_ref(1));
            float term7 = 0.125 * L3 ** 2 * m3 * math.sin(2* (joint_pos_ref(1) + joint_pos_ref(2))) * joint_vel_ref(0);
            float term8 = 0.25 * L3 ** 2 * m3 * (joint_acc_ref(1) + joint_acc_ref(2));
            float term9 = 0.5 * L3 * g * m3 * math.cos(joint_pos_ref(1) + joint_pos_ref(2));

            float ff_1 =  term1 + term2 + term3 + term4 + term5 + term6 + term7 + term8 + term9;

            /// Joint_2
            float term1 = -0.5*(I_2(0, 0) + I_2(1, 1))*math.sin(2 * joint_pos_ref(1)) * joint_vel_ref(0) ** 2 + I_2(2, 2) * joint_ref_acc(2) - 0.5 * (I_3(1, 1)-I_3(0, 0))*math.sin(2*joint_pos_ref(1)+2*joint_pos_ref(2)) * joint_vel_ref(0) ** 2;
            float term2 = I_3(2, 2) * (joint_acc_ref(2) + joint_acc_ref(2)) + 0.125 * L_2 ** 2 * m_2 * math.sin(2 * joint_pos_ref(1)) * joint_vel_ref(0) ** 2;
            float term3 = 0.25 * L_2 ** 2 * m2 * joint_acc_ref(1) + 0.5 * L_2**2 * m3 * math.sin(2 * joint_pos_ref(1)) * joint_vel_ref(0) ** 2;
            float term4 = L_2 ** 2 * m3 * joint_acc_ref(2) + 0.5 * L2 * L3 * m3* math.sin(2*joint_pos_ref(1) + joint_pos_ref(2))*joint_vel_ref(0) ** 2;
            float term5 = -1 * L_2 * L_3 * m3 * math.sin(joint_pos_ref(2)) * joint_vel_ref(1) * joint_vel_ref(2) - 0.5 * L_2 * L_3 *m3 * math.sin(joint_pos_ref(2)) * joint_vel_ref(2)**2;
            float term6 = L_2 * L_3 *m3 * math.cos(joint_pos_ref(2)) * joint_acc_ref(1) + 0.5 * L_2 * L_3 *m3 * math.cos(joint_pos_ref(2)) * joint_acc_ref(2);
            float term7 = 0.5 * L_2 * g * m2 * math.cos(joint_pos_ref(1)) + 1.0 * L_2 * g * m3 * math.cos(joint_pos_ref(1));
            float term8 = 0.125 * L_3 ** 2 * m3 * math.sin(2 * (joint_pos_ref(1) + joint_pos_ref(2))) * joint_vel_ref(0) ** 2;
            float term9 = 0.25 * L_3 ** 2 * m3 * (joint_acc_ref(1) + joint_acc_ref(2)) + 0.5 * L_3 * g * m3 * math.cos(joint_pos_ref(1) + joint_pos_ref(2));

            float ff_2 =  term1 + term2 + term3 + term4 + term5 + term6 + term7 + term8 + term9;

            // Joint_3
            float term1 = I_1(2, 2) * joint_acc_ref(0) + I_2(0, 0) * math.sin(joint_pos_ref) ** 2 * joint_acc_ref(0) + (-1 * I_2(0, 0) + I_2(1, 1)) * math.sin(2 * joint_pos_ref(1)) * joint_vel_ref(0) * joint_vel_ref(1);
            float term2 = I_2(1, 1) * math.cos(joint_pos_ref(1)) ** 2 * joint_acc_ref(0) + I_3(0, 0) * (joint_vel_ref(0) + joint_vel_ref(1)) * math.sin(2 * (joint_pos_ref(1) + joint_pos_ref(2))) * joint_vel_ref(0);
            float term3 = I_3(0, 0) * math.sin(joint_pos_ref(1) + joint_pos_ref(2)) * joint_acc_ref(0) - I_3(1, 1) * (joint_vel_ref(1)+ joint_vel_ref(2)) * math.sin(2 * (joint_pos_ref(1) + joint_pos_ref(2))) * joint_vel_ref(0);
            float term4 = I_3(1, 1) * math.cos(joint_pos_ref(1) + joint_pos_ref(2))**2 * joint_acc_ref(0) - 0.25 * L_2 **2 * m2 * math.sin(2*joint_pos_ref(1))*joint_vel_ref(0) * joint_vel_ref(1);
            float term5 = 0.25 * L_2 ** 2 * m2 * math.cos(joint_pos_ref(1)) ** 2 * joint_acc_ref(0);
            float term6 = 0.25 * m3 * (2 * L_2 * math.cos(joint_pos_ref(1)) + L_3 * math.cos(joint_pos_ref(2) + joint_pos_ref(1)))**2*joint_acc_ref(0);
            float term7 = -0.5 * m3 * (2 * L_2 * math.cos(joint_pos_ref(1)) + L_3 * math.cos(joint_pos_ref(2) + joint_pos_ref(1))) * (2 * L_2 * math.sin(joint_pos_ref(1)) * joint_vel_ref(1) + L_3 * (joint_vel_ref(1) + joint_vel_ref(2)) *math.sin(joint_pos_ref(1) + joint_pos_ref(2)) ) * joint_vel_ref(0);

            float ff_3 = term1 + term2 + term3 + term4 + term5 + term6 + term7;

            Eigen::VectorXf ff_torque = {ff_1, ff_2, ff_3};

            return ff_torque;
        }

        Eigen::VectorXf get_fb_torque(){
            
            Eigen::VectorXf fb = KP * (joint_pos_ref - joint_pos) + KD * (joint_vel_ref - joint_vel);
            return fb;

        };

        Eigen::VectorXf get_total_torque(){
            
            Eigen::VectorXf torque = get_ff_torque() + get_fb_torque();
            return torque;

        };



}

