#include <eigen3/Eigen/Dense>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <man_controller/Traj.h>
#include <math.h>
#include "gp.h"

GP_fit::GP_fit(){
    max_size = 10;
    
    // M_l = Eigen::MatrixXf::Zero(9, 9);

    states = Eigen::VectorXf::Zero(9);

    // M_l <<0.1, 0, 0, 0, 0, 0, 0, 0, 0,
    //     0, 0.10, 0, 0, 0, 0, 0, 0, 0,
    //     0, 0, 0.10, 0, 0, 0, 0, 0, 0,
    //     0, 0, 0, 10, 0, 0, 0, 0, 0,
    //     0, 0, 0, 0, 10, 0, 0, 0, 0,
    //     0, 0, 0, 0, 0, 10, 0, 0, 0,
    //     0, 0, 0, 0, 0, 0, 10, 0, 0,
    //     0, 0, 0, 0, 0, 0, 0, 10, 0,
    //     0, 0, 0, 0, 0, 0, 0, 0, 10;

    M_l = 0.4 * Eigen::MatrixXf::Identity(9, 9);
    M_l(6, 6) = 0.5*90;
    M_l(7, 7) = 0.5*90;
    M_l(8, 8) = 0.5*90;
    

    sigma_n = 1;
    sigma_w = 1e-3;

    data_acc = Eigen::MatrixXf(9, 0);

    J = Eigen::VectorXf(0);
    flag = 0;
    K = Eigen::MatrixXf::Zero(max_size, max_size);

}

float GP_fit::kernel(Eigen::VectorXf d1, Eigen::VectorXf d2){
    // std::cout<<"D1 is: \n"<<d1.transpose()<<"D2 is: \n"<<d2.transpose()<<std::endl;
    // std::cout<<"M_l is: \n"<<M_l<<std::endl;
    float exp_n = (d1 - d2).transpose() * M_l.inverse() * M_l.inverse() * (d1 - d2);
    // std::cout<<"exp_n is \n"<<exp_n<<std::endl;
    float val = sigma_n * sigma_n * exp(-0.5 * exp_n);
    // std::cout<<"val is \n"<<val<<std::endl;
    return val;
}

Eigen::MatrixXf GP_fit::get_K(){
    
    Eigen::MatrixXf K_new(max_size, max_size);
    K_new = Eigen::MatrixXf::Zero(max_size, max_size);
    // std::cout<<" Data collected "<<std::endl<<data_acc<<std::endl;

    for(int i = 0; i<data_acc.cols(); i++){
        for(int j = i; j<data_acc.cols(); j++){
            float k_val = kernel(data_acc.col(i), data_acc.col(j));
            // std::cout<<"Kernel value is: "<<k_val<<std::endl;
            K_new(i, j) = k_val;
            K_new(j, i) = k_val;
        }
    }

    return K_new;
}

void GP_fit::add_data(Eigen::VectorXf states, float obs){
    int size = data_acc.cols(); 
    int n_states = data_acc.rows();
    // std::cout<<"Current size of the input data: "<<size<<std::endl;

    if(size<max_size){
        // std::cout<<"Checkpoint 8"<<std::endl;
        data_acc.conservativeResize(n_states, size+1);
        // std::cout<<"Checkpoint 9"<<" "<<data_acc.cols()<<std::endl;
        data_acc.col(size) = states;
        // std::cout<<"Checkpoint 10"<<std::endl;
        J.conservativeResize(J.size()+1);
        // std::cout<<"J is "<<std::endl<<J.transpose()<<std::endl;
        J(J.size()-1) = obs;
    } else {
        // std::cout<<"Checkpoint 6"<<std::endl;
        data_acc.col(pop_element) = states;
        J(pop_element) = obs;
        pop_element++;
        pop_element = pop_element%max_size;
        // std::cout<<"Checkpoint 7"<<std::endl;
        flag = 1;
        K = get_K();
        // std::cout<<"K is: "<<std::endl<<K<<std::endl;
    }

    // std::cout<<"Checkpoint 10"<<std::endl;

}

Eigen::VectorXf GP_fit::get_Kn(Eigen::VectorXf new_state){
    Eigen::VectorXf K_n(max_size);
    for(int i=0;i<data_acc.cols();i++){
        K_n(i) = kernel(new_state, data_acc.col(i));
    }
    return K_n;
}

Eigen::MatrixXf GP_fit::get_prediction(Eigen::VectorXf new_state){

    // std::cout<<"Checkpoint 11"<<std::endl;
    Eigen::VectorXf K_n = get_Kn(new_state);
    // std::cout<<"J: \n"<<J.transpose()<<std::endl;
    mean = K_n.transpose() * (K + Eigen::MatrixXf::Identity(K.cols(), K.cols()) * sigma_w).inverse() * J;
    // std::cout<<"Checkpoint 12"<<std::endl;
    std_dev = kernel(new_state, new_state) - K_n.transpose() * (K + Eigen::MatrixXf::Identity(K.cols(), K.cols()) * sigma_w).inverse() * K_n;

    // std::cout<<"Type and size of mean and std_dev: "<<mean<<" "<<std_dev<<std::endl;

    Eigen::VectorXf result(2);
    result(0) = mean;
    result(1) = std_dev;

    return result;
}