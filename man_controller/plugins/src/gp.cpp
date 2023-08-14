#include <eigen3/Eigen/Dense>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <man_controller/Traj.h>
#include <math.h>
#include "gp.h"

GP_fit::GP_fit(){
    max_size = 10;
    M <<10, 0, 0, 0, 0, 0, 0,
        0, 10, 0, 0, 0, 0, 0,
        0, 0, 10, 0, 0, 0, 0,
        0, 0, 0, 10, 0, 0, 0,
        0, 0, 0, 0, 10, 0, 0,
        0, 0, 0, 0, 0, 10, 0,
        0, 0, 0, 0, 0, 0, 10;

}

float GP_fit::kernel(Eigen::Vector7f d1, Eigen::Vector7f d2){
    float exp_n = (d1 - d2).transpose() * M * (d1 - d2);
    float val = sigma_n * sigma_n * exp(-0.5 * exp_n);
    return val;
}

Eigen::MatrixXf GP_fit::get_K(){
    
    for(i = 0; i<data_acc.cols(); i++){
        for(j = i+1; j<data_acc.cols(); j++){
            float k_val = kernel(data_acc.col(i), data_acc.col(j));
            K(i, j) = k_val;
            K(j, i) = k_val;
        }
    }

    return K;
}

void GP_fit::add_data(Eigen::Vector7f states, float obs){
    int size = data_acc.cols();
    int n_states = data_acc.rows();
    cout<<"Current size of the input data: "<<size<<std::endl;

    if(size<max_size){
        data_acc.conservativeResize(n_states, size+1)
        data_acc.col(size+1) = states;
        J(J.size() + 1) = obs;
    } else {
        data_acc.col(pop_element) = states;
        J(pop_element) = obs;
        pop_element++;
        pop_element = pop_element%max_size;
    }

    // Reconstructing K and kn matrices
    Eigen::MatrixXf = get_K();

}

Eigen::VectorXf GP_fit::get_Kn(Eigen::VectorXf new_state){
    Eigen::VectorXf K_n;
    for(int i=0;i<acc_data.cols();i++){
        K_n(i) = kernel(new_state, acc_data.col(i));
    }

    return K_n;
}

Eigen::MatrixXf GP_fit::get_prediction(Eigen::VectorXf new_state){

    Eigen::VectorXf K_n = get_Kn(new_state);
    mean = K_n * (K + Eigen::MatrixXf::identity(K.cols()) * sigma_w).inverse() * J;
    std_dev = kernel(new_state, new_state) - K_n * (K + Eigen::MatrixXf::identity(K.cols()) * sigma_w).inverse() * K_n.transpose();

    Eigen::MatrixXf result;
    result(0) = mean;
    result(1) = mean;

    return result;
}