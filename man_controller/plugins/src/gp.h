#include <eigen3/Eigen/Dense>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <man_controller/Traj.h>
#include <math.h>

class GP_fit{
    public:
        int max_size = 10;
        Eigen::VectorXf states;
        int pop_element = 0;
        float sigma_w = 10;
        float sigma_n = 10;
        Eigen::MatrixXf data_acc;
        float mean;
        float std_dev;
        bool flag; 
        Eigen::VectorXf robust_corr;
        Eigen::MatrixXf K;
        Eigen::MatrixXf Kn;
        Eigen::MatrixXf M;
        Eigen::VectorXf J;

        GP_fit();
        Eigen::MatrixXf get_prediction(Eigen::VectorXf new_state);
        Eigen::VectorXf get_Kn(Eigen::VectorXf new_state);
        Eigen::MatrixXf get_K();
        float kernel(Eigen::VectorXf d1, Eigen::VectorXf d2);
        void add_data(Eigen::VectorXf states, float obs);

};