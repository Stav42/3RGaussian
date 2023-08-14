#include <eigen3/Eigen/Dense>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <man_controller/Traj.h>
#include <math.h>

class GP_fit{
    public:
        int max_size = 10;
        int states = 7;
        int pop_element = 0;
        Eigen::MatrixXf data_acc;
        Eigen::VectorXf mean;
        Eigen::VectorXf std_dev;
        Eigen::VectorXf robust_corr;
        Eigen::MatrixXf K;
        Eigen::MatrixXf Kn;
        Eigen::MatrixXf get_K;
        Eigen::Matrix7f M;


        GP_fit();
        Eigen::MatrixXf get_prediction();
        Eigen::VectorXf get_correction();
        
        void fit_data();
        void add_data();

}