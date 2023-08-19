// #include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp"

 
int main()
{

  const std::string urdf_filename = std::string("/Users/stav.42/ws/src/man_controller/eom/src/urdf2eom/URDFs/manipulator.urdf");

  // Load the urdf model
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_filename,model);
  std::cout << "model name: " << model.name << std::endl;

  pinocchio::Data data(model);
 
  Eigen::VectorXd q = pinocchio::neutral(model);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
 
  const Eigen::VectorXd & tau = pinocchio::rnea(model,data,q,v,a);
  std::cout << "tau = " << tau.transpose() << std::endl;
}