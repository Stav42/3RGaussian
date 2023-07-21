#include <vector>
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <thread>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Time.h"
#include "rosgraph_msgs/Clock.h"

#include <man_controller/Traj.h>
#include <boost/bind.hpp>

I