cmake_minimum_required(VERSION 3.10)

project(binding_test)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)

# Find Python
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

# Find Eigen
find_package(Eigen3 REQUIRED NO_MODULE)

# Include NLopt
# find_package(NLopt REQUIRED)
# include_directories(${NLopt_INCLUDE_DIRS})
include_directories(/Users/stav.42/miniconda3/envs/ros_env/lib/python3.9/site-packages/numpy/core/include)

# Add your source file
add_executable(test test.cpp)

# Link libraries
target_link_libraries(test Eigen3::Eigen ${PYTHON_LIBRARIES} ${NLopt_LIBRARIES})

# If you have a specific path for NumPy, you can include it
# include_directories(/path/to/numpy)
