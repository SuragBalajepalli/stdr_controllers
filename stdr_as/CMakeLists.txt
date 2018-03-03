cmake_minimum_required(VERSION 2.8.3)
project(stdr_as)

find_package(catkin_simple REQUIRED)
# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)
catkin_simple()

# Executables
cs_add_executable(stdr_as src/stdr_as.cpp)
cs_add_executable(stdr_ac src/stdr_ac.cpp)
cs_install()
cs_export()
    
