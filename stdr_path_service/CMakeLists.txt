cmake_minimum_required(VERSION 2.8.3)
project(stdr_path_service)

find_package(catkin_simple REQUIRED)

catkin_simple()

# Executables
cs_add_executable(server src/server.cpp)
cs_add_executable(path_client src/path_client.cpp)
cs_install()
cs_export()
    
