cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

# build messages and services
rosbuild_genmsg()
rosbuild_gensrv()
