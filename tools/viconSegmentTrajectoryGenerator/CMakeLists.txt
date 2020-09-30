# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

cmake_minimum_required(VERSION 3.14)

project(ViconSegmentTrajectoryGenerator CXX)

find_package(Eigen3 3.2.92 REQUIRED)
find_package(matlogger2 REQUIRED)
find_package(PCL REQUIRED common io registration kdtree visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(app_name viconSegmentTrajectoryGenerator)
set(src_files src/main.cpp
              src/MatHandler.cpp
              include/MatHandler.hpp
              include/PointCloudUtils.hpp
              include/SegmentData.hpp)

include_directories(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include/)
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/ResourceFolderPath.h.in" "${CMAKE_CURRENT_BINARY_DIR}/ResourceFolderPath.h" @ONLY)

add_executable(${app_name} ${src_files})
target_include_directories(${app_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(${app_name} PUBLIC matio
                                          matlogger2::matlogger2
                                          Eigen3::Eigen
                                          ${PCL_LIBRARIES})


