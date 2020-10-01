/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include <iostream>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <fstream>

#include "MatHandler.hpp"
#include "PointCloudUtils.hpp"
#include <ResourceFolderPath.h>
#include "SegmentData.hpp"

#include <pcl/visualization/cloud_viewer.h>

using namespace std::chrono_literals;

namespace BenchmarkUtils
{
void decomposeHomogeneousTransform(Eigen::Ref<const Eigen::Matrix4f> T,
                                   Eigen::Ref<Eigen::Vector3f> xyz,
                                   Eigen::Ref<Eigen::Vector3f> rpy)
{
    Eigen::Quaternionf quat;
    quat = T.block<3, 3>(0, 0);

    rpy = quat.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
    xyz = T.block<3, 1>(0, 3);
}

void setupLoggerVars(BenchmarkUtils::MatHandler& ml, const std::string& prefix = "")
{
    ml.m_logger->create(prefix+"time", 1);
    ml.m_logger->create(prefix+"xyz", 3);
    ml.m_logger->create(prefix+"rpy", 3);
}

void updateLoggerVars(BenchmarkUtils::MatHandler& ml,
                      const double& time,
                      Eigen::Ref<const Eigen::Matrix4f> T,
                      const std::string& prefix = "")
{
    ml.m_logger->add(prefix+"time", time);
    Eigen::Vector3f xyz, rpy;
    decomposeHomogeneousTransform(T, xyz, rpy);
    ml.m_logger->add(prefix+"xyz", xyz);
    ml.m_logger->add(prefix+"rpy", rpy);
}

}

int main(int argc, char** argv)
{
    std::string modelPCDPath{getViconMountPCDPath()};
    std::string matFilePath{getDatasetPath()};

    auto refCloud = boost::make_shared<BenchmarkUtils::PCLXYZ>();

    if (!BenchmarkUtils::readPCDFile(modelPCDPath, refCloud, /*printDetails = */ true))
    {
        return -1;
    }

    BenchmarkUtils::MatHandler ml;
    if (!ml.openMatio(matFilePath))
    {
        return -1;
    }

    BenchmarkUtils::SegmentMarkersTrajectory segTraj;
    if (!segTraj.populateMarkerTrajFromMat(ml))
    {
        return -1;
    }

    ml.setLoggerPrefix("vicon");
    ml.openMatLogger();
    BenchmarkUtils::setupLoggerVars(ml);

    std::cout << "Nr markers read from mat file: " << segTraj.nrMarkers <<  std::endl;
    std::cout << "Minimum trajectory start time: " << segTraj.minStartTime <<  " ns." << std::endl;
    std::cout << "Maximum trajectory end time: " << segTraj.maxEndTime <<  " ns." <<std::endl;
    std::cout << "Total trajectory duration: " << segTraj.maxEndTime - segTraj.minStartTime <<  " s." <<std::endl;

    size_t nrMarkers{segTraj.nrMarkers};
    double startTime{segTraj.minStartTime};
    double endTime{segTraj.maxEndTime};
    double currentTime{startTime};
    std::vector<double> timeVector;
    segTraj.getMergedTimeVector(timeVector);
//     double dt{0.02};
    double tol_for_fusiontemporal{1e-4};

    size_t totalIter{0};
    size_t failIter{0};
    std::unordered_map<size_t, size_t> currentMarkerIter;
    std::unordered_map<size_t, bool> canUseMarkerPosition;
    // initialize currentMarkerIters
    for (size_t idx = 1; idx <= nrMarkers; idx++)
    {
        currentMarkerIter[idx] = 0;
        canUseMarkerPosition[idx] = false;
    }


    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    viewer.setBackgroundColor(0, 0, 0);

//     viewer.addCoordinateSystem(1.0);
    Eigen::Matrix4f Base_H_ViconWorld = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f initialGuessB_H_Vicon = Eigen::Matrix4f::Identity();

//     while (currentTime < endTime)
    while (totalIter < timeVector.size())
//     while (totalIter < 2)
    {
        currentTime = timeVector[totalIter];

        segTraj.getCurrentTimeMarkerData(currentTime, tol_for_fusiontemporal, canUseMarkerPosition, currentMarkerIter);

        // build point cloud
        auto srcCloud = boost::make_shared<BenchmarkUtils::PCLXYZ>();

        segTraj.buildPointCloudFromCurrentMarkerData(canUseMarkerPosition, currentMarkerIter, srcCloud);
        if (srcCloud->width < 3)
        {
            totalIter++;
            failIter++;
            continue;
        }

        // run icp here
        auto startComp = std::chrono::high_resolution_clock::now();
        double fitnessScore{0.0};

        bool verbose{true};
        auto finalCloud = boost::make_shared<BenchmarkUtils::PCLXYZ>();
        BenchmarkUtils::runICP(srcCloud, refCloud, fitnessScore, initialGuessB_H_Vicon, Base_H_ViconWorld, verbose, finalCloud);
        initialGuessB_H_Vicon = Base_H_ViconWorld;
        auto endComp = std::chrono::high_resolution_clock::now();
        double compTime = std::chrono::duration_cast<std::chrono::microseconds>( endComp - startComp ).count();
        BenchmarkUtils::updateLoggerVars(ml, currentTime, Base_H_ViconWorld);

        std::ofstream outFile;
        std::string fileName{"srcCloud/"+std::to_string(totalIter)+".xyz"};
        outFile.open(fileName);

        for (const auto& pt : srcCloud->points)
        {
            outFile << pt.x << ", " << pt.y << ", " << pt.z << "\n";
        }
        outFile.close();

        if (totalIter > 0)
        {
            viewer.removePointCloud("srcCloud");
            viewer.removePointCloud("refCloud");
            viewer.removePointCloud("aligned");
            viewer.removeText3D("srcCloudTxt");
            viewer.removeText3D("refCloudTxt");
            viewer.removeText3D("alignedTxt");
        }
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green_color(srcCloud, 0, 255, 0);
        viewer.addPointCloud<pcl::PointXYZ>(srcCloud, green_color, "srcCloud");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 16, "srcCloud");


        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color(refCloud, 255, 0, 0);
        viewer.addPointCloud<pcl::PointXYZ>(refCloud, red_color, "refCloud");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 16, "refCloud");


        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue_color(finalCloud, 0, 0, 255);
        viewer.addPointCloud<pcl::PointXYZ>(finalCloud, blue_color, "aligned");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 16, "aligned");
        double txtScale{0.01};
        viewer.addText3D("Src Cloud", srcCloud->points[0], txtScale, 0, 255, 0, "srcCloudTxt");
        viewer.addText3D("Ref Cloud", refCloud->points[0], txtScale, 255, 0, 0, "refCloudTxt");
        viewer.addText3D("Aligned Cloud", finalCloud->points[0], txtScale, 0, 0, 255, "alignedTxt");
        viewer.spinOnce(100);
        std::this_thread::sleep_for(0.02s);
//         do
//  {
//    cout << '\n' << "Press a key to continue...";
//  } while (cin.get() != '\n');

//         currentTime += dt;
        totalIter++;
        for (size_t idx = 1; idx <= nrMarkers; idx++)
        {
            if (canUseMarkerPosition[idx])
            {
                if (currentMarkerIter[idx] < segTraj.markerMaxIter[idx])
                {
                    currentMarkerIter[idx]++;
                }
                canUseMarkerPosition[idx] = false;
            }
        }
    }


    ml.closeMatio();
    std::cout << "Total Iterations: " << totalIter << std::endl;
    std::cout << "Iterations without pointcloud: " << failIter << std::endl;

    return 0;
}

