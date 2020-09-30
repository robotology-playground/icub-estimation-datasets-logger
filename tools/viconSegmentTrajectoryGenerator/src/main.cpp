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

#include "MatHandler.hpp"
#include "PointCloudUtils.hpp"
#include <ResourceFolderPath.h>
#include "SegmentData.hpp"

#include <pcl/visualization/cloud_viewer.h>

using namespace std::chrono_literals;

namespace BenchmarkUtils
{
void temporalAdjustment(const std::string& measure_type, const double& current_est_time,
                        const std::vector<double>& measure_time, const size_t& max_measure_iter,
                        bool& can_set_measure, size_t& current_measure_iter, const double& tol)
{
    // temporal calibration of measurements to achieve a clean sensor fusion
    // for setting corresponding measurements to the estimator
    if (current_measure_iter < max_measure_iter)
    {
        if ( std::abs(current_est_time - measure_time[current_measure_iter]) < tol )
        {
            can_set_measure = true;
        }
        else
        {
            // skip through measurements to catch up with the estimator time
            if (measure_time[current_measure_iter] < current_est_time)
            {
                size_t temp_iter;
                for (temp_iter = current_measure_iter; measure_time[temp_iter] < current_est_time ;temp_iter++) {
                    if (temp_iter >= max_measure_iter)
                    {
                        temp_iter = max_measure_iter;
                        break;
                    }
                }
                current_measure_iter = temp_iter;
                if ( std::abs(current_est_time - measure_time[current_measure_iter]) < tol )
                {
                    can_set_measure = true;
                }
            }
        }
    }
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

    std::cout << "Nr markers read from mat file: " << segTraj.nrMarkers <<  std::endl;
    std::cout << "Minimum trajectory start time: " << segTraj.minStartTime <<  " ns." << std::endl;
    std::cout << "Maximum trajectory end time: " << segTraj.maxEndTime <<  " ns." <<std::endl;
    std::cout << "Total trajectory duration: " << segTraj.maxEndTime - segTraj.minStartTime <<  " s." <<std::endl;

    size_t nrMarkers{segTraj.nrMarkers};
    double startTime{segTraj.minStartTime};
    double endTime{segTraj.maxEndTime};
    double currentTime{startTime};
    double dt{0.02};
    double tol_for_fusiontemporal{0.02};

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


    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    while (currentTime < endTime)
    {
        // flags to check if current iteration time and measurement time at the measurement iteration index
        // are close to each other in order to set the measurement and increment the measurement iteration index


        // temporal adjustment of measurements to wait for or catch up with estimator runtime
        for (size_t idx = 1; idx <= nrMarkers; idx++)
        {
            std::string markerName{"marker"+ std::to_string(idx)};
            auto& markerIter = currentMarkerIter[idx];
            auto& useMarker = canUseMarkerPosition[idx];
            BenchmarkUtils::temporalAdjustment(markerName, currentTime, segTraj.markerTime[idx], segTraj.markerMaxIter[idx], useMarker, markerIter, tol_for_fusiontemporal);
        }

        // build point cloud
        auto srcCloud = boost::make_shared<BenchmarkUtils::PCLXYZ>();
        segTraj.buildPointCloudFromCurrentMarkerData(canUseMarkerPosition, currentMarkerIter, srcCloud);
        if (srcCloud->width < 3)
        {
//             std::cerr << "PCL at time: " << currentTime << " has only "<< srcCloud->width << " points. Skipping forward by one step ... " << std::endl;
            currentTime += dt;
            totalIter++;
            failIter++;
            continue;
        }

        viewer.showCloud(srcCloud);
        std::this_thread::sleep_for(0.02s);

        // run icp here
        auto startComp = std::chrono::high_resolution_clock::now();
        double fitnessScore{0.0};
        Eigen::Matrix4f Base_H_ViconWorld = Eigen::Matrix4f::Identity();
        bool verbose{true};
        BenchmarkUtils::runICP(refCloud, srcCloud, fitnessScore, Base_H_ViconWorld, verbose);

        auto endComp = std::chrono::high_resolution_clock::now();
        double compTime = std::chrono::duration_cast<std::chrono::microseconds>( endComp - startComp ).count();

        currentTime += dt;
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

    while(!viewer.wasStopped())
    {
    }

    std::cout << "Total Iterations: " << totalIter << std::endl;
    std::cout << "Iterations without pointcloud: " << failIter << std::endl;

    return 0;
}

