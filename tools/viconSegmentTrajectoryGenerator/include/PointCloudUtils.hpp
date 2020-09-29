/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BM_PCL_UTILS_H
#define BM_PCL_UTILS_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <Eigen/Dense>

namespace PointCloudUtils
{

using PCLXYZ = pcl::PointCloud<pcl::PointXYZ>;
using PCLXYZPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

bool readPCDFile(const std::string& pcdFilePath,
                 PCLXYZPtr& cloud,
                 bool printDetails = false)
{
    if (pcl::io::loadPCDFile(pcdFilePath, *cloud) == -1)
    {
        std::cerr << "Unable to read from PCD file in "
                  << pcdFilePath << std::endl;
        return false;
    }

    if (printDetails)
    {
        std::cout << "Loaded "
                  << cloud->width*cloud->height
                  << " data points from " << pcdFilePath
                  << " with the following fields" << std::endl;
        for (const auto& point : *cloud)
        {
            std::cout << "   " << point.x
                      << " "   << point.y
                      << " "   << point.z << std::endl;
        }
    }

    return true;
}

bool runICP(const PCLXYZPtr& srcCloud,
            const PCLXYZPtr& targetCloud,
            double& fitnessScore,
            Eigen::Ref<Eigen::Matrix4f> T,
            bool verbose = false,
            PCLXYZPtr finalCloud = nullptr)
{
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(srcCloud);
    icp.setInputTarget(targetCloud);

    if (finalCloud == nullptr)
    {
        PCLXYZ final;
        icp.align(final);
    }
    else
    {
        icp.align(*finalCloud);
    }

    fitnessScore = icp.getFitnessScore();
    T = icp.getFinalTransformation();
    auto converged = icp.hasConverged();


    if (verbose)
    {
        std::cout << "===+=== " << std::endl;
        std::cout << "Final transformation between point clouds: " << std::endl;
        std::cout << "ICP has converged: " << converged << " score: "
                  << fitnessScore << std::endl;
        std::cout << "Final transformation between point clouds: " << std::endl;
        std::cout << T << std::endl;
        std::cout << "===o=== " << std::endl;
    }

    return converged;
}


} // end namespace PointCloudUtils

#endif //BM_PCL_UTILS_H
