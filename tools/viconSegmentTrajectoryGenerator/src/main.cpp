/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include <iostream>

#include "PointCloudUtils.hpp"
#include <ResourceFolderPath.h>


int main(int argc, char** argv)
{
    std::string modelPCDPath{getViconMountPCDPath()};
    std::string matFilePath{getDatasetPath()};

    auto refCloud = boost::make_shared<PointCloudUtils::PCLXYZ>();

    if (!PointCloudUtils::readPCDFile(modelPCDPath, refCloud, /*printDetails = */ true))
    {
        return -1;
    }

    return 0;
}

