/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BM_SEGMENT_DATA_H
#define BM_SEGMENT_DATA_H

#include <vector>
#include <unordered_map>
#include <limits>
#include <algorithm>

#include <Eigen/Dense>
#include "MatHandler.hpp"
#include "PointCloudUtils.hpp"

namespace BenchmarkUtils
{
    struct SegmentMarkersTrajectory
    {
        std::unordered_map<std::size_t, size_t > markerMaxIter;
        std::unordered_map<std::size_t, std::vector<double> > markerTime;
        std::unordered_map<std::size_t, std::vector<Eigen::VectorXd> > markerTraj;
        std::size_t nrMarkers{8};
        double minStartTime{std::numeric_limits<double>::max()};
        double maxEndTime{std::numeric_limits<double>::min()};

        bool populateMarkerTrajFromMat(MatHandler& ml)
        {
            bool ok{true};

            for (std::size_t idx = 1; idx <= nrMarkers; idx++)
            {
                std::string time{"marker"+std::to_string(idx)+"time"};
                std::string xyz{"marker"+std::to_string(idx)+"xyz"};
                ok = ok && ml.readVectorOfScalars(time, markerTime[idx]);
                ok = ok && ml.readVectorOfVectorDoubles(xyz, markerTraj[idx]);

                if (markerTime.at(idx).size() != markerTraj.at(idx).size())
                {
                    std::cerr << "Size mismatch in read from mat file" << std::endl;
                    ok = false;
                    break;
                }

                markerMaxIter[idx] = markerTime.at(idx).size();

                if (!ok)
                {
                    break;
                }

                if (markerTime.at(idx)[0] < minStartTime)
                {
                    minStartTime = markerTime.at(idx)[0];
                }

                if (markerTime.at(idx)[markerMaxIter[idx]-1] > maxEndTime)
                {
                    maxEndTime = markerTime.at(idx)[markerMaxIter[idx]-1];
                }
            }

            if (!ok)
            {
                std::cerr << "Could not read from mat file" << std::endl;
                return false;
            }
            return true;
        }

        void buildPointCloudFromCurrentMarkerData(const std::unordered_map<size_t, bool>& canUseMarkerPosition,
                                                  const std::unordered_map<size_t, size_t>& currentMarkerIter,
                                                  PCLXYZPtr cloud)
        {
            auto count = 0;
            for (auto&& use : canUseMarkerPosition)
            {
                if (use.second)
                {
                    auto currentIter = currentMarkerIter.at(use.first);
                    Eigen::VectorXd xyz = markerTraj.at(use.first)[currentIter];
                    if (xyz.size() != 3)
                    {
                        continue;
                    }

                    pcl::PointXYZ point;
                    point.x = xyz[0];
                    point.y = xyz[1];
                    point.z = xyz[2];

                    cloud->points.emplace_back(point);
                    ++count;
                }
            }
            cloud->width = count;
            cloud->height = 1;
            cloud->is_dense = false;
        }

        void getMergedTimeVector(std::vector<double>& time)
        {
            time.clear();
            for (const auto& data : markerTime)
            {
                std::vector<double> temp;
                const auto& timeVec = data.second;
                std::merge(timeVec.begin(), timeVec.end(),
                           time.begin(), time.end(),
                           std::back_inserter(temp));
                time.clear();
                time = temp;
            }

            time.erase(std::unique(time.begin(), time.end()), time.end());
        }

        void getCurrentTimeMarkerData(const double& currentTime,
                                      const double& timeTolerance,
                                      std::unordered_map<size_t, bool>& canUseMarkerPosition,
                                      std::unordered_map<size_t, size_t>& currentMarkerIter)
        {
            for (const auto& data : markerTime)
            {
                const auto& timeVec = data.second;
                auto const it = std::find(timeVec.begin(), timeVec.end(), currentTime);
                if (it == timeVec.end())
                {
                    canUseMarkerPosition[data.first] = false;
                    continue;
                }

                auto index = std::distance(timeVec.begin(), it);


                    std::cout << "========\n" << std::setprecision(15) << "Current Time: " << currentTime << " \nMarker idx: " << data.first
                              << " \nTime idx: " << index
                              << " \nTime at index: " << timeVec[index]
                              << " \n Diff: " <<  std::abs(currentTime - timeVec[index])
                              << std::endl;



                // check for current index
                if (std::abs(currentTime - timeVec[index]) < timeTolerance)
                {
                    canUseMarkerPosition[data.first] = true;
                    currentMarkerIter[data.first] = index;
                    continue;
                }
                canUseMarkerPosition[data.first] = false;
            }
        }

    };

}

#endif // BM_SEGMENT_DATA_H
