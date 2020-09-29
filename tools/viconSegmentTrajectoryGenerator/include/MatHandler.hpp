/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BM_MATLOADER_H
#define BM_MATLOADER_H

#include <matlogger2/matlogger2.h>
#include <matlogger2/utils/mat_appender.h>
#include "matio.h"

#include <string>
#include <iostream>
#include <chrono>
#include <vector>

#include <Eigen/Dense>

namespace BenchmarkUtils
{
class MatHandler
{
public:
    MatHandler();
    ~MatHandler() { };

    XBot::MatLogger2::Ptr m_logger{nullptr};

    bool openMatio(const std::string& filename);
    void setLoggerPrefix(const std::string& prefix) {m_logger_prefix = prefix;}
    void openMatLogger();
    void closeMatio();
    void closeMatLogger();

    void flush_logger_data() { m_logger->flush_available_data(); }
    std::string getLoggerPrefix() const { return m_logger_prefix; }
    std::string getLoggerFileName() const { return m_logger->get_filename(); }

    bool readVectorOfVectorDoubles(const std::string& var_name, std::vector<Eigen::VectorXd>& vec);

    template <typename T>
    inline bool readVectorOfScalars(const std::string& var_name, std::vector<T>& vec)
    {
        matvar_t* X{nullptr};
        if (!readVarInfo(var_name, X)) { return false; }

        if (X->rank != 2)
        {
            std::cout << "MatHandler is expecting a vector." << std::endl;
            Mat_VarFree(X);
            return false;
        }

        auto d0{X->dims[0]};
        auto d1{X->dims[1]};

        if (d0 == 1)
        {
            vec.resize(d1);
        }
        else if (d1 == 1)
        {
            vec.resize(d0);
        }
        else
        {
            std::cout << "MatHandler is expecting a nx1  or 1xn vector." << std::endl;
            Mat_VarFree(X);
            return false;
        }

        if (!readVarData(X, d0, d1, var_name, vec.data()))
        {
            Mat_VarFree(X);
            return false;
        }

        Mat_VarFree(X);
        return true;
    }

private:
    bool readVarInfo(const std::string& var_name, matvar_t*& X);

    template <typename T>
    inline bool readVarData(matvar_t*& X, const size_t& d0, const size_t& d1, const std::string& var_name, T* data)
    {
        int start[2] = {0, 0};
        int stride[2] = {1, 1};
        int edge[2] = {static_cast<int>(d0), static_cast<int>(d1)};
        if (Mat_VarReadData(matfp, X, data, start, stride, edge))
        {
            std::cout << "MatHandler could not read "<< var_name <<" vector." << std::endl;
            return false;
        }

        return true;
    }

    mat_t* matfp{nullptr};
    std::string m_logger_prefix{"benchmark-base-estimator"};
};

}
#endif

