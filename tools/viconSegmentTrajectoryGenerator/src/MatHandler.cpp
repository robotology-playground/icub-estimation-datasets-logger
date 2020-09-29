/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */



#include "MatHandler.hpp"


namespace BenchmarkUtils
{

    MatHandler::MatHandler() { }

    bool MatHandler::openMatio(const std::string& filename)
    {
        matfp = Mat_Open(filename.c_str(), MAT_ACC_RDWR);
        if (matfp == nullptr)
        {
            std::cerr << "MatHandler could not open "<< filename <<" file." << std::endl;
            return false;
        }
        return true;
    }

    void MatHandler::closeMatio()
    {
        Mat_Close(matfp);
    }

    void MatHandler::closeMatLogger()
    {
        m_logger->~MatLogger2();
    }

    void MatHandler::openMatLogger()
    {
        m_logger = XBot::MatLogger2::MakeLogger(m_logger_prefix);
        m_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
    }

    bool MatHandler::readVectorOfVectorDoubles(const std::string& var_name, std::vector<Eigen::VectorXd>& vec)
    {
        matvar_t* X{nullptr};
        if (!readVarInfo(var_name, X)) { return false; }

        if (X->rank != 2)
        {
            std::cout << "MatHandler is expecting a vector of  vector for variable " << var_name  << std::endl;
            Mat_VarFree(X);
            return false;
        }

        auto d0{X->dims[0]};
        auto d1{X->dims[1]};

        if (d0 == 1 || d1 == 1)
        {
            std::cout << "MatHandler is expecting a nxm vector  where n is the nr. of vectors and m is nr. of vector elements." << std::endl;
            Mat_VarFree(X);
            return false;
        }

        std::vector<double > temp;
        temp.resize(d0*d1);
        if (!readVarData(X, d0, d1, var_name, temp.data()))
        {
            Mat_VarFree(X);
            return false;
        }

        vec.resize(d0);
        for (size_t idx = 0; idx < d0; idx++)
        {
            vec[idx].resize(d1);
        }

        size_t temp_iter=0;
        for (size_t idx = 0; idx < d1; idx++)
        {
            for (size_t loop = 0; loop < d0; loop++)
            {
                if (temp_iter > temp.size())
                {
                    Mat_VarFree(X);
                    return false;
                }

                vec[loop](idx) = temp[temp_iter];
                temp_iter++;
            }
        }

        Mat_VarFree(X);
        return true;
    }


    bool MatHandler::readVarInfo(const std::string& var_name, matvar_t*& X)
    {
        X = Mat_VarReadInfo(matfp, var_name.c_str());

        if (X == nullptr)
        {
            std::cout << "MatHandler could not find variable "<< var_name  << std::endl;
            return false;
        }
        return true;
    }

}

