/*
* This file is part of ArmarX.
*
* ArmarX is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* ArmarX is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @author     Simon Ottenhaus (simon dot ottenhaus at kit dot edu)
* @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
*             GNU General Public License
*/

#pragma once

#include "MathForwardDefinitions.h"
#include "DataR3R1.h"
#include "SimpleAbstractFunctionR3R1.h"

namespace math
{

class GaussianImplicitSurface3D :
        public SimpleAbstractFunctionR3R1
{
public:
    //GaussianImplicitSurface3D();
    void Calculate(std::vector<DataR3R1> samples, float noise);
    float Get(Eigen::Vector3f pos) override;

private:
    Eigen::MatrixXf covariance;
    Eigen::VectorXf alpha;
    std::vector<DataR3R1> samples;
    float R;
    Eigen::Vector3f mean;


    float Predict(Eigen::Vector3f pos);
    static Eigen::MatrixXf CalculateCovariance(std::vector<Eigen::Vector3f> points, float R, float noise);
    //static VectorXf Cholesky(MatrixXf matrix);
    //static VectorXf FitModel(MatrixXf L, List<DataR3R1> targets);
    //VectorXf SpdMatrixSolve(MatrixXf a, bool IsUpper, VectorXf b);
    static Eigen::VectorXf MatrixSolve(Eigen::MatrixXf a, Eigen::VectorXf b);
    static float Kernel(Eigen::Vector3f p1, Eigen::Vector3f p2, float R);
    //static double[,] Transpose(double[,] a)
    static Eigen::Vector3f Average(std::vector<DataR3R1> samples);
};
}

