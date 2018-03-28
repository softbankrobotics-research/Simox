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
* @author     Martin Miller (martin dot miller at student dot kit dot edu)
* @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
*             GNU General Public License
*/

#ifndef math_GaussianImplicitSurface3D
#define math_GaussianImplicitSurface3D

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
    float Get(Vec3 pos) override;

private:
    MatrixXf covariance;
    VectorXf alpha;
    std::vector<DataR3R1> samples;
    float R;
    Vec3 mean;


    float Predict(Vec3 pos);
    static MatrixXf CalculateCovariance(std::vector<Vec3> points, float R, float noise);
    //static VectorXf Cholesky(MatrixXf matrix);
    //static VectorXf FitModel(MatrixXf L, List<DataR3R1> targets);
    //VectorXf SpdMatrixSolve(MatrixXf a, bool IsUpper, VectorXf b);
    static VectorXf MatrixSolve(MatrixXf a, VectorXf b);
    static float Kernel(Vec3 p1, Vec3 p2, float R);
    //static double[,] Transpose(double[,] a)
    static Vec3 Average(std::vector<DataR3R1> samples);
};
}

#endif // math_GaussianImplicitSurface3D
