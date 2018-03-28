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
* @author     Andreea Tulbure (andreea dot tulbure at student dot kit dot edu)
* @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
*             GNU General Public License
*/

#ifndef math_GaussianImplicitSurfaceNormals3D
#define math_GaussianImplicitSurfaceNormals3D

#include "MathForwardDefinitions.h"
#include "Contact.h"
#include "Helpers.h"
#include "SimpleAbstractFunctionR3R1.h"
#include "ContactList.h"

namespace math
{

class GaussianImplicitSurface3DNormals :
        public SimpleAbstractFunctionR3R1
{
public:
    GaussianImplicitSurface3DNormals();
    void Calculate(ContactList samples, float noise, float normalNoise, float normalScale);
    float Get(Vec3 pos) override;

private:
    MatrixXf covariance;
    VectorXf alpha;
    ContactList samples;
    float R;
    float R3;
    Vec3 mean;

    MatrixXf Cinv;

    float Predict(Vec3 pos);
    static MatrixXf CalculateCovariance(std::vector<Vec3> points, float R, float noise, float normalNoise);
    //static VectorXf Cholesky(MatrixXf matrix);
    //static VectorXf FitModel(MatrixXf L, List<DataR3R1> targets);
    //VectorXf SpdMatrixSolve(MatrixXf a, bool IsUpper, VectorXf b);
    static VectorXf MatrixSolve(MatrixXf a, VectorXf b);
    static float Kernel(Vec3 p1, Vec3 p2, float R);
    static float Kernel(Vec3 p1, Vec3 p2, float R, int i, int j);
    static float Kernel_dx(float x, float y, float z, float r, float R);
    static float Kernel_dxdy(float x, float y, float z, float r, float R);
    static float Kernel_ddx(float x, float y, float z, float r, float R);
    static float Kernel_di(Vec3 p1, Vec3 p2, float R, int i);
    static float Kernel_dj(Vec3 p1, Vec3 p2, float R, int i);
    static float Kernel_didj(Vec3 p1, Vec3 p2, float R, int i, int j);
    static void swap(float &x, float &y,float &z, int index);
    static Vec3 Average(ContactList samples);
    VectorXf getCux(Vec3 pos);
};
}

#endif // math_GaussianImplicitSurface3DNormals
