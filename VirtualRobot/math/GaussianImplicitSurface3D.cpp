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

#include "GaussianImplicitSurface3D.h"

using namespace math;




void GaussianImplicitSurface3D::Calculate(std::vector<DataR3R1> samples, float noise)
{
    std::vector<DataR3R1> shiftedSamples;
    std::vector<Vec3> points;
    VectorXf values(samples.size());
    int i = 0;
    mean = Average(samples);
    for(DataR3R1 d : samples){
        Vec3 shiftedPos = d.Position()- mean;
        shiftedSamples.push_back(DataR3R1(shiftedPos, d.Value()));
        points.push_back(shiftedPos);
        values(i++) = d.Value();
    }
    R = 0;
    for(Vec3 p1 : points){
        for(Vec3 p2 : points){
            if((p1-p2).norm() > R) R= (p1-p2).norm();
        }
    }
    R = std::sqrt(R);

    covariance = CalculateCovariance(points, R, noise);
    alpha = MatrixSolve(covariance, values);
    //L = Cholesky(covariance);
    //alpha = FitModel(L, samples);
    this->samples = shiftedSamples;

}

float GaussianImplicitSurface3D::Get(Vec3 pos)
{
    return Predict(pos);
}

float GaussianImplicitSurface3D::Predict(Vec3 pos)
{
    VectorXf Cux(samples.size());
    int i = 0;
    pos = pos - mean;
    for (DataR3R1 d : samples){
        Cux(i++) = Kernel(pos, d.Position(), R);
    }
    return Cux.dot( alpha);

}

MatrixXf GaussianImplicitSurface3D::CalculateCovariance(std::vector<Vec3> points, float R, float noise)
{
    MatrixXf covariance = MatrixXf(points.size(), points.size());

    for (size_t i = 0; i < points.size(); i++)
    {
        for (size_t j = i; j < points.size(); j++)
        {
            float cov = Kernel(points.at(i), points.at(j), R);
            covariance(i, j) = cov;
            covariance(j, i) = cov;
        }
    }
    for (size_t i = 0; i < points.size(); i++)
    {
        covariance(i, i) += noise * noise;
    }
    return covariance;
}



VectorXf GaussianImplicitSurface3D::MatrixSolve(MatrixXf a, VectorXf b)
{
    return a.colPivHouseholderQr().solve(b);
}


float GaussianImplicitSurface3D::Kernel(Vec3 p1, Vec3 p2, float R)
{
    float r = (p1-p2).norm();
    return 2 * r*r*r + 3 * R * r*r + R*R*R;
}

Vec3 GaussianImplicitSurface3D::Average(std::vector<DataR3R1> samples)
{
    Vec3 sum = Vec3(0,0,0);
    if (samples.size() == 0) return sum;
    for(DataR3R1 d : samples){
        sum += d.Position();
    }
    return sum / samples.size();
}
