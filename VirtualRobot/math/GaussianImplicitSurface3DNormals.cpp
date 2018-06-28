/**
 * This file is part of Simox.
 *
 * Simox is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * Simox is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @author     Simon Ottenhaus (simon dot ottenhaus at kit dot edu)
 * @copyright  2018 Simon Ottenhaus
 *             GNU Lesser General Public License
 */

#include "GaussianImplicitSurface3DNormals.h"
#include <cmath>

using namespace math;

GaussianImplicitSurface3DNormals::GaussianImplicitSurface3DNormals(KernelType kernelType)
    : kernel(KernelWithDerivatives::Create(kernelType)) {}

void GaussianImplicitSurface3DNormals::Calculate(const ContactList& samples, float noise, float normalNoise, float normalScale)
{
    ContactList shiftedSamples;
    std::vector<Eigen::Vector3f> points;
    std::vector<Eigen::Vector3f> pointsOriginal;
    for(const auto& d : samples){        
        Eigen::Vector3f shiftedPos = d.Position();
        shiftedSamples.push_back(Contact(shiftedPos, d.Normal()*normalScale));
        points.push_back(shiftedPos);
        pointsOriginal.push_back(d.Position());
    }
    R = 0;
    for (const Eigen::Vector3f& p1 : points) {
        for (const Eigen::Vector3f& p2 : points) {
            R = std::max(R, (p1-p2).squaredNorm());
        }
    }
    R = std::sqrt(R);

    covariance = CalculateCovariance(pointsOriginal, R, noise, normalNoise);
    Eigen::VectorXf y(samples.size()*4);
    for (uint i = 0; i < samples.size(); i++)
    {
         y(i*4) = 0;
         y(i * 4 + 1) = samples.at(i).Normal().x();
         y(i * 4 + 2) = samples.at(i).Normal().y();
         y(i * 4 + 3) = samples.at(i).Normal().z();
    }
    alpha = MatrixSolve(covariance, y);
    this->samples = shiftedSamples;
}


float GaussianImplicitSurface3DNormals::Get(Eigen::Vector3f pos)
{
    return Predict(pos);
}

Eigen::VectorXf GaussianImplicitSurface3DNormals::getCux(const Eigen::Vector3f& pos)
{
     Eigen::VectorXf Cux (samples.size() * 4);
     for (std::size_t i = 0; i < samples.size(); i++)
     {
        Cux(i * 4) = kernel->Kernel(pos, samples.at(i).Position(), R);
        Cux(i * 4 + 1) = kernel->Kernel_dj(pos, samples.at(i).Position(), R, 0);
        Cux(i * 4 + 2) = kernel->Kernel_dj(pos, samples.at(i).Position(), R, 1);
        Cux(i * 4 + 3) = kernel->Kernel_dj(pos, samples.at(i).Position(), R, 2);
    }
    return Cux;//VectorXf;
}
float GaussianImplicitSurface3DNormals::Predict(const Eigen::Vector3f& pos)
{
    Eigen::VectorXf Cux(4*samples.size());
    Cux=getCux(pos);
    return Cux.dot(alpha);

}
Eigen::MatrixXf GaussianImplicitSurface3DNormals::CalculateCovariance(const std::vector<Eigen::Vector3f>& points, float R, float noise, float normalNoise)
{
    Eigen::MatrixXf covariance = Eigen::MatrixXf(points.size()*4, points.size()*4);

    for (size_t i = 0; i < points.size()*4; i++)
    {
        for (size_t j = i; j < points.size()*4; j++)
        {
            float cov = kernel->Kernel(points.at(i/4), points.at(j/4), R, i%4 , j%4);
            covariance(i, j) = cov;
            covariance(j, i) = cov;
        }
    }
    for (size_t i = 0; i < points.size(); i++)
    {
         covariance(i, i) += i % 4 == 0 ? noise*noise : normalNoise*normalNoise;
    }
    return covariance;   
}    
Eigen::VectorXf GaussianImplicitSurface3DNormals::MatrixSolve(Eigen::MatrixXf a, Eigen::VectorXf b)
{
    return a.colPivHouseholderQr().solve(b);
}
