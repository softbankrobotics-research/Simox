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
#include <cmath>
#include <iostream>

using namespace math;

GaussianImplicitSurface3D::GaussianImplicitSurface3D(std::unique_ptr<KernelWithDerivatives> kernel)
    : kernel(std::move(kernel)) {}


void GaussianImplicitSurface3D::Calculate(const std::vector<DataR3R1>& samples, float noise)
{
    std::vector<DataR3R2> samples2;
    for(const DataR3R1& d: samples)
    {
        samples2.push_back(DataR3R2(d.Position(), d.Value(), noise));
    }
    Calculate(samples2);
}

void GaussianImplicitSurface3D::Calculate(const std::vector<DataR3R2>& samples)
{
    this->samples = samples;
    std::vector<Eigen::Vector3f> points;
    Eigen::VectorXd values(samples.size());
    std::vector<float> noise;
    int i = 0;

    for(const auto& d: samples){
        points.push_back(d.Position());
        values(i++) = d.Value1();
        noise.push_back(d.Value2());
    }

    R = 0;

    for (const Eigen::Vector3f& p1 : points) {
        for (const Eigen::Vector3f& p2 : points) {
            R = std::max(R, (p1 - p2).squaredNorm());
        }
    }
    R = std::sqrt(R);

    CalculateCovariance(points, R, noise);
    MatrixInvert(values);
}

float GaussianImplicitSurface3D::Get(Eigen::Vector3f pos)
{
    return Predict(pos);
}

float GaussianImplicitSurface3D::GetVariance(const Eigen::Vector3f& pos)
{
    Eigen::VectorXd Cux(samples.size());
    int i = 0;
    for (const auto& d : samples)
    {
        Cux(i++) = kernel->Kernel(pos, d.Position(), R);
    }

    return kernel->Kernel(pos, pos, R) - Cux.dot(covariance_inv * Cux);
}

float GaussianImplicitSurface3D::Predict(const Eigen::Vector3f& pos) const
{
    Eigen::VectorXd Cux(samples.size());
    int i = 0;
    for (const auto& d : samples)
    {
        Cux(i++) = kernel->Kernel(pos, d.Position(), R);
    }
    return Cux.dot(alpha);
}

void GaussianImplicitSurface3D::CalculateCovariance(const std::vector<Eigen::Vector3f>& points, float R, const std::vector<float>& noise)
{
    covariance = Eigen::MatrixXd(points.size(), points.size());

    for (size_t i = 0; i < points.size(); i++)
    {
        for (size_t j = i; j < points.size(); j++)
        {
            float cov = kernel->Kernel(points.at(i), points.at(j), R);
            covariance(i, j) = cov;
            covariance(j, i) = cov;
        }
    }
    for (size_t i = 0; i < points.size(); i++)
    {
        covariance(i, i) += noise.at(i) * noise.at(i);
    }
}

void GaussianImplicitSurface3D::MatrixInvert(const Eigen::VectorXd& b)
{
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr = covariance.colPivHouseholderQr();
    covariance_inv = qr.inverse();
    alpha = covariance_inv * b;
}
