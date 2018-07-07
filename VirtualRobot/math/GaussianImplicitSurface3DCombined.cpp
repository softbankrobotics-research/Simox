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

#include "GaussianImplicitSurface3DCombined.h"
#include <cmath>

using namespace math;

GaussianImplicitSurface3DCombined::GaussianImplicitSurface3DCombined(std::unique_ptr<KernelWithDerivatives> kernel)
    : kernel(std::move(kernel)) {}

void GaussianImplicitSurface3DCombined::Calculate(const ContactList& normalSamples, const std::vector<DataR3R1>& samples, float noise, float normalNoise, float normalScale)
{
    ContactList shiftedNormalSamples;
    std::vector<DataR3R1> shiftedSamples;
    std::vector<Eigen::Vector3f> points;
    std::vector<Eigen::Vector3f> pointsNormalOriginal;
    std::vector<Eigen::Vector3f> pointsOriginal;
    for (const auto& d : normalSamples)
    {        
        const Eigen::Vector3f pos = d.Position();
        shiftedNormalSamples.emplace_back(pos, d.Normal() * normalScale);
        points.push_back(pos);
        pointsNormalOriginal.push_back(pos);
    }

    for (const auto& d : samples)
    {        
        const Eigen::Vector3f pos = d.Position();
        shiftedSamples.emplace_back(pos, d.Value());
        points.push_back(pos);
        pointsOriginal.push_back(pos);
    }

    R = 0;
    for (Eigen::Vector3f p1 : points)
    {
        for (Eigen::Vector3f p2 : points)
        {
            R = std::max(R, (p1-p2).squaredNorm());
        }
    }
    R = std::sqrt(R);

    CalculateCovariance(pointsOriginal, pointsNormalOriginal, R, noise, normalNoise);

    Eigen::VectorXd y(samples.size() + normalSamples.size() * 4);
    for (size_t i = 0; i < samples.size(); i++)
    {
        y(i) = samples.at(i).Value();
    }

    for (uint i = 0; i < normalSamples.size(); i++)
    {
         y(samples.size() + i * 4) = 0;
         y(samples.size() + i * 4 + 1) = normalSamples.at(i).Normal().x();
         y(samples.size() + i * 4 + 2) = normalSamples.at(i).Normal().y();
         y(samples.size() + i * 4 + 3) = normalSamples.at(i).Normal().z();
    }

    MatrixInvert(y);

    this->samples = shiftedSamples;
    this->normalSamples = shiftedNormalSamples;
}


float GaussianImplicitSurface3DCombined::Get(Eigen::Vector3f pos)
{
    return Predict(pos);
}

Eigen::VectorXd GaussianImplicitSurface3DCombined::getCux(const Eigen::Vector3f& pos)
{
    Eigen::VectorXd Cux (samples.size() + normalSamples.size() * 4);
    for (std::size_t i = 0; i < samples.size(); i++)
    {
        Cux(i) = kernel->Kernel(pos, samples.at(i).Position(), R);
    }

    for (std::size_t i = 0; i < normalSamples.size(); i++)
    {
        Cux(samples.size() + i * 4) = kernel->Kernel(pos, normalSamples.at(i).Position(), R);
        Cux(samples.size() + i * 4 + 1) = kernel->Kernel_dj(pos, normalSamples.at(i).Position(), R, 0);
        Cux(samples.size() + i * 4 + 2) = kernel->Kernel_dj(pos, normalSamples.at(i).Position(), R, 1);
        Cux(samples.size() + i * 4 + 3) = kernel->Kernel_dj(pos, normalSamples.at(i).Position(), R, 2);
    }
    return Cux;
}

float GaussianImplicitSurface3DCombined::Predict(const Eigen::Vector3f& pos)
{
    Eigen::VectorXd Cux(samples.size() + normalSamples.size() * 4);
    Cux = getCux(pos);
    return Cux.dot(alpha);
}

void GaussianImplicitSurface3DCombined::CalculateCovariance(const std::vector<Eigen::Vector3f>& points, const std::vector<Eigen::Vector3f>& normalPoints, float R, float noise, float normalNoise)
{
    covariance = Eigen::MatrixXd(points.size() + normalPoints.size() * 4, points.size() + normalPoints.size() * 4);

    // Just points
    for (size_t i = 0; i < points.size(); i++)
    {
        for (size_t j = i; j < points.size(); j++)
        {
            const float cov = kernel->Kernel(points.at(i), points.at(j), R);
            covariance(i, j) = cov;
            covariance(j, i) = cov;
        }
    }

    // Points + Normal
    for (size_t i = 0; i < normalPoints.size() * 4; i++)
    {
        for (size_t j = i; j < normalPoints.size() * 4; j++)
        {
            const float cov = kernel->Kernel(normalPoints.at(i/4), normalPoints.at(j/4), R, i%4 , j%4);
            covariance(points.size() + i, points.size() + j) = cov;
            covariance(points.size() + j, points.size() + i) = cov;
        }
    }

    // Mix of both
    for (size_t i = 0; i < points.size(); i++)
    {
        for (size_t j = 0; j < normalPoints.size() * 4; j++)
        {
            const float cov = kernel->Kernel(points.at(i), normalPoints.at(j/4), R, 0, j%4);
            covariance(i, points.size() + j) = cov;
            covariance(points.size() + j, i) = cov;
        }
    }

    // Noise for points
    for (size_t i = 0; i < points.size(); i++)
    {
        covariance(i, i) += noise * noise;
    }

    // Noise for Points + Normal
    for (size_t i = 0; i < normalPoints.size(); i++)
    {
        covariance(points.size() + i, points.size() + i) += (i % 4 == 0 ? noise*noise : normalNoise*normalNoise);
    }
}

float GaussianImplicitSurface3DCombined::GetVariance(const Eigen::Vector3f& pos)
{
    Eigen::VectorXd Cux(4 * samples.size());
    Cux = getCux(pos);

    return kernel->Kernel(pos, pos, R) - Cux.dot(covariance_inv * Cux);
}

void GaussianImplicitSurface3DCombined::MatrixInvert(const Eigen::VectorXd& b)
{
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr = covariance.colPivHouseholderQr();
    covariance_inv = qr.inverse();
    alpha = covariance_inv * b;
}