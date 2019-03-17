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

#include "Kernels.h"
#include "Helpers.h"
#include <cmath>
#include <iostream>

using namespace math;

float KernelWithDerivatives::Kernel(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, float R, int i, int j) const
{
    if (i == 0 && j == 0) return Kernel(p1, p2, R);
    if (i == 0) return Kernel_dj(p1, p2, R, j - 1);
    if (j == 0) return Kernel_di(p1, p2, R, i - 1);
    return Kernel_didj(p1, p2, R, i - 1, j - 1);
}

float KernelWithDerivatives::Kernel_di(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, float R, int i) const
{
    float r = (p1-p2).norm();
    float x = p1.x() - p2.x();
    float y = p1.y() - p2.y();
    float z = p1.z() - p2.z();
    swap(x, y, z, i);
    return Kernel_dx(x, y, z, r, R);
}
float KernelWithDerivatives::Kernel_dj(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, float R, int j) const
{
   float r = (p1-p2).norm();
   float x = p1.x() - p2.x();
   float y = p1.y() - p2.y();
   float z = p1.z() - p2.z();
   swap(x, y, z, j);
   return -Kernel_dx(x, y, z, r, R);
}

float KernelWithDerivatives::Kernel_didj(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, float R, int i, int j) const
{
   float r = (p1-p2).norm();
   float x = p1.x() - p2.x();
   float y = p1.y() - p2.y();
   float z = p1.z() - p2.z();
   if (i == j)
   {
       swap(x, y, z, i);       
       return -Kernel_ddx(x, y, z, r, R);
   }
    std::vector<float> a(3);
    a.at(0)=x;
    a.at(1)=y;
    a.at(2)=z;

    x = a.at(i);
    y = a.at(j);
    z = a.at(3 - i - j);
    return -Kernel_dxdy(x, y, z, r, R);
}

void KernelWithDerivatives::swap(float &x, float &y, float &z, int index) const
{
   switch (index)
   {
       case 0: break;
       case 1: math::Helpers::Swap(x, y); break;
       case 2: math::Helpers::Swap(x, z); break;
       default: throw std::invalid_argument("index must be >= 0 and <= 2" );
    }
}

GaussianKernel::GaussianKernel(float lengthScale)
    : lengthScale(lengthScale) {}

float GaussianKernel::Kernel(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, float /*R*/) const
{
    const float tmp = (p1 - p2).squaredNorm() / (2*lengthScale*lengthScale);
    //std::cout << (2*lengthScale*lengthScale) << " " << (p1 - p2).squaredNorm() << " " <<  tmp << std::endl;
    return std::exp(-tmp);
}

float GaussianKernel::Kernel_dx(float /*x*/, float /*y*/, float /*z*/, float /*r*/, float /*R*/) const
{
    throw std::runtime_error("function not implemented");
}

float GaussianKernel::Kernel_ddx(float /*x*/, float /*y*/, float /*z*/, float /*r*/, float /*R*/) const
{
    throw std::runtime_error("function not implemented");
}

float GaussianKernel::Kernel_dxdy(float /*x*/, float /*y*/, float /*z*/, float /*r*/, float /*R*/) const
{
    throw std::runtime_error("function not implemented");
}


float WilliamsPlusKernel::Kernel(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, float R) const
{
    float r = (p1-p2).norm();
    return 2 * r*r*r + 3 * R * r*r + R*R*R;
}

float WilliamsPlusKernel::Kernel_dx(float x, float /*y*/, float /*z*/, float r, float R) const
{
    return 6 * x * (r + R);
}

float WilliamsPlusKernel::Kernel_ddx(float x, float /*y*/, float /*z*/, float r, float R) const
{
    if (r == 0) return 6 * R;
    return 6 * (R + r) + 6 * x * x / r;
}

float WilliamsPlusKernel::Kernel_dxdy(float x, float y, float /*z*/, float r, float /*R*/) const
{
    if (r == 0) return 0;
    return 6 * x * y / r;
}

float WilliamsMinusKernel::Kernel(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, float R) const
{
    float r = (p1-p2).norm();
    return 2 * r*r*r - 3 * R * r*r + R*R*R;
}

float WilliamsMinusKernel::Kernel_dx(float x, float /*y*/, float /*z*/, float r, float R) const
{
    return 6 * x * (r - R);
}

float WilliamsMinusKernel::Kernel_ddx(float x, float /*y*/, float /*z*/, float r, float R) const
{
    if (r == 0) return 6 * R;
    return -6 * (r - R) + 6 * x * x / r;
}

float WilliamsMinusKernel::Kernel_dxdy(float x, float y, float /*z*/, float r, float /*R*/) const
{
    if (r == 0) return 0;
    return -6 * x * y / r;
}
