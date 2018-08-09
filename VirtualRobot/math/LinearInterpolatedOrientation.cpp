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

#include "Helpers.h"
#include "LinearInterpolatedOrientation.h"

//#include <iostream>

using namespace math;

LinearInterpolatedOrientation::LinearInterpolatedOrientation(const Eigen::Quaternionf &startOri, const Eigen::Quaternionf &endOri, float startT, float endT, bool clamp)
    :startOri(startOri), endOri(endOri), startT(startT), endT(endT), clamp(clamp)
{
    Eigen::AngleAxisf aa(endOri * startOri.inverse());
    angle = Helpers::AngleModPI(aa.angle());
    axis = aa.axis();
    Eigen::Vector3f oriDelta = fabs(angle) < 0.01 ? Eigen::Vector3f::Zero() : Eigen::Vector3f(aa.axis() * angle);

    derivative = oriDelta / (endT - startT);
}

LinearInterpolatedOrientation::LinearInterpolatedOrientation(const Eigen::Matrix3f &startOri, const Eigen::Matrix3f &endOri, float startT, float endT, bool clamp)
    : LinearInterpolatedOrientation(Eigen::Quaternionf(startOri), Eigen::Quaternionf(endOri), startT, endT, clamp)
{
}


Eigen::Quaternionf math::LinearInterpolatedOrientation::Get(float t)
{
    if(derivative.squaredNorm() < 0.001)
    {
        return startOri;
    }
    float f = Helpers::ILerp(startT, endT, t);
    //std::cout << "before clamp: " << f << std::endl;
    if(clamp)
    {
        f = Helpers::Clamp(0, 1, f);
    }
    //std::cout << "after clamp: " << f << std::endl;
    //Helpers::Lerp(startOri, endOri, f);
    Eigen::AngleAxisf aa(f * angle, axis);
    return Eigen::Quaternionf(aa * startOri);
}

Eigen::Vector3f math::LinearInterpolatedOrientation::GetDerivative(float t)
{
    if(clamp && (t < startT || t > endT))
    {
        return Eigen::Vector3f::Zero();
    }
    return derivative;

}
