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


#include "TransformedFunctionR2R3.h"
#include "Helpers.h"

using namespace math;



TransformedFunctionR2R3::TransformedFunctionR2R3(const Eigen::Matrix4f& transformation, AbstractFunctionR2R3Ptr func)
    : transformation(transformation), inv(transformation.inverse()), func(func)
{

}



Eigen::Vector3f math::TransformedFunctionR2R3::GetPoint(float u, float v)
{
    return Helpers::TransformPosition(transformation, func->GetPoint(u, v));
}

Eigen::Vector3f math::TransformedFunctionR2R3::GetDdu(float u, float v)
{
    return Helpers::TransformDirection(transformation, func->GetDdu(u, v));
}

Eigen::Vector3f math::TransformedFunctionR2R3::GetDdv(float u, float v)
{
    return Helpers::TransformDirection(transformation, func->GetDdv(u, v));
}

void math::TransformedFunctionR2R3::GetUV(Eigen::Vector3f pos, float& u, float& v)
{
    func->GetUV(Helpers::TransformPosition(inv, pos), u, v);
}
