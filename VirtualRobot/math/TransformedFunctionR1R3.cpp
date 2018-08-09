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


#include "TransformedFunctionR1R3.h"
#include "Helpers.h"

using namespace math;



TransformedFunctionR1R3::TransformedFunctionR1R3(const Eigen::Matrix4f& transformation, AbstractFunctionR1R3Ptr func)
    : transformation(transformation), func(func)
{

}


Eigen::Vector3f TransformedFunctionR1R3::Get(float t)
{
    return Helpers::TransformPosition(transformation, func->Get(t));
}

Eigen::Vector3f TransformedFunctionR1R3::GetDerivative(float t)
{
    return Helpers::TransformDirection(transformation, func->GetDerivative(t));
}
