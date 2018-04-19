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

#pragma once

#include "MathForwardDefinitions.h"




namespace math
{

class AbstractFunctionR1R2
{
public:
    AbstractFunctionR1R2();
    virtual Eigen::Vector2f Get(float t) = 0;
    virtual Eigen::Vector2f GetDerivative(float t)= 0;
    Eigen::Vector2f GetNormalizedNormal(float t);

private:
};
}

