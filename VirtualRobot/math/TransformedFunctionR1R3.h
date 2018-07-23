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

#pragma once


#include "AbstractFunctionR1R3.h"


namespace math
{
    class TransformedFunctionR1R3 :
            public AbstractFunctionR1R3
    {
    public:
        TransformedFunctionR1R3(const Eigen::Matrix4f& transformation, AbstractFunctionR1R3Ptr func);
        Eigen::Vector3f Get(float t) override;
        Eigen::Vector3f GetDerivative(float t) override;

    private:
        const Eigen::Matrix4f transformation;
        const AbstractFunctionR1R3Ptr func;
    };
}

