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

#include "AbstractFunctionR1Ori.h"
#include "AbstractFunctionR1R3.h"
#include "AbstractFunctionR1R6.h"
#include "MathForwardDefinitions.h"

namespace math
{

    class CompositeFunctionR1R6
            : public AbstractFunctionR1R6
    {
    public:
        CompositeFunctionR1R6(const AbstractFunctionR1R3Ptr& position, const AbstractFunctionR1OriPtr& orientation, float startT, float endT);

        Eigen::Vector3f GetPosition(float t) override;
        Eigen::Quaternionf GetOrientation(float t) override;
        Eigen::Vector3f GetPositionDerivative(float t) override;
        Eigen::Vector3f GetOrientationDerivative(float t) override;

        static CompositeFunctionR1R6Ptr CreateLine(const Eigen::Vector3f& startPos, const Eigen::Vector3f& endPos, const Eigen::Quaternionf& startOri, const Eigen::Quaternionf& endOri, float startT, float endT);
        static CompositeFunctionR1R6Ptr CreateLine(const Eigen::Vector3f& startPos, const Eigen::Vector3f& endPos, const Eigen::Quaternionf& ori, float startT, float endT);

    private:
        AbstractFunctionR1R3Ptr position;
        AbstractFunctionR1OriPtr orientation;
        float startT;
        float endT;
    };
}
