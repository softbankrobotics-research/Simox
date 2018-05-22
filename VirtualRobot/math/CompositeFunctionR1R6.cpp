/*
 * This file is part of ArmarX.
 * 
 * Copyright (C) 2012-2016, High Performance Humanoid Technologies (H2T),
 * Karlsruhe Institute of Technology (KIT), all rights reserved.
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

#include "CompositeFunctionR1R6.h"
#include "Helpers.h"
#include "Line.h"
#include "LinearInterpolatedOrientation.h"

using namespace math;

CompositeFunctionR1R6::CompositeFunctionR1R6(const AbstractFunctionR1R3Ptr &position, const AbstractFunctionR1OriPtr &orientation, float startT, float endT)
    : position(position), orientation(orientation), startT(startT), endT(endT)
{ }


Eigen::Vector3f CompositeFunctionR1R6::GetPosition(float t)
{
    return position->Get(Helpers::ILerp(startT, endT, t));
}


Eigen::Quaternionf CompositeFunctionR1R6::GetOrientation(float t)
{
    return orientation->Get(Helpers::ILerp(startT, endT, t));
}

Eigen::Vector3f CompositeFunctionR1R6::GetPositionDerivative(float t)
{
    return position->GetDerivative(Helpers::ILerp(startT, endT, t)) / (endT - startT);
}

Eigen::Vector3f CompositeFunctionR1R6::GetOrientationDerivative(float t)
{
    return orientation->GetDerivative(Helpers::ILerp(startT, endT, t)) / (endT - startT);
}

class ConstantOrientation
        : public AbstractFunctionR1Ori
{
public:
    ConstantOrientation(const Eigen::Quaternionf &ori)
        :ori(ori)
    { }
    Eigen::Quaternionf Get(float t) override
    {
        return ori;
    }
    Eigen::Vector3f GetDerivative(float t) override
    {
        return Eigen::Vector3f::Zero();
    }
private:
    Eigen::Quaternionf ori;
};


CompositeFunctionR1R6Ptr CompositeFunctionR1R6::CreateLine(const Eigen::Vector3f &startPos, const Eigen::Vector3f &endPos, const Eigen::Quaternionf &startOri, const Eigen::Quaternionf &endOri, float startT, float endT)
{

    LinePtr line(new Line(startPos, endPos - startPos));
    LinearInterpolatedOrientationPtr ori(new LinearInterpolatedOrientation(startOri, endOri, 0, 1, true));

    CompositeFunctionR1R6Ptr func(new CompositeFunctionR1R6(line, ori, startT, endT));
    return func;
}

CompositeFunctionR1R6Ptr CompositeFunctionR1R6::CreateLine(const Eigen::Vector3f &startPos, const Eigen::Vector3f &endPos, const Eigen::Quaternionf &ori, float startT, float endT)
{
    LinePtr line(new Line(startPos, endPos - startPos));
    boost::shared_ptr<ConstantOrientation> constOri(new ConstantOrientation(ori));

    CompositeFunctionR1R6Ptr func(new CompositeFunctionR1R6(line, constOri, startT, endT));
    return func;
}
