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
#include "AbstractFunctionR1Ori.h"
#include "MathForwardDefinitions.h"

#ifndef Q_MOC_RUN // workaround for some bug in some QT/boost versions
#include <boost/shared_ptr.hpp>
#endif
namespace math
{

    class LinearInterpolatedOrientation
            : public AbstractFunctionR1Ori
    {
    public:
        LinearInterpolatedOrientation(const Eigen::Quaternionf& startOri, const Eigen::Quaternionf& endOri, float startT, float endT, bool clamp);
        LinearInterpolatedOrientation(const Eigen::Matrix3f& startOri, const Eigen::Matrix3f& endOri, float startT, float endT, bool clamp);
        Eigen::Quaternionf Get(float t) override;
        Eigen::Vector3f GetDerivative(float t) override;

    private:
        Eigen::Quaternionf startOri;
        Eigen::Quaternionf endOri;
        float startT;
        float endT;
        bool clamp;
        Eigen::Vector3f derivative;
        Eigen::Vector3f axis;
        float angle;

    };
}
