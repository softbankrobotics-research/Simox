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

#pragma once

#include "AbstractFunctionR1Ori.h"
#include "AbstractFunctionR1Ori.h"
#include "MathForwardDefinitions.h"

#include <boost/shared_ptr.hpp>

namespace math
{

    class LinearInterpolatedOrientation
            : public AbstractFunctionR1Ori
    {
    public:
        LinearInterpolatedOrientation(const Eigen::Quaternionf& startOri, const Eigen::Quaternionf& endOri, float startT, float endT, bool clamp);
        Eigen::Quaternionf Get(float t) override;
        Eigen::Vector3f GetDerivative(float t) override;

    private:
        Eigen::Quaternionf startOri;
        Eigen::Quaternionf endOri;
        float startT;
        float endT;
        bool clamp;
        Eigen::Vector3f derivative;

    };
}
