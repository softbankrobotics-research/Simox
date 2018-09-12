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

#include "../VirtualRobot.h"
#include "MathForwardDefinitions.h"
#include "DataR3R1.h"
#include "DataR3R2.h"
#include "SimpleAbstractFunctionR3R1.h"
#include "Kernels.h"
#include <memory>

namespace math
{

class VIRTUAL_ROBOT_IMPORT_EXPORT GaussianImplicitSurface3D :
        public SimpleAbstractFunctionR3R1
{
public:
    GaussianImplicitSurface3D(std::unique_ptr<KernelWithDerivatives> kernel);
    void Calculate(const std::vector<DataR3R1>& samples, float noise);
    void Calculate(const std::vector<DataR3R2>& samples);
    float Get(Eigen::Vector3f pos) override;
    float GetVariance(const Eigen::Vector3f& pos);

private:
    Eigen::MatrixXd covariance;
    Eigen::MatrixXd covariance_inv;
    Eigen::VectorXd alpha;
    std::vector<DataR3R2> samples;
    float R;
    
    std::unique_ptr<KernelWithDerivatives> kernel;

    float Predict(const Eigen::Vector3f& pos) const;
    void CalculateCovariance(const std::vector<Eigen::Vector3f>& points, float R, const std::vector<float>& noise);
    void MatrixInvert(const Eigen::VectorXd& b);
};
}

