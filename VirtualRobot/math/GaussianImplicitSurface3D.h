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
#include "DataR3R1.h"
#include "DataR3R2.h"
#include "SimpleAbstractFunctionR3R1.h"
#include "Kernels.h"
#include <memory>

namespace math
{

class GaussianImplicitSurface3D :
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

