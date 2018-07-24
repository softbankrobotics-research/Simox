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
* @author     Andreea Tulbure (andreea dot tulbure at student dot kit dot edu)
* @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
*             GNU General Public License
*/

#pragma once

#include "MathForwardDefinitions.h"
#include "DataR3R1.h"
#include "DataR3R2.h"
#include "Contact.h"
#include "Helpers.h"
#include "SimpleAbstractFunctionR3R1.h"
#include "ContactList.h"
#include "Kernels.h"

namespace math
{

class GaussianImplicitSurface3DCombined :
        public SimpleAbstractFunctionR3R1
{
public:
    GaussianImplicitSurface3DCombined(std::unique_ptr<KernelWithDerivatives> kernel);
    void Calculate(const ContactList& normalSamples, const std::vector<DataR3R1>& samples, float noise, float normalNoise, float normalScale);
    void Calculate(const ContactList& normalSamples, const std::vector<DataR3R2>& samples, float normalNoise, float normalScale);
    float Get(Eigen::Vector3f pos) override;
    float GetVariance(const Eigen::Vector3f& pos);

private:
    Eigen::MatrixXd covariance;
    Eigen::MatrixXd covariance_inv;
    Eigen::VectorXd alpha;

    ContactList normalSamples;
    std::vector<DataR3R2> samples;

    float R;
    float R3;
    std::unique_ptr<KernelWithDerivatives> kernel;

    float Predict(const Eigen::Vector3f& pos);
    void CalculateCovariance(const std::vector<Eigen::Vector3f>& points, const std::vector<Eigen::Vector3f>& normalPoints, float R, const std::vector<float>& noise, float normalNoise);
    //static VectorXf Cholesky(MatrixXf matrix);
    //static VectorXf FitModel(MatrixXf L, List<DataR3R1> targets);
    //VectorXf SpdMatrixSolve(MatrixXf a, bool IsUpper, VectorXf b);
    void MatrixInvert(const Eigen::VectorXd& b);
    Eigen::VectorXd getCux(const Eigen::Vector3f& pos);
};
}
