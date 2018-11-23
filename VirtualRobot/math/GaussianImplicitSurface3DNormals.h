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
#include "Contact.h"
#include "Helpers.h"
#include "SimpleAbstractFunctionR3R1.h"
#include "ContactList.h"
#include "Kernels.h"

namespace math
{

class VIRTUAL_ROBOT_IMPORT_EXPORT GaussianImplicitSurface3DNormals :
        public SimpleAbstractFunctionR3R1
{
public:
    GaussianImplicitSurface3DNormals(std::unique_ptr<KernelWithDerivatives> kernel);
    void Calculate(const ContactList& samples, float noise, float normalNoise, float normalScale);
    float Get(Eigen::Vector3f pos) override;
    float GetVariance(const Eigen::Vector3f& pos);

private:
    Eigen::MatrixXd covariance;
    Eigen::MatrixXd covariance_inv;
    Eigen::VectorXd alpha;

    ContactList samples;
    float R;
    float R3;
    std::unique_ptr<KernelWithDerivatives> kernel;

    float Predict(const Eigen::Vector3f& pos);
    void CalculateCovariance(const std::vector<Eigen::Vector3f>& points, float R, float noise, float normalNoise);
    //static VectorXf Cholesky(MatrixXf matrix);
    //static VectorXf FitModel(MatrixXf L, List<DataR3R1> targets);
    //VectorXf SpdMatrixSolve(MatrixXf a, bool IsUpper, VectorXf b);
    void MatrixInvert(const Eigen::VectorXd& b);
    static Eigen::Vector3f Average(const ContactList& samples);
    Eigen::VectorXd getCux(const Eigen::Vector3f& pos);
};
}

