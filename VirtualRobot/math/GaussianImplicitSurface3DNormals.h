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
#include "Contact.h"
#include "Helpers.h"
#include "SimpleAbstractFunctionR3R1.h"
#include "ContactList.h"
#include "Kernels.h"

namespace math
{

class GaussianImplicitSurface3DNormals :
        public SimpleAbstractFunctionR3R1
{
public:
    GaussianImplicitSurface3DNormals(KernelType kernelType);
    void Calculate(const ContactList& samples, float noise, float normalNoise, float normalScale);
    float Get(Eigen::Vector3f pos) override;

private:
    Eigen::MatrixXf covariance;
    Eigen::VectorXf alpha;
    ContactList samples;
    float R;
    float R3;
    std::unique_ptr<KernelWithDerivatives> kernel;

    Eigen::MatrixXf Cinv;

    float Predict(const Eigen::Vector3f& pos);
    Eigen::MatrixXf CalculateCovariance(const std::vector<Eigen::Vector3f>& points, float R, float noise, float normalNoise);
    //static VectorXf Cholesky(MatrixXf matrix);
    //static VectorXf FitModel(MatrixXf L, List<DataR3R1> targets);
    //VectorXf SpdMatrixSolve(MatrixXf a, bool IsUpper, VectorXf b);
    static Eigen::VectorXf MatrixSolve(Eigen::MatrixXf a, Eigen::VectorXf b);
    static Eigen::Vector3f Average(const ContactList& samples);
    Eigen::VectorXf getCux(const Eigen::Vector3f& pos);
};
}

