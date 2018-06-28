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

#include "GaussianObjectModel.h"
#include "MathForwardDefinitions.h"
#include "Kernels.h"

using namespace math;

GaussianObjectModel::GaussianObjectModel(float noise)
   : noise(noise)
{
    model = gpModel = GaussianImplicitSurface3DPtr(new GaussianImplicitSurface3D(std::unique_ptr<WilliamsPlusKernel>(new WilliamsPlusKernel)));
}

void GaussianObjectModel::AddContact(Contact contact)
{
     ImplicitObjectModel::AddContact(contact);
}



void GaussianObjectModel::Update()
{
    gpModel->Calculate(CreateSamples(),noise);
}

std::vector<float> GaussianObjectModel::GetContactWeights()
{
    std::vector<float> result;
    for (size_t i = 0; i < contacts->size(); ++i) {
        result.push_back(1.f);
    }
    return result;
}

std::vector<DataR3R1> GaussianObjectModel::CreateSamples()
{
    std::vector<DataR3R1> samples;
    for(Contact c: *contacts) {
        samples.push_back(DataR3R1(c.Position(),0));
        samples.push_back(DataR3R1(c.Position()- c.Normal()*0.05,-1));
        samples.push_back(DataR3R1(c.Position()+ c.Normal()*0.05,+1));
    }
    return samples;
}
