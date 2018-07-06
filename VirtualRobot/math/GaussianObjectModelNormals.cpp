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

#include <iostream>

#include "GaussianObjectModelNormals.h"
#include "MathForwardDefinitions.h"
#include "Kernels.h"

using namespace math;

GaussianObjectModelNormals::GaussianObjectModelNormals(float noise, float normalNoise, float normalScale)
   : noise(noise), normalNoise(normalNoise), normalScale(normalScale)
{
    model = gpModel = GaussianImplicitSurface3DNormalsPtr(new GaussianImplicitSurface3DNormals(std::unique_ptr<WilliamsPlusKernel>(new WilliamsPlusKernel)));
}
void GaussianObjectModelNormals::AddContact(Contact contact)
{
     //std::cout<<"addContact"<<std::endl;
     ImplicitObjectModel::AddContact(contact);
}
void GaussianObjectModelNormals::Update()
{
    std::cout<<"contacts"<<contacts->size()<<std::endl;
    if(contacts->size()==1){
        Contact c = contacts->at(0);
        Eigen::Vector3f dir1, dir2;
        contacts->push_back(Contact(c.Position()+Helpers::GetOrthonormalVectors(c.Normal(),dir1,dir2),c.Normal()));
    }
    gpModel->Calculate(*contacts,noise,normalNoise,normalScale);
}

std::vector<float> GaussianObjectModelNormals::GetContactWeights()
{
    std::vector<float> result;
    for (size_t i = 0; i < contacts->size(); ++i) {
        result.push_back(1.f);
    }
    return result;
}

