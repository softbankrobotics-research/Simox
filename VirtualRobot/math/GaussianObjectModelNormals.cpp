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

#include <iostream>

#include "GaussianObjectModelNormals.h"
#include "MathForwardDefinitions.h"

using namespace math;

GaussianObjectModelNormals::GaussianObjectModelNormals(float noise, float normalNoise, float normalScale)
   : noise(noise), normalNoise(normalNoise), normalScale(normalScale)
{
    model = gpModel = GaussianImplicitSurface3DNormalsPtr(new GaussianImplicitSurface3DNormals());
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

