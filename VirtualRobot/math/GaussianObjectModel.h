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

#ifndef math_GaussianObjectModel
#define math_GaussianObjectModel

#include "MathForwardDefinitions.h"

#include "Contact.h"
#include "DataR3R1.h"
#include "GaussianImplicitSurface3D.h"
#include "ImplicitObjectModel.h"

namespace math
{

    class GaussianObjectModel :
            public ImplicitObjectModel
    {
    public:
        float Noise(){ return noise;}
        GaussianObjectModel(float noise);
        void AddContact(Contact contact);
        void Update() override;
        std::vector<float> GetContactWeights() override;


    private:
        float noise;
        GaussianImplicitSurface3DPtr gpModel;
        std::vector<DataR3R1> CreateSamples();

            };
}

#endif // math_GaussianObjectModel
