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

#include "MathForwardDefinitions.h"

#include "Contact.h"
#include "GaussianImplicitSurface3DNormals.h"
#include "ImplicitObjectModel.h"

namespace math
{

    class GaussianObjectModelNormals :
            public ImplicitObjectModel
    {
    public:
        float Noise(){ return noise;}
        GaussianObjectModelNormals(float noise, float normalNoise, float normalScale);
        void AddContact(Contact contact);
        void Update() override;
        std::vector<float> GetContactWeights() override;


    private:
        float noise;
        float normalNoise;
        float normalScale;
        GaussianImplicitSurface3DNormalsPtr gpModel;
        //ContactList contacts;

     };
}

