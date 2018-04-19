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
* @author     miller (martin dot miller at student dot kit dot edu)
* @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
*             GNU General Public License
*/

#include "GridCacheFloat3.h"
#include "Array3D.h"

using namespace math;



GridCacheFloat3::GridCacheFloat3(int size, std::function<float (Index3)>& getData)
{
    this->size = size;
    this->getData = getData;
    data = Array3DFloatPtr(new Array3D<float>(size));
    valid = Array3DBoolPtr(new Array3D<bool>(size));

    for (int x = 0; x < size; x++)
    {
        for (int y = 0; y < size; y++)
        {
            for (int z = 0; z < size; z++)
            {
                valid->Set(x,y,z, false);
            }
        }
    }

}


float GridCacheFloat3::Get(int x, int y, int z)
{
    if (valid->Get(x,y,z))
    {
        return data->Get(x,y,z);
    }
    else
    {
        float val = getData(Index3(x, y, z));
        data->Set(x,y,z,val);
        valid->Set(x,y,z, true);
        return val;
    }
}
