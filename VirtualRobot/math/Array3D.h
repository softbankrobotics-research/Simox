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

#include "Index3.h"


namespace math
{
    template<class T>
    class Array3D
    {
    public:
        Array3D(int size) :
            size(size)
        {
            data = boost::shared_ptr<std::vector<T>>(new std::vector<T>);
            data->resize(size*size*size);
        }

        T Get(Index3 index)
        {
            return data->at(index.X() +
                    size * index.Y() +
                    size * size * index.Z());
        }

        T Get(int i, int j, int k)
        {
            return data->at(i +
                    size * j +
                    size * size * k );

        }

        void Set(Index3 index, T value)
        {
            data->at(             index.X() +
                    size * index.Y() +
                    size * size * index.Z()) = value;
        }
        void Set(int i, int j, int k, T value)
        {
            data->at(            i +
                    size * j +
                    size * size * k )= value;
        }

    private:
        boost::shared_ptr<std::vector<T>> data;
        //std::vector<T> data;
        int size;
    };
}

