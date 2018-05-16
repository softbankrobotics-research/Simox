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
* @author     Simon Ottenhaus (simon dot ottenhaus at kit dot edu)
* @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
*             GNU General Public License
*/

#pragma once

#include "MathForwardDefinitions.h"





namespace math
{

struct Index3
{
private:
    int x;
    int y;
    int z;

public :
    int X() {  return x; }
    int Y() {  return y;}
    int Z() { return z; }
    void SetX(int value){ x = value; }
    void SetY(int value){ y = value; }
    void SetZ(int value){ z = value; }

    Index3(int x, int y, int z);

std::string ToString();

};

/*static Index3 operator -(Index3 a);
static Index3 operator +(Index3 a, Index3 b);
static Index3 operator -(Index3 a, Index3 b);
static Index3 operator *(Index3 a, int f);
static Index3 operator *(int f, Index3 a);
static Index3 operator /(Index3 a, int f);*/

}

