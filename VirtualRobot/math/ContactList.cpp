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

#include "ContactList.h"
#include <sstream>


using namespace math;

ContactList::ContactList()
    : std::vector<Contact>()
{
}

std::vector<Vec3> ContactList::GetPoints()
{
    std::vector<Vec3> points;
    for(Contact c: *this){
        points.push_back(c.Position());
    }
    return points;
}

Contact ContactList::Last()
{
    if (size() == 0){
        throw std::runtime_error("size == 0");
    }else{
        return this->at(size()-1);
    }
}

std::string ContactList::ToString()
{
    std::stringstream ss;
    bool first1 = true;
    for(Contact c : *this)
    {
        if (first1)
        {
            ss << "[" << c.ToString() << "]";
            first1 = false;
        }
        else
        {
            ss << ", [" << c.ToString() << "]";
        }
    }
    return ss.str();

}


