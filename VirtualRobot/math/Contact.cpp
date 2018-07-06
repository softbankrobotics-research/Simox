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

#include "Contact.h"
#include "Helpers.h"
using namespace math;


Contact::Contact()
    : position(Eigen::Vector3f(0,0,0)), normal(Eigen::Vector3f(0,0,0))
{
}

Contact::Contact(Eigen::Vector3f position, Eigen::Vector3f normal)
    : position(position), normal(normal)
{
}

Contact::Contact(float px, float py, float pz, float nx, float ny, float nz)
    : position(px, py, pz), normal(nx, ny, nz)
{

}

Contact Contact::Normalized()
{
    return Contact(position, normal.normalized());
}

std::string Contact::ToString()
{
    std::stringstream ss;
    ss << "(" << position.transpose() << ") (" << normal.transpose() << ")";
    return ss.str();
}

Contact Contact::Lerp(Contact a, Contact b, float f)
{
    return Contact(Helpers::Lerp(a.position, b.position, f),
                   Helpers::Lerp(a.normal, b.normal, f).normalized());
}
