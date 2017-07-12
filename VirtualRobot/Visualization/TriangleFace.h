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
* @package    VirtualRobot
* @author     Manfred Kroehnert, Nikolaus Vahrenkamp
* @copyright  2010,2011 Manfred Kroehnert, Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_TriangleFace_h_
#define _VirtualRobot_TriangleFace_h_

#include "../VirtualRobot.h"
#include <Eigen/Core>
#include <vector>
#include <utility>

namespace VirtualRobot
{
	class VIRTUAL_ROBOT_IMPORT_EXPORT TriangleFace
	{
	public:
		TriangleFace()
			: id1(UINT_MAX), id2(UINT_MAX), id3(UINT_MAX),
			idColor1(UINT_MAX), idColor2(UINT_MAX), idColor3(UINT_MAX),
			idNormal1(UINT_MAX), idNormal2(UINT_MAX), idNormal3(UINT_MAX),
			idMaterial(UINT_MAX) {}

		/**
		* Flips the orientation of the contained vertex and the normal.
		*/
		void flipOrientation()
		{
			std::swap(id3, id1);
			normal *= -1.0f;
		}
		void set(unsigned int id1, unsigned int id2, unsigned int id3)
		{
			this->id1 = id1;
			this->id2 = id2;
			this->id3 = id3;
		}
		void setColor(unsigned int idColor1, unsigned int idColor2, unsigned int idColor3)
		{
			this->idColor1 = idColor1;
			this->idColor2 = idColor2;
			this->idColor3 = idColor3;
		}
		void setNormal(unsigned int idNormal1, unsigned int idNormal2, unsigned int idNormal3)
		{
			this->idNormal1 = idNormal1;
			this->idNormal2 = idNormal2;
			this->idNormal3 = idNormal3;
		}
		void setMaterial(unsigned int idMaterial)
		{
			this->idMaterial = idMaterial;
		}

		// id == position in vertex array
		unsigned int id1;
		unsigned int id2;
		unsigned int id3;

		// idColor == position in color array
		unsigned int idColor1;
		unsigned int idColor2;
		unsigned int idColor3;

		//idNormal == position in normal array
		unsigned int idNormal1;
		unsigned int idNormal2;
		unsigned int idNormal3;

		// idMaterial == position in material array
		unsigned int idMaterial;

		// per face normal (used when no idNormals are given)
		Eigen::Vector3f normal;
	};

} // namespace VirtualRobot

#endif 
