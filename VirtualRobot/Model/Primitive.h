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
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_Primitive_h_
#define _VirtualRobot_Primitive_h_

#include "VirtualRobot/Model/Model.h"
#include <Eigen/Geometry>


namespace VirtualRobot
{
    namespace Primitive
    {

        class VIRTUAL_ROBOT_IMPORT_EXPORT Primitive
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            static const int TYPE = 0;
            const int type;
            Eigen::Matrix4f transform;

            virtual std::string toXMLString(int tabs) = 0;

        protected:
            Primitive(int type) : type(type), transform(Eigen::Matrix4f::Identity()) {}
            std::string getTransformString(int tabs = 0);
            std::string getXMLString(const std::string& type, const std::string& params, int tabs = 0);
        private:
            Primitive() : type(TYPE) {}
        };

        class VIRTUAL_ROBOT_IMPORT_EXPORT Box : public Primitive
        {
        public:
            static const int TYPE = 1;
            Box() : Primitive(TYPE) {}
            Box(float width, float height, float depth) : Primitive(TYPE), width(width), height(height), depth(depth) {}
            float width;
            float height;
            float depth;
            std::string toXMLString(int tabs = 0);
        };

        class VIRTUAL_ROBOT_IMPORT_EXPORT Sphere : public Primitive
        {
        public:
            static const int TYPE = 2;
            Sphere() : Primitive(TYPE) {}
            Sphere(float radius) : Primitive(TYPE), radius(radius) {}
            float radius;
            std::string toXMLString(int tabs = 0);
        };

        class VIRTUAL_ROBOT_IMPORT_EXPORT Cylinder : public Primitive
        {
        public:
            static const int TYPE = 3;
            Cylinder() : Primitive(TYPE) {}
            Cylinder(float radius, float height) : Primitive(TYPE), radius(radius), height(height) {}
            float radius;
            float height;
            std::string toXMLString(int tabs = 0);
        };

        typedef std::shared_ptr<Primitive> PrimitivePtr;

    } //namespace Primitive
} //namespace VirtualRobot

#endif // PRIMITIVE_H
