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
* @author     Manfred Kroehnert, Nikolaus Vahrenkamp, Adrian Knobloch
* @copyright  2010,2011,2017 Manfred Kroehnert, Nikolaus Vahrenkamp, Adrian Knobloch
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_VisualizationFactory_h_
#define _VirtualRobot_VisualizationFactory_h_

#include "../Model/Model.h"
#include "../Tools/BoundingBox.h"
#include "../Model/Primitive.h"
#include "../Tools/MathTools.h"
#include "../Model/Nodes/ModelLink.h"
#include "../EndEffector/EndEffector.h"
#include "Visualization.h"
#include "VisualizationSet.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <memory>

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT VisualizationFactory
    {
    public:
        /*!
        * Use this method to get the VisualizationFactory singleton according to your compile setup.
        * @see CoinVisualizationFactory
        * Usually there is only one VisualizationFactory type registered, so we can safely return the first entry.
        */
        static VisualizationFactoryPtr getInstance();
    protected:
        VisualizationFactory() = default;
    public:
        virtual ~VisualizationFactory() = default;

        virtual void init(int &argc, char* argv[], const std::string &appName);

        virtual VisualizationPtr createVisualizationFromPrimitives(const std::vector<Primitive::PrimitivePtr>& primitives, bool boundingBox = false) const;
        virtual VisualizationPtr createVisualizationFromFile(const std::string& filename, bool boundingBox = false) const;
        virtual VisualizationPtr createVisualizationFromFile(const std::ifstream& ifs, bool boundingBox = false) const;

        virtual VisualizationSetPtr createVisualisationSet(const std::vector<VisualizationPtr>& visualizations = std::vector<VisualizationPtr>()) const;

        /*!
            A box, dimensions are given in mm.
        */
        virtual VisualizationPtr createBox(float width, float height, float depth) const;
        virtual VisualizationPtr createLine(const Eigen::Vector3f& from, const Eigen::Vector3f& to, float width = 1.0f) const;
        virtual VisualizationPtr createLine(const Eigen::Matrix4f& from, const Eigen::Matrix4f& to, float width = 1.0f) const;
        virtual VisualizationSetPtr createLineSet(const std::vector<Eigen::Vector3f>& from, const std::vector<Eigen::Vector3f>& to, float width = 1.0f) const;
        virtual VisualizationSetPtr createLineSet(const std::vector<Eigen::Vector3f>& points, float width = 1.0f) const;
        virtual VisualizationSetPtr createLineSet(const std::vector<Eigen::Matrix4f>& from, const std::vector<Eigen::Matrix4f>& to, float width = 1.0f) const;
        virtual VisualizationPtr createSphere(float radius) const;
        virtual VisualizationPtr createCircle(float radius, float circleCompletion, float width, size_t numberOfCircleParts = 30) const;

        virtual VisualizationPtr createTorus(float radius, float tubeRadius, float completion = 1.0f, int sides = 8, int rings = 30) const;

        virtual VisualizationPtr createCircleArrow(float radius, float tubeRadius, float completion = 1, int sides = 8, int rings = 30) const;

        virtual VisualizationPtr createCylinder(float radius, float height) const;
        virtual VisualizationPtr createCoordSystem(std::string* text = NULL, float axisLength = 100.0f, float axisSize = 3.0f, int nrOfBlocks = 10) const;
        virtual VisualizationPtr createPoint(float radius) const;
        virtual VisualizationSetPtr createPointCloud(const std::vector<Eigen::Matrix4f>& points, float radius) const;
        virtual VisualizationSetPtr createPointCloud(const std::vector<Eigen::Vector3f>& points, float radius) const;
        virtual VisualizationPtr createTriMeshModel(const TriMeshModelPtr& model) const;
        virtual VisualizationPtr createPolygon(const std::vector<Eigen::Vector3f>& points) const;
        inline VisualizationPtr createPlane(const MathTools::Plane& p, float extend, const std::string& texture = "") const
        {
            return createPlane(p.p, p.n, extend, texture);
        }
        virtual VisualizationPtr createPlane(const Eigen::Vector3f& point, const Eigen::Vector3f& normal, float extend, const std::string& texture = "") const;
        inline VisualizationPtr createGrid(const MathTools::Plane& p, float extend) const
        {
            return createPlane(p, extend, "images/Floor.png");
        }
        inline VisualizationPtr createGrid(const Eigen::Vector3f& point, const Eigen::Vector3f& normal, float extend) const
        {
            return createPlane(point, normal, extend, "images/Floor.png");
        }
        virtual VisualizationPtr createArrow(const Eigen::Vector3f& n, float length = 50.0f, float width = 2.0f) const;
        virtual VisualizationPtr createText(const std::string& text, bool billboard = false, float offsetX = 20.0f, float offsetY = 20.0f, float offsetZ = 0.0f) const;
        virtual VisualizationPtr createCone(float baseRadius, float height) const;
        /*!
            Creates an coordinate axis aligned ellipse
            \param x The extend in x direction must be >= 1e-6
            \param y The extend in y direction must be >= 1e-6
            \param z The extend in z direction must be >= 1e-6
            \return A VisualizationNode containing the visualization.
        */
        virtual VisualizationPtr createEllipse(float x, float y, float z) const;

        virtual VisualizationPtr createContactVisualization(const VirtualRobot::EndEffector::ContactInfoVector& contacts, float frictionConeHeight = 30.0f,  float frictionConeRadius = 15.0f, bool scaleAccordingToApproachDir = true) const;
        virtual VisualizationPtr createConvexHull2DVisualization(const MathTools::ConvexHull2DPtr& hull, const MathTools::Plane& p, const Eigen::Vector3f& offset = Eigen::Vector3f::Zero()) const;

        /*!
            Create an empty VisualizationNode.
        */
        virtual VisualizationPtr createVisualization() const;

        /*!
            Here, a manual cleanup can be called, visualization engine access may not be possible after calling this method.
            Usually no need to call cleanup explicitly, since cleanup is performed automatically at application exit.
        */
        virtual void cleanup();

        /**
         * A dynamicly bound version of getName().
         * @return the visualization type that is supported by this factory.
         */
        virtual std::string getVisualizationType() const;
    };

} // namespace VirtualRobot

#endif // _VirtualRobot_VisualizationFactory_h_
