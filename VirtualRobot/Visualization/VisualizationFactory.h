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
#include "../Tools/AbstractFactoryMethod.h"
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

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT VisualizationFactory  : public ::AbstractFactoryMethod<VisualizationFactory, void*>
    {
    public:
        VisualizationFactory();
        virtual ~VisualizationFactory();

        virtual void init(int &/*argc*/, char* /*argv*/[], const std::string &/*appName*/);

        virtual VisualizationPtr createVisualizationFromPrimitives(const std::vector<Primitive::PrimitivePtr>& /*primitives*/, bool /*boundingBox*/ = false);
        virtual VisualizationPtr createVisualizationFromFile(const std::string& /*filename*/, bool /*boundingBox*/ = false);
        virtual VisualizationPtr createVisualizationFromFile(const std::ifstream& /*ifs*/, bool /*boundingBox*/ = false);

        virtual VisualizationSetPtr createVisualisationSet(const std::vector<VisualizationPtr>& visualizations) const;

        /*!
            A box, dimensions are given in mm.
        */
        virtual VisualizationPtr createBox(float /*width*/, float /*height*/, float /*depth*/);
        virtual VisualizationPtr createLine(const Eigen::Vector3f& /*from*/, const Eigen::Vector3f& /*to*/, float /*width*/ = 1.0f);
        virtual VisualizationPtr createLine(const Eigen::Matrix4f& /*from*/, const Eigen::Matrix4f& /*to*/, float /*width*/ = 1.0f);
        virtual VisualizationSetPtr createLineSet(const std::vector<Eigen::Vector3f>& from, const std::vector<Eigen::Vector3f>& to, float width = 1.0f);
        virtual VisualizationSetPtr createLineSet(const std::vector<Eigen::Matrix4f>& from, const std::vector<Eigen::Matrix4f>& to, float width = 1.0f);
        virtual VisualizationPtr createSphere(float /*radius*/, float /*colorR*/ = 0.5f, float /*colorG*/ = 0.5f, float /*colorB*/ = 0.5f);
        virtual VisualizationPtr createCircle(float /*radius*/, float /*circleCompletion*/, float /*width*/, size_t /*numberOfCircleParts*/ = 30);

        virtual VisualizationPtr createTorus(float /*radius*/, float /*tubeRadius*/, float /*completion*/ = 1.0f, int /*sides*/ = 8, int /*rings*/ = 30);

        virtual VisualizationPtr createCircleArrow(float /*radius*/, float /*tubeRadius*/, float /*completion*/ = 1, int /*sides*/ = 8, int /*rings*/ = 30);

        virtual VisualizationPtr createCylinder(float /*radius*/, float /*height*/);
        virtual VisualizationPtr createCoordSystem(float /*scaling*/ = 1.0f, std::string* /*text*/ = NULL, const Eigen::Matrix4f &/*pose*/ = Eigen::Matrix4f::Identity(), float /*axisLength*/ = 100.0f, float /*axisSize*/ = 3.0f, int /*nrOfBlocks*/ = 10);
        virtual VisualizationPtr createVertexVisualization(const Eigen::Vector3f& /*position*/, float /*radius*/);
        virtual VisualizationPtr createTriMeshModelVisualization(TriMeshModelPtr /*model*/, Eigen::Matrix4f& /*pose*/, bool /*showNormals*/ = false, bool /*showLines*/ = true);
        virtual VisualizationPtr createPlane(const Eigen::Vector3f& /*position*/, const Eigen::Vector3f& /*normal*/, float /*extend*/);
        inline VisualizationPtr createPlane(const MathTools::Plane& plane, float extend)
        {
            return createPlane(plane.p, plane.n, extend);
        }
        virtual VisualizationPtr createArrow(const Eigen::Vector3f& /*n*/, float /*length*/ = 50.0f, float /*width*/ = 2.0f);
        virtual VisualizationPtr createText(const std::string& /*text*/, bool /*billboard*/ = false, float /*offsetX*/ = 20.0f, float /*offsetY*/ = 20.0f, float /*offsetZ*/ = 0.0f);
        /*!
            Creates an coordinate axis aligned ellipse
            \param x The extend in x direction must be >= 1e-6
            \param y The extend in y direction must be >= 1e-6
            \param z The extend in z direction must be >= 1e-6
            \param showAxes If true, the axes are visualized
            \param axesHeight The height of the axes (measured from the body surface)
            \param axesWidth The width of the axes.
            \return A VisualizationNode containing the visualization.
        */
        virtual VisualizationPtr createEllipse(float /*x*/, float /*y*/, float /*z*/, bool /*showAxes*/ = true, float /*axesHeight*/ = 4.0f, float /*axesWidth*/ = 8.0f);

        virtual VisualizationPtr createContactVisualization(VirtualRobot::EndEffector::ContactInfoVector& /*contacts*/, float /*frictionConeHeight*/ = 30.0f,  float /*frictionConeRadius*/ = 15.0f, bool /*scaleAccordingToApproachDir*/ = true);

        virtual VisualizationPtr createReachabilityVisualization(WorkspaceRepresentationPtr /*reachSpace*/, const VirtualRobot::ColorMapPtr /*cm*/, bool /*transformToGlobalPose*/ = true, float /*maxZGlobal*/ = 1e10);

        /*!
            Create an empty VisualizationNode.
        */
        virtual VisualizationPtr createVisualization();

        /*!
            Here, a manual cleanup can be called, visualization engine access may not be possible after calling this method.
            Usually no need to call cleanup explicitly, since cleanup is performed automatically at application exit.
        */
        virtual void cleanup();

        /*! 
        * Use this method to get the VisualizationFactory singleton according to your compile setup.
        * @see CoinVisualizationFactory
        * Usually there is only one VisualizationFactory type registered, so we can safely return the first entry.
        */
        static VisualizationFactoryPtr getGlobalVisualizationFactory();

        /**
         * A dynamicly bound version of getName().
         * @return the visualization type that is supported by this factory.
         */
        virtual std::string getVisualizationType();
    };

} // namespace VirtualRobot

#endif // _VirtualRobot_VisualizationFactory_h_
