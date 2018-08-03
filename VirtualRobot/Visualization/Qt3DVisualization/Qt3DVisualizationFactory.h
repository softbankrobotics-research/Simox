/*!
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
* @author     Philipp Schmidt
* @copyright  2017 Philipp Schmidt
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_Qt3DVisualizationFactory_h_
#define _VirtualRobot_Qt3DVisualizationFactory_h_

#include "../VisualizationFactory.h"

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT Qt3DVisualizationFactory  : public VisualizationFactory
    {
    public:
        Qt3DVisualizationFactory();
        ~Qt3DVisualizationFactory();

        virtual void init(int &argc, char *argv[], const std::string &appName) override;
        virtual VisualizationPtr createVisualizationFromFile(const std::string &filename, bool boundingBox) const override;
        virtual VisualizationPtr createVisualizationFromFile(const std::ifstream &ifs, bool boundingBox) const override;
        virtual VisualizationSetPtr createVisualisationSet(const std::vector<VisualizationPtr> &visualizations) const override;
        virtual VisualizationPtr createBox(float width, float height, float depth) const override;
        virtual VisualizationPtr createLine(const Eigen::Vector3f &from, const Eigen::Vector3f &to, float width) const override;
        virtual VisualizationPtr createSphere(float radius) const override;
        virtual VisualizationPtr createTorus(float radius, float tubeRadius, float completion, int sides, int rings) const override;
        virtual VisualizationPtr createCircleArrow(float radius, float tubeRadius, float completion, int sides, int rings) const override;
        virtual VisualizationPtr createCylinder(float radius, float height) const override;
        virtual VisualizationPtr createPoint(float radius) const override;
        virtual VisualizationPtr createTriMeshModel(const TriMeshModelPtr &model) const override;
        virtual VisualizationPtr createArrow(const Eigen::Vector3f &n, float length, float width) const override;
        virtual VisualizationPtr createText(const std::string &text, bool billboard, float offsetX, float offsetY, float offsetZ) const override;
        virtual VisualizationPtr createCone(float baseRadius, float height) const override;
        virtual VisualizationPtr createEllipse(float x, float y, float z) const override;
        virtual VisualizationPtr createContactVisualization(const EndEffector::ContactInfoVector &contacts, float frictionConeHeight, float frictionConeRadius, bool scaleAccordingToApproachDir) const override;
        virtual VisualizationPtr createVisualization() const override;
        virtual void cleanup() override;
        virtual std::string getVisualizationType() const override;

        static std::string getName();
    };

    typedef std::shared_ptr<Qt3DVisualizationFactory> Qt3DVisualizationFactoryPtr;
}

#endif
