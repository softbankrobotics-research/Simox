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
* @author     Adrian Knobloch
* @copyright  2017 Adrian Knobloch
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_VisualizationGroup_h_
#define _VirtualRobot_VisualizationGroup_h_

#include "Visualization.h"
#include "../Tools/BoundingBox.h"

#include <vector>
#include <string>
#include <Eigen/Core>

namespace VirtualRobot
{

    class VIRTUAL_ROBOT_IMPORT_EXPORT VisualizationGroup
    {
    protected:
        VisualizationGroup(const std::vector<VisualizationPtr>& visualizations);

    public:
        virtual ~VisualizationGroup();

        virtual void addVisualization(const VisualizationPtr& visu);
        virtual bool containsVisualization(const VisualizationPtr& visu) const;
        virtual bool removeVisualization(const VisualizationPtr& visu);
        virtual bool removeVisualization(size_t index);
        virtual std::vector<VisualizationPtr> getVisualizations() const;
        virtual VisualizationPtr at(size_t index) const;
        virtual VisualizationPtr operator[] (size_t index) const;
        virtual bool empty() const;
        virtual size_t size() const;

        virtual Eigen::Matrix4f getGlobalPose() const;
        virtual void setGlobalPose(const Eigen::Matrix4f &m);
        virtual void applyDisplacement(const Eigen::Matrix4f &dp);

        virtual void setVisible(bool showVisualization);
        virtual bool isVisible() const;

        virtual void setUpdateVisualization(bool enable);
        virtual bool getUpdateVisualizationStatus() const;

        virtual void setStyle(Visualization::DrawStyle s);
        virtual Visualization::DrawStyle getStyle() const;

        virtual void setColor(const Visualization::Color &c);
        virtual Visualization::Color getColor() const;

        virtual void setMaterial(const Visualization::MaterialPtr& material);
        virtual Visualization::MaterialPtr getMaterial() const;

        virtual void scale(const Eigen::Vector3f &scaleFactor);

        virtual BoundingBox getBoundingBox() const;

        virtual TriMeshModelPtr getTriMeshModel() const;

        virtual std::vector<Primitive::PrimitivePtr> getPrimitives() const;

        virtual int getNumFaces() const;

        virtual void print() const;

    protected:
        std::vector<VisualizationPtr> visualizations;
    };
} // namespace VirtualRobot

#endif // _VirtualRobot_VisualizationGroup_h_
