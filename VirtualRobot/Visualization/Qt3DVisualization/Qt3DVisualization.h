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
* @author     Philipp Schmidt
* @copyright  2017 Philipp Schmidt
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_Qt3DVisualization_h_
#define _VirtualRobot_Qt3DVisualization_h_

#include "../Visualization.h"

#include <Gui/Qt3D/Qt3DViewer.h>
#include "Qt3DElement.h"

#include <Qt3DCore/QEntity>
#include <Qt3DCore/QTransform>
#include <Qt3DRender/QMaterial>

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT Qt3DVisualization : public Visualization, public Qt3DElement
    {
    public:
        Qt3DVisualization();
        ~Qt3DVisualization();

        virtual void setGlobalPose(const Eigen::Matrix4f &m) override;
        virtual size_t addPoseChangedCallback(std::function<void (const Eigen::Matrix4f &)> f) override;
        virtual void removePoseChangedCallback(size_t id) override;
        virtual void setVisible(bool showVisualization) override;
        virtual bool isVisible() const override;
        virtual void setUpdateVisualization(bool enable) override;
        virtual bool getUpdateVisualizationStatus() const override;
        virtual void setStyle(DrawStyle s) override;
        virtual DrawStyle getStyle() const override;
        virtual void setColor(const Color &c) override;
        virtual Color getColor() const override;
        virtual void setMaterial(const MaterialPtr &material) override;
        virtual MaterialPtr getMaterial() const override;
        virtual void setSelected(bool selected) override;
        virtual bool isSelected() const override;
        virtual size_t addSelectionChangedCallback(std::function<void (bool)> f) override;
        virtual void removeSelectionChangedCallback(size_t id) override;
        virtual void scale(const Eigen::Vector3f &scaleFactor) override;
        virtual void shrinkFatten(float offset) override;
        virtual bool hasManipulator(ManipulatorType t) const override;
        virtual std::vector<ManipulatorType> getAddedManipulatorTypes() const override;
        virtual std::vector<Primitive::PrimitivePtr> getPrimitives() const override;
        virtual void setFilename(const std::string &filename, bool boundingBox) override;
        virtual std::string getFilename() const override;
        virtual bool usedBoundingBoxVisu() const override;
        virtual BoundingBox getBoundingBox() const override;
        virtual TriMeshModelPtr getTriMeshModel() const override;
        virtual int getNumFaces() const override;
        virtual VisualizationPtr clone() const override;
        virtual void print() const override;
        virtual std::string toXML(const std::string &basePath, int tabs) const override;
        virtual std::string toXML(const std::string &basePath, const std::string &filename, int tabs) const override;
        virtual bool saveModel(const std::string &modelPath, const std::string &filename) override;

        virtual Qt3DCore::QEntity* getEntity() const override;
    protected:
        virtual void createTriMeshModel();
        virtual void _addManipulator(ManipulatorType t) override;
        virtual void _removeManipulator(ManipulatorType t) override;
        virtual void _removeAllManipulators() override;

    private:
        Qt3DCore::QEntity* entity;
        Qt3DCore::QTransform* transformation;
        Qt3DRender::QMaterial* material;
    };

    typedef std::shared_ptr<Qt3DVisualization> Qt3DVisualizationPtr;
}

#endif
