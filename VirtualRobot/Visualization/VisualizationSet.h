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
* @copyright  2010,2011,2017 Manfred Kroehnert, Nikolaus Vahrenkamp Adrian Knobloch
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_VisualizationSet_h_
#define _VirtualRobot_VisualizationSet_h_

#include "Visualization.h"
#include "../Tools/BoundingBox.h"

#include <vector>
#include <string>
#include <Eigen/Core>

namespace VirtualRobot
{

    class VIRTUAL_ROBOT_IMPORT_EXPORT VisualizationSet : virtual public Visualization
    {
    protected:
        VisualizationSet(const std::vector<VisualizationPtr>& visualizations);

    public:
        virtual ~VisualizationSet() override;

        virtual VisualizationPtr clone() const override = 0;

        virtual void addVisualization(const VisualizationPtr& visu);
        virtual bool containsVisualization(const VisualizationPtr& visu) const;
        virtual bool removeVisualization(const VisualizationPtr &visu);
        virtual bool removeVisualization(size_t index);
        virtual std::vector<VisualizationPtr> getVisualizations() const;
        virtual VisualizationPtr at(size_t index) const;
        virtual VisualizationPtr operator[] (size_t index) const;
        virtual bool empty() const;
        virtual size_t size() const;

        //virtual Eigen::Matrix4f getGlobalPose() const override;
        virtual void setGlobalPose(const Eigen::Matrix4f &m) override;
        virtual void setGlobalPoseNoUpdate(const Eigen::Matrix4f &m);
        virtual void applyDisplacement(const Eigen::Matrix4f &dp) override;

        virtual void setVisible(bool showVisualization) override;
        virtual bool isVisible() const override;

        virtual void setUpdateVisualization(bool enable) override;
        virtual bool getUpdateVisualizationStatus() const override;

        virtual void setStyle(DrawStyle s) override;
        virtual DrawStyle getStyle() const override;

        virtual void setColor(const Visualization::Color &c) override;
        virtual Visualization::Color getColor() const override;

        virtual void setMaterial(const MaterialPtr& material) override;
        virtual MaterialPtr getMaterial() const override;

        virtual void setSelected(bool selected) override;
        virtual bool isSelected() const override;

        virtual void scale(const Eigen::Vector3f &scaleFactor) override;

        virtual void shrinkFatten(float offset) override;

    protected:
        virtual void _addManipulator(ManipulatorType t) override = 0;
        virtual void _removeManipulator(ManipulatorType t) override = 0;
        virtual void _removeAllManipulators() override = 0;
    public:
        virtual bool hasManipulator(ManipulatorType t) const override = 0;
        virtual std::vector<ManipulatorType> getAddedManipulatorTypes() const override = 0;

        virtual std::vector<Primitive::PrimitivePtr> getPrimitives() const override;

        virtual void setFilename(const std::string &filename, bool boundingBox) override = 0;
        virtual std::string getFilename() const override = 0;
        virtual bool usedBoundingBoxVisu() const override = 0;

        virtual BoundingBox getBoundingBox() const override;

        virtual TriMeshModelPtr getTriMeshModel() const override;

        virtual int getNumFaces() const override;

        virtual void print() const override;

        virtual std::string toXML(const std::string &basePath, int tabs) const override = 0;
        virtual std::string toXML(const std::string &basePath, const std::string &filename, int tabs) const override = 0;

        virtual bool saveModel(const std::string &modelPath, const std::string &filename) override = 0;

    protected:
        std::vector<VisualizationPtr> visualizations;
    };

    class VIRTUAL_ROBOT_IMPORT_EXPORT DummyVisualizationSet : public VisualizationSet
    {
        friend class VisualizationFactory;
    protected:
        DummyVisualizationSet(const std::vector<VisualizationPtr>& visualizations);

    public:
        virtual ~DummyVisualizationSet() override = default;

        virtual VisualizationPtr clone() const override;

    protected:
        virtual void _addManipulator(ManipulatorType t) override;
        virtual void _removeManipulator(ManipulatorType t) override;
        virtual void _removeAllManipulators() override;
    public:
        virtual bool hasManipulator(ManipulatorType t) const override;
        virtual std::vector<ManipulatorType> getAddedManipulatorTypes() const override;

        virtual void setFilename(const std::string &filename, bool boundingBox) override;
        virtual std::string getFilename() const override;
        virtual bool usedBoundingBoxVisu() const override;

        virtual std::string toXML(const std::string &basePath, int tabs) const override;
        virtual std::string toXML(const std::string &basePath, const std::string &filename, int tabs) const override;

        virtual bool saveModel(const std::string &modelPath, const std::string &filename) override;

    protected:
        bool selected;
        std::set<ManipulatorType> addedManipulators;
        std::string filename;
        bool usedBoundingBox;
    };

} // namespace VirtualRobot

#endif // _VirtualRobot_VisualizationSet_h_
