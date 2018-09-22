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
#pragma once

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
        virtual void init() override;
        virtual ~VisualizationSet() override;

        virtual VisualizationPtr clone() const override;

        virtual void addVisualization(const VisualizationPtr& visu);
        virtual bool containsVisualization(const VisualizationPtr& visu) const;
        virtual bool removeVisualization(const VisualizationPtr &visu);
        virtual bool removeVisualization(size_t index);
        virtual void removeAllVisualizations();
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
        virtual void setSelectionGroup(const SelectionGroupPtr& group) override;
        virtual SelectionGroupPtr getSelectionGroup() const override;

        virtual void scale(const Eigen::Vector3f &scaleFactor) override;

        virtual void shrinkFatten(float offset) override;

        virtual std::vector<Primitive::PrimitivePtr> getPrimitives() const override;

        virtual void setFilename(const std::string &filename, bool boundingBox) override;
        virtual std::string getFilename() const override;
        virtual bool usedBoundingBoxVisu() const override;
        virtual void getTextureFiles(std::vector<std::string>& storeFilenames) const override;

        virtual BoundingBox getBoundingBox() const override;

        virtual TriMeshModelPtr getTriMeshModel() const override;

        virtual int getNumFaces() const override;

        virtual void print() const override;

        virtual std::string toXML(const std::string &basePath, int tabs) const override = 0;
        virtual std::string toXML(const std::string &basePath, const std::string &filename, int tabs) const override = 0;

        virtual bool saveModel(const std::string &modelPath, const std::string &filename) override = 0;

        size_t addVisualizationAddedCallback(std::function<void (const VisualizationPtr&)> f);
        void removeVisualizationAddedCallback(size_t id);

        size_t addVisualizationRemovedCallback(std::function<void (const VisualizationPtr&)> f);
        void removeVisualizationRemovedCallback(size_t id);

    protected:
        std::vector<VisualizationPtr> visualizations;
        std::map<VisualizationPtr, size_t> childVisualizationChangedCallbacks;
        std::string filename;
        bool usedBoundingBox;

        std::map<size_t, std::function<void(const VisualizationPtr&)>> visualizationAddedCallbacks;
        std::map<size_t, std::function<void(const VisualizationPtr&)>> visualizationRemovedCallbacks;
    };

    class VIRTUAL_ROBOT_IMPORT_EXPORT DummyVisualizationSet : public VisualizationSet
    {
        friend class VisualizationFactory;
    protected:
        DummyVisualizationSet(const std::vector<VisualizationPtr>& visualizations);

    public:
        virtual ~DummyVisualizationSet() override = default;

        virtual std::string toXML(const std::string &basePath, int tabs) const override;
        virtual std::string toXML(const std::string &basePath, const std::string &filename, int tabs) const override;

        virtual bool saveModel(const std::string &modelPath, const std::string &filename) override;
    };

} // namespace VirtualRobot

