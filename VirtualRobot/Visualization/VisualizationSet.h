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

        virtual void addVisualization(const VisualizationPtr& visu);
        virtual bool containsVisualization(const VisualizationPtr& visu) const;
        virtual void removeVisualization(const VisualizationPtr& visu);
        virtual void removeVisualization(size_t id);
        virtual std::vector<VisualizationPtr> getVisualizations() const;
        virtual VisualizationPtr getVisualizationAt(size_t index) const;
        virtual bool empty() const;
        virtual size_t size() const;

        virtual VisualizationPtr clone() const override = 0;

        virtual Eigen::Matrix4f getGlobalPose() const override;
        virtual Eigen::Matrix4f getGlobalPose(size_t id) const;
        virtual void setGlobalPose(const Eigen::Matrix4f &m) override;
        virtual void setGlobalPose(size_t id, const Eigen::Matrix4f &m);
        //virtual void applyDisplacement(const Eigen::Matrix4f &dp) override;
        virtual void applyDisplacement(size_t id, const Eigen::Matrix4f &dp);

        virtual void setVisible(bool showVisualization) override;
        virtual void setVisible(size_t id, bool showVisualization);
        virtual bool isVisible() const override;
        virtual bool isVisible(size_t id) const;

        virtual void setUpdateVisualization(bool enable) override;
        virtual void setUpdateVisualization(size_t id, bool enable);
        virtual bool getUpdateVisualizationStatus() const override;
        virtual bool getUpdateVisualizationStatus(size_t id) const;

        virtual void setStyle(DrawStyle s) override;
        virtual void setStyle(size_t id, DrawStyle s);
        virtual DrawStyle getStyle() const override;
        virtual DrawStyle getStyle(size_t id) const;

        virtual void setColor(const Visualization::Color &c) override;
        virtual void setColor(size_t id,const Visualization::Color &c);
        virtual Visualization::Color getColor() const override;
        virtual Visualization::Color getColor(size_t id) const;

        virtual void setMaterial(const MaterialPtr& material) override;
        virtual void setMaterial(size_t id, const MaterialPtr& material);
        virtual MaterialPtr getMaterial() const override;
        virtual MaterialPtr getMaterial(size_t id) const;

    protected:
        virtual void _setSelected(bool selected) override;
    public:
        virtual bool isSelected() const override = 0;

        virtual void setScalingFactor(const Eigen::Vector3f &scaleFactor) override;
        virtual void setScalingFactor(size_t id, Eigen::Vector3f &scaleFactor);
        virtual Eigen::Vector3f getScalingFactor() const override;
        virtual Eigen::Vector3f getScalingFactor(size_t id) const;

        virtual void shrinkFatten(float offset) override;
        virtual void shrinkFatten(size_t id, float offset);

    protected:
        virtual void _addManipulator(ManipulatorType t) override;
        virtual void _removeManipulator(ManipulatorType t) override;
    public:
        virtual bool hasManipulator(ManipulatorType t) const override = 0;
        virtual std::vector<ManipulatorType> getAddedManipulatorTypes() const override = 0;
    protected:
        virtual void _removeAllManipulators() override;
    public:

        virtual std::vector<Primitive::PrimitivePtr> getPrimitives() const override;

        virtual void setFilename(const std::string &filename, bool boundingBox) override = 0;
        virtual std::string getFilename() const override = 0;
        virtual bool usedBoundingBoxVisu() const override = 0;

        virtual BoundingBox getBoundingBox() const override;
        virtual BoundingBox getBoundingBox(size_t id) const;

        virtual TriMeshModelPtr getTriMeshModel() const override;
        virtual TriMeshModelPtr getTriMeshModel(size_t id) const;
        virtual void createTriMeshModel() override;

        virtual int getNumFaces() const override;
        virtual int getNumFaces(size_t id) const;

        virtual void print() const override = 0;
        virtual void print(size_t id) const;

        virtual std::string toXML(const std::string &basePath, int tabs) const override = 0;
        virtual std::string toXML(size_t id, const std::string &basePath, int tabs) const;
        virtual std::string toXML(const std::string &basePath, const std::string &filename, int tabs) const override = 0;
        virtual std::string toXML(size_t id, const std::string &basePath, const std::string &filename, int tabs) const;

        virtual bool saveModel(const std::string &modelPath, const std::string &filename) override = 0;
        virtual bool saveModel(size_t id, const std::string &modelPath, const std::string &filename);

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

        //virtual void addVisualization(const VisualizationPtr& visu) override;
        //virtual void containsVisualization(const VisualizationPtr& visu) override;
        //virtual void removeVisualization(const VisualizationPtr& visu) override;
        //virtual void removeVisualization(size_t id) override;
        //virtual std::vector<VisualizationPtr> getVisualizations() const override;
        //virtual VisualizationPtr getVisualizationAt(size_t index) const override;

        virtual VisualizationPtr clone() const override;

        //virtual Eigen::Matrix4f getGlobalPose() const override;
        //virtual Eigen::Matrix4f getGlobalPose(size_t id) const override;
        virtual void setGlobalPose(const Eigen::Matrix4f &m) override;
        //virtual void setGlobalPose(size_t id, const Eigen::Matrix4f &m) override;
        //virtual void applyDisplacement(const Eigen::Matrix4f &dp) override;
        //virtual void applyDisplacement(size_t id, const Eigen::Matrix4f &dp) override;
        virtual size_t addPoseChangedCallback(std::function<void (const Eigen::Matrix4f &)> f) override;
        virtual void removePoseChangedCallback(size_t id) override;

        //virtual void setVisible(bool showVisualization) override;
        //virtual void setVisible(size_t id, bool showVisualization) override;
        //virtual bool isVisible() const override;
        //virtual bool isVisible(size_t id) const override;

        //virtual void setUpdateVisualization(bool enable) override;
        //virtual void setUpdateVisualization(size_t id, bool enable) override;
        //virtual bool getUpdateVisualizationStatus() const override;
        //virtual bool getUpdateVisualizationStatus(size_t id) const override;

        //virtual void setStyle(DrawStyle s) override;
        //virtual void setStyle(size_t id, DrawStyle s) override;
        //virtual DrawStyle getStyle() const override;
        //virtual DrawStyle getStyle(size_t id) const override;

        //virtual void setColor(const Visualization::Color &c) override;
        //virtual void setColor(size_t id,const Visualization::Color &c) override;
        //virtual Visualization::Color getColor() const override;
        //virtual Visualization::Color getColor(size_t id) const override;

        //virtual void setMaterial(const MaterialPtr& material) override;
        //virtual void setMaterial(size_t id, const MaterialPtr& material) override;
        //virtual MaterialPtr getMaterial() const override;
        //virtual MaterialPtr getMaterial(size_t id) const override;

    protected:
        virtual void _setSelected(bool selected) override;
    public:
        virtual bool isSelected() const override;
        virtual size_t addSelectionChangedCallback(std::function<void(bool)> f) override;
        virtual void removeSelectionChangedCallback(size_t id) override;

        //virtual void setScalingFactor(const Eigen::Vector3f &scaleFactor) override;
        //virtual void setScalingFactor(size_t id, Eigen::Vector3f &scaleFactor) override;
        //virtual Eigen::Vector3f getScalingFactor() const override;

        //virtual void shrinkFatten(float offset) override;
        //virtual void shrinkFatten(size_t id, float offset) override;

    protected:
        virtual void _addManipulator(ManipulatorType t) override;
        virtual void _removeManipulator(ManipulatorType t) override;
        virtual void _removeAllManipulators() override;
    public:
        virtual bool hasManipulator(ManipulatorType t) const override;
        virtual std::vector<ManipulatorType> getAddedManipulatorTypes() const override;

        //virtual std::vector<Primitive::PrimitivePtr> getPrimitives() const override;

        virtual void setFilename(const std::string &filename, bool boundingBox) override;
        virtual std::string getFilename() const override;
        virtual bool usedBoundingBoxVisu() const override;

        //virtual BoundingBox getBoundingBox() const override;
        //virtual BoundingBox getBoundingBox(size_t id) const override;

        //virtual TriMeshModelPtr getTriMeshModel() const override;
        //virtual TriMeshModelPtr getTriMeshModel(size_t id) const override;
        //virtual void createTriMeshModel() override;

        //virtual int getNumFaces() const override;
        //virtual int getNumFaces(size_t id) const override;

        virtual void print() const override;
        //virtual void print(size_t id) const override;

        virtual std::string toXML(const std::string &basePath, int tabs) const override;
        //virtual std::string toXML(size_t id, const std::string &basePath, int tabs) const override;
        virtual std::string toXML(const std::string &basePath, const std::string &filename, int tabs) const override;
        //virtual std::string toXML(size_t id, const std::string &basePath, const std::string &filename, int tabs) const override;

        virtual bool saveModel(const std::string &modelPath, const std::string &filename) override;
        //virtual bool saveModel(size_t id, const std::string &modelPath, const std::string &filename) override;

    protected:
        bool selected;
        std::set<ManipulatorType> addedManipulators;
        std::string filename;
        bool usedBoundingBox;
        std::map<unsigned int, std::function<void(const Eigen::Matrix4f&)>> poseChangedCallbacks;
    };

} // namespace VirtualRobot

#endif // _VirtualRobot_VisualizationSet_h_
