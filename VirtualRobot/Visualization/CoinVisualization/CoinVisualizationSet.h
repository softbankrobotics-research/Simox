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
* @copyright  2010, 2011, 2017 Manfred Kroehnert, Nikolaus Vahrenkamp, Adrian Knobloch
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_CoinVisualizationSet_h_
#define _VirtualRobot_CoinVisualizationSet_h_

#include "../../Model/Model.h"
#include "../VisualizationSet.h"
#include "CoinElement.h"

class SoNode;
class SoSeparator;

namespace VirtualRobot
{
    /*!
        A Coin3D based implementation of a visualization.
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT CoinVisualizationSet : public VisualizationSet, public CoinElement
    {
        friend class CoinVisualizationFactory;
    protected:
        CoinVisualizationSet(const std::vector<VisualizationPtr>& visualizations);

    public:
        virtual ~CoinVisualizationSet() override;

        virtual SoNode* getMainNode() const override;

        virtual void addVisualization(const VisualizationPtr& visu) override;
        virtual bool removeVisualization(const VisualizationPtr& visu) override;
        virtual bool removeVisualization(size_t index) override;

        virtual VisualizationPtr clone() const override;

        virtual void setGlobalPose(const Eigen::Matrix4f &m) override;
        virtual size_t addPoseChangedCallback(std::function<void (const Eigen::Matrix4f &)> f) override;
        virtual void removePoseChangedCallback(size_t id) override;

        virtual void setSelected(bool selected) override;
        virtual bool isSelected() const override;
        virtual size_t addSelectionChangedCallback(std::function<void (bool)> f) override;
        virtual void removeSelectionChangedCallback(size_t id) override;

    protected:
        virtual void _addManipulator(ManipulatorType t) override;
        virtual void _removeManipulator(ManipulatorType t) override;
        virtual void _removeAllManipulators() override;
    public:
        virtual bool hasManipulator(ManipulatorType t) const override;
        virtual std::vector<ManipulatorType> getAddedManipulatorTypes() const override;
    public:
        virtual void setFilename(const std::string &filename, bool boundingBox) override;
        virtual std::string getFilename() const override;
        virtual bool usedBoundingBoxVisu() const override;

        virtual std::string toXML(const std::string &basePath, int tabs) const override;
        virtual std::string toXML(const std::string &basePath, const std::string &filename, int tabs) const override;

        virtual bool saveModel(const std::string &modelPath, const std::string &filename) override;

    protected:
        SoSeparator* setNode;
        std::string filename;
        bool usedBoundingBox;
        std::map<unsigned int, std::function<void(const Eigen::Matrix4f&)>> poseChangedCallbacks;
    };

    typedef std::shared_ptr<CoinVisualizationSet> CoinVisualizationSetPtr;

} // namespace VirtualRobot

#endif // _VirtualRobot_CoinVisualizationSet_h_
