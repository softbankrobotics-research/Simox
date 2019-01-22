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
* @copyright  2018 Adrian Knobloch
*             GNU Lesser General Public License
*
*/
#pragma once

#include "Visualization.h"
#include "../VirtualRobot.h"

#include <vector>
#include <memory>

namespace VirtualRobot
{
    enum ManipulatorType
    {
        None,
        Translate,
        Rotate,
        TranslateRotate
    };

    class VIRTUAL_ROBOT_IMPORT_EXPORT SelectionGroup : public std::enable_shared_from_this<SelectionGroup>
    {
        friend class Visualization;
        friend class VisualizationFactory;
    protected:
        SelectionGroup();

    public:
        virtual ~SelectionGroup();

        inline void select()
        {
            this->setSelected(true);
        }
        inline void deselect()
        {
            this->setSelected(false);
        }
        virtual bool isSelected() const;
        virtual void setSelected(bool selected);

        virtual std::vector<VisualizationPtr> getVisualizations();

        size_t addManipulationStartedCallback(std::function<void (const SelectionGroupPtr &)> f);
        void removeManipulationStartedCallback(size_t id);
        void executeManipulationStartedCallbacks();

        size_t addManipulationPoseUpdatedCallback(std::function<void (const SelectionGroupPtr&, const Eigen::Matrix4f &)> f);
        void removeManipulationPoseUpdatedCallback(size_t id);
        void executeManipulationPoseUpdatedCallback(const Eigen::Matrix4f& transformSinceManipulationStarted);

        size_t addManipulationFinishedCallback(std::function<void (const SelectionGroupPtr &, const Eigen::Matrix4f &)> f);
        void removeManipulationFinishedCallback(size_t id);
        void executeManipulationFinishedCallbacks(const Eigen::Matrix4f& transformSinceManipulationStarted);

        void addDefaultManipulationCallbacks();
        void removeDefaultManipulationCallbacks();

        size_t addManipulatorSetCallback(std::function<void (const SelectionGroupPtr &, ManipulatorType t)> f);
        void removeManipulatorSetCallback(size_t id);
        void setManipulator(ManipulatorType t);
        ManipulatorType getManipulator() const;

    protected:
        virtual void addVisualization(const VisualizationPtr& visu);
        virtual void removeVisualization(const VisualizationPtr& visu);

    private:
        std::vector<VisualizationWeakPtr> visualizations;
        bool selected;

        std::map<size_t, std::function<void(const SelectionGroupPtr &)>> manipulationStartedCallbacks;
        std::map<size_t, std::function<void(const SelectionGroupPtr &, const Eigen::Matrix4f&)>> manipulationPoseUpdatedCallbacks;
        std::map<size_t, std::function<void(const SelectionGroupPtr &, const Eigen::Matrix4f&)>> manipulationFinishedCallbacks;
        std::map<size_t, std::function<void(const SelectionGroupPtr &, ManipulatorType t)>> manipulatorSetCallbacks;
        ManipulatorType manipulator;

        bool defaultManipulationCallbacksAdded;
        size_t defaultManipulationStartedCallbackId;
        size_t defaultManipulationPoseUpdatedCallbackId;
        size_t defaultManipulationFinishedCallbackId;
        std::map<Visualization*, Eigen::Matrix4f> manipulationCallbackCache;
    };

} // namespace VirtualRobot
