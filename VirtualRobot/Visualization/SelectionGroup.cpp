#include "SelectionGroup.h"
#include "SelectionManager.h"

namespace VirtualRobot
{
    SelectionGroup::SelectionGroup() :
        selected(false),
        manipulator(ManipulatorType::None),
        defaultManipulationCallbacksAdded(false)
    {
        addDefaultManipulationCallbacks();
    }

    SelectionGroup::~SelectionGroup()
    {
        removeDefaultManipulationCallbacks();
    }

    bool SelectionGroup::isSelected() const
    {
        return selected;
    }

    void SelectionGroup::setSelected(bool selected)
    {
        if (selected && SelectionManager::getInstance()->getSelectionMode() == SelectionManager::SelectionMode::eNone)
        {
            VR_WARNING << "Selection disabled." << std::endl;
            return;
        }
        if (this->selected != selected)
        {
            for (const auto& visuWeak : visualizations)
            {
                auto visu = visuWeak.lock();
                if (visu)
                {
                    visu->executeSelectionChangedCallbacks(selected);
                }
                else
                {
                    VR_WARNING << "Could not lock weak pointer. ignoring..." << std::endl;
                }
            }
        }
        this->selected = selected;
        SelectionManager::getInstance()->setSelected(shared_from_this(), selected);
    }

    std::vector<VisualizationPtr> SelectionGroup::getVisualizations()
    {
        std::vector<VisualizationPtr> visus;
        for (const auto& visuWeak : visualizations)
        {
            auto visu = visuWeak.lock();
            if (visu)
            {
                visus.push_back(visu);
            }
            else
            {
                VR_WARNING << "Could not lock weak pointer. ignoring..." << std::endl;
            }
        }
        return visus;
    }

    size_t SelectionGroup::addManipulationStartedCallback(std::function<void (const SelectionGroupPtr &)> f)
    {
        static size_t id = 0;
        manipulationStartedCallbacks[id] = f;
        return id++;
    }

    void SelectionGroup::removeManipulationStartedCallback(size_t id)
    {
        auto it = manipulationStartedCallbacks.find(id);
        if (it != manipulationStartedCallbacks.end())
        {
            manipulationStartedCallbacks.erase(it);
        }
    }

    void SelectionGroup::executeManipulationStartedCallbacks()
    {
        auto thisPtr = shared_from_this();
        for (const auto& f : manipulationStartedCallbacks)
        {
            f.second(thisPtr);
        }
    }

    size_t SelectionGroup::addManipulationPoseUpdatedCallback(std::function<void (const SelectionGroupPtr &, const Eigen::Matrix4f &)> f)
    {
        static size_t id = 0;
        manipulationPoseUpdatedCallbacks[id] = f;
        return id++;
    }

    void SelectionGroup::removeManipulationPoseUpdatedCallback(size_t id)
    {
        auto it = manipulationPoseUpdatedCallbacks.find(id);
        if (it != manipulationPoseUpdatedCallbacks.end())
        {
            manipulationPoseUpdatedCallbacks.erase(it);
        }
    }

    void SelectionGroup::executeManipulationPoseUpdatedCallback(const Eigen::Matrix4f& transformSinceManipulationStarted)
    {
        auto thisPtr = shared_from_this();
        for (const auto& f : manipulationPoseUpdatedCallbacks)
        {
            f.second(thisPtr, transformSinceManipulationStarted);
        }
    }

    size_t SelectionGroup::addManipulationFinishedCallback(std::function<void (const SelectionGroupPtr &, const Eigen::Matrix4f &)> f)
    {
        static size_t id = 0;
        manipulationFinishedCallbacks[id] = f;
        return id++;
    }

    void SelectionGroup::removeManipulationFinishedCallback(size_t id)
    {
        auto it = manipulationFinishedCallbacks.find(id);
        if (it != manipulationFinishedCallbacks.end())
        {
            manipulationFinishedCallbacks.erase(it);
        }
    }

    void SelectionGroup::executeManipulationFinishedCallbacks(const Eigen::Matrix4f& transformSinceManipulationStarted)
    {
        auto thisPtr = shared_from_this();
        for (const auto& f : manipulationFinishedCallbacks)
        {
            f.second(thisPtr, transformSinceManipulationStarted);
        }
    }

    void SelectionGroup::addDefaultManipulationCallbacks()
    {
        if (defaultManipulationCallbacksAdded)
        {
            return;
        }
        defaultManipulationStartedCallbackId = addManipulationStartedCallback([](const SelectionGroupPtr& group)
        {
            for (const auto& visu : group->getVisualizations())
            {
                group->manipulationCallbackCache[visu.get()] = visu->getGlobalPose();
            }
        });
        defaultManipulationPoseUpdatedCallbackId = addManipulationPoseUpdatedCallback([](const SelectionGroupPtr& group, const Eigen::Matrix4f& m)
        {
            for (const auto& visu : group->getVisualizations())
            {
                auto it = group->manipulationCallbackCache.find(visu.get());
                if (it != group->manipulationCallbackCache.end())
                {
                    visu->setGlobalPose(m * it->second);
                }
            }
        });
        defaultManipulationFinishedCallbackId = addManipulationFinishedCallback([](const SelectionGroupPtr& group, const Eigen::Matrix4f& m)
        {
            for (const auto& visu : group->getVisualizations())
            {
                auto it = group->manipulationCallbackCache.find(visu.get());
                if (it != group->manipulationCallbackCache.end())
                {
                    visu->setGlobalPose(m * it->second);
                    group->manipulationCallbackCache.erase(it);
                }
            }
        });

        defaultManipulationCallbacksAdded = true;
    }

    void SelectionGroup::removeDefaultManipulationCallbacks()
    {
        if (!defaultManipulationCallbacksAdded)
        {
            return;
        }

        removeManipulationStartedCallback(defaultManipulationStartedCallbackId);
        removeManipulationPoseUpdatedCallback(defaultManipulationPoseUpdatedCallbackId);
        removeManipulationFinishedCallback(defaultManipulationFinishedCallbackId);

        defaultManipulationCallbacksAdded = false;
    }

    size_t SelectionGroup::addManipulatorSetCallback(std::function<void (const SelectionGroupPtr &, ManipulatorType)> f)
    {
        static size_t id = 0;
        manipulatorSetCallbacks[id] = f;
        return id++;
    }

    void SelectionGroup::removeManipulatorSetCallback(size_t id)
    {
        auto it = manipulatorSetCallbacks.find(id);
        if (it != manipulatorSetCallbacks.end())
        {
            manipulatorSetCallbacks.erase(it);
        }
    }

    void SelectionGroup::setManipulator(ManipulatorType t)
    {
        if (t == manipulator)
        {
            return;
        }
        manipulator = t;
        auto thisPtr = shared_from_this();
        for (const auto& f : manipulatorSetCallbacks)
        {
            f.second(thisPtr, t);
        }
    }

    ManipulatorType SelectionGroup::getManipulator() const
    {
        return manipulator;
    }

    void SelectionGroup::addVisualization(const VisualizationPtr &visu)
    {
        if (visualizations.empty() && isSelected())
        {
            SelectionManager::getInstance()->setSelected(shared_from_this(), true);
        }
        visualizations.push_back(visu);
    }

    void SelectionGroup::removeVisualization(const VisualizationPtr &visu)
    {
        auto it = std::find_if(visualizations.begin(), visualizations.end(), [&](const VisualizationWeakPtr& v){
            return visu == v.lock();
        });
        if (it != visualizations.end())
        {
            visualizations.erase(it);
        }
        if (visualizations.empty())
        {
            SelectionManager::getInstance()->removeSelectionGroup(shared_from_this());
        }
    }
}
