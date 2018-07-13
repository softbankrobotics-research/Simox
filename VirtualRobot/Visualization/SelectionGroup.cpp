#include "SelectionGroup.h"
#include "SelectionManager.h"

namespace VirtualRobot
{
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
