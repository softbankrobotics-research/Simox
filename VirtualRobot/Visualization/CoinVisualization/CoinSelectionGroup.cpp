#include "CoinSelectionGroup.h"

namespace VirtualRobot
{

    CoinSelectionGroup::CoinSelectionGroup() :
        SelectionGroup()
    {
    }

    void CoinSelectionGroup::setSelected(bool selected)
    {
        if (selected != isSelected())
        {
            for (auto& f : selectionChangedCallbacks)
            {
                f.second(selected);
            }
        }
        SelectionGroup::setSelected(selected);
    }

    size_t CoinSelectionGroup::addVisualizationAddedCallback(std::function<void (const VisualizationPtr &)> f)
    {
        static size_t id = 0;
        visualizationAddedCallbacks[id] = f;
        return id++;
    }

    void CoinSelectionGroup::removeVisualizationAddedCallback(size_t id)
    {
        auto it = visualizationAddedCallbacks.find(id);
        if (it != visualizationAddedCallbacks.end())
        {
            visualizationAddedCallbacks.erase(it);
        }
    }

    size_t CoinSelectionGroup::addSelectionChangedCallbacks(std::function<void (bool)> f)
    {
        static size_t id = 0;
        selectionChangedCallbacks[id] = f;
        return id++;
    }

    void CoinSelectionGroup::removeSelectionChangedCallbacks(size_t id)
    {
        auto it = selectionChangedCallbacks.find(id);
        if (it != selectionChangedCallbacks.end())
        {
            selectionChangedCallbacks.erase(it);
        }
    }

    void CoinSelectionGroup::addVisualization(const VisualizationPtr &visu)
    {
        SelectionGroup::addVisualization(visu);
        for (auto& f : visualizationAddedCallbacks)
        {
            f.second(visu);
        }
    }
}
