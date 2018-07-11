#include "CoinSelectionGroup.h"

namespace VirtualRobot
{

    CoinSelectionGroup::CoinSelectionGroup() :
        SelectionGroup()
    {
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

    void CoinSelectionGroup::addVisualization(const VisualizationPtr &visu)
    {
        SelectionGroup::addVisualization(visu);
        for (auto& f : visualizationAddedCallbacks)
        {
            f.second(visu);
        }
    }
}
