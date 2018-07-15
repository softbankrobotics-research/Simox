#include "CoinSelectionGroup.h"
#include "../SelectionManager.h"

namespace VirtualRobot
{

    CoinSelectionGroup::CoinSelectionGroup() :
        SelectionGroup()
    {
    }

    void CoinSelectionGroup::setSelected(bool selected)
    {
        if (selected != isSelected() && (!selected || SelectionManager::getInstance()->getSelectionMode() != SelectionManager::SelectionMode::eNone))
        {
            for (auto& f : selectionChangedCallbacks)
            {
                f.second(selected);
            }
        }
        SelectionGroup::setSelected(selected);
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
}
