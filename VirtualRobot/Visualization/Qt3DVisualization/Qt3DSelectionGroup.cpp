#include "Qt3DSelectionGroup.h"
#include "../SelectionManager.h"
#include "Qt3DVisualization.h"

#include <Qt3DCore/QNode>

#include <memory>

namespace VirtualRobot
{

    Qt3DSelectionGroup::Qt3DSelectionGroup() :
        SelectionGroup()
    {
    }

    void Qt3DSelectionGroup::setSelected(bool selected)
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

    size_t Qt3DSelectionGroup::addSelectionChangedCallbacks(std::function<void (bool)> f)
    {
        static size_t id = 0;
        selectionChangedCallbacks[id] = f;
        return id++;
    }

    void Qt3DSelectionGroup::removeSelectionChangedCallbacks(size_t id)
    {
        auto it = selectionChangedCallbacks.find(id);
        if (it != selectionChangedCallbacks.end())
        {
            selectionChangedCallbacks.erase(it);
        }
    }
}
