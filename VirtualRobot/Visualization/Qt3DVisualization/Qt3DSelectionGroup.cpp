#include "Qt3DSelectionGroup.h"
#include "../SelectionManager.h"
#include "Qt3DVisualization.h"

#include <Qt3DCore/QNode>

#include <memory>

namespace VirtualRobot
{

    Qt3DSelectionGroup::Qt3DSelectionGroup() :
        SelectionGroup(),
        node(new Qt3DCore::QNode)
    {
    }

    Qt3DSelectionGroup::~Qt3DSelectionGroup()
    {
        delete node;
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

    void Qt3DSelectionGroup::addVisualization(const VisualizationPtr &visu)
    {
        SelectionGroup::addVisualization(visu);
        auto qt3dVisu = std::dynamic_pointer_cast<Qt3DVisualization>(visu);
        VR_ASSERT(qt3dVisu); // no VisualizationSet should be added here!!!
        qt3dVisu->getEntity()->setParent(node);
    }

    void Qt3DSelectionGroup::removeVisualization(const VisualizationPtr &visu)
    {
        SelectionGroup::removeVisualization(visu);
        auto qt3dVisu = std::dynamic_pointer_cast<Qt3DVisualization>(visu);
        VR_ASSERT(qt3dVisu); // no VisualizationSet should be added here!!!
        qt3dVisu->getEntity()->setParent(static_cast<Qt3DCore::QNode*>(nullptr)); // remove parent
    }

    Qt3DCore::QNode *Qt3DSelectionGroup::getNode() const
    {
        return node;
    }
}
