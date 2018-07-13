#include "SelectionManager.h"
#include "SelectionGroup.h"

using GlobalFactory = VirtualRobot::SelectionManager;

namespace VirtualRobot
{

    SelectionManagerPtr SelectionManager::getInstance()
    {
        static SelectionManagerPtr instance(new GlobalFactory);
        return instance;
    }

    size_t SelectionManager::addSelectionGroupChangedCallback(std::function<void(const VisualizationPtr&, const SelectionGroupPtr&, const SelectionGroupPtr&)> f)
    {
        static size_t id = 0;
        selectionGroupChangedCallbacks[id] = f;
        return id++;
    }

    void SelectionManager::removeSelectionGroupChangedCallback(size_t id)
    {
        auto it = selectionGroupChangedCallbacks.find(id);
        if (it != selectionGroupChangedCallbacks.end())
        {
            selectionGroupChangedCallbacks.erase(it);
        }
    }

    void SelectionManager::emitSlectionGroupChanged(const VisualizationPtr &visu, const SelectionGroupPtr &oldSelectionGroup, const SelectionGroupPtr &newSelectionGroup) const
    {
        for (auto& f : selectionGroupChangedCallbacks)
        {
            f.second(visu, oldSelectionGroup, newSelectionGroup);
        }
    }

    void SelectionManager::setSelectionMode(SelectionManager::SelectionMode mode)
    {
        selectionMode = mode;

        SelectionGroupPtr tmp;
        if (selectionMode == eSingle && selectedVisualizations.size() >= 1)
        {
            tmp = selectedVisualizations.back().lock();
            selectedVisualizations.pop_back();
            VR_ASSERT(tmp);
        }
        if (selectionMode == eNone || selectionMode == eSingle)
        {
            auto copy = selectedVisualizations;
            for (const auto& sw : copy)
            {
                auto s = sw.lock();
                if (s)
                {
                    s->setSelected(false);
                }
            }
            VR_ASSERT(selectedVisualizations.empty());
        }
        if (selectionMode == eSingle && tmp)
        {
            selectedVisualizations.push_back(tmp);
        }

        for (auto& f : selectionModeChangedCallbacks)
        {
            f.second(selectionMode);
        }
    }

    SelectionManager::SelectionMode SelectionManager::getSelectionMode() const
    {
        return selectionMode;
    }

    size_t SelectionManager::addSelectionModeChangedCallback(std::function<void (SelectionManager::SelectionMode)> f)
    {
        static size_t id = 0;
        selectionModeChangedCallbacks[id] = f;
        return id++;
    }

    void SelectionManager::removeSelectionModeChangedCallback(size_t id)
    {
        auto it = selectionModeChangedCallbacks.find(id);
        if (it != selectionModeChangedCallbacks.end())
        {
            selectionModeChangedCallbacks.erase(it);
        }
    }

    std::vector<VisualizationPtr> SelectionManager::getAllSelected() const
    {
        std::vector<VisualizationPtr> visus;
        for (const auto& sw : selectedVisualizations)
        {
            auto s = sw.lock();
            VR_ASSERT(s);
            for (const auto& v : s->getVisualizations())
            {
                visus.push_back(v);
            }
        }
        return visus;
    }

    void SelectionManager::setSelected(const SelectionGroupPtr &visu, bool selected)
    {
        if (selectionMode == eNone && selected)
        {
            VR_WARNING << "Selection disabled." << std::endl;
            return;
        }
        if (selected)
        {
            if (selectionMode == eSingle)
            {
                auto copy = selectedVisualizations;
                for (const auto& sw : copy)
                {
                    auto s = sw.lock();
                    VR_ASSERT(s);
                    if (s != visu)
                    {
                        s->setSelected(false);
                    }
                }
                VR_ASSERT(selectedVisualizations.size() <= 1);
                if (selectedVisualizations.empty())
                {
                    selectedVisualizations.push_back(visu);
                }
                else
                {
                    VR_ASSERT(selectedVisualizations.at(0).lock() == visu);
                }
            }
            else
            {
                auto it = std::find_if(selectedVisualizations.begin(), selectedVisualizations.end(), [visu](const SelectionGroupWeakPtr& w)
                {
                    auto v = w.lock();
                    VR_ASSERT(v);
                    return visu == v;

                });
                if (it == selectedVisualizations.end())
                {
                    selectedVisualizations.push_back(visu);
                }
            }
        }
        else
        {
            removeSelectionGroup(visu);
        }
    }

    void SelectionManager::removeSelectionGroup(const SelectionGroupPtr &group)
    {
        auto it = std::find_if(selectedVisualizations.begin(), selectedVisualizations.end(), [group](const SelectionGroupWeakPtr& w)
        {
            auto v = w.lock();
            VR_ASSERT(v);
            return group == v;
        });
        if (it != selectedVisualizations.end())
        {
            selectedVisualizations.erase(it);
        }
    }

}
