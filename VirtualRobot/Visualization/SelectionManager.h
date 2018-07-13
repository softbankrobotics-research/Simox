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
#ifndef _VirtualRobot_SelectionManager_h_
#define _VirtualRobot_SelectionManager_h_

#include "Visualization.h"
#include "../VirtualRobot.h"

#include <vector>

namespace VirtualRobot
{

    class VIRTUAL_ROBOT_IMPORT_EXPORT SelectionManager
    {
        friend class SelectionGroup;
        friend class Visualization;
    public:
        /*!
        * Use this method to get the VisualizationFactory singleton according to your compile setup.
        * @see CoinVisualizationFactory
        * Usually there is only one VisualizationFactory type registered, so we can safely return the first entry.
        */
        static SelectionManagerPtr getInstance();
    protected:
        SelectionManager() = default;
    public:
        virtual ~SelectionManager() = default;

        size_t addSelectionGroupChangedCallback(std::function<void(const VisualizationPtr&, const SelectionGroupPtr&, const SelectionGroupPtr&)> f);
        void removeSelectionGroupChangedCallback(size_t id);
    protected:
        virtual void emitSlectionGroupChanged(const VisualizationPtr& visu, const SelectionGroupPtr& oldSelectionGroup, const SelectionGroupPtr& newSelectionGroup) const;

    public:
        enum SelectionMode
        {
            eNone,
            eSingle,
            eMultiple
        };
        virtual void setSelectionMode(SelectionMode mode);
        virtual SelectionMode getSelectionMode() const;
        size_t addSelectionModeChangedCallback(std::function<void(SelectionMode)> f);
        void removeSelectionModeChangedCallback(size_t id);

        std::vector<VisualizationPtr> getAllSelected() const;
    protected:
        virtual void setSelected(const SelectionGroupPtr& visu, bool selected);

    protected:
        void removeSelectionGroup(const SelectionGroupPtr& group);

        std::map<size_t, std::function<void(const VisualizationPtr&, const SelectionGroupPtr&, const SelectionGroupPtr&)>> selectionGroupChangedCallbacks;
        std::map<size_t, std::function<void(SelectionMode)>> selectionModeChangedCallbacks;
        SelectionMode selectionMode = eNone;
        std::vector<SelectionGroupWeakPtr> selectedVisualizations;
    };

} // namespace VirtualRobot

#endif // _VirtualRobot_SelectionManager_h_
