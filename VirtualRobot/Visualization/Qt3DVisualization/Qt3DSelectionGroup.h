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
#ifndef _VirtualRobot_Qt3DSelectionGroup_h_
#define _VirtualRobot_Qt3DSelectionGroup_h_

#include "../SelectionGroup.h"

namespace Qt3DCore {
    class QNode;
}

namespace VirtualRobot
{

    class VIRTUAL_ROBOT_IMPORT_EXPORT Qt3DSelectionGroup : public SelectionGroup
    {
        friend class Qt3DVisualizationFactory;
    protected:
        Qt3DSelectionGroup();

    public:
        virtual ~Qt3DSelectionGroup() override = default;
        virtual void setSelected(bool selected) override;

        size_t addSelectionChangedCallbacks(std::function<void (bool)> f);
        void removeSelectionChangedCallbacks(size_t id);

    protected:
        std::map<size_t, std::function<void(bool)>> selectionChangedCallbacks;
    };

} // namespace VirtualRobot

#endif // _VirtualRobot_CoinSelectionGroup_h_
