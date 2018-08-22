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
#pragma once

#include "Visualization.h"
#include "../VirtualRobot.h"

#include <vector>
#include <memory>

namespace VirtualRobot
{

    class VIRTUAL_ROBOT_IMPORT_EXPORT SelectionGroup : public std::enable_shared_from_this<SelectionGroup>
    {
        friend class Visualization;
        friend class VisualizationFactory;
    protected:
        SelectionGroup() = default;

    public:
        virtual ~SelectionGroup() = default;

        inline void select()
        {
            this->setSelected(true);
        }
        inline void deselect()
        {
            this->setSelected(false);
        }
        virtual bool isSelected() const;
        virtual void setSelected(bool selected);

        virtual std::vector<VisualizationPtr> getVisualizations();

    protected:
        virtual void addVisualization(const VisualizationPtr& visu);
        virtual void removeVisualization(const VisualizationPtr& visu);

        std::vector<VisualizationWeakPtr> visualizations;
        bool selected;
    };

} // namespace VirtualRobot
