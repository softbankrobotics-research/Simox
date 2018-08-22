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
* @author     Manfred Kroehnert, Nikolaus Vahrenkamp, Adrian Knobloch
* @copyright  2010, 2011, 2017 Manfred Kroehnert, Nikolaus Vahrenkamp, Adrian Knobloch
*             GNU Lesser General Public License
*
*/
#pragma once

#include "../../Model/Model.h"
#include "../VisualizationSet.h"

class SoNode;
class SoSeparator;

namespace VirtualRobot
{
    /*!
        A Coin3D based implementation of a visualization.
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT CoinVisualizationSet : public VisualizationSet
    {
        friend class CoinVisualizationFactory;
    protected:
        CoinVisualizationSet(const std::vector<VisualizationPtr>& visualizations);

    public:
        virtual ~CoinVisualizationSet() override = default;

        virtual std::string toXML(const std::string &basePath, int tabs) const override;
        virtual std::string toXML(const std::string &basePath, const std::string &filename, int tabs) const override;

        virtual bool saveModel(const std::string &modelPath, const std::string &filename) override;
    };

    typedef std::shared_ptr<CoinVisualizationSet> CoinVisualizationSetPtr;

} // namespace VirtualRobot
