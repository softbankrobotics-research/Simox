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
* @author     Nikolaus Vahrenkamp
* @copyright  2017 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_EndEffectorIO_h_
#define _VirtualRobot_EndEffectorIO_h_

#include "../VirtualRobot.h"
#include "../Model/Model.h"
#include "../Tools/Units.h"
#include "../Tools/MathTools.h"
#include "../Model/Model.h"
#include "../Model/Nodes/ModelNode.h"
#include "BaseIO.h"

#include <string>
#include <vector>
#include <map>
#include <fstream>



// using forward declarations here, so that the rapidXML header does not have to be parsed when this file is included
namespace rapidxml
{
    template<class Ch>
    class xml_node;
}

namespace VirtualRobot
{

    class VIRTUAL_ROBOT_IMPORT_EXPORT EndEffectorIO : public BaseIO
    {
    public:

        static bool createEndEffectorsFromString(const RobotPtr &robot, const std::string &xmlString);
        static bool loadEndEffectors(const RobotPtr &robot, const std::string& filename);

        static EndEffectorPtr processEndeffectorNode(rapidxml::xml_node<char>* endeffectorXMLNode, RobotPtr robo);

    protected:
        // instantiation not allowed
        EndEffectorIO();
        virtual ~EndEffectorIO();

        static EndEffectorActorPtr processEndeffectorActorNode(rapidxml::xml_node<char>* endeffectorActorXMLNode, RobotPtr robo);
        static void processEndeffectorStaticNode(rapidxml::xml_node<char>* endeffectorStaticXMLNode, RobotPtr robo, std::vector<ModelLinkPtr>& staticNodesList);
        static EndEffectorActor::CollisionMode processEEFColAttributes(rapidxml::xml_node<char>* node, bool allowOtherAttributes = false);
        static void processActorNodeList(rapidxml::xml_node<char>* parentNode, RobotPtr robot, std::vector<EndEffectorActor::ActorDefinition>& actorList, bool clearList = true);
    };

}

#endif
