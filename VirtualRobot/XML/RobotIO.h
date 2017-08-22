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
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_RobotIO_h_
#define _VirtualRobot_RobotIO_h_

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

    class VIRTUAL_ROBOT_IMPORT_EXPORT RobotIO : public BaseIO
    {
    public:

        enum RobotDescription
        {
            eFull,              // load complete robot definition
            eCollisionModel,    // skip visualization tags and load only collision model
            eStructure          // load only the structure of the robot, ignore visualization and collision tags -> faster access when robot is only used for coordinate transformations
        };


        /*!
                Loads robot from meta description file.
                @param xmlFile The file
                @param loadMode Standard: eFull, When eStructure is used no visualization and collision models are loaded for faster access.
                @return Returns an empty pointer, when file access failed.
        */
        static RobotPtr loadRobotModel(const std::string& xmlFile, RobotDescription loadMode = eFull);
        static RobotPtr createRobotModelFromString(const std::string& xmlFile, const std::string& basePath = "", RobotDescription loadMode = eFull);

        static ModelPtr processModelDescription(rapidxml::xml_node<char>* robotXMLNode, const std::string& basePath, RobotIO::RobotDescription loadMode);

    protected:
        // instantiation not allowed
        RobotIO();
        virtual ~RobotIO();

        //static ModelPtr processModel(rapidxml::xml_node<char>* robotModelNode);

        /*!
         * \brief searchFile searches for relative file (to basePath) and on failure tries to find a global file by checking the data dirs
         * \param filename Overwrites with valid filename on success
         * \param basePath
         * \return true on success
         */
        static bool searchFile(std::string& filename, const std::string& basePath);
    };

}

#endif // _VirtualRobot_RobotIO_h_
