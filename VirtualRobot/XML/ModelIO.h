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
#ifndef _VirtualRobot_ModelIO_h_
#define _VirtualRobot_ModelIO_h_

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

    class VIRTUAL_ROBOT_IMPORT_EXPORT ModelIO : public BaseIO
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
        static ModelPtr loadModel(const std::string& xmlFile, RobotDescription loadMode = eFull);
        static ModelPtr createRobotModelFromString(const std::string& xmlFile, const std::string& basePath = "", RobotDescription loadMode = eFull);

        static ModelPtr processModelDescription(rapidxml::xml_node<char>* robotXMLNode, const std::string& basePath, ModelIO::RobotDescription loadMode);

        static VisualizationPtr processVisualizationTag(rapidxml::xml_node<char>* visuXMLNode, const std::string& tagName, const std::string& basePath, bool& useAsColModel);
        static CollisionModelPtr processCollisionTag(rapidxml::xml_node<char>* colXMLNode, const std::string& tagName, const std::string& basePath);
        static std::vector<Primitive::PrimitivePtr> processPrimitives(rapidxml::xml_node<char>* primitivesXMLNode);
        static void processPhysicsTag(rapidxml::xml_node<char>* physicsXMLNode, const std::string& nodeName, ModelLink::Physics& physics);
        static RobotNodeSetPtr processModelNodeSet(rapidxml::xml_node<char>* setXMLNode, ModelPtr robo, const std::string& robotRootNode, int& robotNodeSetCounter);
        static std::vector<VisualizationPtr> processVisuFiles(rapidxml::xml_node<char>* visualizationXMLNode, const std::string& basePath);

        /*!
         * \brief loadNodeSets Loads one or multiple ModelNodeSets / JointSets / LinkSets from the given file and registers them to the robot
         * \param robot
         * \param filename
         * \return
         */
        static bool loadNodeSets(const ModelPtr &robot, const std::string &filename);
        /*!
         * \brief createNodeSetFromString Extracts one or multiple ModelNodeSets / JointSets / LinkSets from the given string and registers them to the robot
         * \param robot
         * \param xmlString
         * \return true on success
         */
        static bool createNodeSetsFromString(const ModelPtr &robot, const std::string &xmlString);

        /*!
         * \brief loadFrames Load one or multiple frames from file and registers them to the corresponding model nodes
         * \param robot
         * \param filename
         * \return
         */
        static bool loadFrames(const ModelPtr &robot, const std::string &filename);

        /*!
         * \brief createFramesString Parse given string to extract frames. Frames are registered to the robot/model.
         * \param robot
         * \param xmlString
         * \return
         */
        static bool createFramesFromString(const ModelPtr &robot, const std::string &xmlString);


        static FramePtr processFrame(rapidxml::xml_node<char>* frameXMLNode, const ModelPtr &robo);

        static bool processConfiguration(rapidxml::xml_node<char> *configXMLNode, ModelPtr model);
        static bool createConfigurationsFromString(const ModelPtr &robot, const std::string &xmlString);
        static bool loadConfigurations(const ModelPtr &robot, const std::string &filename);


        static bool processSensor(ModelPtr model, rapidxml::xml_node<char> *sensorXMLNode, RobotDescription loadMode, const ModelNodePtr &modelNode = ModelNodePtr());
        static bool createSensorsFromString(const ModelPtr &robot, const std::string &xmlString, RobotDescription loadMode);
        static bool loadSensors(const ModelPtr &robot, const std::string &filename, RobotDescription loadMode);

    protected:
        // instantiation not allowed
        ModelIO();
        virtual ~ModelIO();

        /*!
         * \brief searchFile searches for relative file (to basePath) and on failure tries to find a global file by checking the data dirs
         * \param filename Overwrites with valid filename on success
         * \param basePath
         * \return true on success
         */
        static bool searchFile(std::string& filename, const std::string& basePath);
    };

}

#endif // _VirtualRobot_ModelIO_h_
