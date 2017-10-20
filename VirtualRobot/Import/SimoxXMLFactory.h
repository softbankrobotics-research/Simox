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
* @copyright  2014 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_SimoxXMLFactory_h_
#define _VirtualRobot_SimoxXMLFactory_h_

#include "../Model/Model.h"
#include "../Model/Model.h"
#include "RobotImporterFactory.h"




namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT SimoxXMLFactory  : public RobotImporterFactory
    {
    public:
        SimoxXMLFactory();
        virtual ~SimoxXMLFactory();

        virtual RobotPtr loadFromFile(const std::string& filename, ModelIO::RobotDescription loadMode = ModelIO::eFull);

        static RobotPtr loadRobotSimoxXML(const std::string& xmlFile, ModelIO::RobotDescription loadMode = ModelIO::eFull);

        /*!
                Creates Robot from string (Simox XML format).
                @param xmlString The input string.
                @param basePath If any \<childFromRobot\> tags are given, the path for searching the robot files can be specified.
                @param loadMode Standard: eFull, When eStructure is used no visualization and collision models are loaded for faster access.
            */
        static RobotPtr createRobotFromSimoxXMLString(const std::string& xmlString, const std::string& basePath = "", ModelIO::RobotDescription loadMode = ModelIO::RobotDescription::eFull);


        /*!
            Creates an XML string that defines the robot and stores it to the file basePath/filename. All visualizations and collision models are stored to the basePath/modeDir directory
            @param robot The robot to save.
            @param filename The filename without path.
            @param basePath The directory to store the robot to
            @param modelDir The local directory where all visualization files should be stored to.
        */
        static bool saveXML(RobotPtr robot, const std::string& filename, const std::string& basePath, const std::string& modelDir = "models", bool storeEEF = true, bool storeRNS = true, bool storeSensors = true, bool storeModelFiles = true);



        // AbstractFactoryMethod
    public:
        static std::string getName();
        static std::shared_ptr<RobotImporterFactory> createInstance(void*);
    private:
        static SubClassRegistry registry;


        // RobotImporterFactory interface
    public:
        virtual std::string getFileFilter();
        virtual std::string getFileExtension();

    protected:


        struct ChildFromRobotDef
        {
            std::string filename;
            bool importEEF;
        };

        static RobotPtr processRobot(rapidxml::xml_node<char>* robotXMLNode, const std::string& basePath, ModelIO::RobotDescription loadMode = ModelIO::RobotDescription::eFull);
        static RobotPtr processRobotAttributes(rapidxml::xml_node<char>* robotXMLNode, std::string& robotRoot);
        static void processRobotChildNodes(rapidxml::xml_node<char>* robotXMLNode,
                                           RobotPtr robo,
                                           const std::string& robotRoot,
                                           const std::string& basePath,
                                           std::map < RobotNodePtr,
                                           std::vector<ChildFromRobotDef> > & childrenFromRobotFilesMap,
                                           std::vector<rapidxml::xml_node<char>* >& robotNodeSetNodes,
                                           std::vector<rapidxml::xml_node<char>* >& endeffectorNodes,
                                           ModelIO::RobotDescription loadMode = ModelIO::RobotDescription::eFull);
        static RobotNodePtr processRobotNode(rapidxml::xml_node<char>* robotNodeXMLNode,
                                             RobotPtr robo,
                                             const std::string& basePath,
                                             int& robotNodeCounter,
                                             std::vector< std::string >& childrenNames,
                                             std::vector< ChildFromRobotDef >& childrenFromRobot,
                                             ModelIO::RobotDescription loadMode = ModelIO::RobotDescription::eFull);

        //static RobotNodeSetPtr processRobotNodeSet(rapidxml::xml_node<char> *setXMLNode, RobotPtr robo, const std::string &rootName, int &robotNodeSetCounter);
        static void processChildNode(rapidxml::xml_node<char>* childXMLNode, std::vector<std::string>& childrenNames);
        static ModelJointPtr processJointNode(rapidxml::xml_node<char>* jointXMLNode,
            const std::string& robotNodeName,
            RobotPtr robot,
            const Eigen::Matrix4f& transformationMatrix);
        static ModelLinkPtr processLinkNode(const std::string& robotNodeName,
            RobotPtr robot,
            VisualizationNodePtr visualizationNode,
            CollisionModelPtr collisionModel,
            ModelLink::Physics& physics,
            const Eigen::Matrix4f& transformationMatrix);

        static void processChildFromRobotNode(rapidxml::xml_node<char>* childXMLNode, const std::string& nodeName, std::vector< ChildFromRobotDef >& childrenFromRobot);
        static void processLimitsNode(rapidxml::xml_node<char>* limitsXMLNode, float& jointLimitLo, float& jointLimitHi, bool &limitless);
        //static bool processSensor(RobotNodePtr rn, rapidxml::xml_node<char>* sensorXMLNode, RobotDescription loadMode, const std::string& basePath);
        static std::map<std::string, int> robot_name_counter;
        static VisualizationNodePtr checkUseAsColModel(rapidxml::xml_node<char>* visuXMLNode, const std::string& robotNodeName, const std::string& basePath);
        static std::mutex mutex;
    };

} // namespace VirtualRobot

#endif // _VirtualRobot_SimoxXMLFactory_h_
