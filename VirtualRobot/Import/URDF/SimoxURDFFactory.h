/*
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

#ifndef _VirtualRobot_SimoxURDFFactory_h_
#define _VirtualRobot_SimoxURDFFactory_h_

#include "../../VirtualRobotImportExport.h"
#include "../../Model/Model.h"
#include "../RobotImporterFactory.h"

#include <urdf_model/model.h>

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT SimoxURDFFactory  : public RobotImporterFactory
    {
    public:
        SimoxURDFFactory();
        virtual ~SimoxURDFFactory();

        /*!
            Load an urdf model and convert it to a simox robot.
            The 3D model files are assumed to be accessible through the VirtualRobot::RuntimeEnvironment paths; be sure to add any 3d model paths there.
            \param filename The urdf model file
            \param loadMode Load only structure, with collision model, or full visualization
        */
        virtual RobotPtr loadFromFile(const std::string& filename, ModelIO::RobotDescription loadMode = ModelIO::eFull);

        /*!
            \param useColModelsIfNoVisuModel If set (standard), a missing visualization is compensated by using the collision model (e.g. when the visu loading failed)
        */
        void set3DModelMode(bool useColModelsIfNoVisuModel);


        /*!
            Convert an urdf model.
            \param urdfModel The model
            \param loadMode Currently only full loading is supported
            \param useColModelsIfNoVisuModel If set, a missing visualization is compensated by using the collision model (e.g. when the visu loading failed)
        */
        RobotPtr createRobot(const urdf::ModelInterface &urdfModel, const std::string& basePath, ModelIO::RobotDescription loadMode, bool useColModelsIfNoVisuModel = true);

        // AbstractFactoryMethod
    public:
        static std::string getName();
        static std::shared_ptr<RobotImporterFactory> createInstance(void*);
    private:
        static SubClassRegistry registry;


        // RobotImporterFactory interface
    public:
        virtual std::string getFileExtension();
        virtual std::string getFileFilter();


    protected:
        RobotNodePtr createBodyNode(RobotPtr robot, boost::shared_ptr<urdf::Link> urdfBody, const std::string& basePath, ModelIO::RobotDescription loadMode, bool useColModelsIfNoVisuModel = true);
        RobotNodePtr createJointNode(RobotPtr robot, boost::shared_ptr<urdf::Joint> urdfJoint);
        Eigen::Matrix4f convertPose(urdf::Pose& p);
        VirtualRobot::VisualizationNodePtr convertVisu(boost::shared_ptr<urdf::Geometry> g, urdf::Pose& pose, const std::string& basePath);
        VirtualRobot::VisualizationNodePtr convertVisuArray(std::vector<boost::shared_ptr<urdf::Visual> > visu_array, const std::string& basePath);
        VirtualRobot::VisualizationNodePtr convertVisuArray(std::vector<boost::shared_ptr<urdf::Collision> > visu_array, const std::string& basePath);
        std::string getFilename(const std::string& f, const std::string& basePath);

        bool useColModelsIfNoVisuModel;
    };

} // namespace VirtualRobot

#endif
