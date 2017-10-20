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

#ifndef _VirtualRobot_Gravity_h_
#define _VirtualRobot_Gravity_h_

#include "../VirtualRobot.h"

namespace VirtualRobot
{
    class Gravity
    {
    public:
        Gravity(VirtualRobot::RobotPtr robot, VirtualRobot::RobotNodeSetPtr rnsJoints, VirtualRobot::RobotNodeSetPtr rnsBodies);
        virtual ~Gravity();

        /*!
         * \brief computeGravityTorque Computes the torque that is needed to counteract gravity.
         * \return name value map with the computed torques
         */
        std::map<std::string, float> computeGravityTorque();

        void computeGravityTorque(std::vector<float> &storeValues);
        void computeGravityTorqueOptimized(std::vector<float> &storeValues);
    protected:
        struct GravityData;
        typedef boost::shared_ptr<GravityData> GravityDataPtr;
        struct GravityData : boost::enable_shared_from_this<GravityData>
        {
            GravityData();
            static GravityDataPtr create(SceneObjectPtr node, const std::vector<VirtualRobot::RobotNodePtr> &joints, const std::vector<VirtualRobot::RobotNodePtr> &bodies, std::vector<GravityDataPtr> &dataVec);
            void init(SceneObjectPtr node, const std::vector<VirtualRobot::RobotNodePtr> &joints, const std::vector<VirtualRobot::RobotNodePtr> &bodies, std::vector<GravityDataPtr> &dataVec);
            std::map<std::string, GravityDataPtr> children;
            SceneObjectPtr node;
            float massSum = 0.0f;
            float torque = 0.0f;
            bool computeTorque = false;
            bool computeCoM = false;
            void computeCoMAndTorque(Eigen::Vector3f& comPositionGlobal);
        };
        std::vector<GravityDataPtr> gravityDataHelperVec;
        GravityDataPtr gravityDataHelperRoot;

        VirtualRobot::RobotPtr robot;

        // this rns is used to update the current pose of the robot
        VirtualRobot::RobotNodeSetPtr rns;

        // this rns is used to update the current pose of the robot
        VirtualRobot::RobotNodeSetPtr rnsBodies;

        std::vector<VirtualRobot::RobotNodePtr> nodes;
        std::vector<VirtualRobot::RobotNodePtr> nodesBodies;

        Eigen::MatrixXi children;
    };

    typedef boost::shared_ptr<Gravity> GravityPtr;


}

#endif

