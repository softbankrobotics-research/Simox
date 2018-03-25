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
        Gravity(ModelPtr robot, JointSetPtr rnsJoints, LinkSetPtr rnsBodies);
        virtual ~Gravity();

        /*!
         * \brief computeGravityTorque Computes the torque that is needed to counteract gravity.
         * \return name value map with the computed torques
         */
        std::map<std::string, float> computeGravityTorque();

        void computeGravityTorque(std::vector<float> &storeValues);
    protected:
        struct GravityData;
        typedef std::shared_ptr<GravityData> GravityDataPtr;
        struct GravityData : std::enable_shared_from_this<GravityData>
        {
            GravityData();
            static GravityDataPtr create(ModelNodePtr node, const std::vector<VirtualRobot::ModelJointPtr> &joints, const std::vector<VirtualRobot::ModelLinkPtr> &bodies, std::vector<GravityDataPtr> &dataVec);
            void init(ModelNodePtr node, const std::vector<VirtualRobot::ModelJointPtr> &joints, const std::vector<VirtualRobot::ModelLinkPtr> &bodies, std::vector<GravityDataPtr> &dataVec);
            std::map<std::string, GravityDataPtr> children;
            ModelNodePtr node;
            ModelLinkPtr nodeLink;
            ModelJointPtr nodeJoint;
            float massSum = 0.0f;
            float torque = 0.0f;
            bool computeTorque = false;
            bool computeCoM = false;
            void computeCoMAndTorque(Eigen::Vector3f& comPositionGlobal);
        };
        std::vector<GravityDataPtr> gravityDataHelperVec;
        GravityDataPtr gravityDataHelperRoot;

        VirtualRobot::ModelPtr robot;

        // this rns is used to update the current pose of the robot
        VirtualRobot::JointSetPtr rns;

        // this rns is used to update the current pose of the robot
        VirtualRobot::LinkSetPtr rnsBodies;

        std::vector<VirtualRobot::ModelJointPtr> nodes;
        std::vector<VirtualRobot::ModelLinkPtr> nodesBodies;
    };

    typedef std::shared_ptr<Gravity> GravityPtr;


}

#endif

