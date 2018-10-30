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
* @copyright  2013 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#pragma once

#include <VirtualRobot/Model/Model.h>
#include <VirtualRobot/Model/Model.h>
#include <VirtualRobot/Model/Nodes/ModelNode.h>
#include <VirtualRobot/Model/ModelNodeSet.h>

namespace VirtualRobot
{

    /*!
        A feet posture comprises information about
            * the robot and the corresponding RobotNodeSets
            * The base node (eg the hip)
            * The TCPs of both feet
            * the Cartesian relation of both feet when applying the posture
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT FeetPosture
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        FeetPosture(JointSetPtr leftLeg,
                    JointSetPtr rightLeg,
					LinkSetPtr leftLegCol,
					LinkSetPtr rightLegCol,
					Eigen::Matrix4f& transformationLeftToRightFoot,
                    ModelNodePtr baseNode,
                    FramePtr leftTCP = FramePtr(),
					FramePtr rightTCP = FramePtr(),
                    JointSetPtr rnsLeft2RightFoot = JointSetPtr()
                   );

        virtual ~FeetPosture();

		JointSetPtr getLeftLeg();
		JointSetPtr getRightLeg();
        LinkSetPtr getLeftLegCol();
		LinkSetPtr getRightLegCol();
		FramePtr getLeftTCP();
		FramePtr getRightTCP();
		ModelNodePtr getBaseNode();
        ModelPtr getRobot();
        Eigen::Matrix4f getTransformationLeftToRightFoot();

        //! Initially the rns of left and right leg are used as collision models. Here you can set other sets for collision detection.
        void setCollisionCheck(LinkSetPtr leftColModel, LinkSetPtr rightColModel);

        //! The collision models are queried if there is a collision for the current config.
        bool icCurrentLegConfigCollisionFree();

        //! Optional: kinematic chain from left foot to waist to right foot (not supported by all kinematic structures)
        JointSetPtr getRNSLeft2RightFoot();
        void setRNSLeft2RightFoot(JointSetPtr rns);

        void print();
    protected:

		JointSetPtr leftLeg;
		JointSetPtr rightLeg;
		LinkSetPtr leftLegCol;
		LinkSetPtr rightLegCol;
		JointSetPtr left2Right;
        Eigen::Matrix4f transformationLeftToRightFoot;
		FramePtr leftTCP;
		FramePtr rightTCP;
        ModelNodePtr baseNode;

    };

    typedef std::shared_ptr<FeetPosture> FeetPosturePtr;

} // namespace VirtualRobot

