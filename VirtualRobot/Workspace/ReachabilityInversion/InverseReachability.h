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
* @copyright  2012 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_InverseReachability_h_
#define _VirtualRobot_InverseReachability_h_

#include "../../VirtualRobot.h"
#include  <VirtualRobot/Workspace/WorkspaceRepresentation.h>
#include  <VirtualRobot/VirtualRobot.h>


namespace VirtualRobot
{

/*!
*/
class VIRTUAL_ROBOT_IMPORT_EXPORT InverseReachability : public VirtualRobot::WorkspaceRepresentation, public boost::enable_shared_from_this<InverseReachability>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/*!
		Invert the given workspace representation.
		\param ws The reachability/manipulability data that should be inverted
		\param factorTranslation The translational discretization factor of the inverse reachability is calculated with ws->getDiscretizeParameterTranslation()*factorTranslation
		\param factorOrientation The orientaional discretization factor of the inverse reachability is calculated with ws->getDiscretizeParameterRotation()*factorOrientation
	*/
    InverseReachability(VirtualRobot::WorkspaceRepresentationPtr ws, float factorTranslation=1.0f, float factorOrientation=2.0f);

	/*!
		Load inverse reachability from file
	*/
    InverseReachability(VirtualRobot::WorkspaceRepresentationPtr ws, const std::string &filename);

	/*!
		Load inverse reachability from file
	*/
	InverseReachability(VirtualRobot::RobotPtr robot, const std::string &filename);

	/*!
		Set the pose of this inverse reachability data. E.g. a grasping pose.
	*/
	void setGlobalPose(const Eigen::Matrix4f &gp);

	/*!
		Returns true, if the corresponding reachability entry is non zero.
	*/
	bool isReachable(const Eigen::Matrix4f &globalPose);

	//! returns a random pose that is covered by the workspace data.
	Eigen::Matrix4f sampleReachablePose();
	    
protected:

	void buildData(float factorTranslation=1.0f, float factorOrientation=2.0f);

	// this data is not linked to a robot's node. Instead it can be linked to an object/grasp etc..
	virtual Eigen::Matrix4f getToLocalTransformation() const;
	virtual Eigen::Matrix4f getToGlobalTransformation() const;

	Eigen::Matrix4f dataPose;

	std::string robotBaseNode;

	VirtualRobot::WorkspaceRepresentationPtr ws;

    void addInverseData(Eigen::Matrix4f &m, unsigned char e);
};


typedef boost::shared_ptr<InverseReachability> InverseReachabilityPtr;
} // namespace VirtualRobot

#endif // _Reachability_h_
