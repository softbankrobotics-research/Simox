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
#ifndef __PoseQualityManipulability_H_
#define __PoseQualityManipulability_H_

#include "PoseQualityMeasurement.h"

namespace VirtualRobot
{

/**
* 
* \class PoseQualityManipulability
*
* This measurement computes the Yoshikawa's manipulability index by computing the Singular value Decomposition (SVD) of the Jacobian.
* Two modes are offered: Either all singular values are multiplied
* or the ratio of minimum and maximum singular Value is returned (also known as Condition number).
*/
class VIRTUAL_ROBOT_IMPORT_EXPORT PoseQualityManipulability :  public PoseQualityMeasurement
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum ManipulabilityIndexType
	{
		eMultiplySV,	// multiply all singular values
		eMinMaxRatio	// ratio of max and min singular value (aka Condition number)
	};

	PoseQualityManipulability(VirtualRobot::RobotNodeSetPtr rns, ManipulabilityIndexType i = eMinMaxRatio);
	~PoseQualityManipulability();

	virtual float getPoseQuality();
	float getManipulability(ManipulabilityIndexType i);

	/*!
		Returns the singular values of the current Jacobian. The values are ordered, starting with the largest value.
	*/
	Eigen::VectorXf getSingularValues();

	/*
		Computes the scaled singular vectors (U) of the Jacobian with SVD.
		The columns of the returned matrix are the Cartesian singular vectors of the Jacobi, scaled with 1/maxSingularValue.
		The columns of U are called 'Cartesian Vectors' since they describe the Cartesian relationship. 
		SVD:  J = USV'
	*/
	Eigen::MatrixXf getSingularVectorCartesian();
protected:

	VirtualRobot::DifferentialIKPtr jacobain;
	ManipulabilityIndexType manipulabilityType;
};

}

#endif // __PoseQualityManipulability_H_
