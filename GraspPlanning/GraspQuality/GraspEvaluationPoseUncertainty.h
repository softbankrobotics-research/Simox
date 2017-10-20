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
* @package    GraspStudio
* @author     Nikolaus Vahrenkamp
* @copyright  2017 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/

#ifndef _GraspStudio_GraspEvaluationPoseUncertainty_h_
#define _GraspStudio_GraspEvaluationPoseUncertainty_h_

#include <VirtualRobot/VirtualRobotCommon.h>

#include "GraspQualityMeasure.h"

#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

namespace GraspStudio
{

/*!
	This class implements the paper:
	Jonathan Weisz and Peter K. Allen, "Pose Error Robust Grasping from Contact Wrench Space Metrics", 2012 IEEE International Conference on Robotics and Automation
*/
class GraspEvaluationPoseUncertainty : public boost::enable_shared_from_this<GraspEvaluationPoseUncertainty>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct PoseUncertaintyConfig
    {
        PoseUncertaintyConfig()
        {
            init();
        }

        void init(float maxPosDelta = 10.0f, float maxOriDelta = 5.0f, bool normalDistribution = true)
		{
            useNormalDistribution = normalDistribution;
            posDeltaMM = maxPosDelta;
            oriDeltaDeg = maxOriDelta;
			for (int i = 0; i < 6; i++)
			{
				enableDimension[i] = true;
			}
			for (int i = 0; i < 3; i++)
			{
                dimExtends[i] = maxPosDelta; //mm
                dimExtends[i + 3] = maxOriDelta / 180.0f * float(M_PI); // degrees
                stepSize[i] = maxPosDelta*0.5f;
                stepSize[i + 3] = maxOriDelta*0.5f / 180.0f * float(M_PI); // 3 degree
			}
		}

		bool enableDimension[6];
		float dimExtends[6];
		float stepSize[6];
        float useNormalDistribution; // normal or uniform distribution
        bool verbose = false;

        float posDeltaMM;
        float oriDeltaDeg;
	};

    struct PoseEvalResult
    {
        bool forceClosure;
        float quality;
        bool initialCollision; // ignored due to initial collision
    };

    struct PoseEvalResults
    {
        int numPosesTested = 0.0;
        int numValidPoses = 0.0;
        int numColPoses = 0.0;        // poses with initial collision
        int numForceClosurePoses = 0.0;        // poses that have force closure
        float forceClosureRate = 0.0; // without collision poses
        float avgQuality = 0.0;       // without collision poses
        float forceClosureRateCol = 0.0; // with collision poses
        float avgQualityCol = 0.0;       // with collision poses

        void print()
        {
            VR_INFO << "Robustness analysis" << endl;
            VR_INFO << "Num Poses Tested:" << numPosesTested << endl;
            VR_INFO << "Num Poses Valid:" << numValidPoses << endl;
            float colPercent = 0.0f;
            if (numPosesTested>0)
                colPercent = float(numColPoses) / float(numPosesTested) * 100.0f;
            VR_INFO << "Num Poses initially in collision:" << numColPoses << " == " << colPercent << "%" << endl;
            VR_INFO << "Avg Quality (only col freeposes):" << avgQuality << endl;
            VR_INFO << "FC rate (only col free poses):" << forceClosureRate * 100.0f << "%" << endl;
            VR_INFO << "Avg Quality (all poses):" << avgQualityCol << endl;
            VR_INFO << "FC rate (all poses):" << forceClosureRateCol * 100.0f << "%" << endl;
        }
    };

	/*!
		@brief Initialize the pose quality calculation

	*/
	GraspEvaluationPoseUncertainty(const PoseUncertaintyConfig& config);
	
	virtual ~GraspEvaluationPoseUncertainty();

	/*!
        Computes the full set of poses according to configuration.
		\param objectGP The pose of the object. 
		\param graspCenterGP This could be the pose of the object or the center of the contact points (as proposed in the paper)
	*/
	std::vector<Eigen::Matrix4f> generatePoses(const Eigen::Matrix4f &objectGP, const Eigen::Matrix4f &graspCenterGP);
	
    /*!
        Computes a set of poses by randomly sampling within the extends of the configuration.
        \param objectGP The pose of the object.
        \param graspCenterGP This could be the pose of the object or the center of the contact points (as proposed in the paper)
        \param numPoses Number of poses to generate
    */
    std::vector<Eigen::Matrix4f> generatePoses(const Eigen::Matrix4f &objectGP, const Eigen::Matrix4f &graspCenterGP, int numPoses);

    std::vector<Eigen::Matrix4f> generatePoses(const Eigen::Matrix4f &objectGP, const VirtualRobot::EndEffector::ContactInfoVector &contacts, int numPoses);

    PoseEvalResult evaluatePose(VirtualRobot::EndEffectorPtr eef, VirtualRobot::ObstaclePtr o, const Eigen::Matrix4f &objectPose, GraspQualityMeasurePtr qm, VirtualRobot::RobotConfigPtr preshape = VirtualRobot::RobotConfigPtr());
    PoseEvalResults evaluatePoses(VirtualRobot::EndEffectorPtr eef, VirtualRobot::ObstaclePtr o, const std::vector<Eigen::Matrix4f> &objectPoses, GraspQualityMeasurePtr qm, VirtualRobot::RobotConfigPtr preshape = VirtualRobot::RobotConfigPtr());

    PoseEvalResults evaluateGrasp(VirtualRobot::GraspPtr g, VirtualRobot::EndEffectorPtr eef, VirtualRobot::ObstaclePtr o, GraspQualityMeasurePtr qm, int numPoses);

protected:

	PoseUncertaintyConfig config;
	
};

typedef boost::shared_ptr<GraspEvaluationPoseUncertainty> GraspEvaluationPoseUncertaintyPtr;

}

#endif
