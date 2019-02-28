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

#pragma once

#include <VirtualRobot/VirtualRobotCommon.h>

#include "GraspQualityMeasure.h"

#include <string>
#include <vector>


namespace GraspStudio
{

/**
 *
 * This class implements the paper:
 *   Jonathan Weisz and Peter K. Allen,
 *   "Pose Error Robust Grasping from Contact Wrench Space Metrics",
 *   2012 IEEE International Conference on Robotics and Automation.
 */
class GRASPSTUDIO_IMPORT_EXPORT GraspEvaluationPoseUncertainty : public boost::enable_shared_from_this<GraspEvaluationPoseUncertainty>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct PoseUncertaintyConfig
    {
        PoseUncertaintyConfig()
        {
            init();
        }

        void init(float maxPosDelta = 10.0f, float maxOriDeltaDeg = 5.0f, bool normalDistribution = true,
                  float stepFactorPos = 0.5f, float stepFactorOri = 0.5f)
		{
            useNormalDistribution = normalDistribution;
            posDeltaMM = maxPosDelta;
            oriDeltaDeg = maxOriDeltaDeg;
			for (int i = 0; i < 6; i++)
			{
				enableDimension[i] = true;
			}
			for (int i = 0; i < 3; i++)
			{
                dimExtends[i] = maxPosDelta;       // mm
                stepSize[i] = maxPosDelta * stepFactorPos;  // 10 mm => 5 mm steps
                
                float maxOriDeltaRad = maxOriDeltaDeg * static_cast<float>(M_PI / 180.0);
                dimExtends[i + 3] = maxOriDeltaRad;
                stepSize[i + 3] = maxOriDeltaRad * stepFactorOri; // 5 deg => degree
			}
		}

		bool enableDimension[6];
		float dimExtends[6];
		float stepSize[6];
        bool useNormalDistribution; // normal or uniform distribution
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
        int numColPoses = 0.0;           // poses with initial collision
        int numForceClosurePoses = 0.0;  // poses that have force closure
        float forceClosureRate = 0.0;    // without collision poses
        float avgQuality = 0.0;          // without collision poses
        float forceClosureRateCol = 0.0; // with collision poses
        float avgQualityCol = 0.0;       // with collision poses

        void print() { VR_INFO << *this << std::endl; }
        friend std::ostream& operator<<(std::ostream& os, const PoseEvalResults& rhs);
    };

    
	/// Construct with the given configuration.
	GraspEvaluationPoseUncertainty(const PoseUncertaintyConfig& config);
	
    /// Destructor.
	virtual ~GraspEvaluationPoseUncertainty();

    
    // Config
    
    PoseUncertaintyConfig& config();
    const PoseUncertaintyConfig& config() const;
    
    // Pose generation
    
	/**
     * Computes the full set of poses according to configuration.
     * \param objectGP The pose of the object. 
     * \param graspCenterGP This could be the pose of the object or the center of the contact points (as proposed in the paper)
	 */
	std::vector<Eigen::Matrix4f> generatePoses(
            const Eigen::Matrix4f &objectGP, const Eigen::Matrix4f &graspCenterGP);
    
    /// Uses the mean of the contact points as grasp center point.
    std::vector<Eigen::Matrix4f> generatePoses(
            const Eigen::Matrix4f &objectGP,
            const VirtualRobot::EndEffector::ContactInfoVector &contacts);
	
    /**
     * Computes a set of poses by randomly sampling within the extends of the configuration.
     * \param objectGP The pose of the object.
     * \param graspCenterGP This could be the pose of the object or the center of the contact points (as proposed in the paper)
     * \param numPoses Number of poses to generate
     */
    std::vector<Eigen::Matrix4f> generatePoses(
            const Eigen::Matrix4f &objectGP, const Eigen::Matrix4f &graspCenterGP, int numPoses);

    /// Uses the mean of the contact points as grasp center point.
    std::vector<Eigen::Matrix4f> generatePoses(
            const Eigen::Matrix4f &objectGP,
            const VirtualRobot::EndEffector::ContactInfoVector &contacts, int numPoses);

    
    // Pose evaluation
    
    PoseEvalResult evaluatePose(
            VirtualRobot::EndEffectorPtr eef, VirtualRobot::ObstaclePtr object, const Eigen::Matrix4f &objectPose,
            GraspQualityMeasurePtr qm, VirtualRobot::RobotConfigPtr preshape = {});
    
    PoseEvalResults evaluatePoses(
            VirtualRobot::EndEffectorPtr eef, VirtualRobot::ObstaclePtr object, const std::vector<Eigen::Matrix4f> &objectPoses,
            GraspQualityMeasurePtr qm, VirtualRobot::RobotConfigPtr preshape = {});

    PoseEvalResults evaluateGrasp(
            VirtualRobot::GraspPtr grasp, VirtualRobot::EndEffectorPtr eef, VirtualRobot::ObstaclePtr object,
            GraspQualityMeasurePtr qm, int numPoses);

    
protected:

    Eigen::Vector3f getMean(const VirtualRobot::EndEffector::ContactInfoVector &contacts) const;
    
    PoseUncertaintyConfig _config;
	
};

typedef boost::shared_ptr<GraspEvaluationPoseUncertainty> GraspEvaluationPoseUncertaintyPtr;

}

