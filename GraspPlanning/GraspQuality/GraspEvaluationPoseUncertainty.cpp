#include <Eigen/Geometry>
#include "GraspEvaluationPoseUncertainty.h"
#include <VirtualRobot/Random.h>
#include <VirtualRobot/math/Helpers.h>

#include <algorithm>
#include <random>
#include <cfloat>
#include <cstdlib>

using namespace Eigen;
using namespace VirtualRobot;

namespace GraspStudio {

GraspEvaluationPoseUncertainty::GraspEvaluationPoseUncertainty(const PoseUncertaintyConfig& config)
{
	this->config = config;
}

GraspEvaluationPoseUncertainty::~GraspEvaluationPoseUncertainty()
= default;


std::vector<Eigen::Matrix4f> GraspEvaluationPoseUncertainty::generatePoses(
        const Eigen::Matrix4f &objectGP, const Eigen::Matrix4f &graspCenterGP)
{
	std::vector<Eigen::Matrix4f> result;
	Eigen::Matrix4f trafoGraspCenterToObjectCenter = objectGP * math::Helpers::InvertedPose(graspCenterGP);
	Eigen::Matrix4f initPose = graspCenterGP;
	Eigen::Vector3f rpy;
	MathTools::eigen4f2rpy(initPose, rpy);
    
	float initPoseRPY[6];
	initPoseRPY[0] = initPose(0, 3);
	initPoseRPY[1] = initPose(1, 3);
	initPoseRPY[2] = initPose(2, 3);
	initPoseRPY[3] = rpy(0);
	initPoseRPY[4] = rpy(1);
	initPoseRPY[5] = rpy(2);

	float start[6];
	float end[6];
	float step[6];
	float tmpPose[6];
    
	for (int i = 0; i < 6; i++)
	{
		if (config.enableDimension[i])
		{
			start[i] = initPoseRPY[i] - config.dimExtends[i];
			end[i] = initPoseRPY[i] + config.dimExtends[i];
			step[i] = config.stepSize[i];
		}
		else
		{
			start[i] = initPoseRPY[i];
			end[i] = initPoseRPY[i];
			step[i] = 1.0f;
		}
	}

	Eigen::Matrix4f m;

	for (float a = start[0]; a <= end[0]; a += step[0])
	{
		tmpPose[0] = a;
		for (float b = start[1]; b <= end[1]; b += step[1])
		{
			tmpPose[1] = b;
			for (float c = start[2]; c <= end[2]; c += step[2])
			{
				tmpPose[2] = c;
				for (float d = start[3]; d <= end[3]; d += step[3])
				{
					tmpPose[3] = d;
					for (float e = start[4]; e <= end[4]; e += step[4])
					{
						tmpPose[4] = e;
						for (float f = start[5]; f <= end[5]; f += step[5])
						{
							tmpPose[5] = f;
							MathTools::posrpy2eigen4f(tmpPose, m);
							m = m * trafoGraspCenterToObjectCenter;
							result.push_back(m);
						}
					}
				}
			}
		}
	}
	return result;
}


std::vector<Eigen::Matrix4f> GraspEvaluationPoseUncertainty::generatePoses(
        const Eigen::Matrix4f &objectGP, const Eigen::Matrix4f &graspCenterGP, int numPoses)
{
    std::vector<Eigen::Matrix4f> result;
    //Eigen::Matrix4f trafoGraspCenterToObjectCenter = objectGP * graspCenterGP.inverse();
    Eigen::Matrix4f trafoGraspCenterToObjectCenter = math::Helpers::InvertedPose(graspCenterGP) * objectGP;
    Eigen::Matrix4f initPose = graspCenterGP;
    Eigen::Vector3f rpy;
    MathTools::eigen4f2rpy(initPose, rpy);
    
    float initPoseRPY[6];
    initPoseRPY[0] = initPose(0, 3);
    initPoseRPY[1] = initPose(1, 3);
    initPoseRPY[2] = initPose(2, 3);
    initPoseRPY[3] = rpy(0);
    initPoseRPY[4] = rpy(1);
    initPoseRPY[5] = rpy(2);

    float start[6];
    float dist[6];
    float tmpPose[6];
    for (int i = 0; i < 6; i++)
    {
        start[i] = initPoseRPY[i];
        if (config.enableDimension[i])
        {
            if (i<3)
                dist[i] = config.posDeltaMM;
            else
                dist[i] = config.oriDeltaDeg/180.0f*float(M_PI);
        }
        else
        {
            dist[i] = 0.0f;
        }
    }

    Eigen::Matrix4f m;
    std::normal_distribution<float> normalDistribution(0.0, 0.5);
    std::uniform_real_distribution<float> uniformDistribution(0.0, 1);
    
    for (int j = 0; j < numPoses; j++)
    {
        for (int i = 0; i < 6; i++)
        {
            float r = config.useNormalDistribution ? normalDistribution(VirtualRobot::PRNG64Bit())
                                                   : uniformDistribution(VirtualRobot::PRNG64Bit());
            tmpPose[i] = start[i] + r*dist[i];
        }
        MathTools::posrpy2eigen4f(tmpPose, m);
        m = m * trafoGraspCenterToObjectCenter;
        result.push_back(m);
    }
    return result;
}

std::vector<Eigen::Matrix4f> GraspEvaluationPoseUncertainty::generatePoses(
        const Eigen::Matrix4f &objectGP,
        const VirtualRobot::EndEffector::ContactInfoVector &contacts,
        int numPoses)
{
    Eigen::Vector3f centerPose = centerPose.Zero();
    if (contacts.empty())
    {
        VR_ERROR << "No contacts" << endl;
        return std::vector<Eigen::Matrix4f>();
    }
    
    for (size_t i = 0; i < contacts.size(); i++)
    {
            if (config.verbose)
                cout << "contact point:" << i << ": \n" << contacts[i].contactPointObstacleGlobal << endl;
            centerPose += contacts[i].contactPointObstacleGlobal;
    }
    centerPose /= contacts.size();
    
    if (config.verbose)
        cout << "using contact center pose:\n" << centerPose << endl;

    return generatePoses(objectGP, math::Helpers::Pose(centerPose), numPoses);
}

GraspEvaluationPoseUncertainty::PoseEvalResult GraspEvaluationPoseUncertainty::evaluatePose(
        EndEffectorPtr eef, ObstaclePtr object, const Matrix4f &objectPose,
        GraspQualityMeasurePtr qm, VirtualRobot::RobotConfigPtr preshape)
{
    PoseEvalResult result;
    result.forceClosure = false;
    result.quality = 0.0f;
    result.initialCollision = false;

     SceneObjectSetPtr eefColModel = eef->createSceneObjectSet();

    if (!eef || !qm)
    {
        VR_ERROR << "Missing parameters" << endl;
        return result;
    }

    if (preshape)
    {
        eef->getRobot()->setJointValues(preshape);
    } else
    {
        eef->openActors();
    }
    object->setGlobalPose(objectPose);

    // check for initial collision
    if (object->getCollisionChecker()->checkCollision(object->getCollisionModel(), eefColModel))
    {
        result.initialCollision = true;
        return result;
    }

    // collision free
    EndEffector::ContactInfoVector cont = eef->closeActors(object);
    qm->setContactPoints(cont);

    result.quality = qm->getGraspQuality();
    result.forceClosure = qm->isGraspForceClosure();

    return result;
}

GraspEvaluationPoseUncertainty::PoseEvalResults GraspEvaluationPoseUncertainty::evaluatePoses(
        EndEffectorPtr eef, ObstaclePtr object, const std::vector<Eigen::Matrix4f> &objectPoses,
        GraspQualityMeasurePtr qm, VirtualRobot::RobotConfigPtr preshape)
{
    PoseEvalResults res;
    res.avgQuality = 0.0f;
    res.forceClosureRate = 0.0f;
    res.avgQualityCol = 0.0f;
    res.forceClosureRateCol = 0.0f;
    res.numPosesTested = 0;
    res.numValidPoses = 0;
    res.numColPoses = 0;


    if (!eef || !qm)
    {
        VR_ERROR << "Missing parameters" << endl;
        return res;
    }

    if (!eef->getRobot())
    {
        VR_WARNING << "missing eef->robot" << endl;
        return res;
    }

    Eigen::Matrix4f eefRobotPoseInit = eef->getRobot()->getGlobalPose();
    Eigen::Matrix4f objectPoseInit = object->getGlobalPose();
    VirtualRobot::RobotConfigPtr initialConf = eef->getConfiguration();

    std::vector<PoseEvalResult> results;
    for (const auto& objectPose : objectPoses)
    {
        results.push_back(evaluatePose(eef, object, objectPose, qm, preshape));
    }

    if (results.empty())
        return res;

    res.numPosesTested = static_cast<int>(results.size());
    for (const auto& result : results)
    {
        if (result.initialCollision)
        {
            res.numColPoses++;
        }
        else
        {
            res.numValidPoses++;
            res.avgQuality += result.quality;
            res.avgQualityCol += result.quality;
            if (result.forceClosure)
            {
                res.forceClosureRate += 1.0f;
                res.forceClosureRateCol += 1.0f;
                res.numForceClosurePoses++;
            }
        }
    }

    if (res.numValidPoses > 0)
    {
        res.forceClosureRate /= static_cast<float>(res.numValidPoses);
        res.avgQuality /= static_cast<float>(res.numValidPoses);
    }
    if (res.numPosesTested > 0)
    {
        res.forceClosureRateCol /= static_cast<float>(res.numPosesTested);
        res.avgQualityCol /= static_cast<float>(res.numPosesTested);
    }

    // restore setup
    eef->getRobot()->setGlobalPose(eefRobotPoseInit);
    object->setGlobalPose(objectPoseInit);
    eef->getRobot()->setConfig(initialConf);

    return res;
}

GraspEvaluationPoseUncertainty::PoseEvalResults GraspEvaluationPoseUncertainty::evaluateGrasp(
        VirtualRobot::GraspPtr grasp, VirtualRobot::EndEffectorPtr eef, VirtualRobot::ObstaclePtr object, 
        GraspQualityMeasurePtr qm, int numPoses)
{
    PoseEvalResults res;
    res.avgQuality = 0.0f;
    res.forceClosureRate = 0.0f;
    res.numPosesTested = 0;
    res.numValidPoses = 0;
    res.numColPoses = 0;

    if (!grasp || !eef || !object || !qm)
    {
        VR_WARNING << "missing parameters"<< endl;
        return res;
    }
    if (!eef->getRobot())
    {
        VR_WARNING << "missing eef->robot" << endl;
        return res;
    }

    Eigen::Matrix4f eefRobotPoseInit = eef->getRobot()->getGlobalPose();
    Eigen::Matrix4f objectPoseInit = object->getGlobalPose();
    VirtualRobot::RobotConfigPtr initialConf = eef->getConfiguration();

    std::string graspPreshapeName = grasp->getPreshapeName();
    VirtualRobot::RobotConfigPtr graspPS;
    if (eef->hasPreshape(graspPreshapeName))
        graspPS = eef->getPreshape(graspPreshapeName);

    Eigen::Matrix4f mGrasp = grasp->getTcpPoseGlobal(object->getGlobalPose());

    // apply grasp
    eef->getRobot()->setGlobalPoseForRobotNode(eef->getTcp(), mGrasp);

    if (graspPS)
    {
        eef->getRobot()->setJointValues(graspPS);
    }
    else
    {
        eef->openActors();
    }
        


    auto contacts = eef->closeActors(object);
    if (contacts.empty())
    {
        VR_INFO << "No contacts for grasp " << grasp->getName() << " found" << std::endl;
        return res;
    }
    
    auto poses = generatePoses(object->getGlobalPose(), contacts, numPoses);
    if (poses.empty())
    {
        VR_INFO << "No poses for grasp found" << std::endl;
        return res;
    }
    res = evaluatePoses(eef, object, poses, qm, graspPS);

    // restore setup
    eef->getRobot()->setGlobalPose(eefRobotPoseInit);
    object->setGlobalPose(objectPoseInit);
    eef->getRobot()->setConfig(initialConf);

    return res;
}

std::ostream& operator<<(std::ostream& os, const GraspEvaluationPoseUncertainty::PoseEvalResults& rhs)
{
    os << "Robustness analysis" << endl;
    os << "Num Poses Tested: \t" << rhs.numPosesTested << endl;
    os << "Num Poses Valid:  \t" << rhs.numValidPoses << endl;
    
    float colPercent = 0.0f;
    if (rhs.numPosesTested > 0)
        colPercent = float(rhs.numColPoses) / float(rhs.numPosesTested) * 100.0f;
    
    os << "Num Poses initially in collision:\t" << rhs.numColPoses << " (" << colPercent << "%)" << endl;
    os << "Avg Quality (only col freeposes):\t" << rhs.avgQuality << endl;
    os << "FC rate (only col free poses):   \t" << rhs.forceClosureRate * 100.0f << "%" << endl;
    os << "Avg Quality (all poses):         \t" << rhs.avgQualityCol << endl;
    os << "FC rate (all poses):             \t" << rhs.forceClosureRateCol * 100.0f << "%" << endl;
    
    return os;
}


}
