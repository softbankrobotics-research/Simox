#include <Eigen/Geometry>
#include "GraspEvaluationPoseUncertainty.h"

#include <algorithm>
#include <float.h>
#include <cstdlib>

using namespace Eigen;
using namespace VirtualRobot;

namespace GraspStudio {

GraspEvaluationPoseUncertainty::GraspEvaluationPoseUncertainty(const PoseUncertaintyConfig& config)
{
	this->config = config;
}

GraspEvaluationPoseUncertainty::~GraspEvaluationPoseUncertainty()
{

}

std::vector<Eigen::Matrix4f> GraspEvaluationPoseUncertainty::generatePoses(const Eigen::Matrix4f &objectGP, const Eigen::Matrix4f &graspCenterGP)
{
	std::vector<Eigen::Matrix4f> result;
	Eigen::Matrix4f trafoGraspCenterToObjectCenter = objectGP * graspCenterGP.inverse();
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


std::vector<Eigen::Matrix4f> GraspEvaluationPoseUncertainty::generatePoses(const Eigen::Matrix4f &objectGP, const Eigen::Matrix4f &graspCenterGP, int numPoses)
{
    std::vector<Eigen::Matrix4f> result;
    //Eigen::Matrix4f trafoGraspCenterToObjectCenter = objectGP * graspCenterGP.inverse();
    Eigen::Matrix4f trafoGraspCenterToObjectCenter = graspCenterGP.inverse() * objectGP;
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
    float dist[6];
    float tmpPose[6];
    for (int i = 0; i < 6; i++)
    {
        if (config.enableDimension[i])
        {
            start[i] = initPoseRPY[i] - config.dimExtends[i];
            end[i] = initPoseRPY[i] + config.dimExtends[i];
            dist[i] = end[i] - start[i];
        }
        else
        {
            start[i] = initPoseRPY[i];
            end[i] = initPoseRPY[i];
            dist[i] = 0.0f;
        }
    }

    Eigen::Matrix4f m;
    std::default_random_engine generator;
    std::normal_distribution<double> normalDistribution(0.0,0.5);
    std::uniform_real_distribution<double> uniformDistribution(0.0,0.5);
    for (int j=0;j<numPoses; j++)
    {
        for (int i = 0; i < 6; i++)
        {
            float r = config.useNormalDistribution ? normalDistribution(generator) : uniformDistribution(generator);
            tmpPose[i] = start[i] + r*dist[i];
        }
        MathTools::posrpy2eigen4f(tmpPose, m);
        m = m * trafoGraspCenterToObjectCenter;
        result.push_back(m);
    }
    return result;
}

std::vector<Eigen::Matrix4f> GraspEvaluationPoseUncertainty::generatePoses(const Eigen::Matrix4f &objectGP, const VirtualRobot::EndEffector::ContactInfoVector &contacts, int numPoses)
{
    Eigen::Vector3f centerPose;
    centerPose.setZero();
    if (contacts.size()==0)
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

    Eigen::Matrix4f centerPoseM = Eigen::Matrix4f::Identity();
    centerPoseM.block(0, 3, 3, 1) = centerPose;

    return generatePoses(objectGP, centerPoseM, numPoses);
}

GraspEvaluationPoseUncertainty::PoseEvalResult GraspEvaluationPoseUncertainty::evaluatePose(EndEffectorPtr eef, ObstaclePtr o, const Matrix4f &objectPose, GraspQualityMeasurePtr qm, VirtualRobot::RobotConfigPtr preshape)
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
        eef->openActors();
    o->setGlobalPose(objectPose);

    // check for initial collision
    if (o->getCollisionChecker()->checkCollision(o->getCollisionModel(), eefColModel))
    {
        result.initialCollision = true;
        return result;
    }

    // collision free
    EndEffector::ContactInfoVector cont = eef->closeActors(o);
    qm->setContactPoints(cont);

    result.quality = qm->getGraspQuality();
    result.forceClosure = qm->isGraspForceClosure();

    return result;
}

GraspEvaluationPoseUncertainty::PoseEvalResults GraspEvaluationPoseUncertainty::evaluatePoses(EndEffectorPtr eef, ObstaclePtr o, const std::vector<Eigen::Matrix4f> &objectPoses, GraspQualityMeasurePtr qm, VirtualRobot::RobotConfigPtr preshape)
{
    PoseEvalResults res;
    res.avgQuality = 0.0f;
    res.forceClosureRate = 0.0f;
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
    Eigen::Matrix4f objectPoseInit = o->getGlobalPose();
    VirtualRobot::RobotConfigPtr initialConf = eef->getConfiguration();

    std::vector<PoseEvalResult> results;
    for (size_t i=0;i<objectPoses.size();i++)
    {
        results.push_back(evaluatePose(eef,o,objectPoses.at(i),qm,preshape));
    }

    if (results.size()==0)
        return res;

    res.numPosesTested = results.size();
    for (size_t i=0;i<results.size();i++)
    {
        if (results.at(i).initialCollision)
        {
            res.numColPoses++;
        }
        else
        {
            res.numValidPoses++;
            res.avgQuality += results.at(i).quality;
            if (results.at(i).forceClosure)
            {
                res.forceClosureRate += 1.0f;
                res.numForceClosurePoses++;
            }
        }
    }

    if (res.numValidPoses>0)
    {
        res.forceClosureRate /= float(res.numValidPoses);
        res.avgQuality /= float(res.numValidPoses);
    }

    // restore setup
    eef->getRobot()->setGlobalPose(eefRobotPoseInit);
    o->setGlobalPose(objectPoseInit);
    eef->getRobot()->setConfig(initialConf);

    return res;
}

GraspEvaluationPoseUncertainty::PoseEvalResults GraspEvaluationPoseUncertainty::evaluateGrasp(VirtualRobot::GraspPtr g, VirtualRobot::EndEffectorPtr eef, VirtualRobot::ObstaclePtr o, GraspQualityMeasurePtr qm, int numPoses)
{
    PoseEvalResults res;
    res.avgQuality = 0.0f;
    res.forceClosureRate = 0.0f;
    res.numPosesTested = 0;
    res.numValidPoses = 0;
    res.numColPoses = 0;

    if (!g || !eef || !o || !qm)
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
    Eigen::Matrix4f objectPoseInit = o->getGlobalPose();
    VirtualRobot::RobotConfigPtr initialConf = eef->getConfiguration();

    std::string graspPreshapeName = g->getPreshapeName();
    VirtualRobot::RobotConfigPtr graspPS;
    if (eef->hasPreshape(graspPreshapeName))
        graspPS = eef->getPreshape(graspPreshapeName);

    Eigen::Matrix4f mGrasp = g->getTcpPoseGlobal(o->getGlobalPose());

    // apply grasp
    eef->getRobot()->setGlobalPoseForRobotNode(eef->getTcp(), mGrasp);

    if (graspPS)
    {
        eef->getRobot()->setJointValues(graspPS);
    } else
        eef->openActors();


    auto contacts = eef->closeActors(o);
    if(contacts.size() == 0)
    {
        VR_INFO << "No contacts for grasp " << g->getName() << " found" << std::endl;
        return res;
    }
    auto poses = generatePoses(o->getGlobalPose(), contacts, numPoses);
    if(poses.empty())
    {
        VR_INFO << "No poses for grasp found" << std::endl;
        return res;
    }
    res = evaluatePoses(eef, o, poses, qm, graspPS);

    // restore setup
    eef->getRobot()->setGlobalPose(eefRobotPoseInit);
    o->setGlobalPose(objectPoseInit);
    eef->getRobot()->setConfig(initialConf);

    return res;
}

}
