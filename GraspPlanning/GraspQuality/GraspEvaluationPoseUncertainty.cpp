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

    for (int j=0;j<numPoses; j++)
    {
        for (int i = 0; i < 6; i++)
        {
            float r = float(rand()) / float(RAND_MAX);
            tmpPose[i] = start[i] + r*dist[i];
        }
        MathTools::posrpy2eigen4f(tmpPose, m);
        m = m * trafoGraspCenterToObjectCenter;
        result.push_back(m);
    }
    return result;
}

}
