#include "Reachability.h"
#include "../VirtualRobotException.h"
#include "../Model/Model.h"
#include "../Model/ModelNodeSet.h"
#include "../Model/ManipulationObject.h"
#include "../Grasping/Grasp.h"
#include "../Grasping/GraspSet.h"
#include <fstream>
#include <cmath>
#include <float.h>
#include <limits.h>

namespace VirtualRobot
{



    Reachability::Reachability(RobotPtr robot) : WorkspaceRepresentation(robot)
    {
        type = "Reachability";
    }

    bool Reachability::isReachable(const Eigen::Matrix4f& globalPose)
    {
        return isCovered(globalPose);
    }

    VirtualRobot::GraspSetPtr Reachability::getReachableGrasps(GraspSetPtr grasps, ManipulationObjectPtr object)
    {
        THROW_VR_EXCEPTION_IF(!object, "no object");
        THROW_VR_EXCEPTION_IF(!grasps, "no grasps");

        GraspSetPtr result(new GraspSet(grasps->getName(), grasps->getRobotType(), grasps->getEndEffector()));

        for (unsigned int i = 0; i < grasps->getSize(); i++)
        {
            Eigen::Matrix4f m = grasps->getGrasp(i)->getTcpPoseGlobal(object->getGlobalPose());

            if (isReachable(m))
            {
                result->addGrasp(grasps->getGrasp(i));
            }
        }

        return result;
    }

    Eigen::Matrix4f Reachability::sampleReachablePose()
    {
        return sampleCoveredPose();
    }

    VirtualRobot::WorkspaceRepresentationPtr Reachability::clone()
    {
        VirtualRobot::ReachabilityPtr res(new Reachability(robot));
        res->setOrientationType(this->orientationType);
        res->versionMajor = this->versionMajor;
        res->versionMinor = this->versionMinor;
        res->nodeSet = this->nodeSet;
        res->type = this->type;

        res->baseNode = this->baseNode;
        res->tcpNode = this->tcpNode;
        res->staticCollisionModel = this->staticCollisionModel;
        res->dynamicCollisionModel = this->dynamicCollisionModel;
        res->buildUpLoops = this->buildUpLoops;
        res->collisionConfigs = this->collisionConfigs;
        res->discretizeStepTranslation = this->discretizeStepTranslation;
        res->discretizeStepRotation = this->discretizeStepRotation;
        memcpy(res->minBounds, this->minBounds, sizeof(float) * 6);
        memcpy(res->maxBounds, this->maxBounds, sizeof(float) * 6);
        memcpy(res->numVoxels, this->numVoxels, sizeof(float) * 6);
        memcpy(res->achievedMinValues, this->achievedMinValues, sizeof(float) * 6);
        memcpy(res->achievedMaxValues, this->achievedMaxValues, sizeof(float) * 6);
        memcpy(res->spaceSize, this->spaceSize, sizeof(float) * 6);

        res->adjustOnOverflow = this->adjustOnOverflow;
        res->data.reset(this->data->clone());

        return res;
    }

} // namespace VirtualRobot
