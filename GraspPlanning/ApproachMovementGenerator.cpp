#include "ApproachMovementGenerator.h"
#include <VirtualRobot/Model/Model.h>
#include <VirtualRobot/Model/Nodes/ModelLink.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <VirtualRobot/Model/Nodes/ModelNode.h>
#include <VirtualRobot/EndEffector/EndEffector.h>

#include <VirtualRobot/Tools/MathTools.h>
#include <VirtualRobot/CollisionDetection/CollisionModel.h>
#include <VirtualRobot/Model/ModelConfig.h>

#include <iostream>
using namespace std;

namespace GraspPlanning
{

    ApproachMovementGenerator::ApproachMovementGenerator(VirtualRobot::ModelPtr obj, VirtualRobot::EndEffectorPtr endef, std::string graspPres)
        : object{std::move(obj)}, eef{std::move(endef)}, graspPreshape{std::move(graspPres)}
    {
        name = "ApproachMovementGenerator";
        THROW_VR_EXCEPTION_IF(!object, "NULL object?!");
        THROW_VR_EXCEPTION_IF(object->getLinks().size()!=1 || !object->getLinks().at(0)->getCollisionModel(), "Number of collision models != 1 or no collision model for object " << object->getName());
        THROW_VR_EXCEPTION_IF(!eef, "NULL eef?!");
        THROW_VR_EXCEPTION_IF(!eef->getGCP(), "Need a GraspCenterPoint Node defined in EEF " << eef->getName());

        objectModel = object->getLinks().at(0)->getCollisionModel()->getTriMeshModel();
        THROW_VR_EXCEPTION_IF(!objectModel, "NULL trimeshmodel of object " << object->getName());
        THROW_VR_EXCEPTION_IF(objectModel->faces.size() == 0, "no faces in trimeshmodel of object " << object->getName());

        eefRobot = eef->createEefRobot(eef->getName(), eef->getName());
        THROW_VR_EXCEPTION_IF(!eefRobot, "Failed cloning EEF " << eef->getName());


        eef_cloned = eefRobot->getEndEffector(eef->getName());
        THROW_VR_EXCEPTION_IF(!eef_cloned, "No EEF with name " << eef->getName() << " in cloned robot?!");
        THROW_VR_EXCEPTION_IF(!eef_cloned->getGCP(), "No GCP in EEF with name " << eef->getName());

        if (!graspPreshape.empty())
        {
            THROW_VR_EXCEPTION_IF(!eef_cloned->hasPreshape(graspPreshape), "Preshape with name " << graspPreshape << " not present in EEF");
            eef_cloned->setPreshape(graspPreshape);
        }
        approachDirGlobal << 1.0f, 0, 0;
    }

    ApproachMovementGenerator::~ApproachMovementGenerator()
    {
    }


    const VirtualRobot::RobotPtr& ApproachMovementGenerator::getEEFRobotClone()
    {
        return eefRobot;
    }

    bool ApproachMovementGenerator::setEEFPose(const Eigen::Matrix4f& pose)
    {
        //eefRobot->setGlobalPoseForModelNode(eef_cloned->getGCP(), pose);
        VirtualRobot::FramePtr tcp;
        if (!graspPreshape.empty() && eef_cloned->hasPreshape(graspPreshape) && eef_cloned->getPreshape(graspPreshape)->getTCP())
            tcp = eef_cloned->getPreshape(graspPreshape)->getTCP();
        else
            tcp = eef_cloned->getGCP();
        eefRobot->setGlobalPoseForModelNode(tcp, pose);
        return true;
    }

    bool ApproachMovementGenerator::updateEEFPose(const Eigen::Vector3f& deltaPosition)
    {
        Eigen::Matrix4f deltaPose;
        deltaPose.setIdentity();
        deltaPose.block(0, 3, 3, 1) = deltaPosition;
        return updateEEFPose(deltaPose);
    }

    bool ApproachMovementGenerator::updateEEFPose(const Eigen::Matrix4f& deltaPose)
    {
        Eigen::Matrix4f pose = eef_cloned->getGCP()->getGlobalPose();
        pose = deltaPose * pose;
        return setEEFPose(pose);
    }

    Eigen::Matrix4f ApproachMovementGenerator::getEEFPose()
    {
        VirtualRobot::FramePtr tcp;
        if (!graspPreshape.empty() && eef_cloned->hasPreshape(graspPreshape) && eef_cloned->getPreshape(graspPreshape)->getTCP())
            tcp = eef_cloned->getPreshape(graspPreshape)->getTCP();
        else
            tcp = eef_cloned->getGCP();
        return tcp->getGlobalPose();
    }

    bool ApproachMovementGenerator::setEEFToRandomApproachPose()
    {
        Eigen::Matrix4f pose = createNewApproachPose();
        return setEEFPose(pose);
    }

    std::string ApproachMovementGenerator::getGCPJoint()
    {
        return eef_cloned->getGCP()->getName();
    }

    const VirtualRobot::ModelPtr& ApproachMovementGenerator::getObject()
    {
        return object;
    }

    const VirtualRobot::EndEffectorPtr& ApproachMovementGenerator::getEEF()
    {
        return eef_cloned;
    }

    const VirtualRobot::EndEffectorPtr& ApproachMovementGenerator::getEEFOriginal()
    {
        return eef;
    }

    const Eigen::Vector3f& ApproachMovementGenerator::getApproachDirGlobal()
    {
        return approachDirGlobal;
    }

    const std::string& ApproachMovementGenerator::getName()
    {
        return name;
    }


    void ApproachMovementGenerator::openHand()
    {
        if (eef_cloned)
        {
            if (!graspPreshape.empty())
            {
                eef_cloned->setPreshape(graspPreshape);
            }
            else
            {
                eef_cloned->openActors();
            }
        }
    }

}
