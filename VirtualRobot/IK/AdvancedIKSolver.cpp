#include <Eigen/Geometry>
#include "AdvancedIKSolver.h"
#include "../Model/Model.h"
#include "../VirtualRobotException.h"
#include "../VirtualRobotException.h"
#include "../Model/Obstacle.h"
#include "../Model/ManipulationObject.h"
#include "../Grasping/Grasp.h"
#include "../Grasping/GraspSet.h"
#include "../Workspace/Reachability.h"
#include "../EndEffector/EndEffector.h"
#include "../Model/ModelConfig.h"
#include "../CollisionDetection/CollisionChecker.h"
#include "../CollisionDetection/CDManager.h"

#include <algorithm>

using namespace Eigen;

namespace VirtualRobot
{

    AdvancedIKSolver::AdvancedIKSolver(const JointSetPtr &rns, const LinkSetPtr &collisionModels) :
        IKSolver(rns), colSet(collisionModels)
    {
        THROW_VR_EXCEPTION_IF(!rns, "Null data");
        THROW_VR_EXCEPTION_IF(!tcp, "no tcp");
        setMaximumError();
    }

    void AdvancedIKSolver::collisionDetectionModelLinks(const LinkSetPtr &collisionModels)
	{
		colSet = collisionModels;
	}

    void AdvancedIKSolver::collisionDetection(const ModelPtr &avoidCollisionsWith)
    {
        cdm.reset();

        if (avoidCollisionsWith && colSet)
        {
            cdm.reset(new CDManager(avoidCollisionsWith->getCollisionChecker()));
            cdm->addCollisionModel(avoidCollisionsWith->getLinkSet());
            cdm->addCollisionModel(colSet);
        }
    }

    void AdvancedIKSolver::collisionDetection(const ObstaclePtr &avoidCollisionsWith)
    {
		ModelPtr so = std::dynamic_pointer_cast<Model>(avoidCollisionsWith);
        collisionDetection(so);
    }


    void AdvancedIKSolver::collisionDetection(const ModelLinkPtr &avoidCollisionsWith)
	{
		cdm.reset();

		if (avoidCollisionsWith && colSet)
		{
			cdm.reset(new CDManager(avoidCollisionsWith->getCollisionChecker()));
			cdm->addCollisionModel(avoidCollisionsWith);
			cdm->addCollisionModel(colSet);
		}
	}

    void AdvancedIKSolver::collisionDetection(const LinkSetPtr &avoidCollisionsWith)
    {
        cdm.reset();

        if (avoidCollisionsWith && colSet)
        {
            cdm.reset(new CDManager(avoidCollisionsWith->getCollisionChecker()));
            cdm->addCollisionModel(avoidCollisionsWith);
            cdm->addCollisionModel(colSet);
        }
    }

    void AdvancedIKSolver::collisionDetection(const CDManagerPtr &avoidCollisions)
    {
        cdm = avoidCollisions;
    }

    std::vector<float> AdvancedIKSolver::solveNoRNSUpdate(const Eigen::Matrix4f& globalPose, CartesianSelection selection)
    {
        std::vector<float> result;
        std::vector<float> v;
        rns->getJointValues(v);

        if (solve(globalPose, selection))
        {
            rns->getJointValues(result);
        }

		rns->setJointValues(v);
        return result;
    }

    bool AdvancedIKSolver::solve(const Eigen::Vector3f& globalPosition)
    {
        Eigen::Matrix4f t;
        t.setIdentity();
        t.block(0, 3, 3, 1) = globalPosition;
        return solve(t, Position);
    }

    GraspPtr AdvancedIKSolver::solve(const ManipulationObjectPtr &object, CartesianSelection selection /*= All*/, int maxLoops)
    {
        THROW_VR_EXCEPTION_IF(!object, "NULL object");
        // first get a compatible EEF
        RobotPtr robot = rns->getModel();
        THROW_VR_EXCEPTION_IF(!robot, "NULL robot");
        std::vector< EndEffectorPtr > eefs = robot->getEndEffectors();
        EndEffectorPtr eef;

        for (size_t i = 0; i < eefs.size(); i++)
        {
            if (eefs[i]->getTcp() == rns->getTCP())
            {
                if (eef)
                {
                    VR_ERROR << " Two end effectors with tcp " << rns->getTCP()->getName() << " defined in robot " << robot->getName() << ". taking the first one?!" << endl;
                }
                else
                {
                    eef = eefs[i];
                }
            }
        }

        if (!eef)
        {
            VR_ERROR << " No end effector with tcp " << rns->getTCP()->getName() << " defined in robot " << robot->getName() << ". Aborting..." << endl;
            return GraspPtr();
        }

        GraspSetPtr gs = object->getGraspSet(eef)->clone();

        if (!gs || gs->getSize() == 0)
        {
            VR_ERROR << " No grasps defined for eef " << eef->getName() << " defined in object " << object->getName() << ". Aborting..." << endl;
            return GraspPtr();
        }

        bool updateStatus = robot->getUpdateVisualization();
        robot->setUpdateVisualization(false);

        // check all grasps if there is an IK solution
        while (gs->getSize() > 0)
        {
            GraspPtr g = sampleSolution(object, gs, selection, true, maxLoops);

            if (g)
            {
                robot->setUpdateVisualization(updateStatus);
                robot->applyJointValues();
                return g;
            }
        }

        robot->setUpdateVisualization(updateStatus);

        // when here, no grasp was successful
        return GraspPtr();
    }

    bool AdvancedIKSolver::solve(const ManipulationObjectPtr &object, GraspPtr grasp, CartesianSelection selection /*= All*/, int maxLoops)
    {
        THROW_VR_EXCEPTION_IF(!object, "NULL object");
        THROW_VR_EXCEPTION_IF(!grasp, "NULL grasp");
        RobotPtr robot = rns->getModel();
        THROW_VR_EXCEPTION_IF(!robot, "NULL robot");
        bool updateStatus = robot->getUpdateVisualization();
        robot->setUpdateVisualization(false);

        std::vector<float> v;
        rns->getJointValues(v);
        Eigen::Matrix4f m = grasp->getTcpPoseGlobal(object->getGlobalPose());

        if (_sampleSolution(m, selection, maxLoops))
        {
            robot->setUpdateVisualization(updateStatus);
            robot->applyJointValues();
            return true;
        }

        rns->setJointValues(v);
        robot->setUpdateVisualization(updateStatus);
        return false;
    }


    GraspPtr AdvancedIKSolver::sampleSolution(const ManipulationObjectPtr &object, const GraspSetPtr &graspSet, CartesianSelection selection /*= All*/, bool removeGraspFromSet, int maxLoops)
    {
        THROW_VR_EXCEPTION_IF(!object, "NULL object");
        //THROW_VR_EXCEPTION_IF(!eef,"NULL eef");
        THROW_VR_EXCEPTION_IF(!graspSet, "NULL graspSet");

        if (graspSet->getSize() == 0)
        {
            return GraspPtr();
        }

        std::vector<float> v;
        rns->getJointValues(v);

        int pos = rand() % graspSet->getSize();
        GraspPtr g = graspSet->getGrasp(pos);
        Eigen::Matrix4f m = g->getTcpPoseGlobal(object->getGlobalPose());

#if 0

        // assuming that reachability is checked in the solve() method, so we don't have ot check it here (->avoid double checks)
        if (checkReachable(m))
#endif
            if (_sampleSolution(m, selection, maxLoops))
            {
                return g;
            }

        // did not succeed, reset joint values and remove grasp from temporary set
        rns->setJointValues(v);

        if (removeGraspFromSet)
        {
            graspSet->removeGrasp(g);
        }

        return GraspPtr();
    }

    void AdvancedIKSolver::setMaximumError(float maxErrorPositionMM /*= 1.0f*/, float maxErrorOrientationRad /*= 0.02*/)
    {
        this->maxErrorPositionMM = maxErrorPositionMM;
        this->maxErrorOrientationRad = maxErrorOrientationRad;
    }

    void AdvancedIKSolver::setReachabilityCheck(const ReachabilityPtr &reachabilitySpace)
    {
        this->reachabilitySpace = reachabilitySpace;

        if (reachabilitySpace)
        {
            if (reachabilitySpace->getTCP() != tcp)
            {
                VR_ERROR << "Reachability representation has different tcp RobotNode (" << reachabilitySpace->getTCP()->getName() << ") than IK solver (" << tcp->getName() << ") ?! " << endl << "Reachability results may not be valid!" << endl;
            }

            if (reachabilitySpace->getJointSet() != rns)
            {
                VR_ERROR << "Reachability representation is defined for a different RobotNodeSet (" << reachabilitySpace->getJointSet()->getName() << ") than IK solver uses (" << rns->getName() << ") ?! " << endl << "Reachability results may not be valid!" << endl;
            }
        }
    }

    bool AdvancedIKSolver::checkReachable(const Eigen::Matrix4f& globalPose)
    {
        if (!reachabilitySpace)
        {
            return true;
        }

        return reachabilitySpace->isReachable(globalPose);
    }

    LinkSetPtr AdvancedIKSolver::getLinkSet() const
    {
        return colSet;
    }

} // namespace VirtualRobot
