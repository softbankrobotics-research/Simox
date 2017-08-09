#include "JointLimitAvoidanceJacobi.h"
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Model/Model.h>
#include <VirtualRobot/Model/Nodes/ModelJoint.h>

using namespace VirtualRobot;
using namespace std;


namespace VirtualRobot
{


    JointLimitAvoidanceJacobi::JointLimitAvoidanceJacobi(JointSetPtr rns, JacobiProvider::InverseJacobiMethod invJacMethod)
        : JacobiProvider(rns, invJacMethod)
    {
        nodes = rns->getJoints();
        initialized = true; // no need of spiecial initialization
        VR_ASSERT(nodes.size() > 0);
    }


    Eigen::MatrixXf JointLimitAvoidanceJacobi::getJacobianMatrix()
    {
        size_t nDoF = nodes.size();
        Eigen::MatrixXf Jacobian(nDoF, nDoF);
        Jacobian.setIdentity();
        return Jacobian;
    }

    Eigen::VectorXf JointLimitAvoidanceJacobi::getError(float stepSize)
    {
        size_t nDoF = nodes.size();
        Eigen::VectorXf error(nDoF);

        for (size_t i = 0; i < nDoF; i++)
        {
            if (nodes[i]->getType() == ModelNode::ModelNodeType::JointRevolute)
            {
                float l = nodes[i]->getJointLimitHigh() - nodes[i]->getJointLimitLow();
                float target = nodes[i]->getJointLimitLow() + l * 0.5f;
                error(i) = (target - nodes[i]->getJointValue()) * stepSize;
            }
            else
            {
                error(i) = 0.0f;
            }
        }

        return error;
    }

    bool JointLimitAvoidanceJacobi::checkTolerances()
    {
        return false;
    }

    Eigen::MatrixXf JointLimitAvoidanceJacobi::getJacobianMatrix(FramePtr /*tcp*/)
    {
        return getJacobianMatrix();
    }


} // namespace VirtualRobot
