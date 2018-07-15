#include "CoMIK.h"
#include "DifferentialIK.h"
#include "../Model/Nodes/ModelJointRevolute.h"
#include "../Model/Nodes/ModelJointPrismatic.h"
#include "../VirtualRobotException.h"
#include "../Model/Model.h"
#include "../Model/Nodes/ModelLink.h"
#include "../Model/LinkSet.h"
#include "../Tools/MathTools.h"

#include <float.h>

namespace VirtualRobot
{
    CoMIK::CoMIK(const JointSetPtr &rnsJoints, const LinkSetPtr &rnsBodies, const FramePtr &coordSystem, int dimensions)
        : JacobiProvider(rnsJoints), coordSystem(coordSystem)
    {
        VR_ASSERT(rns);
        VR_ASSERT(rnsBodies);
        convertMMtoM = false;
        name = "CoMIK";
        checkImprovement = false;
        this->rnsBodies = rnsBodies;
        numDimensions = dimensions;

        bodyNodes = rnsBodies->getLinks();

        for (size_t i = 0; i < bodyNodes.size(); i++)
        {
            // get all joints that influence the body
            std::vector<RobotNodePtr> parentsN = bodyNodes[i]->getAllParents(rns);
            // maybe this node is joint and body
            if (rnsJoints->hasNode(bodyNodes[i]->getName()))
                parentsN.push_back(bodyNodes[i]);
            bodyNodeParents[bodyNodes[i]] = parentsN;
        }

        if (rnsBodies->getMass() == 0)
        {
            VR_ERROR << "The RNS does not contain any bodies or masses are not specified (mass==0)" << endl;
        }
    }

    void CoMIK::setGoal(const Eigen::VectorXf& goal, float tolerance)
    {
        target = goal;
        this->tolerance = tolerance;
        initialized = true;
    }

    Eigen::MatrixXf CoMIK::getJacobianOfCoM(const ModelLinkPtr &node)
    {
        // Get number of degrees of freedom
        size_t nDoF = rns->getJoints().size();

        // Create matrices for the position and the orientation part of the jacobian.
        Eigen::MatrixXf position = Eigen::MatrixXf::Zero(3, nDoF);

        const std::vector<ModelNodePtr> parentsN = bodyNodeParents[node];

        // Iterate over all degrees of freedom
        for (size_t i = 0; i < nDoF; i++)
        {
            RobotNodePtr dof = rns->getNode((int)i);// bodyNodes[i];

            // Check if the tcp is affected by this DOF
            if (find(parentsN.begin(), parentsN.end(), dof) != parentsN.end())
            {
                // Calculus for rotational joints is different as for prismatic joints.
                if (dof->getType() == ModelNode::JointRevolute)
                {
                    // get axis
                    std::shared_ptr<ModelJointRevolute> revolute
                        = std::dynamic_pointer_cast<ModelJointRevolute>(dof);
                    THROW_VR_EXCEPTION_IF(!revolute, "Internal error: expecting revolute joint");
                    // todo: find a better way of handling different joint types
                    Eigen::Vector3f axis = revolute->getJointRotationAxis(coordSystem);

                    // For CoM-Jacobians only the positional part is necessary
                    //Eigen::Vector3f toTCP = node->getCoMLocal() + node->getGlobalPose().block(0, 3, 3, 1)
                    //                        - dof->getGlobalPose().block(0, 3, 3, 1);
                    Eigen::Vector3f toTCP = node->getCoMGlobal()
                                            - dof->getGlobalPose().block(0, 3, 3, 1);

                    if (convertMMtoM)
                    {
                        toTCP /= 1000.0f;
                    }

                    position.block(0, i, 3, 1) = axis.cross(toTCP);
                }
                else if (dof->getType() == ModelNode::JointPrismatic)
                {
                    // -> prismatic joint
                    std::shared_ptr<ModelJointPrismatic> prismatic
                        = std::dynamic_pointer_cast<ModelJointPrismatic>(dof);
                    THROW_VR_EXCEPTION_IF(!prismatic, "Internal error: expecting prismatic joint");
                    // todo: find a better way of handling different joint types
                    Eigen::Vector3f axis = prismatic->getJointTranslationDirection(coordSystem);

                    position.block(0, i, 3, 1) = axis;
                }
            }
        }

        if (target.rows() == 2)
        {
            Eigen::MatrixXf result(2, nDoF);
            result.row(0) = position.row(0);
            result.row(1) = position.row(1);
            return result;
        }
        else if (target.rows() == 1)
        {
            VR_INFO << "One dimensional CoMs not supported." << endl;
        }

        return position;
    }

    void CoMIK::convertModelScalingtoM(bool enable)
    {
        convertMMtoM = enable;
    }


    Eigen::MatrixXf CoMIK::getJacobianMatrix(const FramePtr &/*tcp*/)
    {
        // ignoring tcp
        return getJacobianMatrix();
    }

    Eigen::MatrixXf CoMIK::getJacobianMatrix()
    {
        Eigen::MatrixXf Jsum(0, 0);

        for (std::vector<ModelLinkPtr>::const_iterator n = bodyNodes.begin(); n != bodyNodes.end(); n++)
        {
            // Retrieve (positional) Jacobian for this node's CoM
            // Depeding on the set target, the Jacobian is already projected to the XY-plane
            Eigen::MatrixXf J = getJacobianOfCoM(*n);

            // Sum up the Jacobians weighted by the respective node masses
            if (Jsum.rows() == 0)
            {
                Jsum = (*n)->getMass() * J;
            }
            else
            {
                Jsum += (*n)->getMass() * J;
            }
        }

        if (rnsBodies->getMass() != 0)
        {
            Jsum /= rnsBodies->getMass();
        }

        return Jsum;
    }

    Eigen::VectorXf CoMIK::getError(float /*stepSize*/)
    {
        return (target - rnsBodies->getCoM().head(target.rows()));
    }

    Eigen::VectorXf CoMIK::computeStep(float stepSize)
    {
        Eigen::VectorXf error = (target - rnsBodies->getCoM().head(target.rows())) * stepSize;
        return getPseudoInverseJacobianMatrix() * error;
    }

    bool CoMIK::checkTolerances()
    {
        float error = (target - rnsBodies->getCoM().head(target.rows())).norm();
        return error < tolerance;
    }

    void CoMIK::checkImprovements(bool enable)
    {
        checkImprovement = enable;
    }

    bool CoMIK::isValid(const Eigen::VectorXf& v) const
    {
        return MathTools::isValid(v);
    }

    bool CoMIK::computeSteps(float stepSize, float minumChange, int maxNStep)
    {
        std::vector<ModelJointPtr> rn = rns->getJoints();
        ModelPtr robot = rns->getModel();
        std::vector<float> jv(rns->getSize(), 0.0f);
        int step = 0;
        checkTolerances();
        float lastDist = FLT_MAX;

        while (step < maxNStep)
        {
            Eigen::VectorXf dTheta = this->computeStep(stepSize);

            // Check for singularities
            if (!isValid(dTheta))
            {
                VR_INFO << "Singular Jacobian" << endl;
                return false;
            }

            for (unsigned int i = 0; i < rn.size(); i++)
            {
                jv[i] = (rn[i]->getJointValue() + dTheta[i]);
            }

            rns->setJointValues(jv);

            // check tolerances
            if (checkTolerances())
            {
                VR_INFO << "Tolerances ok, loop:" << step << endl;
                return true;
            }

            float d = dTheta.norm();

            if (dTheta.norm() < minumChange)
            {
                VR_INFO << "Could not improve result any more (dTheta.norm()=" << d << "), loop:" << step << endl;
                return false;
            }

            if (checkImprovement && d > lastDist)
            {
                VR_INFO << "Could not improve result any more (dTheta.norm()=" << d << ", last loop's norm:" << lastDist << "), loop:" << step << endl;
                return false;
            }

            lastDist = d;
            step++;
        }

#ifndef NO_FAILURE_OUTPUT
        //VR_INFO << "IK failed, loop:" << step << endl;
        //VR_INFO << "error:" << (target - rns->getCoM().head(target.rows())).norm() << endl;
#endif
        return false;
    }

    bool CoMIK::solveIK(float stepSize, float minChange, int maxSteps)
    {
        return computeSteps(stepSize, minChange, maxSteps);
    }

    void CoMIK::print()
    {
        JacobiProvider::print();

        if (coordSystem)
        {
            cout << "Coordsystem: " << coordSystem->getName() << endl;
        }
        else
        {
            cout << "Coordsystem: global" << endl;
        }

        cout << "Target:" << endl << this->target.transpose() << endl;
        cout << "Tolerances pos: " << this->tolerance << endl;
        cout << "RNS bodies:" << rnsBodies->getName() << endl;
        cout << "CoM:" << rnsBodies->getCoM().head(target.rows()).transpose() << endl;
        cout << "Error:" << endl << getError().transpose() << endl;

    }
}
