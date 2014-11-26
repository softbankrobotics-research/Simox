#ifndef _DYNAMICS_H_
#define _DYNAMICS_H__

#include "../VirtualRobotImportExport.h"

#include "../Nodes/RobotNode.h"
#include "../RobotNodeSet.h"
#include <rbdl/rbdl.h>

#include <boost/shared_ptr.hpp>


namespace VirtualRobot
{

    /** @brief
     * Encapsulates dynamics simulations for the virtual robot.
     */
    class Dynamics
    {
    public:
        /// Creates a Dynamics object given a RobotNodeSet
        Dynamics(RobotNodeSetPtr rns);
        /// Calculates the Inverse Dynamics for given motion state defined by q, qdot and qddot
        Eigen::VectorXd getInverseDynamics(Eigen::VectorXd q, Eigen::VectorXd qdot, Eigen::VectorXd qddot);
        /// Calculates the joint space inertia matrix given a joint position vector q
        Eigen::MatrixXd getInertiaMatrix(Eigen::VectorXd q);
        /// Sets the gravity vector of the dynamics system
        void setGravity(Eigen::Vector3d gravity);
        /// returns the number of Degrees of Freedom of the dynamics system
        int getnDoF();

    protected:
        RobotNodeSetPtr rns;
        boost::shared_ptr<RigidBodyDynamics::Model> model;
        Eigen::Vector3d gravity;

    private:
        void toRBDL(boost::shared_ptr<RigidBodyDynamics::Model> model, RobotNodePtr node, RobotNodePtr parentNode = RobotNodePtr(), int parentID = 0);
    };
}

#endif
