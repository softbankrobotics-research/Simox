#include <VirtualRobot/Dynamics/Dynamics.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include <chrono>
#include <VirtualRobot/Tools/Gravity.h>
#include <rbdl/Kinematics.h>
using std::cout;
using namespace VirtualRobot;




int main(int argc, char* argv[])
{
    std::string filename("robots/ArmarIII/ArmarIII.xml");
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filename);
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    cout << "Using robot at " << filename << endl;
    RobotPtr rob;

    try
    {
        rob = RobotIO::loadRobot(filename, RobotIO::eStructure);
    }
    catch (VirtualRobotException& e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }

    if (rob)
    {

        RobotNodeSetPtr ns = rob->getRobotNodeSet("RightArm");
        RobotNodeSetPtr bodyNs = rob->getRobotNodeSet("RightArmHandColModel");
//        RobotNodeSetPtr bodyNs = rob->getRobotNodeSet("RightArmCol");

        Gravity g(rob, ns, bodyNs);
        for(auto& pair:g.getMasses())
        {
            cout << pair.first <<": " << pair.second << endl;
        }
        VirtualRobot::Dynamics dynamics = VirtualRobot::Dynamics(ns, bodyNs, true);
        dynamics.print();
        int nDof = dynamics.getnDoF();
        Eigen::VectorXd q = Eigen::VectorXd::Zero(nDof);
        ns->setJointValues(q.cast<float>());
        q = ns->getJointValuesEigen().cast<double>(); // get joint values with joint limits applied
        Eigen::VectorXd qdot = Eigen::VectorXd::Random(nDof);
        Eigen::VectorXd qddot = Eigen::VectorXd::Random(nDof);
        Eigen::VectorXd tau = Eigen::VectorXd::Random(nDof);

        auto start = std::chrono::system_clock::now();
        int c = 1000;
        for (int i = 0; i < c; ++i) {
            Eigen::VectorXd q = Eigen::VectorXd::Random(nDof);
            Eigen::VectorXd qdot = Eigen::VectorXd::Random(nDof);
            Eigen::VectorXd qddot = Eigen::VectorXd::Random(nDof);
            Eigen::VectorXd tau = Eigen::VectorXd::Random(nDof);
            Eigen::VectorXd invDyn = dynamics.getInverseDynamics(q, qdot, qddot);            
        }
        auto end = std::chrono::system_clock::now();
        auto elapsed =
            std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        std::cout << "duration:" << (elapsed.count()/c) << '\n';

        for (int i = 0; i < c; ++i) {
//            break;
            Eigen::VectorXd q = Eigen::VectorXd::Random(nDof);
            Eigen::VectorXd qdot = Eigen::VectorXd::Random(nDof);
            Eigen::VectorXd qddot = Eigen::VectorXd::Random(nDof);
            Eigen::VectorXd tau = Eigen::VectorXd::Random(nDof);
            ns->setJointValues(q.cast<float>());
            q = ns->getJointValuesEigen().cast<double>(); // get joint values with joint limits applied
            Eigen::VectorXd gravityRBDL = dynamics.getGravityMatrix(q);
            int d=0;
            std::vector<float> gravityVR;
            g.computeGravityTorque(gravityVR);
            for(auto & val: gravityVR)
            {
                auto diff = val- gravityRBDL(d);
                if(std::abs(diff)> 0.01)
                    throw std::runtime_error((std::to_string(i) + " dim: " + std::to_string(d) + " diff: " + std::to_string(diff).c_str()));
                d++;
            }

        }

        Eigen::VectorXd invDyn = dynamics.getInverseDynamics(q, qdot, qddot);
        cout << "Joint values:\n" << q << endl;
        ns->setJointValues(q.cast<float>());
        cout << "Joint values in VR:\n" << q << endl;
        std::vector<float> gravityVR;
        g.computeGravityTorque(gravityVR);

//        cout << "joint torques from inverse dynamics: " << endl << invDyn << endl;
        cout << "joint space inertia matrix: " << endl << dynamics.getInertiaMatrix(q) << endl;
        cout << "joint space gravitational matrix:" << endl << dynamics.getGravityMatrix(q) << endl;
        cout << "joint space VR gravity :" << endl;
        int i=0;
        for(auto & val: gravityVR)
        {
            cout << ns->getNode(i)->getName() << ": " << val << endl;
            i++;
        }
        cout << "joint space coriolis matrix:" << endl << dynamics.getCoriolisMatrix(q, qdot) << endl;
        cout << "joint space accelerations from forward dynamics:" << endl << dynamics.getForwardDynamics(q, qdot, tau) << endl;
//        cout << "Identifier for Elbow R:" << endl << dynamics.getIdentifier("Elbow R") << endl;

    }
    else
    {
        cout << " ERROR while creating robobt" << endl;
    }
}



