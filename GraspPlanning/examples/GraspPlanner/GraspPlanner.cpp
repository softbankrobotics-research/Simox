#include <VirtualRobot/Model/Model.h>
#include <VirtualRobot/Tools/RuntimeEnvironment.h>

#include <string>
#include <iostream>

using std::cout;
using std::endl;
using namespace VirtualRobot;

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "GraspPlannerWindow.h"


int main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "Simox Grasp Planner");
    cout << " --- START --- " << endl;
    cout << endl << "Hint: You can start this demo for different hands:" << endl;
    cout << "GraspPlanner --robot robots/iCub/iCub.xml --endeffector \"Left Hand\" --preshape \"Grasp Preshape\"" << endl;
    cout << "GraspPlanner --robot robots/Shadow_Dexterous_Hand/shadowhand.xml --endeffector \"SHADOWHAND\" --preshape \"Grasp Preshape\"" << endl;
    cout << "GraspPlanner --robot robots/Schunk_SAH/SAH_RightHand.xml --endeffector \"Right Hand\" --preshape \"Grasp Preshape\"" << endl;

    // --robot robots/iCub/iCub.xml --endeffector "Left Hand" --preshape "Grasp Preshape"


    VirtualRobot::RuntimeEnvironment::considerKey("robot");
    VirtualRobot::RuntimeEnvironment::considerKey("object");
    VirtualRobot::RuntimeEnvironment::considerKey("endeffector");
    VirtualRobot::RuntimeEnvironment::considerKey("preshape");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    std::string objFile = VirtualRobot::RuntimeEnvironment::checkValidFileParameter("object", "objects/iv/WaterBottleSmall.xml");
    cout << "Object file: " << objFile << endl;

    std::string robFile = VirtualRobot::RuntimeEnvironment::checkValidFileParameter("robot", "robots/Armar3/Armar3.xml");
    cout << "Robot file: " << robFile << endl;

    std::string eef("Hand R");
    std::string preshape("");

    std::string eefname = VirtualRobot::RuntimeEnvironment::getValue("endeffector");

    if (!eefname.empty())
    {
        eef = eefname;
    }

    std::string ps = VirtualRobot::RuntimeEnvironment::getValue("preshape");

    if (!ps.empty())
    {
        preshape = ps;
    }

    cout << "End effector:" << eef << ", preshape:" << preshape << endl;


    GraspPlannerWindow rw(robFile, eef, preshape, objFile);

    rw.main();

    return 0;
}
