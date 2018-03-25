
#include <SimDynamics/DynamicsEngine/BulletEngine/BulletOpenGLViewer.h>
#include <SimDynamics/DynamicsWorld.h>
#include <SimDynamics/DynamicsEngine/BulletEngine/BulletEngineFactory.h>

#include <VirtualRobot/Model/Obstacle.h>
#include <VirtualRobot/XML/ModelIO.h>
#include <VirtualRobot/XML/ObjectIO.h>
#include <VirtualRobot/Tools/RuntimeEnvironment.h>

using namespace std;
using namespace VirtualRobot;
using namespace SimDynamics;


int main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "Bullet Debug Viewer");

    VirtualRobot::RuntimeEnvironment::considerKey("robot");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    VR_INFO << " --- START --- " << endl;
    std::string robFile = VirtualRobot::RuntimeEnvironment::checkValidFileParameter("robot", "robots/Armar3/Armar3.xml");
    VR_INFO << "Using robot file " << robFile << endl;

    SimDynamics::DynamicsWorldPtr world = SimDynamics::DynamicsWorld::Init();
    SIMDYNAMICS_ASSERT(world);

    world->createFloorPlane();

    VirtualRobot::ObstaclePtr o = VirtualRobot::Obstacle::createBox(100.0f, 100.0f, 100.0f);
    o->getFirstLink()->setSimulationType(ModelLink::Physics::eDynamic);
    Eigen::Matrix4f gp = Eigen::Matrix4f::Identity();
    gp.block(0, 3, 3, 1) = Eigen::Vector3f(2800, 10400, 5000.0f);
    o->setGlobalPose(gp);
    o->setMass(1.0f); // 1kg

    SimDynamics::DynamicsModelPtr dynObj = world->CreateDynamicsModel(o);
    world->addModel(dynObj);
    o->print();

    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFile);
    VirtualRobot::RobotPtr robot = VirtualRobot::ModelIO::loadModel(robFile);

    if (robot)
    {
        DynamicsModelPtr dynRob = world->CreateDynamicsModel(robot);
        ActuationMode mode;
        mode.modes.position = 1;
        dynRob->enableActuation(mode);
        world->addModel(dynRob);
    }

    BulletOpenGLViewer viewer(world);
    //viewer.enableContraintsDebugDrawing();

#if 0
    cout << "TEST7" << endl;
    ObstaclePtr o = Obstacle::createBox(10, 10, 1500);
    DynamicsObjectPtr do1 = DynamicsWorld::GetWorld()->CreateDynamicsObject(o, DynamicsObject::eStatic);
    ObstaclePtr o2 = Obstacle::createBox(10, 10, 1000);
    Eigen::Matrix4f gpxy = Eigen::Matrix4f::Identity();
    //gpxy(1,3) -= 213.0f;
    gpxy(0, 3) += 3000.0f;
    o2->setGlobalPose(gpxy);
    DynamicsObjectPtr do2 = DynamicsWorld::GetWorld()->CreateDynamicsObject(o2, DynamicsObject::eStatic);
    DynamicsEnginePtr e = DynamicsWorld::GetWorld()->getEngine();
    e->disableCollision(do1.get());
    e->disableCollision(do2.get());
    /*
    std::vector<DynamicsObjectPtr> dos = e->getObjects();
    for (size_t i=0;i<dos.size();i++)
    {
        e->disableCollision(do1.get(),dos[i].get());
        e->disableCollision(do2.get(),dos[i].get());
        if (e->checkCollisionEnabled(do1.get(),dos[i].get()))
        {
            cout << "OOPS" << endl;
        }
        if (e->checkCollisionEnabled(do2.get(),dos[i].get()))
        {
            cout << "OOPS" << endl;
        }
    }*/
    e->addObject(do1);
    e->addObject(do2);
#endif

    return glutmain(argc, argv, 640, 480, "Show SimDynamics Scene", &viewer);
}
