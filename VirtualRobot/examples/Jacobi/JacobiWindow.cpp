
#include "JacobiWindow.h"
#include "VirtualRobot/XML/ModelIO.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/IK/DifferentialIK.h"
#include "Gui/ViewerFactory.h"

#include <time.h>
#include <vector>
#include <iostream>
#include <sstream>

using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 30.0f;

// load static factories from SimoxGui-lib.
// TODO this workaround is actually something one should avoid
#ifdef Simox_USE_COIN_VISUALIZATION
    #include <Gui/Coin/CoinViewerFactory.h>
    SimoxGui::CoinViewerFactory f;
#endif

JacobiWindow::JacobiWindow(std::string& sRobotFilename)
    : QMainWindow(NULL), boxVisuLayer("box-layer")
{
    VR_INFO << " start " << endl;
    //this->setCaption(QString("ShowRobot - KIT - Humanoids Group"));
    //resize(1100, 768);

    useColModel = false;
    robotFilename = sRobotFilename;
    setupUI();

    loadRobot();

    box = Obstacle::createBox(30.0f, 30.0f, 30.0f);
    viewer->addVisualization(boxVisuLayer, "box", box->getVisualization());

    box2 = Obstacle::createBox(30.0f, 30.0f, 30.0f);
    viewer->addVisualization(boxVisuLayer, "box2", box2->getVisualization());

    box3 = Obstacle::createBox(30.0f, 30.0f, 30.0f);
    viewer->addVisualization(boxVisuLayer, "box3", box3->getVisualization());

    box2TCP();

    viewer->viewAll();
}


JacobiWindow::~JacobiWindow()
{
}

void JacobiWindow::setupUI()
{
    UI.setupUi(this);
    viewer = SimoxGui::ViewerFactory::fromName(VisualizationFactory::getGlobalVisualizationFactory()->getVisualizationType(), NULL)->createViewer(UI.frameViewer);
    viewer->viewAll();

    connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
    connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(loadRobot()));

    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(collisionModel()));
    connect(UI.comboBoxKC, SIGNAL(activated(int)), this, SLOT(selectKC(int)));

    connect(UI.pushButtonBox2TCP, SIGNAL(clicked()), this, SLOT(box2TCP()));
    connect(UI.pushButtonJacobiTest, SIGNAL(clicked()), this, SLOT(jacobiTest()));
    connect(UI.pushButtonJacobiTest_2, SIGNAL(clicked()), this, SLOT(jacobiTestBi()));
    connect(UI.pushButtonJacobiTest_3, SIGNAL(clicked()), this, SLOT(jacobiTest2()));


    connect(UI.horizontalSliderX, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderY, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderZ, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderA, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderB, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderG, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));

    connect(UI.horizontalSliderX_2, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderY_2, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderZ_2, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderA_2, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderB_2, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderG_2, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));

    connect(UI.horizontalSliderX_3, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderY_3, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderZ_3, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderA_3, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderB_3, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderG_3, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateCB()));
    timer->start(30.0f);
}

void JacobiWindow::updateCB()
{
    float x = UI.horizontalSliderX->value();
    float y = UI.horizontalSliderY->value();
    float z = UI.horizontalSliderZ->value();
    float a = UI.horizontalSliderA->value();
    float b = UI.horizontalSliderB->value();
    float g = UI.horizontalSliderG->value();

    if (x != 0 || y != 0 || z != 0 || a != 0 || b != 0 || g != 0)
    {
        updatBoxPos(x / 100.0f, y / 100.0f, z / 100.0f, a / 2000.0f, b / 2000.0f, g / 2000.0f);
    }

    x = UI.horizontalSliderX_2->value();
    y = UI.horizontalSliderY_2->value();
    z = UI.horizontalSliderZ_2->value();
    a = UI.horizontalSliderA_2->value();
    b = UI.horizontalSliderB_2->value();
    g = UI.horizontalSliderG_2->value();

    if (x != 0 || y != 0 || z != 0 || a != 0 || b != 0 || g != 0)
    {
        updatBox2Pos(x / 100.0f, y / 100.0f, z / 100.0f, a / 2000.0f, b / 2000.0f, g / 2000.0f);
    }

    x = UI.horizontalSliderX_3->value();
    y = UI.horizontalSliderY_3->value();
    z = UI.horizontalSliderZ_3->value();
    a = UI.horizontalSliderA_3->value();
    b = UI.horizontalSliderB_3->value();
    g = UI.horizontalSliderG_3->value();

    if (x != 0 || y != 0 || z != 0 || a != 0 || b != 0 || g != 0)
    {
        updatBoxBiPos(x / 100.0f, y / 100.0f, z / 100.0f, a / 2000.0f, b / 2000.0f, g / 2000.0f);
    }
}

void JacobiWindow::updatBoxPos(float x, float y, float z, float a, float b, float g)
{
    if (!box)
    {
        return;
    }

    Eigen::Matrix4f m = box->getGlobalPose();
    Eigen::Matrix4f mR;
    MathTools::rpy2eigen4f(a, b, g, mR);
    mR = m * mR;
    mR(0, 3) = m(0, 3) + x;
    mR(1, 3) = m(1, 3) + y;
    mR(2, 3) = m(2, 3) + z;
    box->setGlobalPose(mR);
}

void JacobiWindow::updatBoxBiPos(float x, float y, float z, float a, float b, float g)
{
    if (!box3)
    {
        return;
    }

    Eigen::Matrix4f m = box3->getGlobalPose();
    Eigen::Matrix4f mR;
    MathTools::rpy2eigen4f(a, b, g, mR);
    mR = m * mR;
    mR(0, 3) = m(0, 3) + x;
    mR(1, 3) = m(1, 3) + y;
    mR(2, 3) = m(2, 3) + z;
    box3->setGlobalPose(mR);
}

void JacobiWindow::updatBox2Pos(float x, float y, float z, float a, float b, float g)
{
    if (!box2)
    {
        return;
    }

    Eigen::Matrix4f m = box2->getGlobalPose();
    Eigen::Matrix4f mR;
    MathTools::rpy2eigen4f(a, b, g, mR);
    mR = m * mR;
    mR(0, 3) = m(0, 3) + x;
    mR(1, 3) = m(1, 3) + y;
    mR(2, 3) = m(2, 3) + z;
    box2->setGlobalPose(mR);
}

QString JacobiWindow::formatString(const char* s, float f)
{
    QString str1(s);

    if (f >= 0)
    {
        str1 += " ";
    }

    if (fabs(f) < 1000)
    {
        str1 += " ";
    }

    if (fabs(f) < 100)
    {
        str1 += " ";
    }

    if (fabs(f) < 10)
    {
        str1 += " ";
    }

    QString str1n;
    str1n.setNum(f, 'f', 3);
    str1 = str1 + str1n;
    return str1;
}


void JacobiWindow::resetSceneryAll()
{
    if (!robot)
    {
        return;
    }

    std::map<ModelJointPtr,float> configMap;
    for (ModelJointPtr joint : robot->getJoints())
    {
        configMap[joint] = 0.0f;
    }
    robot->setJointValues(configMap);

    viewer->viewAll();
}



void JacobiWindow::collisionModel()
{
    if (!robot)
    {
        return;
    }

    useColModel = UI.checkBoxColModel->checkState() == Qt::Checked;
    ModelLink::VisualizationType colModel = useColModel ? ModelLink::VisualizationType::Collision : ModelLink::VisualizationType::Full;

    VisualizationSetPtr visualization = robot->getVisualization(colModel);

    if (visualization)
    {
        viewer->addVisualization(boxVisuLayer, "colModel", visualization);
    }

    viewer->viewAll();
}


void JacobiWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}


int JacobiWindow::main()
{
    viewer->start(this);
    return 0;
}


void JacobiWindow::quit()
{
    std::cout << "JacobiWindow: Closing" << std::endl;
    this->close();
    timer->stop();
    viewer->stop();
}

void JacobiWindow::updateKCBox()
{
    UI.comboBoxKC->clear();

    if (!robot)
    {
        return;
    }

    std::vector<JointSetPtr> rns = robot->getJointSets();
    kinChains.clear();

    for (unsigned int i = 0; i < rns.size(); i++)
    {
        // ModelNodeSet::isKinematicChain() is not supported at the time of writing.
        if (/*rns[i]->isKinematicChain()*/ true)
        {
            UI.comboBoxKC->addItem(QString(rns[i]->getName().c_str()));
            kinChains.push_back(rns[i]);
        }
    }
}

void JacobiWindow::selectKC(int nr)
{
    cout << "Selecting kinematic chain nr " << nr << endl;

    if (nr < 0 || nr >= (int)kinChains.size())
    {
        return;
    }

    kc = kinChains[nr];
    kc->print();
    QString nameQ = "TCP: ";
    tcp = kc->getTCP();

    if (tcp)
    {
        QString n(tcp->getName().c_str());
        nameQ += n;
    }

    float d = kc->getSize();
    QString qd = "Nr of joints: ";
    qd += QString::number(d);

    UI.label_TCP->setText(nameQ);
    UI.label_NrJoints->setText(qd);
    std::vector<RobotNodePtr> nodes = kc->getModelNodes();
    elbow.reset();

    for (size_t i = 0; i < nodes.size(); i++)
    {
        if ((nodes[i])->getName() == std::string("Elbow L_joint"))
        {
            elbow = nodes[i];
        }

        if ((nodes[i])->getName() == std::string("Elbow R_joint"))
        {
            elbow = nodes[i];
        }

        if ((nodes[i])->getName() == std::string("TCP L_joint"))
        {
            elbow = nodes[i];
        }

        if ((nodes[i])->getName() == std::string("TCP R_joint"))
        {
            elbow = nodes[i];
        }
    }

    tcp2.reset();

    if (kc->getTCP()->getName() == std::string("TCP L_joint"))
    {
        tcp2 = robot->getModelNode(std::string("TCP R_joint"));
    }

    if (kc->getTCP()->getName() == std::string("TCP R_joint"))
    {
        tcp2 = robot->getModelNode(std::string("TCP L_joint"));
    }

    box2TCP();
}

void JacobiWindow::sliderReleased()
{
    UI.horizontalSliderX->setSliderPosition(0);
    UI.horizontalSliderY->setSliderPosition(0);
    UI.horizontalSliderZ->setSliderPosition(0);
    UI.horizontalSliderA->setSliderPosition(0);
    UI.horizontalSliderB->setSliderPosition(0);
    UI.horizontalSliderG->setSliderPosition(0);
    UI.horizontalSliderX_2->setSliderPosition(0);
    UI.horizontalSliderY_2->setSliderPosition(0);
    UI.horizontalSliderZ_2->setSliderPosition(0);
    UI.horizontalSliderA_2->setSliderPosition(0);
    UI.horizontalSliderB_2->setSliderPosition(0);
    UI.horizontalSliderG_2->setSliderPosition(0);
    UI.horizontalSliderX_3->setSliderPosition(0);
    UI.horizontalSliderY_3->setSliderPosition(0);
    UI.horizontalSliderZ_3->setSliderPosition(0);
    UI.horizontalSliderA_3->setSliderPosition(0);
    UI.horizontalSliderB_3->setSliderPosition(0);
    UI.horizontalSliderG_3->setSliderPosition(0);
}


void JacobiWindow::jacobiTest()
{
    if (!kc)
    {
        return;
    }

    cout << "---- TEST JACOBI ----" << endl;
    JointSetPtr jointSet = JointSet::createJointSet(kc->getModel(), kc->getName(),
                             kc->getModelJoints(), kc->getKinematicRoot(), kc->getTCP());
    DifferentialIKPtr j(new DifferentialIK(jointSet));

    Eigen::Matrix4f targetPose = box->getGlobalPose();

    j->setGoal(targetPose, RobotNodePtr(), IKSolver::All);
    j->computeSteps(0.2f, 0, 50);
    viewer->viewAll();
    cout << "---- END TEST JACOBI ----" << endl;
}

void JacobiWindow::jacobiTest2()
{
    if (!kc || !elbow)
    {
        return;
    }

    cout << "---- TEST JACOBI ----" << endl;
    //std::vector<RobotNodePtr> n;
    //n.push_back(tcp);
    //n.push_back(elbow);
    //RobotNodeSetPtr rns = RobotNodeSet::createRobotNodeSet(robot,std::string("jacobiTest"),n);
    JointSetPtr jointSet = JointSet::createJointSet(kc->getModel(), kc->getName(),
                             kc->getModelJoints(), kc->getKinematicRoot(), kc->getTCP());
    DifferentialIKPtr j(new DifferentialIK(jointSet));

    Eigen::Matrix4f targetPose = box->getGlobalPose();
    Eigen::Matrix4f targetPose2 = box2->getGlobalPose();

    j->setGoal(targetPose, tcp, IKSolver::Position);
    j->setGoal(targetPose2, elbow, IKSolver::Z);
    j->computeSteps(0.2f, 0, 40);
    viewer->viewAll();

    cout << "---- END TEST JACOBI ----" << endl;
}

void JacobiWindow::jacobiTestBi()
{
    if (!kc || !tcp || !tcp2)
    {
        return;
    }

    cout << "---- TEST JACOBI ----" << endl;
    //std::vector<RobotNodePtr> n;
    //n.push_back(tcp);
    //n.push_back(tcp2);
    //RobotNodeSetPtr rns = RobotNodeSet::createRobotNodeSet(robot,std::string("jacobiTest"),n);
    std::vector<RobotNodePtr> nBi;
    nBi.push_back(robot->getModelNode(std::string("Shoulder 1 L_joint")));
    nBi.push_back(robot->getModelNode(std::string("Shoulder 1 R_joint")));
    nBi.push_back(robot->getModelNode(std::string("Shoulder 2 L_joint")));
    nBi.push_back(robot->getModelNode(std::string("Shoulder 2 R_joint")));
    nBi.push_back(robot->getModelNode(std::string("Underarm L_joint")));
    nBi.push_back(robot->getModelNode(std::string("Underarm R_joint")));
    nBi.push_back(robot->getModelNode(std::string("Elbow L_joint")));
    nBi.push_back(robot->getModelNode(std::string("Elbow R_joint")));
    nBi.push_back(robot->getModelNode(std::string("Upperarm L_joint")));
    nBi.push_back(robot->getModelNode(std::string("Upperarm R_joint")));
    nBi.push_back(robot->getModelNode(std::string("Wrist 1 L_joint")));
    nBi.push_back(robot->getModelNode(std::string("Wrist 1 R_joint")));
    nBi.push_back(robot->getModelNode(std::string("Wrist 2 L_joint")));
    nBi.push_back(robot->getModelNode(std::string("Wrist 2 R_joint")));
    nBi.push_back(robot->getModelNode(std::string("Hip Roll_joint")));
    nBi.push_back(robot->getModelNode(std::string("Hip Pitch_joint")));
    nBi.push_back(robot->getModelNode(std::string("Hip Yaw_joint")));
    JointSetPtr kcBi = JointSet::createJointSet(robot, std::string("jacobiTestBi"), nBi);

    DifferentialIKPtr j(new DifferentialIK(kcBi));

    Eigen::Matrix4f targetPose = box->getGlobalPose();
    Eigen::Matrix4f targetPose2 = box3->getGlobalPose();

    j->setGoal(targetPose, tcp, IKSolver::Position);
    j->setGoal(targetPose2, tcp2, IKSolver::Position);
    j->computeSteps(0.2f, 0, 50);
    viewer->viewAll();

    cout << "---- END TEST JACOBI ----" << endl;
}

void JacobiWindow::box2TCP()
{
    if (!tcp || !box || !box2)
    {
        return;
    }

    Eigen::Matrix4f m = tcp->getGlobalPose();

    box->setGlobalPose(m);

    if (elbow)
    {
        m = elbow->getGlobalPose();
        box2->setGlobalPose(m);
    }

    if (tcp2)
    {
        m = tcp2->getGlobalPose();
        box3->setGlobalPose(m);
    }

}

void JacobiWindow::sliderPressed()
{
    cout << "GG ";
}

void JacobiWindow::loadRobot()
{
    std::cout << "JacobiWindow: Loading robot" << std::endl;
    cout << "Loading Robot from " << robotFilename << endl;

    try
    {
        robot = ModelIO::loadModel(robotFilename);
    }
    catch (VirtualRobotException& e)
    {
        cout << " ERROR while creating robot" << endl;
        cout << e.what();
        return;
    }

    if (!robot)
    {
        cout << " ERROR while creating robot" << endl;
        return;
    }

    updateKCBox();

    if (kinChains.size() == 0)
    {
        selectKC(-1);
    }
    else
    {
        selectKC(0);
    }

    // build visualization
    collisionModel();
    viewer->viewAll();
}

