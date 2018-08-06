
#include "GenericIKWindow.h"
#include "VirtualRobot/Visualization/VisualizationFactory.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "Gui/ViewerFactory.h"

#ifdef USE_NLOPT
#include "VirtualRobot/IK/ConstrainedOptimizationIK.h"
#include "VirtualRobot/IK/constraints/PoseConstraint.h"
#endif

#include <time.h>
#include <vector>
#include <iostream>
#include <sstream>

using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 30.0f;

GenericIKWindow::GenericIKWindow(std::string& sRobotFilename)
    : QMainWindow(nullptr), boxVisuLayer("box-layer"), robotVisuLayer("robot-layer")
{
    VR_INFO << " start " << endl;

    useColModel = false;
    robotFilename = sRobotFilename;
    setupUI();
    loadRobot();

    box = Obstacle::createBox(30.0f, 30.0f, 30.0f);
    box->attachFrames();
    viewer->addVisualization(box->getVisualization(), boxVisuLayer);

    box2TCP();

    viewer->viewAll();
}


GenericIKWindow::~GenericIKWindow()
{
    timer->stop();
    delete timer;
}

void GenericIKWindow::setupUI()
{
    UI.setupUi(this);

    SimoxGui::ViewerFactoryPtr factory = SimoxGui::ViewerFactory::getInstance();
    viewer = factory->createViewer(UI.frameViewer);
    viewer->viewAll();

    connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
    connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(loadRobot()));

    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(collisionModel()));
    connect(UI.comboBoxKC, SIGNAL(activated(int)), this, SLOT(selectKC(int)));
    connect(UI.comboBoxIKMethod, SIGNAL(activated(int)), this, SLOT(selectIK(int)));

    connect(UI.pushButtonBox2TCP, SIGNAL(clicked()), this, SLOT(box2TCP()));
    connect(UI.pushButtonSolve, SIGNAL(clicked()), this, SLOT(solve()));

    connect(UI.horizontalSliderX, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderY, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderZ, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderA, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderB, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderG, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));

    UI.comboBoxIKMethod->setCurrentIndex(1);

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateCB()));
    timer->start(30.0f);
}

void GenericIKWindow::updateCB()
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
}

void GenericIKWindow::updatBoxPos(float x, float y, float z, float a, float b, float g)
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

QString GenericIKWindow::formatString(const char* s, float f)
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


void GenericIKWindow::resetSceneryAll()
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
}



void GenericIKWindow::collisionModel()
{
    if (!robot)
    {
        return;
    }

    viewer->clearLayer(robotVisuLayer);
    useColModel = UI.checkBoxColModel->checkState() == Qt::Checked;
    ModelLink::VisualizationType colModel = useColModel ? ModelLink::VisualizationType::Collision : ModelLink::VisualizationType::Full;

    VisualizationSetPtr visualization = robot->getVisualization(colModel);
    if (visualization)
    {
        viewer->addVisualization(visualization, robotVisuLayer);
    }
}


void GenericIKWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}


void GenericIKWindow::quit()
{
    std::cout << "GenericIKWindow: Closing" << std::endl;
    this->close();
    timer->stop();
}

void GenericIKWindow::updateKCBox()
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

void GenericIKWindow::selectKC(int nr)
{
    cout << "Selecting kinematic chain nr " << nr << endl;

    if (nr < 0 || nr >= (int)kinChains.size())
    {
        return;
    }

    /*if (tcp)
    {   
        tcp->showCoordinateSystem(false);
    }+/

    /*std::vector<RobotNodePtr> nodes0;
    if (kc)
        nodes0 = kc->getAllRobotNodes();
    for (size_t i=0;i<nodes0.size();i++)
    {
        nodes0[i]->showBoundingBox(false,true);
    }*/
    kc = kinChains[nr];
    kc->print();
    QString nameQ = "TCP: ";
    tcp = kc->getTCP();

    if (tcp)
    {
        QString n(tcp->getName().c_str());
        nameQ += n;
        //tcp->showCoordinateSystem(true);
    }

    unsigned int d = kc->getSize();
    QString qd = "Nr of joints: ";
    qd += QString::number(d);

    UI.label_TCP->setText(nameQ);
    UI.label_NrJoints->setText(qd);
    /*std::vector<RobotNodePtr> nodes = kc->getAllRobotNodes();
    for (size_t i=0;i<nodes.size();i++)
    {
        nodes[i]->showBoundingBox(true,true);
    }*/
    box2TCP();

    JointSetPtr jointSet = JointSet::createJointSet(kc->getModel(), kc->getName(), kc->getJoints(), kc->getKinematicRoot(), kc->getTCP());
    if (kc->getNode(kc->getSize() - 1)->isTranslationalJoint())
    {
        ikGazeSolver.reset(new GazeIK(jointSet, std::dynamic_pointer_cast<ModelJointPrismatic>(kc->getNode(kc->getSize() - 1))));
    }
    else
    {
        ikGazeSolver.reset();
    }

    ikSolver.reset(new GenericIKSolver(jointSet, JacobiProvider::eSVDDamped));
    //ikSolver->getDifferentialIK()->setVerbose(true);
    /*Eigen::VectorXf js(d);
    js.setConstant(1.0f);
    js(js.rows() - 1) = 10.0f;
    ikSolver->getDifferentialIK()->setJointWeights(js);*/
    ikSolver->getDifferentialIK()->setMaxPositionStep(20.0f);
    ikSolver->setupJacobian(0.3f, 100);

    // since it may be that a tcp coord system was created in this method we must re-build the visualization in order to show it
    collisionModel();
}

void GenericIKWindow::selectIK(int nr)
{
    if(nr == 0)
    {
        UI.comboBoxKC->setCurrentIndex(0);
    }
}

void GenericIKWindow::sliderReleased()
{
    UI.horizontalSliderX->setSliderPosition(0);
    UI.horizontalSliderY->setSliderPosition(0);
    UI.horizontalSliderZ->setSliderPosition(0);
    UI.horizontalSliderA->setSliderPosition(0);
    UI.horizontalSliderB->setSliderPosition(0);
    UI.horizontalSliderG->setSliderPosition(0);
}


void GenericIKWindow::solve()
{
    if (!kc || !ikSolver || !tcp)
    {
        return;
    }

    cout << "---- Solve IK ----" << endl;

    IKSolver::CartesianSelection s = IKSolver::All;

    if (UI.radioButton_Pos->isChecked())
    {
        s = IKSolver::Position;
    }

    //if (UI.radioButton_Ori->isChecked())
    //  s = IKSolver::Orientation;
    //ikSolver->setVerbose(true);
    Eigen::Matrix4f targetPose = box->getGlobalPose();

    /*
    if (kc && kc->getNode(kc->getSize() - 1)->isTranslationalJoint() && kc->getNode(kc->getSize() - 1)->getParent())
    {
        // setup gaze IK
        float v = (kc->getNode(kc->getSize() - 1)->getParent()->getGlobalPose().block(0, 3, 3, 1) - targetPose.block(0, 3, 3, 1)).norm();
        cout << "Setting initial value of translation joint to :" << v << endl;
        ikSolver->setupTranslationalJoint(kc->getNode(kc->getSize() - 1), v);
        kc->getNode(kc->getSize() - 1)->setJointValue(v);
    }*/
    clock_t startT = clock();

    if (UI.comboBoxIKMethod->currentIndex() == 0)
    {
        cout << "Solving with Gaze IK" << endl;
        if (!ikGazeSolver)
        {
            VR_WARNING << "GazeIK not supported with currently selected JointSet ('" << kc->getName() << "')." << endl
                       << "Select a JointSet with a translational joint at the end of the chain." << endl;
        }
        else
        {
            ikGazeSolver->solve(targetPose.block(0, 3, 3, 1));
        }
    }
    else if(UI.comboBoxIKMethod->currentIndex() == 1)
    {
        cout << "Solving with Differential IK" << endl;
        ikSolver->solve(targetPose, s, 50);
    }
    else
    {
#ifdef USE_NLOPT
        cout << "Solving with Constrained IK" << endl;
        JointSetPtr jointSet = JointSet::createJointSet(kc->getModel(), kc->getName(), kc->getJoints(), kc->getKinematicRoot(), kc->getTCP());
        ConstrainedOptimizationIK solver(robot, jointSet);

        PoseConstraintPtr pc(new PoseConstraint(robot, jointSet, tcp, targetPose, s));
        solver.addConstraint(pc);

        solver.initialize();
        solver.solve();
#else
        cout << "Constrained IK not available (requires NLopt)" << endl;
#endif
    }

    clock_t endT = clock();

    Eigen::Matrix4f actPose = tcp->getGlobalPose();
    float errorPos = (actPose.block(0, 3, 3, 1) - targetPose.block(0, 3, 3, 1)).norm();
    MathTools::Quaternion q1 = MathTools::eigen4f2quat(actPose);
    MathTools::Quaternion q2 = MathTools::eigen4f2quat(targetPose);
    MathTools::Quaternion d = getDelta(q1, q2);
    float errorOri = fabs(180.0f - (d.w + 1.0f) * 90.0f);

    float diffClock = (float)(((float)(endT - startT) / (float)CLOCKS_PER_SEC) * 1000.0f);
    QString qd = "Time: ";
    qd += QString::number(diffClock, 'f', 2);
    qd += " ms";
    UI.labelTime->setText(qd);
    QString qd2 = "Error Pos: : ";
    qd2 += QString::number(errorPos, 'f', 2);
    qd2 += " mm";
    UI.labelPos->setText(qd2);
    QString qd3 = "Error Ori: : ";
    qd3 += QString::number(errorOri, 'f', 2);
    qd3 += " deg";
    UI.labelOri->setText(qd3);

    cout << "Joint values:" << endl;
    std::vector<ModelJointPtr> joints = kc->getJoints();

    for (size_t i = 0; i < joints.size(); i++)
    {
        cout << joints[i]->getJointValue() << endl;
    }

    /*
    DifferentialIKPtr j(new DifferentialIK(kc));
    j->setGoal(targetPose,RobotNodePtr(),IKSolver::All);
    j->computeSteps(0.2f,0,50);
    */

    cout << "---- END Solve IK ----" << endl;
}

void GenericIKWindow::box2TCP()
{
    if (!tcp || !box)
    {
        return;
    }

    Eigen::Matrix4f m = tcp->getGlobalPose();

    box->setGlobalPose(m);
}

void GenericIKWindow::sliderPressed()
{
}

void GenericIKWindow::loadRobot()
{
    std::cout << "GenericIKWindow: Loading robot" << std::endl;
    viewer->clearLayer(robotVisuLayer);

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

