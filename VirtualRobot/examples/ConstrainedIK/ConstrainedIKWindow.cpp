/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    VirtualRobot
* @author     Peter Kaiser
* @copyright  2016 Peter Kaiser
*             GNU Lesser General Public License
*
*/

#include "ConstrainedIKWindow.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationNode.h"
#include "VirtualRobot/EndEffector/EndEffector.h"

#ifdef USE_NLOPT
#include "VirtualRobot/IK/ConstrainedOptimizationIK.h"
#include "VirtualRobot/IK/constraints/PoseConstraint.h"
#endif

#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoUnits.h>

#include <time.h>
#include <vector>
#include <iostream>
#include <sstream>

using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 30.0f;

ConstrainedIKWindow::ConstrainedIKWindow(std::string& sRobotFilename)
    : QMainWindow(NULL)
{
    VR_INFO << " start " << endl;
    //this->setCaption(QString("ShowRobot - KIT - Humanoids Group"));
    //resize(1100, 768);

    robotFilename = sRobotFilename;
    sceneSep = new SoSeparator();
    sceneSep->ref();
    robotSep = new SoSeparator();
    sceneSep->addChild(robotSep);
    setupUI();

    loadRobot();

    boxSep = new SoSeparator();
    sceneSep->addChild(boxSep);

    tsrSep = new SoSeparator();
    sceneSep->addChild(tsrSep);

    exViewer->viewAll();
}


ConstrainedIKWindow::~ConstrainedIKWindow()
{
    sceneSep->unref();
}

void ConstrainedIKWindow::setupUI()
{
    UI.setupUi(this);
    exViewer = new SoQtExaminerViewer(UI.frameViewer, "", TRUE, SoQtExaminerViewer::BUILD_POPUP);

    // setup
    exViewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
    exViewer->setAccumulationBuffer(true);
    exViewer->setAntialiasing(true, 4);

    exViewer->setTransparencyType(SoGLRenderAction::BLEND);
    exViewer->setFeedbackVisibility(true);
    exViewer->setSceneGraph(sceneSep);
    exViewer->viewAll();

    connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
    connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(loadRobot()));
    connect(UI.comboBoxKC, SIGNAL(activated(int)), this, SLOT(selectKC(int)));
    connect(UI.pushButtonSolve, SIGNAL(clicked()), this, SLOT(solve()));

    connect(UI.tsrLowX, SIGNAL(valueChanged(double)), this, SLOT(updateTSR(double)));
    connect(UI.tsrLowY, SIGNAL(valueChanged(double)), this, SLOT(updateTSR(double)));
    connect(UI.tsrLowZ, SIGNAL(valueChanged(double)), this, SLOT(updateTSR(double)));
    connect(UI.tsrLowPitch, SIGNAL(valueChanged(double)), this, SLOT(updateTSR(double)));
    connect(UI.tsrLowRoll, SIGNAL(valueChanged(double)), this, SLOT(updateTSR(double)));
    connect(UI.tsrLowYaw, SIGNAL(valueChanged(double)), this, SLOT(updateTSR(double)));
    connect(UI.tsrHighX, SIGNAL(valueChanged(double)), this, SLOT(updateTSR(double)));
    connect(UI.tsrHighY, SIGNAL(valueChanged(double)), this, SLOT(updateTSR(double)));
    connect(UI.tsrHighZ, SIGNAL(valueChanged(double)), this, SLOT(updateTSR(double)));
    connect(UI.tsrHighPitch, SIGNAL(valueChanged(double)), this, SLOT(updateTSR(double)));
    connect(UI.tsrHighRoll, SIGNAL(valueChanged(double)), this, SLOT(updateTSR(double)));
    connect(UI.tsrHighYaw, SIGNAL(valueChanged(double)), this, SLOT(updateTSR(double)));

    connect(UI.tsrRandom, SIGNAL(clicked()), this, SLOT(randomTSR()));
    connect(UI.tsrGroup, SIGNAL(clicked()), this, SLOT(enableTSR()));
}

void ConstrainedIKWindow::resetSceneryAll()
{
    if (!robot)
    {
        return;
    }

    std::vector<RobotNodePtr> rn;
    robot->getRobotNodes(rn);
    std::vector<float> jv(rn.size(), 0.0f);
    robot->setJointValues(rn, jv);

    exViewer->render();
}



void ConstrainedIKWindow::collisionModel()
{
    if (!robot)
    {
        return;
    }

    robotSep->removeAllChildren();

    boost::shared_ptr<CoinVisualization> visualization = robot->getVisualization<CoinVisualization>(SceneObject::Full);
    SoNode* visualisationNode = NULL;

    if (visualization)
    {
        visualisationNode = visualization->getCoinVisualization();
    }

    if (visualisationNode)
    {
        robotSep->addChild(visualisationNode);
    }

    exViewer->render();
}


void ConstrainedIKWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}


int ConstrainedIKWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}


void ConstrainedIKWindow::quit()
{
    std::cout << "ConstrainedIKWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}

void ConstrainedIKWindow::updateKCBox()
{
    UI.comboBoxKC->clear();

    if (!robot)
    {
        return;
    }

    std::vector<RobotNodeSetPtr> rns;
    robot->getRobotNodeSets(rns);
    kinChains.clear();

    for (unsigned int i = 0; i < rns.size(); i++)
    {
        if (rns[i]->isKinematicChain())
        {
            UI.comboBoxKC->addItem(QString(rns[i]->getName().c_str()));
            kinChains.push_back(rns[i]);
        }
    }
}

void ConstrainedIKWindow::selectKC(int nr)
{
    cout << "Selecting kinematic chain nr " << nr << endl;

    if (nr < 0 || nr >= (int)kinChains.size())
    {
        return;
    }

    if (tcp)
    {
        tcp->showCoordinateSystem(false);
    }

    kc = kinChains[nr];
    kc->print();
    QString nameQ = "TCP: ";
    tcp = kc->getTCP();

    if (tcp)
    {
        QString n(tcp->getName().c_str());
        nameQ += n;
        tcp->showCoordinateSystem(true);
    }

    unsigned int d = kc->getSize();
    QString qd = "Nr of joints: ";
    qd += QString::number(d);

    UI.label_TCP->setText(nameQ);
    UI.label_NrJoints->setText(qd);

    // since it may be that a tcp coord system was created in this method we must re-build the visualization in order to show it
    collisionModel();
}

void ConstrainedIKWindow::selectIK(int nr)
{
    if(nr == 0)
    {
        UI.comboBoxKC->setCurrentIndex(0);
    }
}

void ConstrainedIKWindow::solve()
{
    if (!kc || !tcp)
    {
        return;
    }

    clock_t startT = clock();

    cout << "Solving with Constrained IK" << endl;
    ConstrainedOptimizationIK solver(robot, kc);

#if 0
    PoseConstraintPtr pc(new PoseConstraint(robot, kc, tcp, targetPose, s));
    solver.addConstraint(pc);
#endif

    solver.initialize();
    solver.solve();

    clock_t endT = clock();

#if 0
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
    std::vector<RobotNodePtr> nodes = kc->getAllRobotNodes();

    for (size_t i = 0; i < nodes.size(); i++)
    {
        cout << nodes[i]->getJointValue() << endl;
    }
#endif

    exViewer->render();
}

void ConstrainedIKWindow::updateTSR(double value)
{
    tsrSep->removeAllChildren();

    SoUnits *u = new SoUnits;
    u->units.setValue(SoUnits::MILLIMETERS);
    tsrSep->addChild(u);

    SoTransform *t = new SoTransform;
    t->translation.setValue(
                UI.tsrHighX->value() + (UI.tsrHighX->value() - UI.tsrLowX->value()) / 2,
                UI.tsrHighY->value() + (UI.tsrHighY->value() - UI.tsrLowY->value()) / 2,
                UI.tsrHighZ->value() + (UI.tsrHighZ->value() - UI.tsrLowZ->value()) / 2
                );
    tsrSep->addChild(t);

    SoMaterial *m = new SoMaterial;
    m->diffuseColor.setValue(1, 0, 0);
    m->ambientColor.setValue(1, 0, 0);
    m->transparency = 0.5;
    tsrSep->addChild(m);

    SoCube *cube = new SoCube;
    cube->width = UI.tsrHighX->value() - UI.tsrLowX->value();
    cube->height = UI.tsrHighY->value() - UI.tsrLowY->value();
    cube->depth = UI.tsrHighZ->value() - UI.tsrLowZ->value();
    tsrSep->addChild(cube);
}

void ConstrainedIKWindow::randomTSR()
{
    // Store joint angles
    RobotConfigPtr originalConfig(new RobotConfig(robot, "original config"));
    kc->getJointValues(originalConfig);

    // Apply random joint angles
    for(unsigned int i = 0; i < kc->getSize(); i++)
    {
        RobotNodePtr node = kc->getNode(i);

        float v = node->getJointLimitLo() + (node->getJointLimitHi() - node->getJointLimitLo()) * (rand()%1000 / 1000.0);
        node->setJointValue(v);
    }

    // Obtain TCP pose
    Eigen::Matrix4f tcpPose = kc->getTCP()->getGlobalPose();

    VR_INFO << "Sampled TCP Pose: " << std::endl << tcpPose << std::endl;

    // Relax TCP pose randomly to form a TSR
    float lowX = tcpPose(0,3) - rand()%100;
    float highX = tcpPose(0,3) + rand()%100;
    float lowY = tcpPose(1,3) - rand()%100;
    float highY = tcpPose(1,3) + rand()%100;
    float lowZ = tcpPose(2,3) - rand()%100;
    float highZ = tcpPose(2,3) + rand()%100;

    Eigen::Vector3f rpy;
    MathTools::eigen4f2rpy(tcpPose, rpy);

    float lowRoll = rpy.x() - (M_PI / 4) * (rand()%100 / 100.0);
    float highRoll = rpy.x() + (M_PI / 4) * (rand()%100 / 100.0);
    float lowPitch = rpy.y() - (M_PI / 4) * (rand()%100 / 100.0);
    float highPitch = rpy.y() + (M_PI / 4) * (rand()%100 / 100.0);
    float lowYaw = rpy.z() - (M_PI / 4) * (rand()%100 / 100.0);
    float highYaw = rpy.z() + (M_PI / 4) * (rand()%100 / 100.0);

    VR_INFO << "Random TSR: " << std::endl
            << "    [" << lowX << ", " << highX << "]," << std::endl
            << "    [" << lowY << ", " << highY << "]," << std::endl
            << "    [" << lowZ << ", " << highZ << "]," << std::endl
            << "    [" << lowRoll << ", " << highRoll << "]," << std::endl
            << "    [" << lowPitch << ", " << highPitch << "]," << std::endl
            << "    [" << lowYaw << ", " << highYaw << "]," << std::endl;

    // Apply TSR
    UI.tsrLowX->setValue(lowX);
    UI.tsrHighX->setValue(highX);
    UI.tsrLowY->setValue(lowY);
    UI.tsrHighY->setValue(highY);
    UI.tsrLowZ->setValue(lowZ);
    UI.tsrHighZ->setValue(highZ);
    UI.tsrLowRoll->setValue(lowRoll);
    UI.tsrHighRoll->setValue(highRoll);
    UI.tsrLowPitch->setValue(lowPitch);
    UI.tsrHighPitch->setValue(highPitch);
    UI.tsrLowYaw->setValue(lowYaw);
    UI.tsrHighYaw->setValue(highYaw);

    updateTSR(0);

    // Restore original joint angles
    kc->setJointValues(originalConfig);
}

void ConstrainedIKWindow::enableTSR()
{
    if(UI.tsrGroup->isChecked())
    {
        updateTSR(0);
    }
    else
    {
        tsrSep->removeAllChildren();
    }
}

void ConstrainedIKWindow::loadRobot()
{
    std::cout << "ConstrainedIKWindow: Loading robot" << std::endl;
    robotSep->removeAllChildren();
    cout << "Loading Robot from " << robotFilename << endl;

    try
    {
        robot = RobotIO::loadRobot(robotFilename);
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
    exViewer->viewAll();
    exViewer->render();
}

