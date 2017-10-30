
#include "stabilityWindow.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/Workspace/Reachability.h"
#include "VirtualRobot/XML/ModelIO.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationNode.h"
#include "VirtualRobot/CollisionDetection/CollisionChecker.h"
#include "VirtualRobot/Tools/BoundingBox.h"
#include "VirtualRobot/IK/CoMIK.h"
#include "VirtualRobot/Model/Nodes/ModelLink.h"
#include "VirtualRobot/Model/Nodes/ModelJoint.h"
#include "VirtualRobot/Import/SimoxXMLFactory.h"

//#ifdef USE_NLOPT
//#include "VirtualRobot/IK/ConstrainedOptimizationIK.h"
//#include "VirtualRobot/IK/constraints/CoMConstraint.h"
//#endif

#include <QFileDialog>
#include <Eigen/Geometry>

#include <time.h>
#include <vector>
#include <iostream>
#include <cmath>

#ifdef Simox_USE_COIN_VISUALIZATION
    #include "../../../Gui/Coin/CoinViewerFactory.h"
    // need this to ensure that static Factory methods are called across library boundaries (otherwise coin Gui lib is not loaded since it is not referenced by us)
    SimoxGui::CoinViewerFactory f;
#endif

#include <sstream>
using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 30.0f;

stabilityWindow::stabilityWindow(const std::string& robotFile, const std::string linkset, const std::string &jointset)
    : QMainWindow(NULL)
{
    VR_INFO << " start " << endl;

    this->robotFile = robotFile;
    useColModel = false;

    comTarget = Eigen::Vector2f::Zero();

    setupUI();

    MathTools::Plane p =  MathTools::getFloorPlane();
    VisualizationNodePtr pv = VisualizationFactory::getGlobalVisualizationFactory()->createPlane(p.p, p.n, 10000.0f, 0.0f);
    viewer->addVisualization("floor", "floor", pv);

    loadRobot();

    selectJointSet(jointset);
    selectLinkSet(linkset);

    viewer->viewAll();
}


stabilityWindow::~stabilityWindow()
{
}


void stabilityWindow::setupUI()
{
    UI.setupUi(this);

    SimoxGui::ViewerFactoryPtr viewerFactory = SimoxGui::ViewerFactory::first(NULL);
    THROW_VR_EXCEPTION_IF(!viewerFactory,"No viewer factory?!");
    viewer = viewerFactory->createViewer(UI.frameViewer);

    connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
    connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(selectRobot()));

    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(collisionModel()));
    connect(UI.checkBoxCoM, SIGNAL(clicked()), this, SLOT(showCoM()));
    connect(UI.checkBoxSupportPolygon, SIGNAL(clicked()), this, SLOT(showSupportPolygon()));
    connect(UI.comboBoxJointSet, SIGNAL(activated(int)), this, SLOT(selectJointSet(int)));
    connect(UI.comboBoxLinkSet, SIGNAL(activated(int)), this, SLOT(selectLinkSet(int)));

    connect(UI.comboBoxJoint, SIGNAL(activated(int)), this, SLOT(selectJoint(int)));
    connect(UI.horizontalSliderPos, SIGNAL(valueChanged(int)), this, SLOT(jointValueChanged(int)));
    connect(UI.horizontalSliderPos, SIGNAL(sliderReleased()), this, SLOT(showSupportPolygon()));

    connect(UI.sliderX, SIGNAL(valueChanged(int)), this, SLOT(comTargetMovedX(int)));
    connect(UI.sliderY, SIGNAL(valueChanged(int)), this, SLOT(comTargetMovedY(int)));
}

void stabilityWindow::resetSceneryAll()
{
    if (!robot)
    {
        return;
    }
    std::vector< ModelJointPtr > nodes;
    nodes = robot->getJoints();
    for (auto &n : nodes)
        n->setJointValue(0.0f);

    updateCoM();
}



void stabilityWindow::collisionModel()
{
    if (!robot)
    {
        return;
    }

    buildVisu();
}

void stabilityWindow::showSupportPolygon()
{
    if (!robot)
    {
        return;
    }

    updateSupportVisu();
}

void stabilityWindow::showCoM()
{
    if (!robot)
    {
        return;
    }

    buildVisu();
}

void stabilityWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}


void stabilityWindow::buildVisu()
{
    if (!robot)
    {
        return;
    }

    viewer->clearLayer("robot");
    useColModel = UI.checkBoxColModel->checkState() == Qt::Checked;
    ModelLink::VisualizationType colModel = (UI.checkBoxColModel->isChecked()) ? ModelLink::VisualizationType::Collision : ModelLink::VisualizationType::Full;

    VisualizationPtr visu = robot->getVisualization(colModel);
    viewer->addVisualization("robot", robot->getName(), visu);

    updateCoM();
    updateSupportVisu();

}

void stabilityWindow::updateCoM()
{
    viewer->clearLayer("com");

    Eigen::Matrix4f globalPoseCoM;
    globalPoseCoM.setIdentity();

    if (currentLinkSet)
    {
        globalPoseCoM.block(0, 3, 3, 1) = currentLinkSet->getCoM();
    }
    else if (robot)
    {
        globalPoseCoM.block(0, 3, 3, 1) = robot->getCoMGlobal();
    }

    if (!UI.checkBoxCoM->isChecked())
        return;

    // Draw CoM
    VisualizationNodePtr sphere = VisualizationFactory::getGlobalVisualizationFactory()->createSphere(30.0f, 1.0f, 0.2f, 0.2f);
    VisualizationNodePtr box = VisualizationFactory::getGlobalVisualizationFactory()->createBox(50.0f, 50.0f, 50.0f, 1.0f, 0.2f, 0.2f);
    VisualizationFactory::getGlobalVisualizationFactory()->applyDisplacement(sphere, globalPoseCoM);
    VisualizationFactory::getGlobalVisualizationFactory()->applyDisplacement(box, globalPoseCoM);
    viewer->addVisualization("com", "com-box", box);
    viewer->addVisualization("com", "com-sphere", sphere);

    // Draw CoM projection
    globalPoseCoM(2, 3) = 0;
    VisualizationNodePtr sphere2 = VisualizationFactory::getGlobalVisualizationFactory()->createSphere(30.0f, 0.2f, 0.2f, 1.0f);
    VisualizationNodePtr box2 = VisualizationFactory::getGlobalVisualizationFactory()->createBox(50.0f, 50.0f, 50.0f, 0.2f, 0.2f, 1.0f);
    VisualizationFactory::getGlobalVisualizationFactory()->applyDisplacement(sphere2, globalPoseCoM);
    VisualizationFactory::getGlobalVisualizationFactory()->applyDisplacement(box2, globalPoseCoM);
    viewer->addVisualization("com", "projected-com-box", box2);
    viewer->addVisualization("com", "projected-com-sphere", sphere2);

    // target com
    Eigen::Matrix4f targCom;
    targCom.setIdentity();
    targCom.block(0,3,2,1) = comTarget;
    VisualizationNodePtr sphere3 = VisualizationFactory::getGlobalVisualizationFactory()->createSphere(30.0f, 0.2f, 1.0f, 0.2f);
    VisualizationNodePtr box3 = VisualizationFactory::getGlobalVisualizationFactory()->createBox(50.0f, 50.0f, 50.0f, 0.2f, 1.0f, 0.2f);
    VisualizationFactory::getGlobalVisualizationFactory()->applyDisplacement(sphere3, targCom);
    VisualizationFactory::getGlobalVisualizationFactory()->applyDisplacement(box3, targCom);
    viewer->addVisualization("com", "target-com-box", box3);
    viewer->addVisualization("com", "target-com-sphere", sphere3);
}


void stabilityWindow::updateSupportVisu()
{
    viewer->clearLayer("support");

    if (UI.checkBoxSupportPolygon->isChecked())
    {
        MathTools::Plane p =  MathTools::getFloorPlane();

        std::vector< CollisionModelPtr > colModels =  robot->getCollisionModels();
        CollisionCheckerPtr colChecker = CollisionChecker::getGlobalCollisionChecker();
        std::vector< CollisionModelPtr >::iterator i = colModels.begin();

        std::vector< MathTools::ContactPoint > points;

        while (i != colModels.end())
        {
            colChecker->getContacts(p, *i, points, 5.0f);
            i++;
        }

        std::vector< Eigen::Vector2f > points2D;

        //MathTools::Plane plane2(Eigen::Vector3f(0,0,0),Eigen::Vector3f(0,1.0f,0));
        for (size_t u = 0; u < points.size(); u++)
        {

            Eigen::Vector2f pt2d = MathTools::projectPointToPlane2D(points[u].p, p);
            points2D.push_back(pt2d);
        }

        MathTools::ConvexHull2DPtr cv = MathTools::createConvexHull2D(points2D);
        VisualizationNodePtr sv = VisualizationFactory::getGlobalVisualizationFactory()->createConvexHull2DVisualization(cv, p, VisualizationFactory::Color::Blue(), VisualizationFactory::Color::Black(), 6.0f, Eigen::Vector3f(0, 0, 2.0f));
        viewer->addVisualization("support", "support-plane", sv);
    }
}

void stabilityWindow::selectJointSet(const string &jointset)
{
    int p = 1;
    for (auto &r : jointSets)
    {
        if (r->getName() == jointset)
        {
            UI.comboBoxJointSet->setCurrentIndex(p);
            selectJointSet(p);
            return;
        }
        p++;
    }
    VR_WARNING << "Could not find joint set with name " << jointset << endl;
}

void stabilityWindow::selectLinkSet(const string &linkset)
{
    int p = 1;
    for (auto &r : linkSets)
    {
        if (r->getName() == linkset)
        {
            UI.comboBoxLinkSet->setCurrentIndex(p);
            selectLinkSet(p);
            return;
        }
        p++;
    }
    VR_WARNING << "Could not find link set with name " << linkset << endl;
}

void stabilityWindow::updateRNSBox()
{
    UI.comboBoxJointSet->clear();
    UI.comboBoxLinkSet->clear();
    UI.comboBoxLinkSet->addItem(QString("<All>"));
    UI.comboBoxJointSet->addItem(QString("<All>"));

    std::vector < VirtualRobot::JointSetPtr > allJointSets = robot->getJointSets();
    std::vector < VirtualRobot::LinkSetPtr > allLinkSets = robot->getLinkSets();
    jointSets.clear();
    linkSets.clear();

    for (size_t i = 0; i < allJointSets.size(); i++)
    {
        jointSets.push_back(allJointSets[i]);
        UI.comboBoxJointSet->addItem(QString(allJointSets[i]->getName().c_str()));
    }
    for (size_t i = 0; i < allLinkSets.size(); i++)
    {
        linkSets.push_back(allLinkSets[i]);
        UI.comboBoxLinkSet->addItem(QString(allLinkSets[i]->getName().c_str()));
    }


    /*for (unsigned int i = 0; i < allRobotNodes.size(); i++)
    {
        allRobotNodes[i]->showBoundingBox(false);
    }*/


    updateJointBox();
}

void stabilityWindow::selectJointSet(int nr)
{
    currentJointSet.reset();
    cout << "Selecting joint set nr " << nr << endl;

    if (nr <= 0)
    {
        // all joints
        currentJoints = allJoints;
    }
    else
    {
        nr--;

        if (nr >= (int)jointSets.size())
        {
            return;
        }

        currentJointSet = jointSets[nr];
        currentJoints = currentJointSet->getJoints();

        // Set CoM target to current CoM position
        if (currentLinkSet)
        {
            Eigen::Vector3f com = currentLinkSet->getCoM();
            UI.sliderX->setValue(com(0));
            UI.sliderY->setValue(com(1));
        }
    }

    updateJointBox();
    selectJoint(0);
    updateCoM();
}

void stabilityWindow::selectLinkSet(int nr)
{
    currentLinkSet.reset();
    cout << "Selecting link set nr " << nr << endl;

    if (nr <= 0)
    {
        // all joints
        currentLinks = allLinks;
    }
    else
    {
        nr--;

        if (nr >= (int)linkSets.size())
        {
            return;
        }

        currentLinkSet = linkSets[nr];
        currentLinks = currentLinkSet->getLinks();

        // Set CoM target to current CoM position
        Eigen::Vector3f com = currentLinkSet->getCoM();
        UI.sliderX->setValue(com(0));
        UI.sliderY->setValue(com(1));
    }

    //updateLinkBox();
    //selectLink(0);
    updateCoM();
}


int stabilityWindow::main()
{
    viewer->start(this);
    return 0;
}


void stabilityWindow::quit()
{
    std::cout << "stabilityWindow: Closing" << std::endl;
    viewer->stop();
    this->close();
}


void stabilityWindow::updateJointBox()
{
    UI.comboBoxJoint->clear();

    for (unsigned int i = 0; i < currentJoints.size(); i++)
    {
        UI.comboBoxJoint->addItem(QString(currentJoints[i]->getName().c_str()));
    }

    selectJoint(0);
}

void stabilityWindow::jointValueChanged(int pos)
{
    int nr = UI.comboBoxJoint->currentIndex();

    if (nr < 0 || nr >= (int)currentJoints.size())
    {
        return;
    }

    float fPos = currentJoints[nr]->getJointLimitLow() + (float)pos / 1000.0f * (currentJoints[nr]->getJointLimitHigh() - currentJoints[nr]->getJointLimitLow());
    currentJoints[nr]->setJointValue(fPos);
    UI.lcdNumberJointValue->display((double)fPos);

    updateCoM();

    /*RobotNodePtr p = robot->getModelNode("Foot2 L");
    if (p)
    {
        BoundingBox bbox = p->getCollisionModel()->getBoundingBox(true);
        supportVisu->addChild(CoinVisualizationFactory::CreateBBoxVisualization(bbox));
    }*/
    // show bbox
    /*if (currentRobotNodeSet)
    {
        for (unsigned int i = 0; i < currentRobotNodeSet->getSize(); i++)
        {
            currentRobotNodeSet->getNode(i)->showBoundingBox(true, true);
        }
    }*/
}



void stabilityWindow::selectJoint(int nr)
{
    currentJoint.reset();
    cout << "Selecting Joint nr " << nr << endl;

    if (nr < 0 || nr >= (int)currentJoints.size())
    {
        return;
    }

    currentJoint = currentJoints[nr];
    currentJoint->print();
    float mi = currentJoint->getJointLimitLow();
    float ma = currentJoint->getJointLimitHigh();
    QString qMin = QString::number(mi);
    QString qMax = QString::number(ma);
    UI.labelMinPos->setText(qMin);
    UI.labelMaxPos->setText(qMax);
    float j = currentJoint->getJointValue();
    UI.lcdNumberJointValue->display((double)j);

    if (fabs(ma - mi) > 0 && (currentJoint->isTranslationalJoint() || currentJoint->isRotationalJoint()))
    {
        UI.horizontalSliderPos->setEnabled(true);
        int pos = (int)((j - mi) / (ma - mi) * 1000.0f);
        UI.horizontalSliderPos->setValue(pos);
    }
    else
    {
        UI.horizontalSliderPos->setValue(500);
        UI.horizontalSliderPos->setEnabled(false);
    }
}

void stabilityWindow::selectRobot()
{
    QString fi = QFileDialog::getOpenFileName(this, tr("Open Robot File"), QString(), tr("XML Files (*.xml)"));
    robotFile = std::string(fi.toLatin1());
    loadRobot();
}

void stabilityWindow::loadRobot()
{
    viewer->clearLayer("robot");
    cout << "Loading Scene from " << robotFile << endl;

    try
    {
        robot = VirtualRobot::ModelIO::loadModel(robotFile);
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

    // get nodes
    allJoints = robot->getJoints();
    allLinks = robot->getLinks();

    updateRNSBox();
    selectLinkSet(0);
    selectJointSet(0);

    if (allJoints.size() == 0)
    {
        selectJoint(-1);
    }
    else
    {
        selectJoint(0);
    }


    // build visualization
    buildVisu();
    viewer->viewAll();
}

void stabilityWindow::performCoMIK()
{
    if (!currentJointSet || !currentLinkSet)
    {
        return;
    }

/*#ifdef USE_NLOPT
    ConstrainedOptimizationIKPtr ik(new ConstrainedOptimizationIK(robot, currentJointSet, 0.01));
    CoMConstraintPtr comConstraint(new CoMConstraint(robot, currentJointSet, currentLinkSet, m_CoMTarget, 5.0f));
    ik->addConstraint(comConstraint);
    ik->initialize();

    if(!ik->solve())
    {
        std::cout << "IK solver did not succeed" << std::endl;
    }

#else*/
    CoMIK comIK(currentJointSet, currentLinkSet);
    comIK.setGoal(comTarget);

    if (!comIK.solveIK(0.3f, 0, 20))
    {
        std::cout << "IK solver did not succeed" << std::endl;
    }
//#endif

    updateCoM();
}

void stabilityWindow::comTargetMovedX(int value)
{
    comTarget(0) = value;
    updateCoM();
    performCoMIK();
}

void stabilityWindow::comTargetMovedY(int value)
{
    comTarget(1) = value;
    updateCoM();
    performCoMIK();
}
