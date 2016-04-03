
#include "showRobotWindow.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/Workspace/Reachability.h"
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/Import/RobotImporterFactory.h>
#include <boost/algorithm/string/predicate.hpp>

#include <QFileDialog>
#include <Eigen/Geometry>

#include <time.h>
#include <vector>
#include <iostream>
#include <cmath>
#include <sstream>

using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 30.0f;

showRobotWindow::showRobotWindow(std::string& sRobotFilename)
    : QMainWindow(NULL)
{
    VR_INFO << " start " << endl;
    //this->setCaption(QString("ShowRobot - KIT - Humanoids Group"));
    //resize(1100, 768);

    useColModel = false;
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(sRobotFilename);
    m_sRobotFilename = sRobotFilename;

    /*SoUnits *u = new SoUnits;
    u->units = SoUnits::MILLIMETERS;
    sceneSep->addChild(u);*/

    /*SoShapeHints * shapeHints = new SoShapeHints;
    shapeHints->vertexOrdering = SoShapeHints::COUNTERCLOCKWISE;
    shapeHints->shapeType = SoShapeHints::UNKNOWN_SHAPE_TYPE;
    sceneSep->addChild(shapeHints);*/
    /*SoLightModel * lightModel = new SoLightModel;
    lightModel->model = SoLightModel::BASE_COLOR;
    sceneSep->addChild(lightModel);*/

    setupUI();

    loadRobot();

    viewer->viewAll();
}


showRobotWindow::~showRobotWindow()
{
    robot.reset();
}

/*
void CShowRobotWindow::saveScreenshot()
{
    static int counter = 0;
    SbString framefile;

    framefile.sprintf("MPL_Render_Frame%06d.png", counter);
    counter++;

    viewer->getSceneManager()->render();
    viewer->getSceneManager()->scheduleRedraw();
    QGLWidget* w = (QGLWidget*)viewer->getGLWidget();

    QImage i = w->grabFrameBuffer();
    bool bRes = i.save(framefile.getString(), "PNG");
    if (bRes)
        cout << "wrote image " << counter << endl;
    else
        cout << "failed writing image " << counter << endl;

}*/

void showRobotWindow::setupUI()
{
    UI.setupUi(this);
    //centralWidget()->setLayout(UI.gridLayoutViewer);

    viewer = new CoinViewer(UI.frameViewer);

    connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
    connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(selectRobot()));

    connect(UI.pushButtonClose, SIGNAL(clicked()), this, SLOT(closeHand()));
    connect(UI.ExportVRML20, SIGNAL(clicked()), this, SLOT(exportVRML()));
    connect(UI.ExportXML, SIGNAL(clicked()), this, SLOT(exportXML()));
    connect(UI.pushButtonOpen, SIGNAL(clicked()), this, SLOT(openHand()));
    connect(UI.comboBoxEndEffector, SIGNAL(activated(int)), this, SLOT(selectEEF(int)));

    connect(UI.checkBoxPhysicsCoM, SIGNAL(clicked()), this, SLOT(displayPhysics()));
    connect(UI.checkBoxPhysicsInertia, SIGNAL(clicked()), this, SLOT(displayPhysics()));

    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(rebuildVisualization()));
    connect(UI.checkBoxRobotSensors, SIGNAL(clicked()), this, SLOT(showSensors()));
    connect(UI.checkBoxStructure, SIGNAL(clicked()), this, SLOT(robotStructure()));
    UI.checkBoxFullModel->setChecked(true);
    connect(UI.checkBoxFullModel, SIGNAL(clicked()), this, SLOT(robotFullModel()));
    connect(UI.checkBoxRobotCoordSystems, SIGNAL(clicked()), this, SLOT(robotCoordSystems()));
    connect(UI.checkBoxShowCoordSystem, SIGNAL(clicked()), this, SLOT(showCoordSystem()));
    connect(UI.comboBoxRobotNodeSet, SIGNAL(activated(int)), this, SLOT(selectRNS(int)));
    connect(UI.comboBoxJoint, SIGNAL(activated(int)), this, SLOT(selectJoint(int)));
    connect(UI.horizontalSliderPos, SIGNAL(valueChanged(int)), this, SLOT(jointValueChanged(int)));

}

QString showRobotWindow::formatString(const char* s, float f)
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


void showRobotWindow::resetSceneryAll()
{
    if (!robot)
    {
        return;
    }

    std::vector<float> jv(allRobotNodes.size(), 0.0f);
    robot->setJointValues(allRobotNodes, jv);

    selectJoint(UI.comboBoxJoint->currentIndex());
}



void showRobotWindow::displayTriangles()
{
    QString text1, text2, text3;
    int trisAllFull, trisRNSFull, trisJointFull;
    trisAllFull = trisRNSFull = trisJointFull = 0;
    int trisAllCol, trisRNSCol, trisJointCol;
    trisAllCol = trisRNSCol = trisJointCol = 0;

    if (robot)
    {
        trisAllFull = robot->getNumFaces(false);
        trisAllCol = robot->getNumFaces(true);
        trisRNSFull = trisAllFull;
        trisRNSCol = trisAllCol;
    }

    if (currentRobotNodeSet)
    {
        trisRNSFull = currentRobotNodeSet->getNumFaces(false);
        trisRNSCol = currentRobotNodeSet->getNumFaces(true);
    }

    if (currentRobotNode)
    {
        trisJointFull = currentRobotNode->getNumFaces(false);
        trisJointCol = currentRobotNode->getNumFaces(true);
    }

    if (UI.checkBoxColModel->checkState() == Qt::Checked)
    {
        text1 = tr("Total\t:") + QString::number(trisAllCol);
        text2 = tr("RobotNodeSet:\t") + QString::number(trisRNSCol);
        text3 = tr("Joint:\t") + QString::number(trisJointCol);
    }
    else
    {
        text1 = tr("Total:\t") + QString::number(trisAllFull);
        text2 = tr("RobotNodeSet:\t") + QString::number(trisRNSFull);
        text3 = tr("Joint:\t") + QString::number(trisJointFull);
    }

    UI.labelInfo1->setText(text1);
    UI.labelInfo2->setText(text2);
    UI.labelInfo3->setText(text3);
}

void showRobotWindow::robotFullModel()
{
    if (!robot)
    {
        return;
    }

    bool showFullModel = UI.checkBoxFullModel->checkState() == Qt::Checked;

    robot->setupVisualization(showFullModel, true);

}

void showRobotWindow::rebuildVisualization()
{
    if (!robot)
    {
        return;
    }

    viewer->clearLayer("robotLayer");

    //setRobotModelShape(UI.checkBoxColModel->state() == QCheckBox::On);
    useColModel = UI.checkBoxColModel->checkState() == Qt::Checked;
    //bool sensors = UI.checkBoxRobotSensors->checkState() == Qt::Checked;
    SceneObject::VisualizationType colModel = (UI.checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;

    VisualizationPtr visu = robot->getVisualization<CoinVisualization>(colModel);
    viewer->addVisualization("robotLayer", "robot", visu);

    selectJoint(UI.comboBoxJoint->currentIndex());

    UI.checkBoxStructure->setEnabled(!useColModel);
    UI.checkBoxRobotSensors->setEnabled(!useColModel);
    UI.checkBoxFullModel->setEnabled(!useColModel);
    UI.checkBoxRobotCoordSystems->setEnabled(!useColModel);

}

void showRobotWindow::showSensors()
{
    if (!robot)
    {
        return;
    }

    bool showSensors = UI.checkBoxRobotSensors->isChecked();

    std::vector<SensorPtr> sensors = robot->getSensors();

    for (size_t i = 0; i < sensors.size(); i++)
    {
        sensors[i]->setupVisualization(showSensors, showSensors);
        sensors[i]->showCoordinateSystem(showSensors);
    }

    // rebuild visualization
    rebuildVisualization();
}



void showRobotWindow::displayPhysics()
{
    if (!robot)
    {
        return;
    }

    physicsCoMEnabled = UI.checkBoxPhysicsCoM->checkState() == Qt::Checked;
    physicsInertiaEnabled = UI.checkBoxPhysicsInertia->checkState() == Qt::Checked;
    robot->showPhysicsInformation(physicsCoMEnabled, physicsInertiaEnabled);

    // rebuild visualization
    rebuildVisualization();

}

void showRobotWindow::exportVRML()
{
    if (!robot)
    {
        return;
    }

#if 0
    resetSceneryAll();

    RobotNodePtr start1 = robot->getRobotNode("Shoulder 2 L");
    RobotNodePtr tcp1 = robot->getRobotNode("Wrist 1 L");

    Eigen::Matrix4f m1 = start1->toLocalCoordinateSystem(tcp1->getGlobalPose());
    cout << "OLD trafo (FW):" << endl << m1 << endl;

    /*
    RobotFactory::robotStructureDef newStructure;
    newStructure.rootName = "TCP L";

    RobotFactory::robotNodeDef rn1;
    rn1.name = "TCP L";
    rn1.children.push_back("Wrist 2 L");
    rn1.invertTransformation = true;
    newStructure.parentChildMapping.push_back(rn1);

    RobotFactory::robotNodeDef rn2;
    rn2.name = "Wrist 2 L";
    rn2.children.push_back("Wrist 1 L");
    rn2.invertTransformation = true;
    newStructure.parentChildMapping.push_back(rn2);

    RobotFactory::robotNodeDef rn3;
    rn3.name = "Wrist 1 L";
    rn3.children.push_back("Underarm L");
    rn3.invertTransformation = true;
    newStructure.parentChildMapping.push_back(rn3);

    RobotFactory::robotNodeDef rn4;
    rn4.name = "Underarm L";
    rn4.children.push_back("Elbow L");
    rn4.invertTransformation = true;
    newStructure.parentChildMapping.push_back(rn4);

    RobotFactory::robotNodeDef rn5;
    rn5.name = "Elbow L";
    rn5.children.push_back("Upperarm L");
    rn5.invertTransformation = true;
    newStructure.parentChildMapping.push_back(rn5);

    RobotFactory::robotNodeDef rn6;
    rn6.name = "Upperarm L";
    rn6.children.push_back("Shoulder 2 L");
    rn6.invertTransformation = true;
    newStructure.parentChildMapping.push_back(rn6);

    RobotFactory::robotNodeDef rn7;
    rn7.name = "Shoulder 2 L";
    rn7.children.push_back("Shoulder 1 L");
    rn7.invertTransformation = true;
    newStructure.parentChildMapping.push_back(rn7);

    robot = RobotFactory::cloneChangeStructure(robot, newStructure);
    */
    robot = RobotFactory::cloneChangeStructure(robot, "Shoulder 1 L", "TCP L");

    updatRobotInfo();

    RobotNodePtr start2 = robot->getRobotNode("Shoulder 2 L");
    RobotNodePtr tcp2 = robot->getRobotNode("Wrist 1 L");

    Eigen::Matrix4f m2 = start2->toLocalCoordinateSystem(tcp2->getGlobalPose());
    cout << "NEW trafo (INV):" << endl << m2 << endl;

    return;

#endif
    // VRML
    QString fi = QFileDialog::getSaveFileName(this, tr("VRML 2.0 File"), QString(), tr("VRML Files (*.wrl)"));
    std::string s = std::string(fi.toLatin1());

    if (!s.empty())
    {
        if (!boost::algorithm::ends_with(s, ".wrl"))
            s += ".wrl";
        SceneObject::VisualizationType colModel = (UI.checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;
        visualization = robot->getVisualization<CoinVisualization>(colModel);
        visualization->exportToVRML2(s);
    }
}


void showRobotWindow::exportXML()
{
    if (!robot)
    {
        return;
    }

    // XML
    QString fi = QFileDialog::getSaveFileName(this, tr("xml File"), QString(), tr("xml Files (*.xml)"));
    std::string s = std::string(fi.toLatin1());

    if (!s.empty())
    {

        boost::filesystem::path p1(s);
        std::string fn = p1.filename().generic_string();
        std::string fnPath = p1.parent_path().generic_string();
        RobotIO::saveXML(robot, fn, fnPath);
    }

}

void showRobotWindow::showRobot()
{
    //m_pGraspScenery->showRobot(m_pShowRobot->state() == QCheckBox::On);
}

void showRobotWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}

int showRobotWindow::main()
{
    viewer->start(this);
    return 0;
}

void showRobotWindow::quit()
{
    std::cout << "CShowRobotWindow: Closing" << std::endl;
    this->close();
    viewer->stop();
}

void showRobotWindow::updateJointBox()
{
    UI.comboBoxJoint->clear();

    for (unsigned int i = 0; i < currentRobotNodes.size(); i++)
    {
        UI.comboBoxJoint->addItem(QString(currentRobotNodes[i]->getName().c_str()));
    }
}

void showRobotWindow::updateRNSBox()
{
    UI.comboBoxRobotNodeSet->clear();
    UI.comboBoxRobotNodeSet->addItem(QString("<All>"));

    for (unsigned int i = 0; i < robotNodeSets.size(); i++)
    {
        UI.comboBoxRobotNodeSet->addItem(QString(robotNodeSets[i]->getName().c_str()));
    }
}

void showRobotWindow::selectRNS(int nr)
{
    currentRobotNodeSet.reset();
    cout << "Selecting RNS nr " << nr << endl;

    if (nr <= 0)
    {
        // all joints
        currentRobotNodes = allRobotNodes;
    }
    else
    {
        nr--;

        if (nr >= (int)robotNodeSets.size())
        {
            return;
        }

        currentRobotNodeSet = robotNodeSets[nr];
        currentRobotNodes = currentRobotNodeSet->getAllRobotNodes();
        /*cout << "HIGHLIGHTING rns " << currentRobotNodeSet->getName() << endl;
        if (visualization)
        {

            robot->highlight(visualization,false);
            currentRobotNodeSet->highlight(visualization,true);
        }*/

    }

    updateJointBox();
    selectJoint(0);
    displayTriangles();
}

void showRobotWindow::selectJoint(int nr)
{
    if (currentRobotNode)
    {
        currentRobotNode->showBoundingBox(false);
    }

    currentRobotNode.reset();
    cout << "Selecting Joint nr " << nr << endl;

    if (nr < 0 || nr >= (int)currentRobotNodes.size())
    {
        return;
    }

    currentRobotNode = currentRobotNodes[nr];
    currentRobotNode->showBoundingBox(true, true);
    currentRobotNode->print();
    float mi = currentRobotNode->getJointLimitLo();
    float ma = currentRobotNode->getJointLimitHi();
    QString qMin = QString::number(mi);
    QString qMax = QString::number(ma);
    UI.labelMinPos->setText(qMin);
    UI.labelMaxPos->setText(qMax);
    float j = currentRobotNode->getJointValue();
    UI.lcdNumberJointValue->display((double)j);

    if (fabs(ma - mi) > 0 && (currentRobotNode->isTranslationalJoint() || currentRobotNode->isRotationalJoint()))
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

    if (currentRobotNodes[nr]->showCoordinateSystemState())
    {
        UI.checkBoxShowCoordSystem->setCheckState(Qt::Checked);
    }
    else
    {
        UI.checkBoxShowCoordSystem->setCheckState(Qt::Unchecked);
    }

    cout << "HIGHLIGHTING node " << currentRobotNodes[nr]->getName() << endl;

    if (visualization)
    {
        robot->highlight(visualization, false);
        currentRobotNode->highlight(visualization, true);
    }

    displayTriangles();
}

void showRobotWindow::jointValueChanged(int pos)
{
    int nr = UI.comboBoxJoint->currentIndex();

    if (nr < 0 || nr >= (int)currentRobotNodes.size())
    {
        return;
    }

    float fPos = currentRobotNodes[nr]->getJointLimitLo() + (float)pos / 1000.0f * (currentRobotNodes[nr]->getJointLimitHi() - currentRobotNodes[nr]->getJointLimitLo());
    robot->setJointValue(currentRobotNodes[nr], fPos);
    UI.lcdNumberJointValue->display((double)fPos);

#if 0
    RobotNodePtr rnl = robot->getRobotNode("LeftLeg_TCP");
    RobotNodePtr rnr = robot->getRobotNode("RightLeg_TCP");

    if (rnl && rnr)
    {
        cout << "LEFT:" << endl;
        MathTools::printMat(rnl->getGlobalPose());
        cout << "RIGHT:" << endl;
        MathTools::printMat(rnr->getGlobalPose());
    }

#endif
}

void showRobotWindow::showCoordSystem()
{
    float size = 0.75f;
    int nr = UI.comboBoxJoint->currentIndex();

    if (nr < 0 || nr >= (int)currentRobotNodes.size())
    {
        return;
    }

    // first check if robot node has a visualization
    /*VisualizationNodePtr visu = robotNodes[nr]->getVisualization();
    if (!visu)
    {
        // create dummy visu
        SoSeparator *s = new SoSeparator();
        VisualizationNodePtr visualizationNode(new CoinVisualizationNode(s));
        robotNodes[nr]->setVisualization(visualizationNode);
        //visualizationNode->showCoordinateSystem(UI.checkBoxShowCoordSystem->checkState() == Qt::Checked, size);

    }*/

    currentRobotNodes[nr]->showCoordinateSystem(UI.checkBoxShowCoordSystem->checkState() == Qt::Checked, size);
    // rebuild visualization
    rebuildVisualization();
}



void showRobotWindow::selectRobot()
{
    string supportedExtensions = RobotImporterFactory::getAllExtensions();
    string supported = "Supported Formats, " + supportedExtensions + " (" + supportedExtensions + ")";
    string filter = supported + ";;" + RobotImporterFactory::getAllFileFilters();
    QString fi = QFileDialog::getOpenFileName(this, tr("Open Robot File"), QString(), tr(filter.c_str()));
    std::string s = m_sRobotFilename = std::string(fi.toLatin1());

    if (!s.empty())
    {
        m_sRobotFilename = s;
        loadRobot();
    }
}

void showRobotWindow::testPerformance(RobotPtr robot, RobotNodeSetPtr rns)
{
    int loops = 10000;
    Eigen::VectorXf limitMin(rns->getSize());
    Eigen::VectorXf limitMax(rns->getSize());
    for (size_t i = 0; i < rns->getSize(); i++)
    {
        limitMin[i] = rns->getNode(i)->getJointLimitLo();
        limitMax[i] = rns->getNode(i)->getJointLimitHi();
    }
    Eigen::VectorXf v(rns->getSize());
    //float minV = rn->getJointLimitLo();
    //float maxV = rn->getJointLimitHi();

    clock_t start = clock();
    robot->setupVisualization(true, false);
    robot->setUpdateVisualization(true);
    robot->setUpdateCollisionModel(true);
    robot->setThreadsafe(true);
    for (int i = 0; i < loops; i++)
    {
        for (size_t k = 0; k < rns->getSize(); k++)
        {
            float p = float(rand() % 1000) / 1000.0f;
            v[k] = limitMin[k] + p * (limitMax[k] - limitMin[k]);
        }
        rns->setJointValues(v);
    }
    clock_t end = clock();
    float timeMS = (float)(end - start) / (float)CLOCKS_PER_SEC * 1000.0f;
    VR_INFO << "Time (visu on, thread on): " << timeMS / (float)loops << endl;

    start = clock();
    robot->setupVisualization(false, false);
    robot->setUpdateVisualization(false);
    robot->setUpdateCollisionModel(false);
    robot->setThreadsafe(true);
    for (int i = 0; i < loops; i++)
    {
        /*float v = float(rand() % 1000) / 1000.0f;
        v = minV + v * (maxV - minV);
        rn->setJointValue(v);*/
        for (size_t k = 0; k < rns->getSize(); k++)
        {
            float p = float(rand() % 1000) / 1000.0f;
            v[k] = limitMin[k] + p * (limitMax[k] - limitMin[k]);
        }
        rns->setJointValues(v);
    }
    end = clock();
    timeMS = (float)(end - start) / (float)CLOCKS_PER_SEC * 1000.0f;
    VR_INFO << "Time (visu off, thread on): " << timeMS / (float)loops << endl;

    start = clock();
    robot->setupVisualization(true, false);
    robot->setUpdateVisualization(true);
    robot->setUpdateCollisionModel(true);
    robot->setThreadsafe(false);
    for (int i = 0; i < loops; i++)
    {
        for (size_t k = 0; k < rns->getSize(); k++)
        {
            float p = float(rand() % 1000) / 1000.0f;
            v[k] = limitMin[k] + p * (limitMax[k] - limitMin[k]);
        }
        rns->setJointValues(v);
    }
    end = clock();
    timeMS = (float)(end - start) / (float)CLOCKS_PER_SEC * 1000.0f;
    VR_INFO << "Time (visu on, thread off): " << timeMS / (float)loops << endl;


    start = clock();
    robot->setupVisualization(false, false);
    robot->setUpdateVisualization(false);
    robot->setUpdateCollisionModel(false);
    robot->setThreadsafe(false);
    for (int i = 0; i < loops; i++)
    {
        for (size_t k = 0; k < rns->getSize(); k++)
        {
            float p = float(rand() % 1000) / 1000.0f;
            v[k] = limitMin[k] + p * (limitMax[k] - limitMin[k]);
        }
        rns->setJointValues(v);
    }
    end = clock();
    timeMS = (float)(end - start) / (float)CLOCKS_PER_SEC * 1000.0f;
    VR_INFO << "Time (visu off, thread off): " << timeMS / (float)loops << endl;

}

void showRobotWindow::loadRobot()
{
    viewer->clearLayer("robotLayer");

    cout << "Loading Robot from " << m_sRobotFilename << endl;
    currentEEF.reset();
    currentRobotNode.reset();
    currentRobotNodes.clear();
    currentRobotNodeSet.reset();
    robot.reset();

    try
    {
        QFileInfo fileInfo(m_sRobotFilename.c_str());
        std::string suffix(fileInfo.suffix().toLatin1());
        RobotImporterFactoryPtr importer = RobotImporterFactory::fromFileExtension(suffix, NULL);

        if (!importer)
        {
            cout << " ERROR while grabbing importer" << endl;
            return;
        }

        robot = importer->loadFromFile(m_sRobotFilename, RobotIO::eFull);


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

    // just a simple test that inverts the kinematic structure of the robot
#if 0 
    if (robot->hasRobotNode("Index L J1"))
        robot = RobotFactory::cloneInversed(robot, "Index L J1");
#endif

#if 0
    if (robot->hasRobotNodeSet("LeftArm"))
        robot = RobotFactory::cloneSubSet(robot, robot->getRobotNodeSet("LeftArm"), "LeftArmRobot");
#endif

#if 0
    if (robot->hasRobotNode("Wrist 2 L") && robot->hasRobotNode("Wrist 2 R"))
    {
        std::vector<std::string> l;
        l.push_back("Wrist 2 L");
        l.push_back("Wrist 2 R");
        robot = RobotFactory::cloneUniteSubsets(robot, "RobotWithUnitedHands", l);
    }
    VR_INFO << "=========== PERFORMANCE orig ============" << endl;
    testPerformance(robot, robot->getRobotNodeSet("Joints_Revolute"));
    if (robot->hasRobotNode("LWy_joint") && robot->hasRobotNode("RWy_joint"))
    {
        std::vector<std::string> l;
        l.push_back("LWy_joint");
        l.push_back("RWy_joint");
        robot = RobotFactory::cloneUniteSubsets(robot, "RobotWithUnitedHands", l);
    }
    VR_INFO << "=========== PERFORMANCE clone ============" << endl;
    testPerformance(robot, robot->getRobotNodeSet("Joints_Revolute"));// ("BPy_joint"));
#endif

    updatRobotInfo();
}

void showRobotWindow::updatRobotInfo()
{
    if (!robot)
    {
        return;
    }

    UI.checkBoxColModel->setChecked(false);
    UI.checkBoxFullModel->setChecked(true);
    UI.checkBoxPhysicsCoM->setChecked(false);
    UI.checkBoxPhysicsInertia->setChecked(false);
    UI.checkBoxRobotCoordSystems->setChecked(false);
    UI.checkBoxShowCoordSystem->setChecked(false);
    UI.checkBoxStructure->setChecked(false);

    // get nodes
    robot->getRobotNodes(allRobotNodes);
    robot->getRobotNodeSets(robotNodeSets);
    robot->getEndEffectors(eefs);
    updateEEFBox();
    updateRNSBox();
    selectRNS(0);

    if (allRobotNodes.size() == 0)
    {
        selectJoint(-1);
    }
    else
    {
        selectJoint(0);
    }

    if (eefs.size() == 0)
    {
        selectEEF(-1);
    }
    else
    {
        selectEEF(0);
    }

    displayTriangles();

#if 0
    RobotPtr r2 = robot->clone(robot->getName(), robot->getCollisionChecker(), 2.0f);
    Eigen::Matrix4f gp = Eigen::Matrix4f::Identity();
    gp(0, 3) += 1000.0f;

    r2->setGlobalPose(gp);
    extraSep->removeAllChildren();
    useColModel = UI.checkBoxColModel->checkState() == Qt::Checked;
    SceneObject::VisualizationType colModel = (UI.checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;

    boost::shared_ptr<VirtualRobot::CoinVisualization> visualization = r2->getVisualization<CoinVisualization>(colModel);
    SoNode* visualisationNode = NULL;

    if (visualization)
    {
        visualisationNode = visualization->getCoinVisualization();
    }

    extraSep->addChild(visualisationNode);
#endif

    // build visualization
    rebuildVisualization();
    robotStructure();
    displayPhysics();
    viewer->viewAll();
}

void showRobotWindow::robotStructure()
{
    if (!robot)
    {
        return;
    }

    structureEnabled = UI.checkBoxStructure->checkState() == Qt::Checked;
    robot->showStructure(structureEnabled);
    // rebuild visualization
    rebuildVisualization();
}

void showRobotWindow::robotCoordSystems()
{
    if (!robot)
    {
        return;
    }

    bool robotAllCoordsEnabled = UI.checkBoxRobotCoordSystems->checkState() == Qt::Checked;
    robot->showCoordinateSystems(robotAllCoordsEnabled);
    // rebuild visualization
    rebuildVisualization();
}

void showRobotWindow::closeHand()
{
    if (currentEEF)
    {
        currentEEF->closeActors();
    }
}

void showRobotWindow::openHand()
{
#if 0

    if (robot)
    {
        float randMult = (float)(1.0 / (double)(RAND_MAX));
        std::vector<RobotNodePtr> rn = robot->getRobotNodes();
        std::vector<RobotNodePtr> rnJoints;

        for (size_t j = 0; j < rn.size(); j++)
        {
            if (rn[j]->isRotationalJoint())
            {
                rnJoints.push_back(rn[j]);
            }
        }

        int loops = 10000;
        clock_t startT = clock();

        for (int i = 0; i < loops; i++)
        {
            std::vector<float> jv;

            for (size_t j = 0; j < rnJoints.size(); j++)
            {
                float t = (float)rand() * randMult; // value from 0 to 1
                t = rnJoints[j]->getJointLimitLo() + (rnJoints[j]->getJointLimitHi() - rnJoints[j]->getJointLimitLo()) * t;
                jv.push_back(t);
            }

            robot->setJointValues(rnJoints, jv);
        }

        clock_t endT = clock();

        float diffClock = (float)(((float)(endT - startT) / (float)CLOCKS_PER_SEC) * 1000.0f);
        cout << "RobotNodes:" << rn.size() << endl;
        cout << "Joints:" << rnJoints.size() << endl;
        cout << "loops:" << loops << ". time (ms):" << diffClock << ". Per loop:" << diffClock / (float)loops << endl;
    }

#endif

    if (currentEEF)
    {
        currentEEF->openActors();
    }
}

void showRobotWindow::selectEEF(int nr)
{
    cout << "Selecting EEF nr " << nr << endl;

    if (nr < 0 || nr >= (int)eefs.size())
    {
        return;
    }

    currentEEF = eefs[nr];
}

void showRobotWindow::updateEEFBox()
{
    UI.comboBoxEndEffector->clear();

    for (unsigned int i = 0; i < eefs.size(); i++)
    {
        UI.comboBoxEndEffector->addItem(QString(eefs[i]->getName().c_str()));
    }
}
