
#include "reachabilityWindow.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/Workspace/Reachability.h"
#include "VirtualRobot/Workspace/Manipulability.h"
#include "VirtualRobot/IK/PoseQualityExtendedManipulability.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h"
#include <VirtualRobot/Tools/RuntimeEnvironment.h>
#include <VirtualRobot/Model/Nodes/ModelJoint.h>
#include <Gui/ViewerFactory.h>

#include <QFileDialog>
#include <Eigen/Geometry>

#include <time.h>
#include <vector>
#include <iostream>
#include <cmath>

#include "ui_reachabilityCreate.h"

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

reachabilityWindow::reachabilityWindow(std::string& sRobotFile, std::string& reachFile, Eigen::Vector3f& axisTCP)
    : QMainWindow(NULL), robotVisuLayerName("robot-layer"), wsVisuLayerName("ws-layer")
{
    VR_INFO << " start " << endl;

    this->axisTCP = axisTCP;
    robotFile = sRobotFile;
    useColModel = false;

    setupUI();

    loadRobot();

    if (!reachFile.empty())
    {
        if (RuntimeEnvironment::getDataFileAbsolute(reachFile))
        {
            loadReachFile(reachFile);
        }
    }

    viewer->viewAll();
}


reachabilityWindow::~reachabilityWindow()
{
}


void reachabilityWindow::setupUI()
{
    UI.setupUi(this);
    // viewerfactories and visualizationfactories of the same type share the same name (e.g. "inventor" for coin).
    SimoxGui::ViewerFactoryPtr viewerFactory = SimoxGui::ViewerFactory::fromName(VisualizationFactory::getGlobalVisualizationFactory()->getVisualizationType(), NULL);

    viewer = viewerFactory->createViewer(UI.frameViewer);
    viewer->viewAll();

    connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
    connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(selectRobot()));
    connect(UI.pushButtonLoadReachability, SIGNAL(clicked()), this, SLOT(loadReach()));
    connect(UI.pushButtonCreateReachability, SIGNAL(clicked()), this, SLOT(createReach()));
    connect(UI.pushButtonExtendReachability, SIGNAL(clicked()), this, SLOT(extendReach()));
    connect(UI.pushButtonSaveReachability, SIGNAL(clicked()), this, SLOT(saveReach()));
    connect(UI.pushButtonFillHoles, SIGNAL(clicked()), this, SLOT(fillHoles()));
    connect(UI.pushButtonBinarize, SIGNAL(clicked()), this, SLOT(binarize()));
    connect(UI.pushButtonVolume, SIGNAL(clicked()), this, SLOT(computeVolume()));

    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(collisionModel()));
    connect(UI.checkBoxReachabilityVisu, SIGNAL(clicked()), this, SLOT(reachVisu()));
    connect(UI.checkBoxReachabilityCut, SIGNAL(clicked()), this, SLOT(reachVisu()));
    connect(UI.comboBoxRNS, SIGNAL(activated(int)), this, SLOT(selectRNS(int)));

    connect(UI.comboBoxJoint, SIGNAL(activated(int)), this, SLOT(selectJoint(int)));
    connect(UI.horizontalSliderPos, SIGNAL(valueChanged(int)), this, SLOT(jointValueChanged(int)));
    connect(UI.sliderCutReach, SIGNAL(sliderReleased()), this, SLOT(reachVisu()));

    UI.sliderCutReach->setValue(50);
    UI.sliderCutReach->setEnabled(false);
    UI.checkBoxReachabilityCut->setEnabled(false);
}

QString reachabilityWindow::formatString(const char* s, float f)
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


void reachabilityWindow::resetSceneryAll()
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



void reachabilityWindow::collisionModel()
{
    if (!robot)
    {
        return;
    }

    buildVisu();
}

void reachabilityWindow::reachVisu()
{
    if (!robot || !reachSpace)
    {
        return;
    }

    viewer->clearLayer(wsVisuLayerName);
    VisualizationNodePtr wsVisuNode;

    if (UI.checkBoxReachabilityVisu->checkState() == Qt::Checked)
    {

        UI.checkBoxReachabilityCut->setEnabled(true);
        float heightPercent = 1.0f;
        if (UI.checkBoxReachabilityCut->checkState() == Qt::Checked)
        {
            // TODO workspace cut visualization currently not supported by the Visualizationfactory interface (only Coin)

//            UI.sliderCutReach->setEnabled(true);
//            int dist = UI.sliderCutReach->maximum() - UI.sliderCutReach->minimum();
//            float pos = (float)(UI.sliderCutReach->value() - UI.sliderCutReach->minimum()) / (float)dist;
//            heightPercent = pos;

//            WorkspaceRepresentation::WorkspaceCut2DPtr cutData = reachSpace->createCut(pos,reachSpace->getDiscretizeParameterTranslation(), true);
//            VR_INFO << "Slider pos: " << pos  << ", maxEntry:" << reachSpace->getMaxSummedAngleReachablity() << ", cut maxCoeff:" << cutData->entries.maxCoeff() << endl;
//            SoNode *reachvisu2 = CoinVisualizationFactory::getCoinVisualization(cutData, VirtualRobot::ColorMap(VirtualRobot::ColorMap::eHot), Eigen::Vector3f::UnitZ(), reachSpace->getMaxSummedAngleReachablity());
//            visualisationNode->addChild(reachvisu2);

        } else
        {
            UI.sliderCutReach->setEnabled(false);
        }

        Eigen::Vector3f minBB, maxBB;
        reachSpace->getWorkspaceExtends(minBB, maxBB);
        float zDist = maxBB(2) - minBB(2);
        float maxZ =  minBB(2) + heightPercent*zDist - reachSpace->getDiscretizeParameterTranslation();
        wsVisuNode = VisualizationFactory::getGlobalVisualizationFactory()->createReachabilityVisualization(reachSpace, ColorMapPtr(new ColorMap(VirtualRobot::ColorMap::eHot)), true, maxZ);

    } else
    {
        UI.sliderCutReach->setEnabled(false);
        UI.checkBoxReachabilityCut->setEnabled(false);
    }
    if (wsVisuNode)
    {
        viewer->addVisualization(wsVisuLayerName, "ws-node", wsVisuNode);
    }
}

void reachabilityWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}


void reachabilityWindow::buildVisu()
{
    if (!robot)
    {
        return;
    }

    viewer->clearLayer(robotVisuLayerName);
    useColModel = UI.checkBoxColModel->checkState() == Qt::Checked;
    ModelLink::VisualizationType visuType = (UI.checkBoxColModel->isChecked()) ? ModelLink::VisualizationType::Collision : ModelLink::VisualizationType::Full;

    visualization = robot->getVisualization(visuType);


    if (visualization)
    {
        viewer->addVisualization(robotVisuLayerName, "robot-node", visualization);
    }
}

int reachabilityWindow::main()
{
    viewer->start(this);
    return 0;
}


void reachabilityWindow::quit()
{
    std::cout << "reachabilityWindow: Closing" << std::endl;
    this->close();
    viewer->stop();
}

void reachabilityWindow::updateRNSBox()
{
    UI.comboBoxRNS->clear();

    //UI.comboBoxRNS->addItem(QString("<All>"));
    for (unsigned int i = 0; i < robotNodeSets.size(); i++)
    {
        UI.comboBoxRNS->addItem(QString(robotNodeSets[i]->getName().c_str()));
    }

    selectRNS(0);
}

void reachabilityWindow::selectRNS(int nr)
{
    currentRobotNodeSet.reset();
    cout << "Selecting RNS nr " << nr << endl;
    std::string tcp = "<not set>";

    if (nr < 0 || nr >= (int)robotNodeSets.size())
    {
        return;
    }

    currentRobotNodeSet = robotNodeSets[nr];
    currentRobotNodes = currentRobotNodeSet->getJoints();

    if (currentRobotNodeSet->getTCP())
    {
        tcp = currentRobotNodeSet->getTCP()->getName();
    }

    QString qTCP("TCP: ");
    qTCP += tcp.c_str();
    UI.labelTCP->setText(qTCP);

    updateQualityInfo();

    //updateJointBox();
    //selectJoint(0);
    //displayTriangles();
}

void reachabilityWindow::updateJointBox()
{
    UI.comboBoxJoint->clear();

    for (unsigned int i = 0; i < allRobotNodes.size(); i++)
    {
        UI.comboBoxJoint->addItem(QString(allRobotNodes[i]->getName().c_str()));
    }

    selectJoint(0);
}

void reachabilityWindow::jointValueChanged(int pos)
{
    int nr = UI.comboBoxJoint->currentIndex();

    if (nr < 0 || nr >= (int)allRobotNodes.size())
    {
        return;
    }

    float lowerLimit = 0;
    float upperLimit = 0;
    if (allRobotNodes[nr]->isJoint())
    {
        lowerLimit = std::static_pointer_cast<ModelJoint>(allRobotNodes[nr])->getJointLimitLow();
        upperLimit = std::static_pointer_cast<ModelJoint>(allRobotNodes[nr])->getJointLimitHigh();
    }

    float fPos = lowerLimit + (float)pos / 1000.0f * (upperLimit - lowerLimit);
    robot->setJointValue(allRobotNodes[nr], fPos);
    UI.lcdNumberJointValue->display((double)fPos);

    updateQualityInfo();
}


void reachabilityWindow::selectJoint(int nr)
{
    currentRobotNode.reset();
    cout << "Selecting Joint nr " << nr << endl;

    if (nr < 0 || nr >= (int)allRobotNodes.size())
    {
        return;
    }

    currentRobotNode = allRobotNodes[nr];
    currentRobotNode->print();
    float mi = 0;
    float ma = 0;
    float j = 0;

    if (currentRobotNode->isJoint())
    {
        mi = std::static_pointer_cast<ModelJoint>(currentRobotNode)->getJointLimitLow();
        ma = std::static_pointer_cast<ModelJoint>(currentRobotNode)->getJointLimitHigh();
        j = std::static_pointer_cast<ModelJoint>(currentRobotNode)->getJointValue();
    }

    QString qMin = QString::number(mi);
    QString qMax = QString::number(ma);
    UI.labelMinPos->setText(qMin);
    UI.labelMaxPos->setText(qMax);
    UI.lcdNumberJointValue->display((double)j);

    if (fabs(ma - mi) > 0 && (currentRobotNode->isJoint()))
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
    updateQualityInfo();
}

/*
void reachabilityWindow::showCoordSystem()
{
    float size = 0.75f;
    int nr = UI.comboBoxJoint->currentIndex();
    if (nr<0 || nr>=(int)currentRobotNodes.size())
        return;

    // first check if robot node has a visualization


    currentRobotNodes[nr]->showCoordinateSystem(UI.checkBoxShowCoordSystem->checkState() == Qt::Checked, size);
    // rebuild visualization
    collisionModel();
}

*/

void reachabilityWindow::selectRobot()
{
    QString fi = QFileDialog::getOpenFileName(this, tr("Open Robot File"), QString(), tr("XML Files (*.xml)"));
    robotFile = std::string(fi.toLatin1());
    loadRobot();
}

void reachabilityWindow::loadRobot()
{
    viewer->clearLayer(robotVisuLayerName);
    cout << "Loading Scene from " << robotFile << endl;

    try
    {
        robot = ModelIO::loadModel(robotFile);
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
    allRobotNodes = robot->getJoints();
    std::vector < VirtualRobot::JointSetPtr > allRNS = robot->getJointSets();
    robotNodeSets.clear();

    for (size_t i = 0; i < allRNS.size(); i++)
    {
        // ModelNodeSet::isKinematicChain() is not supported at the time of writing.
        if (/*allRNS[i]->isKinematicChain()*/ true)
        {
            VR_INFO << " RNS <" << allRNS[i]->getName() << "> is a valid kinematic chain" << endl;
            robotNodeSets.push_back(allRNS[i]);
        }
        else
        {
            VR_INFO << " RNS <" << allRNS[i]->getName() << "> is not a valid kinematic chain" << endl;
        }
    }

    updateRNSBox();
    updateJointBox();

    updateQualityInfo();

    // build visualization
    buildVisu();
    viewer->viewAll();
}
void reachabilityWindow::extendReach()
{
    if (!robot)
    {
        return;
    }

    if (!reachSpace)
    {
        cout << " Please load/create reachability data first..." << endl;
        return;
    }

    int steps = UI.spinBoxExtend->value();

#if 0
    ManipulabilityPtr manipSpace = std::dynamic_pointer_cast<Manipulability>(reachSpace);
    if (manipSpace && manipSpace->getManipulabilityMeasure())
    {
        manipSpace->getManipulabilityMeasure()->setVerbose(true);
    }
#endif
    //reachSpace->addRandomTCPPoses(steps, 1, true);
    reachSpace->addRandomTCPPoses(steps, QThread::idealThreadCount() < 1 ? 1 : QThread::idealThreadCount(), true);

    reachSpace->print();
    UI.checkBoxReachabilityVisu->setChecked(false);
    UI.sliderCutReach->setEnabled(false);
    UI.checkBoxReachabilityCut->setEnabled(false);
    viewer->clearLayer(wsVisuLayerName);
}

void reachabilityWindow::createReach()
{
    if (!robot || !currentRobotNodeSet)
    {
        return;
    }

    // setup window
    Ui::ReachabilityCreate UICreate;
    QDialog diag;
    UICreate.setupUi(&diag);
    RobotNodePtr baseNode = currentRobotNodeSet->getKinematicRoot();
    FramePtr tcpNode = currentRobotNodeSet->getTCP();
    UICreate.labelRNS->setText(QString("RobotNodeSet: ") + QString(currentRobotNodeSet->getName().c_str()));
    UICreate.labelBaseNode->setText(QString("Base: ") + QString(baseNode->getName().c_str()));
    UICreate.labelTCP->setText(QString("TCP: ") + QString(tcpNode->getName().c_str()));
    ReachabilityPtr reachSpaceTest(new Reachability(robot));
    float minB[6];// = {-1000.0f,-1000.0f,-1000.0f,(float)-M_PI,(float)-M_PI,(float)-M_PI};
    float maxB[6];// ={1000.0f,1000.0f,1000.0f,(float)M_PI,(float)M_PI,(float)M_PI};

    JointSetPtr currentJointSet = JointSet::createJointSet(currentRobotNodeSet->getModel(), currentRobotNodeSet->getName(),
                             currentRobotNodeSet->getModelJoints(), currentRobotNodeSet->getKinematicRoot(), currentRobotNodeSet->getTCP());
    reachSpaceTest->checkForParameters(currentJointSet, 1000, minB, maxB, baseNode, tcpNode);

    //float ex = currentRobotNodeSet->getMaximumExtension();
    UICreate.doubleSpinBoxMinX->setValue(minB[0]);
    UICreate.doubleSpinBoxMaxX->setValue(maxB[0]);
    UICreate.doubleSpinBoxMinY->setValue(minB[1]);
    UICreate.doubleSpinBoxMaxY->setValue(maxB[1]);
    UICreate.doubleSpinBoxMinZ->setValue(minB[2]);
    UICreate.doubleSpinBoxMaxZ->setValue(maxB[2]);


    std::vector < VirtualRobot::RobotNodeSetPtr > allRNS = robot->getModelNodeSets();

    for (size_t i = 0; i < allRNS.size(); i++)
    {
        UICreate.comboBoxColModelDynamic->addItem(QString(allRNS[i]->getName().c_str()));
        UICreate.comboBoxColModelStatic->addItem(QString(allRNS[i]->getName().c_str()));
    }

    UICreate.comboBoxQualityMeasure->addItem(QString("Reachability"));
    UICreate.comboBoxQualityMeasure->addItem(QString("Manipulability"));
    UICreate.comboBoxQualityMeasure->addItem(QString("Ext. Manipulability"));

    if (diag.exec())
    {
        reachSpace.reset(new Reachability(robot));

        minB[0] = UICreate.doubleSpinBoxMinX->value();
        minB[1] = UICreate.doubleSpinBoxMinY->value();
        minB[2] = UICreate.doubleSpinBoxMinZ->value();
        minB[3] = UICreate.doubleSpinBoxMinRo->value();
        minB[4] = UICreate.doubleSpinBoxMinPi->value();
        minB[5] = UICreate.doubleSpinBoxMinYa->value();
        maxB[0] = UICreate.doubleSpinBoxMaxX->value();
        maxB[1] = UICreate.doubleSpinBoxMaxY->value();
        maxB[2] = UICreate.doubleSpinBoxMaxZ->value();
        maxB[3] = UICreate.doubleSpinBoxMaxRo->value();
        maxB[4] = UICreate.doubleSpinBoxMaxPi->value();
        maxB[5] = UICreate.doubleSpinBoxMaxYa->value();

        LinkSetPtr staticModel;
        LinkSetPtr dynamicModel;

        if (UICreate.checkBoxColDetecion->isChecked())
        {
            std::string staticM = std::string(UICreate.comboBoxColModelStatic->currentText().toLatin1());
            std::string dynM = std::string(UICreate.comboBoxColModelDynamic->currentText().toLatin1());
            staticModel = robot->getLinkSet(staticM);
            dynamicModel = robot->getLinkSet(dynM);
        }

        float discrTr = UICreate.doubleSpinBoxDiscrTrans->value();
        float discrRo = UICreate.doubleSpinBoxDiscrRot->value();

        std::string measure = std::string(UICreate.comboBoxQualityMeasure->currentText().toLatin1());

        if (measure != "Reachability")
        {
            reachSpace.reset(new Manipulability(robot));
            ManipulabilityPtr manipSpace = std::dynamic_pointer_cast<Manipulability>(reachSpace);
            manipSpace->setMaxManipulability(UICreate.doubleSpinBoxMaxManip->value());
        }

        reachSpace->initialize(currentJointSet, discrTr, discrRo, minB, maxB, staticModel, dynamicModel, baseNode, tcpNode); //200.0f,0.4f,minB,maxB,staticModel,dynamicModel,baseNode);

        if (measure == "Ext. Manipulability")
        {
            ManipulabilityPtr man = std::dynamic_pointer_cast<Manipulability>(reachSpace);
            PoseQualityExtendedManipulabilityPtr manMeasure(new PoseQualityExtendedManipulability(currentJointSet));
            man->setManipulabilityMeasure(manMeasure);
            if (UICreate.checkBoxColDetecion->isChecked() && UICreate.checkBoxSelfDistance->isChecked())
            {
                std::string staticM = std::string(UICreate.comboBoxColModelStatic->currentText().toLatin1());
                std::string dynM = std::string(UICreate.comboBoxColModelDynamic->currentText().toLatin1());
                LinkSetPtr m1 = robot->getLinkSet(staticM);
                LinkSetPtr m2 = robot->getLinkSet(dynM);
                man->initSelfDistanceCheck(m1, m2);
                manMeasure->considerObstacles(true, UICreate.doubleSpinBoxSelfDistA->value(), UICreate.doubleSpinBoxSelfDistB->value());
            }
        }

        reachSpace->print();

        reachSpace->addCurrentTCPPose();
        reachSpace->print();
    }
}

void reachabilityWindow::fillHoles()
{
    if (!robot || !reachSpace)
    {
        return;
    }

    cout << "filling holes of reachability space" << endl;
    int res = reachSpace->fillHoles();
    cout << "Filled " << res << " voxels" << endl;
    reachSpace->print();
}

void reachabilityWindow::binarize()
{
    if (!robot || !reachSpace)
    {
        return;
    }

    cout << "Binarizing reachability space" << endl;
    reachSpace->binarize();
    reachSpace->print();
}

void reachabilityWindow::computeVolume()
{
    if (!reachSpace)
        return;

    VirtualRobot::WorkspaceRepresentation::VolumeInfo vi;
    vi = reachSpace->computeVolumeInformation();

    cout << "Reachability Volume Information:" << endl;
    cout << "Nr 3d Voxels:" << vi.voxelCount3D << endl;
    cout << "Nr filled 3d Voxels:" << vi.filledVoxelCount3D << endl;
    cout << "Nr border 3d Voxels:" << vi.borderVoxelCount3D << endl;
    cout << "Volume per 3d Voxel:" << vi.volumeVoxel3D << " m^3" << endl;
    cout << "Volume of all filled 3d Voxels:" << vi.volumeFilledVoxels3D << " m^3" << endl;
    cout << "Volume of filledVoxels - borderVoxels*0.5:" << vi.volume3D << " m^3" << endl;
}

void reachabilityWindow::saveReach()
{
    if (!robot || !reachSpace)
    {
        return;
    }

    QString fi = QFileDialog::getSaveFileName(this, tr("Save Reachability to File"), QString(), tr("bin Files (*.bin);;all Files (*.*)"));

    if (fi.isEmpty())
    {
        return;
    }

    reachFile = std::string(fi.toLatin1());
    reachSpace->save(reachFile);

}
void reachabilityWindow::loadReachFile(std::string filename)
{
    if (!robot)
    {
        return;
    }

    reachFile = filename;
    bool loadOK = true;

    // try manipulability file
    try
    {
        reachSpace.reset(new Manipulability(robot));
        reachSpace->load(reachFile);
    }
    catch (...)
    {
        loadOK = false;
    }

    if (!loadOK)
    {
        // try reachability file

        loadOK = true;

        try
        {
            reachSpace.reset(new Reachability(robot));
            reachSpace->load(reachFile);
        }
        catch (...)
        {
            loadOK = false;
        }
    }

    if (!loadOK)
    {
        VR_ERROR << "Could not load reach/manip file" << endl;
        reachSpace.reset();
        return;
    }

    reachSpace->print();

    if (reachSpace->getJointSet())
    {
        cout << "Using RNS: " << reachSpace->getJointSet()->getName() << endl;

        for (size_t i = 0; i < robotNodeSets.size(); i++)
        {
            cout << "checking " << robotNodeSets[i]->getName() << endl;

            if (robotNodeSets[i] == reachSpace->getJointSet())
            {
                cout << "Found RNS.." << endl;
                UI.comboBoxRNS->setCurrentIndex(i);
                selectRNS(i);
            }
        }
    }
}

void reachabilityWindow::loadReach()
{
    if (!robot)
    {
        return;
    }

    QString fi = QFileDialog::getOpenFileName(this, tr("Open Reachability File"), QString(), tr("bin Files (*.bin);;all Files (*.*)"));

    if (fi.isEmpty())
    {
        return;
    }

    reachFile = std::string(fi.toLatin1());
    loadReachFile(reachFile);
}


void reachabilityWindow::updateQualityInfo()
{
    if (!currentRobotNodeSet)
        return;



    std::stringstream ss;
    std::stringstream ss2;
    std::stringstream ss3;
    std::stringstream ss4;
    PoseQualityManipulabilityPtr manMeasure(new PoseQualityManipulability(currentRobotNodeSet));
    PoseQualityExtendedManipulabilityPtr extManMeasure(new PoseQualityExtendedManipulability(currentRobotNodeSet));
    float manip = manMeasure->getPoseQuality();
    float extManip = extManMeasure->getPoseQuality();
    ss << "Manipulability: " << manip;
    std::string manipString = ss.str();
    UI.labelManip->setText(manipString.c_str());

    ss2 << "Ext. Manipulability: " << extManip;
    manipString = ss2.str();
    UI.labelExtManip->setText(manipString.c_str());

    float reachManip = 1.0f;
    float poseManip = 1.0f;
    if (reachSpace)
    {
        ManipulabilityPtr p = std::dynamic_pointer_cast<Manipulability>(reachSpace);
        if (p)
        {
            reachManip = p->getManipulabilityAtPose(p->getTCP()->getGlobalPose());
            poseManip = p->measureCurrentPose();
        } else
        {
            if (reachSpace->getEntry(p->getTCP()->getGlobalPose())>0)
                reachManip = 1.0f;
            else
                reachManip = 0.0f;
        }
    }
    ss3 << "Quality in Reach Data: " << reachManip;
    manipString = ss3.str();
    UI.labelWSData->setText(manipString.c_str());

    ss4 << "Quality at pose: " << poseManip;
    manipString = ss4.str();
    UI.labelPose->setText(manipString.c_str());
}

SimoxGui::ViewerInterfacePtr reachabilityWindow::getViewer() const
{
    return viewer;
}
