
#include "showRobotWindow.h"
#include "../../EndEffector/EndEffector.h"
#include "../../Tools/RuntimeEnvironment.h"
#include "../../Import/RobotImporterFactory.h"
#include "../../Model/Nodes/ModelJoint.h"
#include "../../Model/LinkSet.h"
#include "../../Model/JointSet.h"
#include "../../Import/SimoxXMLFactory.h"
#include "../../Visualization/CoinVisualization/CoinVisualizationFactory.h"
#include "../../Model/Nodes/Attachments/ModelNodeAttachmentFactory.h"
#include "../../Model/Nodes/Attachments/ModelFrameFactory.h"

#include <QFileDialog>
#include <Eigen/Geometry>

#include <boost/algorithm/string/predicate.hpp>
#include <time.h>
#include <vector>
#include <iostream>
#include <cmath>

#include <sstream>

#ifdef Simox_USE_COIN_VISUALIZATION
    #include "../../../Gui/Coin/CoinViewerFactory.h"
    // need this to ensure that static Factory methods are called across library boundaries (otherwise coin Gui lib is not loaded since it is not referenced by us)
    SimoxGui::CoinViewerFactory f;
#endif

using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 30.0f;

showRobotWindow::showRobotWindow(std::string& sRobotFilename)
    : QMainWindow(NULL)
{
    useColModel = false;
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(sRobotFilename);
    robotFilename = sRobotFilename;

    setupUI();

    loadRobot();

    viewer->viewAll();
}


showRobotWindow::~showRobotWindow()
{
    robot.reset();
}

void showRobotWindow::setupUI()
{
    UI.setupUi(this);

    SimoxGui::ViewerFactoryPtr viewerFactory = SimoxGui::ViewerFactory::first(NULL);
    THROW_VR_EXCEPTION_IF(!viewerFactory,"No viewer factory?!");
    viewer = viewerFactory->createViewer(UI.frameViewer);

    connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
    connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(selectRobot()));

    connect(UI.pushButtonClose, SIGNAL(clicked()), this, SLOT(closeHand()));
    connect(UI.ExportVRML20, SIGNAL(clicked()), this, SLOT(exportVRML()));
    connect(UI.ExportXML, SIGNAL(clicked()), this, SLOT(exportXML()));
    connect(UI.pushButtonOpen, SIGNAL(clicked()), this, SLOT(openHand()));
    connect(UI.comboBoxEndEffector, SIGNAL(activated(int)), this, SLOT(selectEEF(int)));
    connect(UI.comboBoxEndEffectorPS, SIGNAL(activated(int)), this, SLOT(selectPreshape(int)));

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

void showRobotWindow::resetSceneryAll()
{
    if (!robot)
    {
        return;
    }

    for (auto& j:allNodes)
    {
        ModelJointPtr joint = dynamic_pointer_cast<ModelJoint>(j);
        if (joint)
            joint->setJointValue(0.0f);
    }

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

    LinkSetPtr ls = dynamic_pointer_cast<LinkSet>(currentRobotNodeSet);
    if (ls)
    {
        trisRNSFull = ls->getNumFaces(false);
        trisRNSCol = ls->getNumFaces(true);
    }

    ModelLinkPtr ml = dynamic_pointer_cast<ModelLink>(currentRobotNode);
    if (ml)
    {
        trisJointFull = ml->getNumFaces(false);
        trisJointCol = ml->getNumFaces(true);
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

    useColModel = UI.checkBoxColModel->checkState() == Qt::Checked;
    //bool sensors = UI.checkBoxRobotSensors->checkState() == Qt::Checked;
    ModelLink::VisualizationType colModel = (UI.checkBoxColModel->isChecked()) ? ModelLink::VisualizationType::Collision : ModelLink::VisualizationType::Full;

    VisualizationPtr visu = VisualizationFactory::getGlobalVisualizationFactory()->getVisualization(robot, colModel);
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

    //todo
    /*

    std::vector<SensorPtr> sensors = robot->getSensors();

    for (size_t i = 0; i < sensors.size(); i++)
    {
        sensors[i]->setupVisualization(showSensors, showSensors);
        sensors[i]->showCoordinateSystem(showSensors);
    }
    */

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

    //todo
    /*
    robot->showPhysicsInformation(physicsCoMEnabled, physicsInertiaEnabled);
    */

    // rebuild visualization
    rebuildVisualization();

}

void showRobotWindow::exportVRML()
{
    if (!robot)
    {
        return;
    }


    // todo: remove?

    // VRML
    /*
    QString fi = QFileDialog::getSaveFileName(this, tr("VRML 2.0 File"), QString(), tr("VRML Files (*.wrl)"));
    std::string s = std::string(fi.toLatin1());

    if (!s.empty())
    {
        if (!boost::algorithm::ends_with(s, ".wrl"))
            s += ".wrl";
        ModelLink::VisualizationType colModel = (UI.checkBoxColModel->isChecked()) ? ModelLink::VisualizationType::Collision : ModelLink::VisualizationType::Full;
        visualization = robot->getVisualization<CoinVisualization>(colModel);
        visualization->exportToVRML2(s);
    }
    */
}


void showRobotWindow::exportXML()
{
    if (!robot)
    {
        return;
    }

    // todo
    // XML
    /*
    QString fi = QFileDialog::getSaveFileName(this, tr("xml File"), QString(), tr("xml Files (*.xml)"));
    std::string s = std::string(fi.toLatin1());

    if (!s.empty())
    {

        boost::filesystem::path p1(s);
        std::string fn = p1.filename().generic_string();
        std::string fnPath = p1.parent_path().generic_string();
        ModelIO::saveXML(robot, fn, fnPath);
    }*/
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
    std::cout << "ShowRobotWindow: Closing" << std::endl;
    this->close();
    viewer->stop();
}

void showRobotWindow::updateJointBox()
{
    UI.comboBoxJoint->clear();

    for (unsigned int i = 0; i < currentNodes.size(); i++)
    {
        UI.comboBoxJoint->addItem(QString(currentNodes[i]->getName().c_str()));
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
        currentNodes = allNodes;
    }
    else
    {
        nr--;

        if (nr >= (int)robotNodeSets.size())
        {
            return;
        }

        currentRobotNodeSet = robotNodeSets[nr];

        currentNodes = currentRobotNodeSet->getModelNodes();

        //std::cout << "COM:" << currentRobotSet->getCoM();
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
    //todo
    /*
    if (currentRobotNode)
    {
        currentRobotNode->showBoundingBox(false);
    }
    */

    currentRobotNode.reset();
    cout << "Selecting Joint nr " << nr << endl;

    if (nr < 0 || nr >= (int)currentNodes.size())
    {
        return;
    }

    currentRobotNode = currentNodes[nr];
    //todo
    /*
    currentRobotNode->showBoundingBox(true, true);
    */
    currentRobotNode->print();
    ModelJointPtr joint = dynamic_pointer_cast<ModelJoint>(currentRobotNode);
    float mi = 0.0f;
    float ma = 0.0f;
    float j = 0.0f;
    if (joint)
    {
        mi = joint->getJointLimitLow();
        ma = joint->getJointLimitHigh();
        j = joint->getJointValue();
    }
    QString qMin = QString::number(mi);
    QString qMax = QString::number(ma);
    UI.labelMinPos->setText(qMin);
    UI.labelMaxPos->setText(qMax);

    UI.lcdNumberJointValue->display((double)j);

    if (fabs(ma - mi) > 0 && joint)
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
    //todo
    /*
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
        // todo
        //robot->highlight(visualization, false);
        //currentRobotNode->highlight(visualization, true);
    }
    */
    displayTriangles();
}

void showRobotWindow::jointValueChanged(int pos)
{
    ModelJointPtr joint = dynamic_pointer_cast<ModelJoint>(currentRobotNode);
    if (!joint)
        return;

    float fPos = joint->getJointLimitLow() + (float)pos / 1000.0f * (joint->getJointLimitHigh() - joint->getJointLimitLow());
    joint->setJointValue(fPos);
    UI.lcdNumberJointValue->display((double)fPos);
}

void showRobotWindow::showCoordSystem()
{
    // todo
    /*
    float size = 0.75f;
    currentRobotNode->showCoordinateSystem(UI.checkBoxShowCoordSystem->checkState() == Qt::Checked, size);
    */
    // rebuild visualization
    rebuildVisualization();
}



void showRobotWindow::selectRobot()
{
    string supportedExtensions = RobotImporterFactory::getAllExtensions();
    string supported = "Supported Formats, " + supportedExtensions + " (" + supportedExtensions + ")";
    string filter = supported + ";;" + RobotImporterFactory::getAllFileFilters();
    QString fi = QFileDialog::getOpenFileName(this, tr("Open Robot File"), QString(), tr(filter.c_str()));
    std::string s = robotFilename = std::string(fi.toLatin1());

    if (!s.empty())
    {
        robotFilename = s;
        loadRobot();
    }
}



void showRobotWindow::loadRobot()
{
    viewer->clearLayer("robotLayer");

    cout << "Loading Robot from " << robotFilename << endl;
    currentEEF.reset();
    currentRobotNode.reset();
    currentNodes.clear();
    currentRobotNodeSet.reset();
    robot.reset();

    try
    {
        //QFileInfo fileInfo(robotFilename.c_str());
        //std::string suffix(fileInfo.suffix().toLatin1());
        /*RobotImporterFactoryPtr importer = RobotImporterFactory::fromFileExtension(suffix, NULL);

        if (!importer)
        {
            cout << " ERROR while grabbing importer" << endl;
            return;
        }

        robot = importer->loadFromFile(robotFilename, ModelIO::eFull);*/

        robot = ModelIO::loadModel(robotFilename, ModelIO::eFull);
    }
    catch (VirtualRobotException& e)
    {
    }

    if (!robot)
    {
        try
        {
            QFileInfo fileInfo(robotFilename.c_str());
            std::string suffix(fileInfo.suffix().toLatin1());
            RobotImporterFactoryPtr importer = RobotImporterFactory::fromFileExtension(suffix, NULL);

            if (!importer)
            {
                cout << " ERROR while grabbing importer" << endl;
                return;
            }

            robot = importer->loadFromFile(robotFilename, ModelIO::eFull);
            //robot = SimoxXMLFactory::loadRobotSimoxXML(robotFilename, ModelIO::eFull);
        }
        catch (VirtualRobotException& e)
        {
        }
    }


    if (!robot)
    {
        cout << " ERROR while creating robot" << endl;
        return;
    }

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
    allNodes = robot->getModelNodes();
    robotNodeSets = robot->getModelNodeSets();
    eefs = robot->getEndEffectors();
    updateEEFBox();
    updateRNSBox();
    selectRNS(0);

    if (allNodes.size() == 0)
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

    if (UI.checkBoxStructure->checkState() == Qt::Checked)
        robot->attachStructure(VirtualRobot::CoinVisualizationFactory::getName());
    else
        robot->detachStructure();

    rebuildVisualization();
}

void showRobotWindow::robotCoordSystems()
{
    if (!robot)
    {
        return;
    }

    if (UI.checkBoxRobotCoordSystems->checkState() == Qt::Checked)
        robot->attachFrames(VirtualRobot::CoinVisualizationFactory::getName());
    else
        robot->detachFrames();

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
    if (currentEEF)
    {
        currentEEF->openActors();
    }
}

void showRobotWindow::selectEEF(int nr)
{
    cout << "Selecting EEF nr " << nr << endl;

     UI.comboBoxEndEffectorPS->clear();
     currentEEF.reset();

    if (nr < 0 || nr >= (int)eefs.size())
    {        
        return;
    }
    currentEEF = eefs[nr];

    std::vector<std::string> ps = currentEEF->getPreshapes();
    UI.comboBoxEndEffectorPS->addItem(QString("none"));
    for (unsigned int i = 0; i < ps.size(); i++)
    {
        UI.comboBoxEndEffectorPS->addItem(QString(ps[i].c_str()));
    }
}

void showRobotWindow::selectPreshape(int nr)
{
    cout << "Selecting EEF preshape nr " << nr << endl;

    if (!currentEEF || nr==0)
        return;

    nr--; // first entry is "none"

    std::vector<std::string> ps = currentEEF->getPreshapes();
    if (nr < 0 || nr >= (int)ps.size())
    {
        return;
    }

    VirtualRobot::RobotConfigPtr c = currentEEF->getPreshape(ps.at(nr));

    robot->setConfig(c);
}

void showRobotWindow::updateEEFBox()
{
    UI.comboBoxEndEffector->clear();

    for (unsigned int i = 0; i < eefs.size(); i++)
    {
        UI.comboBoxEndEffector->addItem(QString(eefs[i]->getName().c_str()));
    }
}
