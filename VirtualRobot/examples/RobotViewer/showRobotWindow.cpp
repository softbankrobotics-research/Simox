
#include "showRobotWindow.h"
#include "../../EndEffector/EndEffector.h"
#include "../../Tools/RuntimeEnvironment.h"
#include "../../Import/RobotImporterFactory.h"
#include "../../Model/Nodes/ModelJoint.h"
#include "../../Model/LinkSet.h"
#include "../../Model/JointSet.h"
#include "../../Import/SimoxXMLFactory.h"
#include "../../Model/Nodes/Attachments/ModelNodeAttachmentFactory.h"

#include <QFileDialog>
#include <Eigen/Geometry>

#include <boost/algorithm/string/predicate.hpp>
#include <time.h>
#include <vector>
#include <iostream>
#include <cmath>

#include <sstream>

using namespace std;
using namespace VirtualRobot;

showRobotWindow::showRobotWindow(std::string& sRobotFilename)
    : QMainWindow(nullptr), rederRobotCoM(false), robotLayer("robot-layer")
{
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(sRobotFilename);
    robotFilename = sRobotFilename;

    loadRobot();
    setupUI();

    viewer->viewAll();
}


showRobotWindow::~showRobotWindow()
{
    robot.reset();
}

void showRobotWindow::setupUI()
{
    UI.setupUi(this);

    SimoxGui::ViewerFactoryPtr viewerFactory = SimoxGui::ViewerFactory::getInstance();
    THROW_VR_EXCEPTION_IF(!viewerFactory,"No viewer factory?!");
    viewer = viewerFactory->createViewer(UI.frameViewer);

    updateModelNodeSets();
    updateModelNodeControls();
    updateEEFBox();
    displayTriangles();

    connect(UI.btnLoadRobot, SIGNAL(clicked()), this, SLOT(selectRobot()));
    connect(UI.btnResetRobot, SIGNAL(clicked()), this, SLOT(resetRobot()));

    connect(UI.pushButtonClose, SIGNAL(clicked()), this, SLOT(closeEEF()));
    connect(UI.ExportXML, SIGNAL(clicked()), this, SLOT(exportXML()));
    connect(UI.pushButtonOpen, SIGNAL(clicked()), this, SLOT(openEEF()));
    connect(UI.comboBoxEndEffector, SIGNAL(activated(int)), this, SLOT(selectEEF()));
    connect(UI.comboBoxEndEffectorPS, SIGNAL(activated(int)), this, SLOT(selectPreshape()));

    connect(UI.radioBtnCollisionVisu, SIGNAL(clicked()), this, SLOT(render()));
    connect(UI.radioBtnFullVisu, SIGNAL(clicked()), this, SLOT(render()));
    connect(UI.radioBtnNoVisu, SIGNAL(clicked()), this, SLOT(render()));

    connect(UI.checkBoxRobotSensors, SIGNAL(clicked()), this, SLOT(showSensors()));
    connect(UI.checkBoxStructure, SIGNAL(clicked(bool)), this, SLOT(attachStructure(bool)));
    connect(UI.checkBoxRobotCoordSystems, SIGNAL(clicked(bool)), this, SLOT(attachFrames(bool)));
    connect(UI.checkBoxPhysics, SIGNAL(clicked(bool)), this, SLOT(attachPhysicsInformation(bool)));

    connect(UI.cBoxJointSets, SIGNAL(currentIndexChanged(int)), this, SLOT(updateModelNodeControls()));
    connect(UI.cBoxLinkSets, SIGNAL(currentIndexChanged(int)), this, SLOT(updateModelNodeControls()));
    connect(UI.cBoxLinkSets, SIGNAL(currentIndexChanged(int)), this, SLOT(displayTriangles()));
    connect(UI.listLinks, SIGNAL(currentTextChanged(QString)), this, SLOT(displayTriangles()));
    connect(UI.radioBtnCollisionVisu, SIGNAL(clicked(bool)), this, SLOT(displayTriangles()));
    connect(UI.radioBtnFullVisu, SIGNAL(clicked(bool)), this, SLOT(displayTriangles()));
    connect(UI.radioBtnNoVisu, SIGNAL(clicked(bool)), this, SLOT(displayTriangles()));

    render();
}

void showRobotWindow::resetRobot()
{
    if (!robot) return;

    std::map<std::string, float> jv;
    for (auto& j: robot->getJoints())
    {
        jv[j->getName()] = 0.0f;
    }
    robot->setJointValues(jv);

    updateModelNodeControls();
}

void showRobotWindow::displayTriangles()
{
    if (!robot) return;

    QString text1, text2, text3;
    int trisAllFull, trisRNSFull, trisJointFull;
    trisAllFull = trisRNSFull = trisJointFull = 0;
    int trisAllCol, trisRNSCol, trisJointCol;
    trisAllCol = trisRNSCol = trisJointCol = 0;

    trisAllFull = robot->getNumFaces(false);
    trisAllCol = robot->getNumFaces(true);
    trisRNSFull = trisAllFull;
    trisRNSCol = trisAllCol;

    if (robot->hasLinkSet(UI.cBoxLinkSets->currentText().toStdString()))
    {
        LinkSetPtr ls = robot->getLinkSet(UI.cBoxLinkSets->currentText().toStdString());
        trisRNSFull = ls->getNumFaces(false);
        trisRNSCol = ls->getNumFaces(true);
    }

    if (UI.listLinks->currentItem() && robot->hasLink(UI.listLinks->currentItem()->text().toStdString()))
    {
        ModelLinkPtr ml = robot->getLink(UI.listLinks->currentItem()->text().toStdString());
        trisJointFull = ml->getNumFaces(false);
        trisJointCol = ml->getNumFaces(true);
    }

    if (UI.radioBtnCollisionVisu->isChecked())
    {
        text1 = "Total: " + QString::number(trisAllCol);
        text2 = UI.cBoxLinkSets->currentText() + ": " + QString::number(trisRNSCol);
        text3 = UI.listLinks->currentItem() ? UI.listLinks->currentItem()->text() + ": " + QString::number(trisJointCol) : "";
    }
    else if (UI.radioBtnNoVisu->isChecked())
    {
        text1 = "Total: 0";
        text2 = UI.cBoxLinkSets->currentText() + ": 0";
        text3 = UI.listLinks->currentItem() ? UI.listLinks->currentItem()->text() + ": 0" : "";
    }
    else
    {
        text1 = "Total: " + QString::number(trisAllFull);
        text2 = UI.cBoxLinkSets->currentText() + ": " + QString::number(trisRNSFull);
        text3 = UI.listLinks->currentItem() ? UI.listLinks->currentItem()->text() + ": " + QString::number(trisJointFull) : "";
    }

    UI.labelInfo1->setText(text1);
    UI.labelInfo2->setText(text2);
    UI.labelInfo3->setText(text3);
}

void showRobotWindow::render()
{
    if (!robot)
    {
        return;
    }

    viewer->clearLayer(robotLayer);
    ModelLink::VisualizationType visuType = (UI.radioBtnCollisionVisu->isChecked()) ? ModelLink::VisualizationType::Collision : ModelLink::VisualizationType::Full;

    if (!UI.radioBtnNoVisu->isChecked())
    {
        auto visu = robot->getVisualization(visuType);
        viewer->addVisualization(visu, robotLayer);
    }
    // We always render attachments because they can be unchecked easily anyways
    auto attachmentVisus = robot->getAllAttachmentVisualizations();
    viewer->addVisualization(attachmentVisus, robotLayer);

    if (rederRobotCoM)
    {
        auto comSphere = VisualizationFactory::getInstance()->createSphere(7.05f);
        comSphere->setColor(Visualization::Color::Red());

        auto comText = VisualizationFactory::getInstance()->createText("CoM: Robot", true, 0, 10.0f, 0);
        comText->setColor(Visualization::Color::Blue());

        auto com = VisualizationFactory::getInstance()->createVisualisationSet({comSphere, comText});
        Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
        m.block<3, 1>(0, 3) = robot->getCoMGlobal();
        com->setGlobalPose(m);

        viewer->addVisualization(com);
    }
}

void showRobotWindow::showSensors()
{
    if (!robot) return;

    VR_ERROR_ONCE_NYI;

    render();
}



void showRobotWindow::attachPhysicsInformation(bool attach)
{
    if (!robot) return;

    if (attach)
    {
        robot->attachPhysicsInformation();
        rederRobotCoM = true;
    }
    else
    {
        robot->detachPhysicsInformation();
        rederRobotCoM = false;
    }

    render();
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
        SimoxXMLFactory::saveXML(robot, fn, fnPath);
    }
}


void showRobotWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}


void showRobotWindow::quit()
{
    this->close();
}

void showRobotWindow::selectRobot()
{
    string supportedExtensions = RobotImporterFactory::getAllExtensions();
    string supported = "Supported Formats, " + supportedExtensions + " (" + supportedExtensions + ")";
    string filter = supported + ";;" + RobotImporterFactory::getAllFileFilters();
    QString dir = RuntimeEnvironment::getDataPaths().empty() ? QString() : QString::fromStdString(RuntimeEnvironment::getDataPaths()[0]);
    QString fi = QFileDialog::getOpenFileName(this, tr("Open Robot File"), dir, tr(filter.c_str()));
    std::string s = robotFilename = std::string(fi.toLatin1());

    if (!s.empty())
    {
        robotFilename = s;
        loadRobot();
        updateModelNodeSets();
        updateModelNodeControls();
        updateEEFBox();
        render();
    }
}



void showRobotWindow::loadRobot()
{
    VR_INFO << "Loading Robot from " << robotFilename << endl;
    robot.reset();

    try
    {
        robot = ModelIO::loadModel(robotFilename, ModelIO::eFull);
    }
    catch (VirtualRobotException& /*e*/)
    {
    }

    if (!robot)
    {
        VR_WARNING << " ERROR while creating robot" << endl;
        return;
    }
}

void showRobotWindow::updateModelNodeControls()
{
    if (!robot) return;

    std::vector<ModelJointPtr> joints = (!robot->hasJointSet(UI.cBoxJointSets->currentText().toStdString())) ? robot->getJoints() : robot->getJointSet(UI.cBoxJointSets->currentText().toStdString())->getJoints();
    std::vector<ModelLinkPtr> links = (!robot->hasLinkSet(UI.cBoxLinkSets->currentText().toStdString())) ? robot->getLinks() : robot->getLinkSet(UI.cBoxLinkSets->currentText().toStdString())->getLinks();

    // joints tab
    UI.tableJoints->clear();
    UI.tableJoints->setRowCount(joints.size());
    UI.tableJoints->setColumnCount(2);
    UI.tableJoints->setHorizontalHeaderLabels({"Name", "Joint Value"});
    for (int i = 0; i < joints.size(); i++)
    {
        QTableWidgetItem *item = new QTableWidgetItem(QString::fromStdString(joints[i]->getName()));
        UI.tableJoints->setItem(i, 0, item);

        QSlider *slider = new JointValueSlider(joints[i], Qt::Horizontal);
        slider->setRange(0, 1000);
        float sliderValue = 1000 * ((joints[i]->getJointValue() - joints[i]->getJointLimitLow()) / (joints[i]->getJointLimitHigh() - joints[i]->getJointLimitLow()));
        slider->setValue(sliderValue);
        slider->setTracking(true);
        slider->setEnabled(joints[i]->getJointLimitHigh() - joints[i]->getJointLimitLow() != 0.0);
        UI.tableJoints->setCellWidget(i, 1, slider);

        connect(slider, SIGNAL(valueChanged(int)), this, SLOT(updateJoints()));
    }

    // links tab
    UI.listLinks->clear();
    for (std::size_t i = 0; i < links.size(); i++)
    {
        UI.listLinks->addItem(QString::fromStdString(links[i]->getName()));
    }
}

void showRobotWindow::updateModelNodeSets()
{
    // joint sets
    UI.cBoxJointSets->clear();
    UI.cBoxJointSets->addItem("<All>");
    for (auto & js : robot->getJointSets())
    {
        UI.cBoxJointSets->addItem(QString::fromStdString(js->getName()));
    }
    // link sets
    UI.cBoxLinkSets->clear();
    UI.cBoxLinkSets->addItem("<All>");
    for (auto & ls : robot->getLinkSets())
    {
        UI.cBoxLinkSets->addItem(QString::fromStdString(ls->getName()));
    }
}

void showRobotWindow::updateJoints()
{
    std::map<std::string, float> jointValues;
    for (int i = 0; i < UI.tableJoints->rowCount(); i++)
    {
        std::string name = UI.tableJoints->item(i, 0)->text().toStdString();
        QSlider* slider = ((QSlider*) UI.tableJoints->cellWidget(i, 1));
        float ratio = (float)(slider->value() - slider->minimum()) / (slider->maximum() - slider->minimum());
        float value = robot->getJoint(name)->getJointLimitLow() + ratio * (robot->getJoint(name)->getJointLimitHigh() - robot->getJoint(name)->getJointLimitLow());
        jointValues[name] = value;
    }
    robot->setJointValues(jointValues);
}

void showRobotWindow::attachStructure(bool attach)
{
    if (!robot) return;

    if (attach)
        robot->attachStructure();
    else
        robot->detachStructure();

    render();
}

void showRobotWindow::attachFrames(bool attach)
{
    if (!robot) return;

    if (attach)
        robot->attachCoordinateSystems();
    else
        robot->detachCoordinateSystems();

    render();
}

void showRobotWindow::closeEEF()
{
    if (robot->hasEndEffector(UI.comboBoxEndEffector->currentText().toStdString()))
    {
        EndEffectorPtr currentEEF = robot->getEndEffector(UI.comboBoxEndEffector->currentText().toStdString());
        currentEEF->closeActors();
    }
}

void showRobotWindow::openEEF()
{
    if (robot->hasEndEffector(UI.comboBoxEndEffector->currentText().toStdString()))
    {
        EndEffectorPtr currentEEF = robot->getEndEffector(UI.comboBoxEndEffector->currentText().toStdString());
        if (currentEEF->hasPreshape(UI.comboBoxEndEffectorPS->currentText().toStdString()))
        {
            robot->setConfig(currentEEF->getPreshape(UI.comboBoxEndEffectorPS->currentText().toStdString()));
        }
        else
        {
            currentEEF->openActors();
        }
    }
}

void showRobotWindow::selectEEF()
{
    UI.comboBoxEndEffectorPS->clear();
    if (!robot->hasEndEffector(UI.comboBoxEndEffector->currentText().toStdString()))
    {
        return;
    }

    EndEffectorPtr currentEEF = robot->getEndEffector(UI.comboBoxEndEffector->currentText().toStdString());
    std::vector<std::string> ps = currentEEF->getPreshapes();
    UI.comboBoxEndEffectorPS->addItem(QString("<none>"));
    for (unsigned int i = 0; i < ps.size(); i++)
    {
        UI.comboBoxEndEffectorPS->addItem(QString(ps[i].c_str()));
    }
}

void showRobotWindow::selectPreshape()
{
    if (!robot->hasEndEffector(UI.comboBoxEndEffector->currentText().toStdString()))
    {
        return;
    }
    EndEffectorPtr currentEEF = robot->getEndEffector(UI.comboBoxEndEffector->currentText().toStdString());
    if (!currentEEF->hasPreshape(UI.comboBoxEndEffectorPS->currentText().toStdString()))
    {
        return;
    }

    robot->setConfig(currentEEF->getPreshape(UI.comboBoxEndEffectorPS->currentText().toStdString()));
}

void showRobotWindow::updateEEFBox()
{
    if (!robot) return;

    UI.comboBoxEndEffector->clear();
    for (auto && eef : robot->getEndEffectors())
    {
        UI.comboBoxEndEffector->addItem(QString::fromStdString(eef->getName()));
    }
    selectEEF();
}
