
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

#ifdef Simox_USE_COIN_VISUALIZATION
    #include "../../../Gui/Coin/CoinViewerFactory.h"
    // TODO get rid of this hack.
    // need this to ensure that static Factory methods are called across library boundaries (otherwise coin Gui lib is not loaded since it is not referenced by us)
    SimoxGui::CoinViewerFactory f;
#endif

using namespace std;
using namespace VirtualRobot;

showRobotWindow::showRobotWindow(std::string& sRobotFilename)
    : QMainWindow(nullptr), robotLayer("robot-layer")
{
    useColModel = false;
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

    SimoxGui::ViewerFactoryPtr viewerFactory = SimoxGui::ViewerFactory::first(nullptr);
    THROW_VR_EXCEPTION_IF(!viewerFactory,"No viewer factory?!");
    viewer = viewerFactory->createViewer(UI.frameViewer);

    updateModelNodeSets();
    updateModelNodeControls();
    displayTriangles();

    connect(UI.btnLoadRobot, SIGNAL(clicked()), this, SLOT(selectRobot()));
    connect(UI.btnResetRobot, SIGNAL(clicked()), this, SLOT(resetRobot()));

    connect(UI.pushButtonClose, SIGNAL(clicked()), this, SLOT(closeHand()));
    connect(UI.ExportVRML20, SIGNAL(clicked()), this, SLOT(exportVRML()));
    connect(UI.ExportXML, SIGNAL(clicked()), this, SLOT(exportXML()));
    connect(UI.pushButtonOpen, SIGNAL(clicked()), this, SLOT(openHand()));
    connect(UI.comboBoxEndEffector, SIGNAL(activated(int)), this, SLOT(selectEEF(int)));
    connect(UI.comboBoxEndEffectorPS, SIGNAL(activated(int)), this, SLOT(selectPreshape(int)));

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

    auto visu = robot->getVisualization(visuType);
    viewer->addVisualization("robotLayer", visu);
    robot->setupVisualization(!UI.radioBtnNoVisu->isChecked());
}

void showRobotWindow::showSensors()
{
    if (!robot) return;

    // TODO

    render();
}



void showRobotWindow::attachPhysicsInformation(bool attach)
{
    if (!robot) return;

    if (attach)
        robot->attachPhysicsInformation();
    else
        robot->detachPhysicsInformation();

    render();
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
    std::cout << "ShowRobotWindow: Closing" << std::endl;
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
        render();
    }
}



void showRobotWindow::loadRobot()
{
    VR_INFO << "Loading Robot from " << robotFilename << endl;
    currentEEF.reset();
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
        try
        {
            QFileInfo fileInfo(robotFilename.c_str());
            std::string suffix(fileInfo.suffix().toLatin1());
            RobotImporterFactoryPtr importer = RobotImporterFactory::fromFileExtension(suffix, nullptr);

            if (!importer)
            {
                VR_WARNING << " ERROR while grabbing importer" << endl;
                return;
            }

            robot = importer->loadFromFile(robotFilename, ModelIO::eFull);
        }
        catch (VirtualRobotException& /*e*/)
        {
        }
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
    UI.cBoxJointSets->addItem("All");
    for (auto & js : robot->getJointSets())
    {
        UI.cBoxJointSets->addItem(QString::fromStdString(js->getName()));
    }
    // link sets
    UI.cBoxLinkSets->clear();
    UI.cBoxLinkSets->addItem("All");
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
        robot->attachStructure(VisualizationFactory::getGlobalVisualizationFactory()->getVisualizationType());
    else
        robot->detachStructure();

    render();
}

void showRobotWindow::attachFrames(bool attach)
{
    if (!robot) return;

    if (attach)
        robot->attachFrames(VisualizationFactory::getGlobalVisualizationFactory()->getVisualizationType());
    else
        robot->detachFrames();

    render();
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
