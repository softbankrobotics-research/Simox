
#include "GraspQualityWindow.h"

#include "GraspPlanning/GraspQuality/GraspEvaluationPoseUncertainty.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/Workspace/Reachability.h"
#include "VirtualRobot/Model/ManipulationObject.h"
#include "VirtualRobot/Grasping/Grasp.h"
#include "VirtualRobot/IK/GenericIKSolver.h"
#include "VirtualRobot/Grasping/GraspSet.h"
#include "VirtualRobot/CollisionDetection/CDManager.h"
#include "VirtualRobot/XML/ObjectIO.h"
#include "VirtualRobot/XML/ModelIO.h"
#include "VirtualRobot/Visualization/Visualization.h"

#include <GraspPlanning/Visualization/ConvexHullVisualization.h>

#include <QFileDialog>
#include <Eigen/Geometry>

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

GraspQualityWindow::GraspQualityWindow(std::string& robFile, std::string& objFile)
    : QMainWindow(NULL)
{
    VR_INFO << " start " << endl;

    this->robotFile = robFile;
    this->objectFile = objFile;
    setupUI();


    loadRobot();
    loadObject();

    viewer->viewAll();

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(timerCB()));
    timer->start(TIMER_MS);

}


GraspQualityWindow::~GraspQualityWindow()
{
    timer->stop();
    delete timer;
    viewer.reset();
}


void GraspQualityWindow::timerCB()
{
    float x[6];
    x[0] = (float)UI.horizontalSliderX->value();
    x[1] = (float)UI.horizontalSliderY->value();
    x[2] = (float)UI.horizontalSliderZ->value();
    x[3] = (float)UI.horizontalSliderRo->value();
    x[4] = (float)UI.horizontalSliderPi->value();
    x[5] = (float)UI.horizontalSliderYa->value();
    x[0] /= 10.0f;
    x[1] /= 10.0f;
    x[2] /= 10.0f;
    x[3] /= 300.0f;
    x[4] /= 300.0f;
    x[5] /= 300.0f;

    if (x[0] != 0 || x[1] != 0 || x[2] != 0 || x[3] != 0 || x[4] != 0 || x[5] != 0)
    {
        updateObject(x);
    }
}


void GraspQualityWindow::setupUI()
{
    UI.setupUi(this);

    SimoxGui::ViewerFactoryPtr viewerFactory = SimoxGui::ViewerFactory::first(NULL);
    THROW_VR_EXCEPTION_IF(!viewerFactory,"No viewer factory?!");
    viewer = viewerFactory->createViewer(UI.frameViewer);

    connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
    connect(UI.pushButtonClose, SIGNAL(clicked()), this, SLOT(closeEEF()));
    connect(UI.pushButtonOpen, SIGNAL(clicked()), this, SLOT(openEEF()));
    connect(UI.pushButtonToTCP, SIGNAL(clicked()), this, SLOT(objectToTCP()));
    connect(UI.pushButtonQuality, SIGNAL(clicked()), this, SLOT(graspQuality()));
    connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(selectObject()));
    connect(UI.pushButtonRobustness, SIGNAL(clicked()), this, SLOT(evalRobustness()));

    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.checkBoxCones, SIGNAL(clicked()), this, SLOT(frictionConeVisu()));
    //connect(UI.checkBoxGWS1, SIGNAL(clicked()), this, SLOT(showGWS()));
    //connect(UI.checkBoxGWS2, SIGNAL(clicked()), this, SLOT(showGWS()));
    //connect(UI.checkBoxOWS1, SIGNAL(clicked()), this, SLOT(showOWS()));
    //connect(UI.checkBoxOWS2, SIGNAL(clicked()), this, SLOT(showOWS()));

    connect(UI.horizontalSliderX, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectX()));
    connect(UI.horizontalSliderY, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectY()));
    connect(UI.horizontalSliderZ, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectZ()));
    connect(UI.horizontalSliderRo, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectA()));
    connect(UI.horizontalSliderPi, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectB()));
    connect(UI.horizontalSliderYa, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectG()));

    connect(UI.comboBoxEEF, SIGNAL(activated(int)), this, SLOT(selectEEF(int)));
    connect(UI.comboBoxGrasp, SIGNAL(activated(int)), this, SLOT(selectGrasp(int)));
}


void GraspQualityWindow::resetSceneryAll()
{
}


void GraspQualityWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}


void GraspQualityWindow::buildVisu()
{
    viewer->clearLayer("robotLayer");

    ModelLink::VisualizationType colModel = (UI.checkBoxColModel->isChecked()) ? ModelLink::Collision : ModelLink::Full;

    // robot
    if (robot)
    {
        VisualizationSetPtr visu = robot->getVisualization(colModel);
        viewer->addVisualization("robotLayer", visu);
    }

    // object
    viewer->clearLayer("objectLayer");
    if (object)
    {
        VisualizationSetPtr visu = object->getVisualization(colModel);
        viewer->addVisualization("objectLayer", visu);
    }

    // friction cones
    viewer->clearLayer("frictionLayer");
    bool fc = (UI.checkBoxCones->isChecked());
    if (fc && contacts.size() > 0 && qualityMeasure)
    {

        GraspPlanning::ContactConeGeneratorPtr cg = qualityMeasure->getConeGenerator();
        float radius = cg->getConeRadius();
        float height = cg->getConeHeight();
        float scaling = 30.0f;

        VisualizationPtr visu = VisualizationFactory::getGlobalVisualizationFactory()->createContactVisualization(contacts, height * scaling, radius * scaling, true);
        viewer->addVisualization("frictionLayer", visu);
    }
}


void GraspQualityWindow::quit()
{
    std::cout << "GraspQualityWindow: Closing" << std::endl;
    this->close();
    timer->stop();
}

void GraspQualityWindow::evalRobustness()
{
    int numSamples = UI.sbSamples->value();
    float varMM = UI.sbVariationMM->value();
    float varDeg = UI.sbVariationDeg->value();
    GraspPlanning::GraspEvaluationPoseUncertainty::PoseUncertaintyConfig c;
    c.init(varMM, varDeg);
    GraspPlanning::GraspEvaluationPoseUncertaintyPtr eval(new GraspPlanning::GraspEvaluationPoseUncertainty(c));

    openEEF();
    closeEEF();
    if (contacts.size() == 0)
        return;
    evalPoses = eval->generatePoses(object->getGlobalPose(), contacts, numSamples );


    if (evalPoses.size()==0)
        return;

    /*
    int r = rand() % evalPoses.size();
    Eigen::Matrix4f p = evalPoses.at(r);
    GraspPlanning::GraspEvaluationPoseUncertainty::PoseEvalResult re = eval->evaluatePose(eef, object, p, qualityMeasure);
    cout << "FC: " << re.forceClosure << endl;
    cout << "init col: " << re.initialCollision << endl;
    cout << "QUAL: " << re.quality << endl;
    */
    GraspPlanning::GraspEvaluationPoseUncertainty::PoseEvalResults re = eval->evaluatePoses(eef, object, evalPoses, qualityMeasure);
    if (eef && grasp)
        VR_INFO << "#### Robustness for eef " << eef->getName() << ", grasp " << grasp->getName() << endl;
    re.print();
}

void GraspQualityWindow::selectObject()
{
    QString fi = QFileDialog::getOpenFileName(this, tr("Open Object File"), QString(), tr("XML Files (*.xml)"));
    objectFile = std::string(fi.toLatin1());
    loadObject();
}

void GraspQualityWindow::loadObject()
{
    openEEF();

    if (!objectFile.empty())
    {
        try {
            object = ObjectIO::loadManipulationObject(objectFile);
        } catch (...)
        {
            VR_ERROR << "Could not load file " << objectFile << endl;
        }
    }

    if (!object)
    {
        VR_INFO << "Building standard box" << endl;
        ObstaclePtr o = Obstacle::createBox(50.0f, 50.0f, 10.0f);
        object = ManipulationObject::createFromMesh(o->getFirstLink()->getVisualization()->getTriMeshModel());
    }

    qualityMeasure.reset(new GraspPlanning::GraspQualityMeasureWrenchSpace(object));
    qualityMeasure->calculateObjectProperties();

    selectEEF(0);

    buildVisu();
}

void GraspQualityWindow::loadRobot()
{
    eefs.clear();
    robot.reset();
    robot = ModelIO::loadModel(robotFile);

    if (!robot)
    {
        VR_ERROR << " no robot at " << robotFile << endl;
        return;
    }


    setEEFComboBox();
    selectEEF(0);
    //objectToTCP();

    buildVisu();
}

void GraspQualityWindow::objectToTCP()
{
    if (object && eef && eef->getTcp())
    {
        Eigen::Matrix4f pos =  eef->getTcp()->getGlobalPose();
        object->setGlobalPose(pos);
    }
}


void GraspQualityWindow::objectToGrasp()
{
    if (object && grasp && eef->getTcp())
    {
        VR_INFO << "Setting object pose to grasp " << grasp->getName() << endl;
        Eigen::Matrix4f pos =  eef->getTcp()->getGlobalPose();
        pos = grasp->getObjectTargetPoseGlobal(pos);
        object->setGlobalPose(pos);
    }
}

void GraspQualityWindow::graspQuality()
{
    if (qualityMeasure && object && contacts.size() > 0)
    {
        qualityMeasure->setVerbose(true);

        qualityMeasure->setContactPoints(contacts);
        float volume = qualityMeasure->getVolumeGraspMeasure();
        float epsilon = qualityMeasure->getGraspQuality();
        bool fc = qualityMeasure->isGraspForceClosure();
        cout << "Grasp Quality (epsilon measure):" << epsilon << endl;
        cout << "v measure:" << volume << endl;
        cout << "Force closure:";

        if (fc)
        {
            cout << "yes" << endl;
        }
        else
        {
            cout << "no" << endl;
        }

    }
}

void GraspQualityWindow::selectEEF(int nr)
{
    eef.reset();

    if (nr < 0 || nr >= (int)eefs.size())
    {
        setGraspComboBox();
        selectGrasp(-1);
        objectToTCP();
        return;
    }

    eef = eefs[nr];
    setGraspComboBox();
    selectGrasp(0);
}

void GraspQualityWindow::selectGrasp(int nr)
{
    grasp.reset();

    if (!grasps || nr < 0 || nr >= (int)grasps->getSize())
    {
        objectToTCP();
        return;
    }

    grasp = grasps->getGrasp(nr);
    objectToGrasp();
}

void GraspQualityWindow::setGraspComboBox()
{
    UI.comboBoxGrasp->clear();
    grasp.reset();
    grasps.reset();

    if (!eef || !object)
    {
        return;
    }

    grasps = object->getGraspSet(eef);

    if (!grasps || grasps->getSize()==0)
    {
        VR_INFO << "No grasps found for eef " << eef->getName() << endl;
        return;
    }

    VR_INFO << "Found " << grasps->getSize() << " grasps for eef " << eef->getName() << endl;

    for (size_t i = 0; i < grasps->getSize(); i++)
    {
        QString nameGrasp(grasps->getGrasp(i)->getName().c_str());
        UI.comboBoxGrasp->addItem(nameGrasp);
    }
}


void GraspQualityWindow::setEEFComboBox()
{
    UI.comboBoxEEF->clear();
    eef.reset();
    eefs.clear();

    if (!robot)
    {
        return;
    }

    eefs = robot->getEndEffectors();

    for (size_t i = 0; i < eefs.size(); i++)
    {
        QString nameEEF(eefs[i]->getName().c_str());
        UI.comboBoxEEF->addItem(nameEEF);
    }
}

void GraspQualityWindow::closeEEF()
{
    contacts.clear();

    if (eef)
    {
        contacts = eef->closeActors(object);
    }

    buildVisu();
}

void GraspQualityWindow::openEEF()
{
    contacts.clear();

    if (eef)
    {
        eef->openActors();
    }

    buildVisu();
}



void GraspQualityWindow::updateObject(float x[6])
{
    if (object)
    {
        //cout << "getGlobalPose robot:" << endl << robotEEF->getGlobalPose() << endl;
        //cout << "getGlobalPose TCP:" << endl <<  robotEEF_EEF->getTcp()->getGlobalPose() << endl;
        Eigen::Matrix4f m;
        MathTools::posrpy2eigen4f(x, m);

        m = object->getGlobalPose() * m;
        object->setGlobalPose(m);
        cout << "object " << endl;
        cout << m << endl;

    }
}

void GraspQualityWindow::sliderReleased_ObjectX()
{
    UI.horizontalSliderX->setValue(0);
}

void GraspQualityWindow::sliderReleased_ObjectY()
{
    UI.horizontalSliderY->setValue(0);
}

void GraspQualityWindow::sliderReleased_ObjectZ()
{
    UI.horizontalSliderZ->setValue(0);
}

void GraspQualityWindow::sliderReleased_ObjectA()
{
    UI.horizontalSliderRo->setValue(0);
}

void GraspQualityWindow::sliderReleased_ObjectB()
{
    UI.horizontalSliderPi->setValue(0);
}

void GraspQualityWindow::sliderReleased_ObjectG()
{
    UI.horizontalSliderYa->setValue(0);
}

void GraspQualityWindow::frictionConeVisu()
{
    buildVisu();
}

void GraspQualityWindow::colModel()
{
    buildVisu();
}

void GraspQualityWindow::showGWS()
{
    viewer->clearLayer("gws");
    viewer->clearLayer("ows");

    Eigen::Matrix4f m = object->getGlobalPose();


    if (!qualityMeasure)
    {
        return;
    }

    VirtualRobot::MathTools::ConvexHull6DPtr ch = qualityMeasure->getConvexHullGWS();

    if (!ch)
    {
        return;
    }

    GraspPlanning::ConvexHullVisualizationPtr chv(new GraspPlanning::ConvexHullVisualization(ch, true));
    GraspPlanning::ConvexHullVisualizationPtr chv2(new GraspPlanning::ConvexHullVisualization(ch, false));
    viewer->addVisualization("gws", chv->getVisualization());
    viewer->addVisualization("ows", chv2->getVisualization());
}

void GraspQualityWindow::showOWS()
{

}
