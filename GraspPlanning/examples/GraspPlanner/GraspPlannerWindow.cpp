
#include "GraspPlannerWindow.h"
#include "GraspPlanning/Visualization/CoinVisualization/CoinConvexHullVisualization.h"
#include "GraspPlanning/ContactConeGenerator.h"
#include "GraspPlanning/MeshConverter.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/Workspace/Reachability.h"
#include "VirtualRobot/ManipulationObject.h"
#include "VirtualRobot/Grasping/Grasp.h"
#include "VirtualRobot/IK/GenericIKSolver.h"
#include "VirtualRobot/Grasping/GraspSet.h"
#include "VirtualRobot/CollisionDetection/CDManager.h"
#include "VirtualRobot/XML/ObjectIO.h"
#include "VirtualRobot/XML/RobotIO.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h"
#include "VirtualRobot/Visualization/TriMeshModel.h"
#include <GraspPlanning/GraspQuality/GraspEvaluationPoseUncertainty.h>

#include <QFileDialog>
#include <QProgressDialog>

#include <Eigen/Geometry>

#include <time.h>
#include <vector>
#include <iostream>
#include <cmath>
#include <sstream>

#include "Inventor/actions/SoLineHighlightRenderAction.h"
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/nodes/SoScale.h>


using namespace std;
using namespace VirtualRobot;
using namespace GraspStudio;

float TIMER_MS = 30.0f;

GraspPlannerWindow::GraspPlannerWindow(std::string& robFile, std::string& eefName, std::string& preshape, std::string& objFile)
    : QMainWindow(NULL)
{
    VR_INFO << " start " << endl;

    // init the random number generator
    this->robotFile = robFile;
    this->objectFile = objFile;
    this->eefName = eefName;
    this->preshape = preshape;
    eefVisu = NULL;

    sceneSep = new SoSeparator;
    sceneSep->ref();
    robotSep = new SoSeparator;
    objectSep = new SoSeparator;
    frictionConeSep = new SoSeparator;
    graspsSep = new SoSeparator;
    graspsSep->ref();

#if 0
    SoSeparator* s = CoinVisualizationFactory::CreateCoordSystemVisualization();
    sceneSep->addChild(s);
#endif
    sceneSep->addChild(robotSep);
    sceneSep->addChild(objectSep);
    sceneSep->addChild(frictionConeSep);
    //sceneSep->addChild(graspsSep);

    setupUI();


    loadRobot();
    loadObject();

    buildVisu();



    viewer->viewAll();

    /*SoSensorManager *sensor_mgr = SoDB::getSensorManager();
    SoTimerSensor *timer = new SoTimerSensor(timerCB, this);
    timer->setInterval(SbTime(TIMER_MS/1000.0f));
    sensor_mgr->insertTimerSensor(timer);*/
}


GraspPlannerWindow::~GraspPlannerWindow()
{
    sceneSep->unref();
    graspsSep->unref();

    if (eefVisu)
    {
        eefVisu->unref();
    }
}


/*void GraspPlannerWindow::timerCB(void * data, SoSensor * sensor)
{
    GraspPlannerWindow *ikWindow = static_cast<GraspPlannerWindow*>(data);
    float x[6];
    x[0] = (float)ikWindow->UI.horizontalSliderX->value();
    x[1] = (float)ikWindow->UI.horizontalSliderY->value();
    x[2] = (float)ikWindow->UI.horizontalSliderZ->value();
    x[3]= (float)ikWindow->UI.horizontalSliderRo->value();
    x[4] = (float)ikWindow->UI.horizontalSliderPi->value();
    x[5] = (float)ikWindow->UI.horizontalSliderYa->value();
    x[0] /= 10.0f;
    x[1] /= 10.0f;
    x[2] /= 10.0f;
    x[3] /= 300.0f;
    x[4] /= 300.0f;
    x[5] /= 300.0f;

    if (x[0]!=0 || x[1]!=0 || x[2]!=0 || x[3]!=0 || x[4]!=0 || x[5]!=0)
        ikWindow->updateObject(x);
}*/

void GraspPlannerWindow::setupUI()
{
    UI.setupUi(this);
    viewer = new SoQtExaminerViewer(UI.frameViewer, "", TRUE, SoQtExaminerViewer::BUILD_POPUP);

    // setup
    viewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
    viewer->setAccumulationBuffer(true);
#ifdef WIN32
    viewer->setAntialiasing(true, 8);
#endif
    viewer->setGLRenderAction(new SoLineHighlightRenderAction);
    viewer->setTransparencyType(SoGLRenderAction::SORTED_OBJECT_BLEND);
    viewer->setFeedbackVisibility(true);
    viewer->setSceneGraph(sceneSep);
    viewer->viewAll();

    connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
    connect(UI.pushButtonPlan, SIGNAL(clicked()), this, SLOT(plan()));
    connect(UI.pushButtonPlanBatch, SIGNAL(clicked()), this, SLOT(planObjectBatch()));
    connect(UI.pushButtonSave, SIGNAL(clicked()), this, SLOT(save()));
    connect(UI.pushButtonOpen, SIGNAL(clicked()), this, SLOT(openEEF()));
    connect(UI.pushButtonClose, SIGNAL(clicked()), this, SLOT(closeEEF()));



    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.checkBoxCones, SIGNAL(clicked()), this, SLOT(frictionConeVisu()));
    connect(UI.checkBoxGrasps, SIGNAL(clicked()), this, SLOT(showGrasps()));
}


void GraspPlannerWindow::resetSceneryAll()
{
    grasps->removeAllGrasps();
    graspsSep->removeAllChildren();

    //if (rns)
    //  rns->setJointValues(startConfig);
}


void GraspPlannerWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}


void GraspPlannerWindow::buildVisu()
{

    robotSep->removeAllChildren();
    SceneObject::VisualizationType colModel = (UI.checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;

    if (eefCloned)
    {
        visualizationRobot = eefCloned->getVisualization<CoinVisualization>(colModel);
        SoNode* visualisationNode = visualizationRobot->getCoinVisualization();

        if (visualisationNode)
        {
            robotSep->addChild(visualisationNode);
            visualizationRobot->highlight(UI.checkBoxHighlight->isChecked());
        }
    }

    /*
    if (robot)
    {
        visualizationRobot = robot->getVisualization<CoinVisualization>(colModel);
        SoNode* visualisationNode = visualizationRobot->getCoinVisualization();
        if (visualisationNode)
        {
            robotSep->addChild(visualisationNode);
            //visualizationRobot->highlight(true);
        }
    }
    */
    objectSep->removeAllChildren();

    if (object)
    {

#if 1
        SceneObject::VisualizationType colModel2 = (UI.checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;
        SoNode* visualisationNode = CoinVisualizationFactory::getCoinVisualization(object, colModel2);

        if (visualisationNode)
        {
            objectSep->addChild(visualisationNode);
        }

#else

        if (UI.checkBoxColModel->isChecked())
        {
            VirtualRobot::MathTools::ConvexHull3DPtr ch = ConvexHullGenerator::CreateConvexHull(object->getCollisionModel()->getTriMeshModel());
            CoinConvexHullVisualizationPtr chv(new CoinConvexHullVisualization(ch));
            SoSeparator* s = chv->getCoinVisualization();

            if (s)
            {
                objectSep->addChild(s);
            }
        }
        else
        {
            SoNode* visualisationNode = CoinVisualizationFactory::getCoinVisualization(object, SceneObject::Full);

            if (visualisationNode)
            {
                objectSep->addChild(visualisationNode);
            }
        }

#endif
        /*SoNode *s = CoinVisualizationFactory::getCoinVisualization(object->getCollisionModel()->getTriMeshModel(),true);
        if (s)
            objectSep->addChild(s);   */
    }

    frictionConeSep->removeAllChildren();
    bool fc = (UI.checkBoxCones->isChecked());

    if (fc && contacts.size() > 0 && qualityMeasure)
    {
        ContactConeGeneratorPtr cg = qualityMeasure->getConeGenerator();
        float radius = cg->getConeRadius();
        float height = cg->getConeHeight();
        float scaling = 30.0f;
        SoNode* visualisationNode = CoinVisualizationFactory::getCoinVisualization(contacts, height * scaling, radius * scaling, true);

        if (visualisationNode)
        {
            frictionConeSep->addChild(visualisationNode);
        }

        // add approach dir visu
        for (size_t i = 0; i < contacts.size(); i++)
        {
            SoSeparator* s = new SoSeparator;
            Eigen::Matrix4f ma;
            ma.setIdentity();
            ma.block(0, 3, 3, 1) = contacts[i].contactPointFingerGlobal;
            SoMatrixTransform* m = CoinVisualizationFactory::getMatrixTransformScaleMM2M(ma);
            s->addChild(m);
            s->addChild(CoinVisualizationFactory::CreateArrow(contacts[i].approachDirectionGlobal, 10.0f, 1.0f));
            frictionConeSep->addChild(s);
        }
    }

    if (UI.checkBoxGrasps->isChecked() && sceneSep->findChild(graspsSep) < 0)
    {
        sceneSep->addChild(graspsSep);
    }

    if (!UI.checkBoxGrasps->isChecked() && sceneSep->findChild(graspsSep) >= 0)
    {
        sceneSep->removeChild(graspsSep);
    }

    viewer->scheduleRedraw();
}

int GraspPlannerWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}


void GraspPlannerWindow::quit()
{
    std::cout << "GraspPlannerWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}

void GraspPlannerWindow::loadObject()
{
    if (!objectFile.empty())
    {
        object = ObjectIO::loadManipulationObject(objectFile);
    }

    if (!object)
    {
        object = Obstacle::createBox(50.0f, 50.0f, 10.0f);
    }

#if 0
    TriMeshModelPtr tm = object->getVisualization()->getTriMeshModel();
    MathTools::ConvexHull3DPtr cv = ConvexHullGenerator::CreateConvexHull(tm->vertices);
    object = GraspStudio::MeshConverter::CreateManipulationObject(object->getName(), cv);
#endif

    //Eigen::Vector3f minS,maxS;
    //object->getCollisionModel()->getTriMeshModel()->getSize(minS,maxS);
    //cout << "minS: \n" << minS << "\nMaxS:\n" << maxS << endl;
    qualityMeasure.reset(new GraspStudio::GraspQualityMeasureWrenchSpace(object));
    //qualityMeasure->setVerbose(true);
    qualityMeasure->calculateObjectProperties();
    approach.reset(new GraspStudio::ApproachMovementSurfaceNormal(object, eef, preshape));
    eefCloned = approach->getEEFRobotClone();

    if (robot && eef)
    {
        std::string name = "Grasp Planner - ";
        name += eef->getName();
        grasps.reset(new GraspSet(name, robot->getType(), eefName));
    }

    planner.reset(new GraspStudio::GenericGraspPlanner(grasps, qualityMeasure, approach));
    planner->setVerbose(true);
}

void GraspPlannerWindow::loadRobot()
{
    robot.reset();
    robot = RobotIO::loadRobot(robotFile);

    if (!robot)
    {
        VR_ERROR << " no robot at " << robotFile << endl;
        return;
    }

    eef = robot->getEndEffector(eefName);

    if (!preshape.empty())
    {
        eef->setPreshape(preshape);
    }

    eefVisu = CoinVisualizationFactory::CreateEndEffectorVisualization(eef);
    eefVisu->ref();
}

void GraspPlannerWindow::plan()
{

    float timeout = UI.spinBoxTimeOut->value() * 1000.0f;
    bool forceClosure = UI.checkBoxFoceClosure->isChecked();
    float quality = (float)UI.doubleSpinBoxQuality->value();
    int nrGrasps = UI.spinBoxGraspNumber->value();
    if (planner)
        planner->setParameters(quality, forceClosure);
    else
        planner.reset(new GraspStudio::GenericGraspPlanner(grasps, qualityMeasure, approach, quality, forceClosure));

    int nr = planner->plan(nrGrasps, timeout);
    VR_INFO << " Grasp planned:" << nr << endl;
    int start = (int)grasps->getSize() - nrGrasps - 1;

    if (start < 0)
    {
        start = 0;
    }

    grasps->setPreshape(preshape);

    for (int i = start; i < (int)grasps->getSize() - 1; i++)
    {
        Eigen::Matrix4f m = grasps->getGrasp(i)->getTcpPoseGlobal(object->getGlobalPose());
        SoSeparator* sep1 = new SoSeparator();
        SoMatrixTransform* mt = CoinVisualizationFactory::getMatrixTransformScaleMM2M(m);
        sep1->addChild(mt);
        sep1->addChild(eefVisu);
        graspsSep->addChild(sep1);
    }

    // set to last valid grasp
    if (grasps->getSize() > 0 && eefCloned && eefCloned->getEndEffector(eefName))
    {
        Eigen::Matrix4f mGrasp = grasps->getGrasp(grasps->getSize() - 1)->getTcpPoseGlobal(object->getGlobalPose());
        eefCloned->setGlobalPoseForRobotNode(eefCloned->getEndEffector(eefName)->getTcp(), mGrasp);
    }

    if (nrGrasps > 0)
    {
        openEEF();
        closeEEF();
    }

    planner->getEvaluation().print();
}



void GraspPlannerWindow::closeEEF()
{
#if 0
    static int pp = 0;
    static Eigen::Vector3f approachDir;
    /*object->getCollisionModel()->getTriMeshModel()->print();
    object->getCollisionModel()->getTriMeshModel()->checkAndCorrectNormals(false);*/
    object->getCollisionModel()->getTriMeshModel()->print();

    if (pp == 0)
    {
        Eigen::Vector3f position;
        approach->getPositionOnObject(position, approachDir);

        // set new pose
        approach->setEEFToApproachPose(position, approachDir);
        //eefCloned->getEndEffector(eefName)->getGCP()->showCoordinateSystem(true);
    }
    else
    {
        approach->moveEEFAway(approachDir, 3.0f, 5);
    }

    pp = (pp + 1) % 5;
    return;
#endif
    contacts.clear();

    if (eefCloned && eefCloned->getEndEffector(eefName))
    {
        contacts = eefCloned->getEndEffector(eefName)->closeActors(object);
        float qual = qualityMeasure->getGraspQuality();
        bool isFC = qualityMeasure->isGraspForceClosure();
        std::stringstream ss;
        ss << std::setprecision(3);
        ss << "Grasp Nr " << grasps->getSize() << "\nQuality: " << qual << "\nForce closure: ";

        if (isFC)
        {
            ss << "yes";
        }
        else
        {
            ss << "no";
        }

        UI.labelInfo->setText(QString(ss.str().c_str()));
    }

    buildVisu();
}

void GraspPlannerWindow::openEEF()
{
    contacts.clear();

    if (eefCloned && eefCloned->getEndEffector(eefName))
    {
        if (!preshape.empty())
        {
            eefCloned->getEndEffector(eefName)->setPreshape(preshape);
        } else
        {
            eefCloned->getEndEffector(eefName)->openActors();
        }
    }

    buildVisu();
}

void GraspPlannerWindow::frictionConeVisu()
{
    buildVisu();
}

void GraspPlannerWindow::colModel()
{
    buildVisu();
}

void GraspPlannerWindow::showGrasps()
{
    buildVisu();
}

void GraspPlannerWindow::save()
{
    if (!object)
    {
        return;
    }

    ManipulationObjectPtr objectM(new ManipulationObject(object->getName(), object->getVisualization()->clone(), object->getCollisionModel()->clone()));
    objectM->addGraspSet(grasps);
    QString fi = QFileDialog::getSaveFileName(this, tr("Save ManipulationObject"), QString(), tr("XML Files (*.xml)"));
    objectFile = std::string(fi.toLatin1());
    bool ok = false;

    try
    {
        ok = ObjectIO::saveManipulationObject(objectM, objectFile);
    }
    catch (VirtualRobotException& e)
    {
        cout << " ERROR while saving object" << endl;
        cout << e.what();
        return;
    }

    if (!ok)
    {
        cout << " ERROR while saving object" << endl;
        return;
    }
}
void GraspPlannerWindow::planObjectBatch()
{
    QString fi = QFileDialog::getExistingDirectory(this, tr("Select Base Directory"), QString());
    qApp->processEvents();
    VR_INFO << "Searching for all .moxml files in " << fi.toStdString() << std::endl;
    if (fi.isEmpty())
    {
        return;
    }
    QStringList paths;
    for (boost::filesystem::recursive_directory_iterator end, dir(fi.toUtf8().data(), boost::filesystem::symlink_option::recurse);
         dir != end ; ++dir)
    {
        std::string path(dir->path().c_str());

        // search for all statechart group xml files
        if (dir->path().extension() == ".moxml")
        {
            paths << dir->path().c_str();
        }

    }
    paths.removeDuplicates();
    VR_INFO << "Found:  " << paths.join(", ").toStdString() << std::endl;
    boost::filesystem::path resultsCSVPath("genericgraspplanningresults-" + robot->getName() + ".csv");
    resultsCSVPath = boost::filesystem::absolute(resultsCSVPath);
    std::ofstream fs(resultsCSVPath.string().c_str(), std::ofstream::out);
    fs << "object," << planner->getEvaluation().GetCSVHeader() << ",RobustnessAvgQualityNoCol,RobustnessAvgForceClosureRateNoCol,RobustnessAvgQualityCol,RobustnessAvgForceClosureRateCol";
    QProgressDialog progress("Calculating grasps...", "Abort", 0, paths.size(), this);
    progress.setWindowModality(Qt::WindowModal);
    progress.show();
    int i = 0;
    progress.setValue(0);
    qApp->processEvents();
    size_t bins = 20;
    for (size_t i = 0; i < bins; ++i)
    {
        fs  << "," << "HistogramBin" << i;
    }
    for (size_t i = 0; i < bins; ++i)
    {
        fs   << "," << "HistogramWithCollisionBin" << i;
    }
    fs << std::endl;

    for(auto& path :  paths)
    {
        try
        {
            //resetSceneryAll();
            objectFile = path.toStdString();
            loadObject();
            {
                //planner->setRetreatOnLowContacts(false);
                plan();
//                save(boost::filesystem::path(path.toStdString()).replace_extension(".moxml").string());
                float avgRate = 0;
                float avgForceClosureRate = 0;
                float avgRateCol = 0;
                float avgForceClosureRateCol = 0;
                size_t graspSum = 0;
                std::vector<double> histogramFC(bins,0.0);
                std::vector<double> histogramFCWithCollisions(bins,0.0);
//                if(planner->getPlannedGrasps().empty())
//                {
//                    ARMARX_INFO << "No grasps found for"
//                }
                for(VirtualRobot::GraspPtr& g : planner->getPlannedGrasps())
                {
                    GraspEvaluationPoseUncertainty::PoseEvalResults result;
                    if (!evaluateGrasp(g, eefCloned, eefCloned->getEndEffector(eefName), 100, result))
                        continue;
                    VR_INFO << "Grasp " << graspSum << "/" << planner->getPlannedGrasps().size() << std::endl;
                    histogramFC.at(std::min<int>((int)(result.forceClosureRate * bins), bins-1))++;
                    histogramFCWithCollisions.at(std::min<int>((int)((double)(result.numForceClosurePoses)/result.numPosesTested * bins), bins-1))++;
                    avgRate += result.avgQuality;
                    avgForceClosureRate += result.forceClosureRate;
                    avgRateCol += result.avgQualityCol;
                    avgForceClosureRateCol += result.forceClosureRateCol;
                    graspSum++;
//                    if(graspSum > 10)
//                        break;
                }
                if (graspSum>0)
                {
                    avgRate /= planner->getPlannedGrasps().size();
                    avgForceClosureRate /= planner->getPlannedGrasps().size();
                    avgRateCol /= planner->getPlannedGrasps().size();
                    avgForceClosureRateCol /= planner->getPlannedGrasps().size();
                }
                fs << object->getName() << "," << planner->getEvaluation().toCSVString() << "," << avgRate << "," << avgForceClosureRate << "," << avgRateCol << "," << avgForceClosureRateCol;
                int i = 0;
                for(auto bin : histogramFC)
                {
                    fs  <<  ", " << (double)(bin)/graspSum;
                    cout << i << ": " << bin << ", " << graspSum << ", " <<  (double)(bin)/graspSum << std::endl;
                    i++;
                }
                for(auto bin : histogramFCWithCollisions)
                {
                    fs  <<  ", " << (double)(bin)/graspSum;
                }
                fs << std::endl;
            }
        }
        catch(std::exception & e)
        {
            VR_ERROR << "Failed to plan for " << path.toStdString() << "\nReason: \n" << e.what() << std::endl;
        }
        progress.setValue(++i);
        qApp->processEvents();
        if (progress.wasCanceled())
            break;
    }
    VR_INFO << "Saving CSV results to " << resultsCSVPath.string() << std::endl;
}

bool GraspPlannerWindow::evaluateGrasp(VirtualRobot::GraspPtr g, VirtualRobot::RobotPtr eefRobot, VirtualRobot::EndEffectorPtr eef, int nrEvalLoops, GraspEvaluationPoseUncertainty::PoseEvalResults &results)
{
    if (!g || !eefRobot || !eef)
        return false;

    GraspEvaluationPoseUncertaintyPtr eval(new GraspEvaluationPoseUncertainty(GraspEvaluationPoseUncertainty::PoseUncertaintyConfig()));

    results = eval->evaluateGrasp(g, eef, object, qualityMeasure, nrEvalLoops);

    return true;
}

