
#include "InvReachWindow.h"

#include <VirtualRobot/EndEffector/EndEffector.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/XML/ObjectIO.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/Workspace/Reachability.h>
#include <VirtualRobot/Workspace/WorkspaceGrid.h>
#include <VirtualRobot/XML/SceneIO.h>
#include <VirtualRobot/Trajectory.h>

#include <QFileDialog>
#include <Eigen/Geometry>
#include <time.h>
#include <vector>
#include <iostream>
#include <cmath>

#include "Inventor/actions/SoLineHighlightRenderAction.h"
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoBaseColor.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoUnits.h>

#include <sstream>
using namespace std;
using namespace VirtualRobot;
using namespace Eigen;

float TIMER_MS = 30.0f;

//#define ATLAS

//#define BEQUIET

// test: clear reach data and add one pose at tcp
//#define TEST_AT_TCP
// rotate by this angle (only needed in TEST_AT_TCP mode)
//#define TEST_AT_TCP_TEST_ANGLE 0

InvReachWindow::InvReachWindow(std::string &sRobotFile, std::string &reachFile, std::string &eef, std::string &invReachFile, std::string &rnsNameCollisionDetection, 
                               std::string &rnFootL, std::string &rnFootR, std::vector<std::string> environments, Qt::WFlags flags)
:QMainWindow(NULL)
{
	VR_INFO << " start " << endl;
	currentScene = 0;
	robotFile = sRobotFile;
	VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robotFile);
	this->rnsNameCollisionDetection = rnsNameCollisionDetection;
    envFiles = environments;
	sceneSep = new SoSeparator;
	sceneSep->ref();
	robotVisuSep = new SoSeparator;
	robotVisuSep->ref();
	reachabilityVisuSep = new SoSeparator;
	reachabilityVisuSep->ref();
	reachabilityMapVisuSep = new SoSeparator;
	reachabilityMapVisuSep->ref();
	allGraspsVisuSep = new SoSeparator;
	allGraspsVisuSep->ref();
	targetVisuSep = new SoSeparator;
	targetVisuSep->ref();
	objectVisuSep = new SoSeparator;
	objectVisuSep->ref();
	trajectoryVisuSep = new SoSeparator;
	trajectoryVisuSep->ref();
	footLRVisuSep = new SoSeparator;
	footLRVisuSep->ref();
	reachableFootVisuSep = new SoSeparator;
	reachableFootVisuSep->ref();
    performanceTest= false;

	sceneSep->addChild(objectVisuSep);
	sceneSep->addChild(targetVisuSep);
	sceneSep->addChild(reachabilityVisuSep);
	sceneSep->addChild(reachabilityMapVisuSep);
	sceneSep->addChild(trajectoryVisuSep);
	sceneSep->addChild(reachableFootVisuSep);

	buildObject();

	setupUI();

	loadRobot();

	if (robot)
	{
		if (robot->hasRobotNode(rnFootL))
			footL = robot->getRobotNode(rnFootL);
		else
		{
			VR_WARNING << "Could not find footL:" << rnFootL << endl;
		}
        if (!rnFootL.empty() && robot->hasRobotNode(rnFootR))
			footR = robot->getRobotNode(rnFootR);
		else
		{
            if (!rnFootL.empty())
                VR_WARNING << "Could not find footR:" << rnFootR << endl;
		}
	}

	if (!reachFile.empty())
	{
		if (RuntimeEnvironment::getDataFileAbsolute(reachFile))
			loadReachFile(reachFile);
	}
	VR_ASSERT(reachSpace);

    this->invReachFile = invReachFile;
    if (VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(invReachFile))
    {
        invReach.reset(new InverseReachability(robot,invReachFile));
		invReach->print();
	} else
	{
        cout << "Building IRD..." << endl;
		invReach.reset(new InverseReachability(reachSpace,1.0f,1.0f));
        cout << "Saving IRD file to " << invReachFile << endl;
        invReach->save(invReachFile);
	}
    if (footL && footR)
    {
        InverseReachabilityCoinVisualization::buildFeetPolygons(footLRVisuSep, footL, footR, reachSpace, trafoBaseToFootL, trafoBaseToFootR);
        Eigen::Matrix4f gp = reachSpace->getBaseNode()->getGlobalPose();
        InverseReachabilityCoinVisualization::addFootstepVisu(reachableFootVisuSep,footLRVisuSep, gp);
    } else if (footL)
    {
        InverseReachabilityCoinVisualization::buildFootPolygon(footLRVisuSep, footL, reachSpace, trafoBaseToFootL);
        Eigen::Matrix4f gp = reachSpace->getBaseNode()->getGlobalPose();
        InverseReachabilityCoinVisualization::addFootstepVisu(reachableFootVisuSep,footLRVisuSep, gp);
    }

	selectScene(0);

	updateVisu();

	if (!eef.empty())
	{
		selectEEF(eef);
	}

    setObjectRandom();
	viewer->viewAll();
}


InvReachWindow::~InvReachWindow()
{
	robotVisuSep->unref();
	reachabilityVisuSep->unref();
	reachabilityMapVisuSep->unref();
	allGraspsVisuSep->unref();
	targetVisuSep->unref();
	objectVisuSep->unref();
	footLRVisuSep->unref();
	reachableFootVisuSep->unref();
	sceneSep->unref();
}


void InvReachWindow::setupUI()
{
	 UI.setupUi(this);
	 viewer = new SoQtExaminerViewer(UI.frameViewer,"",TRUE,SoQtExaminerViewer::BUILD_POPUP);

	// setup
	viewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
	viewer->setAccumulationBuffer(true);
	
#ifdef WIN32
	viewer->setAntialiasing(true, 8);
#endif
	viewer->setGLRenderAction(new SoLineHighlightRenderAction);
	viewer->setTransparencyType(SoGLRenderAction::SORTED_OBJECT_SORTED_TRIANGLE_ADD);
	viewer->setFeedbackVisibility(true);
	viewer->setSceneGraph(sceneSep);
	viewer->viewAll();
	UI.comboBoxScene->addItem(QString("Table"));
	UI.comboBoxScene->addItem(QString("Kitchen"));

	connect(UI.pushButtonTargetRandom, SIGNAL(clicked()), this, SLOT(setObjectRandom()));
	//connect(UI.pushButton1, SIGNAL(clicked()), this, SLOT(test1()));
    connect(UI.btnBuildCompleteReachMap, SIGNAL(clicked()), this, SLOT(createFullReachGrid()));
	//connect(UI.pushButtonTraj, SIGNAL(clicked()), this, SLOT(testTrajectory()));
	connect(UI.pushButtonRobotPose, SIGNAL(clicked()), this, SLOT(ikRobotPose()));
	connect(UI.pushButtonIK, SIGNAL(clicked()), this, SLOT(ikAll()));

	connect(UI.checkBoxRobot, SIGNAL(clicked()), this, SLOT(updateVisu()));
	connect(UI.checkBoxTarget, SIGNAL(clicked()), this, SLOT(updateVisu()));
	//connect(UI.checkBoxObject, SIGNAL(clicked()), this, SLOT(updateVisu()));
    connect(UI.checkBoxReachabilityVisu, SIGNAL(clicked()), this, SLOT(updateVisu()));
	connect(UI.checkBoxReachabilityMapVisu, SIGNAL(clicked()), this, SLOT(updateVisu()));
	connect(UI.checkBoxFloor, SIGNAL(clicked()), this, SLOT(updateVisu()));

	//connect(UI.radioButtonAllGrasps, SIGNAL(clicked()), this, SLOT(selectGrasp()));
	//connect(UI.radioButtonOneGrasp, SIGNAL(clicked()), this, SLOT(selectGrasp()));
	//connect(UI.comboBoxGrasp, SIGNAL(currentIndexChanged(int)), this, SLOT(selectGrasp()));
	connect(UI.comboBoxEEF, SIGNAL(currentIndexChanged(int)), this, SLOT(selectEEF()));
	connect(UI.comboBoxScene, SIGNAL(currentIndexChanged(int)), this, SLOT(selectScene()));
	connect(UI.comboBoxTrajectory, SIGNAL(currentIndexChanged(int)), this, SLOT(selectTrajectory()));
	connect(UI.horizontalSliderTrajSol, SIGNAL(sliderMoved(int)), this, SLOT(trajectorySolutionUpdate()));
	
	UI.comboBoxEEF->setEnabled(false);
}


QString InvReachWindow::formatString(const char *s, float f)
{
	QString str1(s);
	if (f>=0)
		str1 += " ";
	if (fabs(f)<1000)
		str1 += " ";
	if (fabs(f)<100)
		str1 += " ";
	if (fabs(f)<10)
		str1 += " ";
	QString str1n;
	str1n.setNum(f,'f',3);
	str1 = str1 + str1n;
	return str1;
}
void InvReachWindow::resetSceneryAll()
{
	if (!robot)
		return;
	std::vector< RobotNodePtr > nodes;
	robot->getRobotNodes(nodes);
	std::vector<float> jv(nodes.size(),0.0f);
	robot->setJointValues(nodes,jv);
}


void InvReachWindow::updateVisu()
{
	if (UI.checkBoxRobot->isChecked())
	{
		if (robotVisuSep->getNumChildren()==0)
		{
			InverseReachabilityCoinVisualization::buildRobotVisu(robotVisuSep,robot);
		}
		if (sceneSep->findChild(robotVisuSep)<0)
			sceneSep->addChild(robotVisuSep);
	} else
	{
		if (sceneSep->findChild(robotVisuSep)>=0)
			sceneSep->removeChild(robotVisuSep);
	}
	/*if (UI.checkBoxTarget->isChecked())
	{
		if (targetVisuSep->getNumChildren()==0)
		{
			buildGraspVisu();
		}
		if (sceneSep->findChild(targetVisuSep)<0)
			sceneSep->addChild(targetVisuSep);
	} else
	{
		if (sceneSep->findChild(targetVisuSep)>=0)
			sceneSep->removeChild(targetVisuSep);
	}*/

	objectVisuSep->removeAllChildren();
	if (UI.checkBoxTarget->isChecked() || UI.checkBoxFloor->isChecked())
	{
		if (objectVisuSep->getNumChildren()==0)
		{
			Eigen::Vector3f p(0,0,0);
            //if (currentScene==1)
            //	p(0) = -5000.0f;
			InverseReachabilityCoinVisualization::buildObjectVisu(objectVisuSep,UI.checkBoxTarget->isChecked(),UI.checkBoxFloor->isChecked(),graspObject,environment,p);
            if (UI.checkBoxTarget->isChecked() && eefClone)
                objectVisuSep->addChild(eefClone->getVisualization<CoinVisualization>()->getCoinVisualization());
		}
		if (sceneSep->findChild(objectVisuSep)<0)
			sceneSep->addChild(objectVisuSep);
	} else
	{
		if (sceneSep->findChild(objectVisuSep)>=0)
			sceneSep->removeChild(objectVisuSep);
	}
    if (UI.checkBoxReachabilityVisu->isChecked())
	{
        if (reachabilityVisuSep->getNumChildren()==0 || true)
		{
			Eigen::Matrix4f gp = getTargetPose();
            cout << gp << endl;
            cout << "Visualizing IRD" << endl;
			InverseReachabilityCoinVisualization::buildReachVisu(reachabilityVisuSep,UI.checkBoxReachabilityVisu->isChecked(),invReach, gp);
		}
        if (sceneSep->findChild(reachabilityVisuSep)<0)
        {
            cout << "Adding IRD-node to scene graph" << endl;
            sceneSep->addChild(reachabilityVisuSep);
        }
	} else
	{
		if (sceneSep->findChild(reachabilityVisuSep)>=0)
			sceneSep->removeChild(reachabilityVisuSep);
	}
	if (UI.checkBoxReachabilityMapVisu->isChecked())
	{
		if (reachabilityMapVisuSep->getNumChildren()==0)
		{
			InverseReachabilityCoinVisualization::buildReachMapVisu(reachabilityMapVisuSep,UI.checkBoxReachabilityMapVisu->isChecked(),invReach,reachGrid,reachableFootVisuSep,footLRVisuSep);
		}
		if (sceneSep->findChild(reachabilityMapVisuSep)<0)
			sceneSep->addChild(reachabilityMapVisuSep);
		if (sceneSep->findChild(reachableFootVisuSep)<0)
			sceneSep->addChild(reachableFootVisuSep);
	} else
	{
		if (sceneSep->findChild(reachabilityMapVisuSep)>=0)
			sceneSep->removeChild(reachabilityMapVisuSep);
		if (sceneSep->findChild(reachableFootVisuSep)>=0)
			sceneSep->removeChild(reachableFootVisuSep);
	}
}

void InvReachWindow::closeEvent(QCloseEvent *event)
{
	quit();
	QMainWindow::closeEvent(event);
}


int InvReachWindow::main()
{
	SoQt::show(this);
	SoQt::mainLoop();
	return 0;
}


void InvReachWindow::quit()
{
	std::cout << "InvReachWindow: Closing" << std::endl;
	this->close();
	SoQt::exitMainLoop();
}

void InvReachWindow::updateEEFBox()
{
	UI.comboBoxEEF->clear();
	if (!robot)
	{
		//selectGrasp();
		return;
	}
	std::vector<EndEffectorPtr> eefs = robot->getEndEffectors();

	for (unsigned int i=0;i<eefs.size();i++)
	{
		UI.comboBoxEEF->addItem(QString(eefs[i]->getName().c_str()));
	}
	selectEEF(0);
}

void InvReachWindow::selectGrasp()
{
	if (!grasps)
		return;

	targetVisuSep->removeAllChildren();
	
	if (invReach && graspObject)
	{
		Eigen::Matrix4f graspGlobal = getTargetPose();
		invReach->setGlobalPose(graspGlobal);
        VR_INFO << "Goal:\n" << graspGlobal << endl;
        eefClone->setGlobalPoseForRobotNode(eefClone->getRobotNode(eef->getTcpName()),graspGlobal);

	}
	buildReachMap();

	/*
	if (UI.radioButtonAllGrasps->isChecked())
	{
		buildReachMapAll();
	} else
	{
		GraspPtr g = grasps->getGrasp(UI.comboBoxGrasp->currentIndex());
		if (invReach && g)
		{
			Eigen::Matrix4f graspGlobal = g->getTcpPoseGlobal(graspObject->getGlobalPose());
			invReach->setGlobalPose(graspGlobal);
		}
		buildReachMap(g);
	}*/
    if (!performanceTest)
	    updateVisu();
}


void InvReachWindow::selectEEF()
{
	selectEEF(UI.comboBoxEEF->currentIndex());
}

void InvReachWindow::selectEEF(int nr)
{
	if (eef && eef->getTcp())
	{
		eef->getTcp()->showCoordinateSystem(false);
	}
	eef.reset();
	grasps.reset();
	//UI.comboBoxGrasp->clear();
	targetVisuSep->removeAllChildren();
	if (!robot)
	{
		//selectGrasp();
		return;
	}
	cout << "Selecting EEF nr " << nr << endl;

	std::vector<EndEffectorPtr> eefs = robot->getEndEffectors();
	std::string tcp = "<not set>";
	if (nr<0 || nr>=(int)eefs.size())
		return;

	eef = eefs[nr];
	if (eef && eef->getTcp())
	{
		eef->getTcp()->showCoordinateSystem(true,2.0f);
    }
    eefClone = eef->createEefRobot(eef->getName(),eef->getName());

	/*if (graspObject)
	{
		grasps = graspObject->getGraspSet(eef);
		if (grasps)
		{
			for (unsigned int i=0;i<grasps->getSize();i++)
			{
				UI.comboBoxGrasp->addItem(QString(grasps->getGrasp(i)->getName().c_str()));
			}
		}
	}*/
	selectGrasp();
}

void InvReachWindow::selectScene()
{
	selectScene(UI.comboBoxScene->currentIndex());
}

void InvReachWindow::selectScene(int nr)
{
	currentScene = nr;
	targetVisuSep->removeAllChildren();
	reachabilityVisuSep->removeAllChildren();
	objectVisuSep->removeAllChildren();

	setupEnvironment(nr);
	UI.pushButtonTargetRandom->setEnabled(nr==0); // only allow random object pose for nr=0 -> table
	UI.comboBoxTrajectory->setEnabled(nr==1); // only allow trajectory for nr=1 -> kitchen
	UI.comboBoxTrajectory->clear();
	trajectories.clear();
	trajectorySolution.valid = false;
	
	if (environment && nr==1)
	{

		// kitchen -> build trajectories
		GraspSetPtr gs = environment->getGraspSet("Dishwasher");
		if (gs && gs->getGrasps().size()>0)
		{
			std::vector< Eigen::Matrix4f > t;
			for (int i=0;i<(int)gs->getGrasps().size();i++)
			{
				GraspPtr g = gs->getGrasp(i);
				Eigen::Matrix4f gp = g->getTcpPoseGlobal(environment->getGlobalPose());
#ifdef ATLAS
				// rotate orientation
				//gp = gp*MathTools::axisangle2eigen4f(Eigen::Vector3f::UnitY(),(float)-M_PI/2.0f);
				//gp = gp*MathTools::axisangle2eigen4f(Eigen::Vector3f::UnitZ(),(float)M_PI/2.0f);
#endif
				t.push_back(gp);
			}
			trajectories["Dishwasher"] = t;
			UI.comboBoxTrajectory->addItem(QString("Dishwasher"));
		}

		gs = environment->getGraspSet("Drawer");
		if (gs && gs->getGrasps().size()>0)
		{
			std::vector< Eigen::Matrix4f > t;
			for (int i=0;i<(int)gs->getGrasps().size();i++)
			{
				GraspPtr g = gs->getGrasp(i);
				Eigen::Matrix4f gp = g->getTcpPoseGlobal(environment->getGlobalPose());
#ifdef ATLAS
				// rotate orientation
				gp = gp*MathTools::axisangle2eigen4f(Eigen::Vector3f::UnitY(),(float)-M_PI/2.0f);
				gp = gp*MathTools::axisangle2eigen4f(Eigen::Vector3f::UnitZ(),(float)M_PI/2.0f);
				gp(2,3) -= 50.0f; // lift the trajectory for better access
#endif
				t.push_back(gp);
			}
			trajectories["Drawer"] = t;
			UI.comboBoxTrajectory->addItem(QString("Drawer"));
		}

		gs = environment->getGraspSet("Fridge");
		if (gs && gs->getGrasps().size()>0)
		{
			std::vector< Eigen::Matrix4f > t;
			for (int i=0;i<(int)gs->getGrasps().size();i++)
			{
				GraspPtr g = gs->getGrasp(i);
				Eigen::Matrix4f gp = g->getTcpPoseGlobal(environment->getGlobalPose());
#ifdef ATLAS
				// rotate orientation
				gp = gp*MathTools::axisangle2eigen4f(Eigen::Vector3f::UnitY(),(float)-M_PI/2.0f);
				gp = gp*MathTools::axisangle2eigen4f(Eigen::Vector3f::UnitZ(),(float)M_PI/2.0f);
#endif
				t.push_back(gp);
			}
			trajectories["Fridge"] = t;
			UI.comboBoxTrajectory->addItem(QString("Fridge"));

		}
		selectTrajectory();
	}
	updateVisu();

}

void InvReachWindow::selectEEF( std::string &eef )
{
	if (!robot)
		return;
	trajectorySolution.valid = false;
	std::vector<EndEffectorPtr> eefs = robot->getEndEffectors();
	for (size_t i=0;i<eefs.size();i++)
	{
		if (eefs[i]->getName() == eef)
		{
			selectEEF((int)i);
			UI.comboBoxEEF->setCurrentIndex((int)i);
		}
	}
}

void InvReachWindow::loadRobot()
{
	robotVisuSep->removeAllChildren();

	cout << "Loading robot from " << robotFile << endl;

	try
	{
		robot = RobotIO::loadRobot(robotFile);
	}
	catch (VirtualRobotException &e)
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

	BoundingBox bb = robot->getBoundingBox();
    float h = -bb.getMin()(2);
	Eigen::Matrix4f gp = Eigen::Matrix4f::Identity();
	gp(2,3) = h;
	robot->setGlobalPose(gp);

	/*RobotNodePtr fbase = robot->getRobotNode("FeetBase");
	RobotNodePtr base = robot->getRobotNode("TorsoBase");
	RobotNodePtr lfoot = robot->getRobotNode("l_foot");
	RobotNodePtr rfoot = robot->getRobotNode("r_foot");
	fbase->showCoordinateSystem(true,3.0f);
	base->showCoordinateSystem(true,3.0f);
	lfoot->showCoordinateSystem(true,3.0f);
	rfoot->showCoordinateSystem(true,3.0f);

	Eigen::Matrix4f lfoot2base = lfoot->toLocalCoordinateSystem(base->getGlobalPose());
	Eigen::Matrix4f rfoot2base = rfoot->toLocalCoordinateSystem(base->getGlobalPose());

	cout << "LFOOT:\n" << lfoot2base << endl;
	cout << "RFOOT:\n" << lfoot2base << endl;*/

    updateEEFBox();
	// build visualization
	updateVisu();
	viewer->viewAll();
}

void InvReachWindow::loadReachFile(std::string filename)
{
	if (!robot)
		return;
	reachFile = filename;
#ifdef MANIPULABILITY
	reachSpace.reset(new Manipulability(robot));
#else
	reachSpace.reset(new Reachability(robot));
#endif

	reachSpace->load(reachFile);
	reachSpace->print();
	currentRobotNodeSet = reachSpace->getNodeSet();
	currentRobotNodeColSet = reachSpace->getCollisionModelDynamic();
}

void InvReachWindow::setObjectRandom()
{
	if (graspObject)
	{
		Eigen::Matrix4f gp;
		gp.setIdentity();
		gp(0,3) = 50.0f + (float)(rand()%1100);
		gp(1,3) = -50.0f - (float)(rand()%720);
		gp(2,3) = 1035.0f;
		float a = float(rand()%1000) / 1000.0f * 2.0f*float(M_PI);
		float b = float(rand()%1000) / 1000.0f * 2.0f*float(M_PI);
		float c = float(rand()%1000) / 1000.0f * 2.0f*float(M_PI);
		Eigen::AngleAxis<float> t(a,Eigen::Vector3f(0,0,1));
		Eigen::Transform<float,3,Eigen::Affine> t2(t);
		Eigen::AngleAxis<float> t3(b,Eigen::Vector3f(0,1,0));
		Eigen::Transform<float,3,Eigen::Affine> t4(t3);
		Eigen::AngleAxis<float> t5(c,Eigen::Vector3f(1,0,0));
		Eigen::Transform<float,3,Eigen::Affine> t6(t5);
        gp = gp * t2.matrix() * t4.matrix() * t6.matrix();

		graspObject->setGlobalPose(gp);
        if (eefClone)
        {
            VR_INFO << "gp:\n" << gp << endl;
            eefClone->setGlobalPoseForRobotNode(eefClone->getRobotNode(eef->getTcpName()),gp);
        }
        selectGrasp();
        updateVisu();
	}
}
void InvReachWindow::setupEnvironment(int nr)
{
	trajectoryVisuSep->removeAllChildren();
    //pipelineScene.reset();
    if (nr < envFiles.size())
	{

        std::string objectFile = envFiles.at(nr);

		if (!RuntimeEnvironment::getDataFileAbsolute(objectFile))
		{
			VR_ERROR << "No path to " << objectFile << endl;
			return;
		}
		try {
			environment = ObjectIO::loadManipulationObject(objectFile);
		} catch (VirtualRobotException e)
		{
			VR_ERROR << "Could not load " << objectFile << endl;
			cout << e.what();
			return;
		}
		if (!environment)
			return;

		Eigen::Matrix4f gp;
		gp.setIdentity();
		if (nr==1)
			gp(2,3) = -60.0f; // move down kitchen
		environment->setGlobalPose(gp);
		setObjectRandom();
	} else
	{
        VR_ERROR << "could not select environment " << nr << endl;
		environment.reset();
		/*
		// setup pipeline environment
		std::string ps("C:/Projects/Simox_Projects/OrientedReachabilityMap/scene/scenePipeline.xml");
		pipelineScene = SceneIO::loadScene(ps);
		if (!pipelineScene)
			return;
		graspObject = pipelineScene->getManipulationObject("Vitalis");
		robot = pipelineScene->getRobot("Armar3");
		updateEEFBox();

		// build visualization
		updateVisu();
		viewer->viewAll();
		*/
	}
	
}

void InvReachWindow::buildObject()
{
	graspObject = ManipulationObject::createSphere(20.0f);
}

bool InvReachWindow::buildReachMapAll()
{
	reachabilityVisuSep->removeAllChildren();
	reachabilityMapVisuSep->removeAllChildren();

	if (!grasps)
		return false;

	Eigen::Vector3f minBB,maxBB;
	reachSpace->getWorkspaceExtends(minBB,maxBB);
	/*reachGrid.reset(new WorkspaceGrid(minBB(0),maxBB(0),minBB(1),maxBB(1),reachSpace->getDiscretizeParameterTranslation()));

	Eigen::Matrix4f gp = graspObject->getGlobalPose();
	reachGrid->setGridPosition(gp(0,3),gp(1,3));

	for (int i=0;i<(int)grasps->getSize();i++)
	{
		GraspPtr g = grasps->getGrasp(i);
		reachGrid->fillGridData(reachSpace, graspObject, g, robot->getRootNode() );
	}
	updateVisu();*/
	return true;
}

bool InvReachWindow::buildReachMap( )
{
	/*
	if (!reachSpace)
		return false;
	reachabilityVisuSep->removeAllChildren();
	reachabilityMapVisuSep->removeAllChildren();
	Eigen::Vector3f minBB,maxBB;
	reachSpace->getWorkspaceExtends(minBB,maxBB);
	/ *reachGrid.reset(new WorkspaceGrid(minBB(0),maxBB(0),minBB(1),maxBB(1),reachSpace->getDiscretizeParameterTranslation()));

	Eigen::Matrix4f gp = graspObject->getGlobalPose();
	reachGrid->setGridPosition(gp(0,3),gp(1,3));

	reachGrid->fillGridData(reachSpace, graspObject, g, robot->getRootNode() );

	updateVisu();*/
	return true;

}

void InvReachWindow::test1()
{
	cout << "\n*********************************************************" << endl;
    cout << "PERFORMING ikALL test" << endl;

    performanceTest = true;
	int loops = 100;

	cout << "Step 1: random object pose (FULL)" << endl;
	if (currentScene==0)
		createRobotPlacementIK(false);
	else
		createRobotPlacementTrajectoryIK(false);
	if (robotPlacementIK)
		robotPlacementIK->resetPerformanceMeasure();
	if (robotPlacementTrajectoryIK)
		robotPlacementTrajectoryIK->resetPerformanceMeasure();

	for (int i=0;i<loops;i++)
	{
		setObjectRandom();
		doIK(false);
	}
	if (currentScene==0)
		robotPlacementIK->printPerformanceMeasure();
	else
		robotPlacementTrajectoryIK->printPerformanceMeasure();
	if (currentScene==0)
	{

		cout << "***********\n\nStep 2: fixed object pose (FULL)" << endl;
		createRobotPlacementIK(false);
		if (robotPlacementIK)
			robotPlacementIK->resetPerformanceMeasure();
		for (int i=0;i<loops;i++)
		{
			doIK(false);
		}
		robotPlacementIK->printPerformanceMeasure();    
	}

	cout << "***********\n\nStep 3: random object pose (LAZY)" << endl;
	if (currentScene==0)
		createRobotPlacementIK(true);
	else
		createRobotPlacementTrajectoryIK(true);
	if (robotPlacementIK)
		robotPlacementIK->resetPerformanceMeasure();
	if (robotPlacementTrajectoryIK)
		robotPlacementTrajectoryIK->resetPerformanceMeasure();

	for (int i=0;i<loops;i++)
	{
		setObjectRandom();
		doIK(false);
	}
	if (currentScene==0)
		robotPlacementIK->printPerformanceMeasure();
	else
		robotPlacementTrajectoryIK->printPerformanceMeasure();

	if (currentScene==0)
	{

		cout << "***********\n\nStep 4: fixed object pose (LAZY)" << endl;
		createRobotPlacementIK(true);
		if (robotPlacementIK)
			robotPlacementIK->resetPerformanceMeasure();

		for (int i=0;i<loops;i++)
		{
			doIK(false);
		}
		robotPlacementIK->printPerformanceMeasure();
	}

    performanceTest = false;
    selectGrasp();
	cout << "\n\n";
    return;

	reachabilityVisuSep->removeAllChildren();
	reachabilityMapVisuSep->removeAllChildren();


#ifdef TEST_AT_TCP
	reachSpace->clear();
    reachSpace->setOrientationType(WorkspaceRepresentation::EulerXYZ);
	Eigen::Matrix4f gp1 = Eigen::Matrix4f::Identity();
	//gp1.block(0,3,3,1) = reachSpace->getTcp()->getGlobalPose().block(0,3,3,1);
	gp1 = reachSpace->getTcp()->getGlobalPose();
	cout << "tcp pose:\n" << gp1 << endl;
	reachSpace->addPose(gp1);
	reachSpace->print();
	Eigen::Matrix4f gp = reachSpace->getTcp()->getGlobalPose();
	invReach.reset(new InverseReachability(reachSpace));

	float tr = reachSpace->getDiscretizeParameterTranslation();
	float rot = reachSpace->getDiscretizeParameterRotation();
	cout << "Discretization trans:" << tr << ", rot:" << rot << endl;
	reachGrid.reset(new OrientedWorkspaceGrid(-3000.0f,3000.0f,-3000.0f,2000.0f,tr,rot));
	reachGrid->setGridPosition(gp(0,3),gp(1,3));

	return;
#endif


	invReach.reset(new InverseReachability(reachSpace));
	//invReach->save(invReachFile);
    /*GraspPtr g;
    if (grasps)
        g = grasps->getGrasp(UI.comboBoxGrasp->currentIndex());
    if (g)
    {
        Eigen::Matrix4f graspGlobal = g->getTcpPoseGlobal(graspObject->getGlobalPose());
        invReach->setGlobalPose(graspGlobal);
    }*/
	invReach->setGlobalPose(getTargetPose());
	//reachGrid->fillGridData(reachSpace, graspObject, g, robot->getRootNode() );
	updateVisu();
}


void InvReachWindow::selectTrajectory()
{
	trajectoryVisuSep->removeAllChildren();
	trajectorySolution.valid = false;

	std::vector<Eigen::Matrix4f> traj;
	//std::string trajName = UI.comboBoxTrajectory->currentText().toStdString();
	std::string trajName = UI.comboBoxTrajectory->currentText().toLocal8Bit().data();
	cout << "Using trajectory " << trajName << endl;
	traj = trajectories[trajName];
	if (traj.size()==0)
		return;


	SoSeparator *eefSep = CoinVisualizationFactory::CreateCoordSystemVisualization(2.0f);
		//CreateEndEffectorVisualization(eef);
	// visualize
	for (int i=0;i<int(traj.size());i++)
	{
		SoSeparator *s = new SoSeparator;
		SoMatrixTransform *t = CoinVisualizationFactory::getMatrixTransformScaleMM2M(traj[i]);
		s->addChild(t);
		s->addChild(eefSep);
		trajectoryVisuSep->addChild(s);	
	}
}

void InvReachWindow::testTrajectory()
{
/*
	if (!invReach)
		return;
	GraspPtr g;
	// select trajectory
	std::vector<Eigen::Matrix4f> traj;
	if (currentScene==0)
	{
		int nrTrajPoints = 10;


		Eigen::Matrix4f start = getTargetPose();
		Eigen::Vector3f dir(-800.0f,0,0);

		for (int i=0;i<nrTrajPoints;i++)
		{
			Eigen::Matrix4f p = start;
			p.block(0,3,3,1) += dir * float(i) / float(nrTrajPoints-1);

			traj.push_back(p);	
		}

	} else
	{
		//std::string trajName = UI.comboBoxTrajectory->currentText().toStdString();
		std::string trajName = UI.comboBoxTrajectory->currentText().toLocal8Bit().data();
		cout << "Using trajectory " << trajName << endl;
		traj = trajectories[trajName];
		if (traj.size()==0)
			return;
	}

	createRobotPlacementTrajectoryIK(true);
	trajectorySolution = robotPlacementTrajectoryIK->solve(traj,VirtualRobot::IKSolver::All,500);
	//robotPlacementTrajectoryIK->setRobotPoseTargetReachable(traj,30);
	reachGrid = robotPlacementTrajectoryIK->getReachGrid();
	InverseReachabilityCoinVisualization::buildReachMapVisu(reachabilityMapVisuSep,UI.checkBoxReachabilityMapVisu->isChecked(),invReach,reachGrid,reachableFootVisuSep,footLRVisuSep);
	*/
}

#define ALL_ROT
void InvReachWindow::createFullReachGrid()
{
	reachabilityMapVisuSep->removeAllChildren();
    if (!invReach)
        return;

    // build reachmap
    float tr = reachSpace->getDiscretizeParameterTranslation();
    float rot = reachSpace->getDiscretizeParameterRotation();
    cout << "Discretization trans:" << tr << ", rot:" << rot << endl;
    reachGrid.reset(new OrientedWorkspaceGrid(-3000.0f,3000.0f,-3000.0f,2000.0f,tr,rot,false/*, getBaseHeight()*/)); // TODO (harry) why tr/rot from reach and not inv-reach ?
    Eigen::Matrix4f gp = getTargetPose();
    reachGrid->setGridPosition(gp(0,3),gp(1,3));

	Eigen::Matrix4f graspGlobal = getTargetPose();
	// get grasp
	GraspPtr g;
	/*GraspPtr g;
	if (grasps)
		g = grasps->getGrasp(UI.comboBoxGrasp->currentIndex());
	if (!g)
	{
		if (!currentRobotNodeSet)
			return;
		graspGlobal = currentRobotNodeSet->getTCP()->getGlobalPose();
	} else
	{
		graspGlobal = g->getTcpPoseGlobal(graspObject->getGlobalPose());
	}*/

#ifdef TEST_AT_TCP
	cout << " TEST at TCP pose" << endl;
	Eigen::Matrix4f gp = Eigen::Matrix4f::Identity();
	//gp.block(0,3,3,1) = reachSpace->getTcp()->getGlobalPose().block(0,3,3,1);
	gp = reachSpace->getTcp()->getGlobalPose();
    Eigen::AngleAxis<float> t(TEST_AT_TCP_TEST_ANGLE,Eigen::Vector3f::UnitZ());
    Eigen::Transform<float,3,Eigen::Affine> trRot(t);
	graspGlobal = gp*trRot.matrix();
	cout << "tcp pose:\n" << gp << endl;
#endif
	invReach->setGlobalPose(graspGlobal);
	clock_t startTime = clock();

	reachGrid->fillData(graspGlobal,invReach,g);
	clock_t startTime2 = clock();
	int timeMS = (int)(startTime2-startTime);
	cout << "Time for building oriented reach grid (ms):" << timeMS << endl;
	cout << "Filled " << reachGrid->getCellsFilled() << " reachgrid cells. Max entry:" << reachGrid->getMaxEntry() << endl;
	cout << "Showing full Reach grid" << endl;
	InverseReachabilityCoinVisualization::buildReachMapVisu(reachabilityMapVisuSep,UI.checkBoxReachabilityMapVisu->isChecked(),invReach,reachGrid,reachableFootVisuSep,footLRVisuSep);
   createRobotPlacementIK(true);
}


void InvReachWindow::ikRobotPose()
{
    if (!invReach)
        return;
    if (!robotPlacementIK)
        createRobotPlacementIK(true);

	Eigen::Matrix4f graspGlobal = getTargetPose();
	// get grasp
	/*GraspPtr g;
	if (grasps)
		g = grasps->getGrasp(UI.comboBoxGrasp->currentIndex());
	if (!g)
	{
		if (!currentRobotNodeSet)
			return;
		graspGlobal = currentRobotNodeSet->getTCP()->getGlobalPose();
	} else
	{
		graspGlobal = g->getTcpPoseGlobal(graspObject->getGlobalPose());
	}*/

	clock_t startTime = clock();
	robotPlacementIK->setRobotPoseTargetReachable(graspGlobal);
	clock_t startTime2 = clock();
	int timeMS = (int)(startTime2-startTime);
	cout << "Time for searching a potential robot base pose (ms):" << timeMS << endl;
	reachGrid = robotPlacementIK->getReachGrid();
	InverseReachabilityCoinVisualization::buildReachMapVisu(reachabilityMapVisuSep,UI.checkBoxReachabilityMapVisu->isChecked(),invReach,reachGrid,reachableFootVisuSep,footLRVisuSep);
}

void InvReachWindow::createRobotPlacementIK(bool lazyGridUpdate)
{
	if (!invReach)
		return;
	clock_t startTime = clock();
	robotPlacementIK.reset(new RobotPlacementIK(reachSpace->getBaseNode(),reachSpace->getNodeSet(),invReach,lazyGridUpdate));
	robotPlacementIK->setupJacobian(0.8f,20);
	clock_t startTime2 = clock();
	int timeMS = (int)(startTime2-startTime);
	cout << "Time for building ik solver (ms):" << timeMS << endl;
}

void InvReachWindow::createRobotPlacementTrajectoryIK(bool lazyGridUpdate)
{
	if (!invReach)
		return;
	clock_t startTime = clock();
	robotPlacementTrajectoryIK.reset(new RobotPlacementTrajectoryIK(reachSpace->getBaseNode(),reachSpace->getNodeSet(),invReach,lazyGridUpdate));
	robotPlacementTrajectoryIK->setupJacobian(0.8f,20);
	clock_t startTime2 = clock();
	int timeMS = (int)(startTime2-startTime);
	cout << "Time for building trajectory ik solver (ms):" << timeMS << endl;
}

void InvReachWindow::ikAll()
{
    doIK(true);
	if (robotPlacementIK)
		cout << "REACH GRID MAX ENTRY: " << robotPlacementIK->getReachGrid()->getMaxEntry() << endl;
/*
	cout << "\n\n ************ quality evaluation (full orm) ****************" << endl;
	createRobotPlacementIK(false);
	cout << "List: maxGridentry, reachspace entry, current real manipulability, reach grid entry  \t;\t " << endl;
	for (int i=0;i<100;i++)
	{
		doIK(false,false,1);
		if (robotPlacementIK)
			cout << (int)robotPlacementIK->getReachGrid()->getMaxEntry() << " , ";
		//	cout << "REACH GRID MAX ENTRY: " << robotPlacementIK->getReachGrid()->getMaxEntry() << endl;
		calculateManipulability(false);
	}
	*/
}

void InvReachWindow::calculateManipulability(bool output)
{
	/*
	manip.reset(new PoseQualityExtendedManipulability(currentRobotNodeSet));
	RobotNodeSetPtr rns1 = robot->getRobotNodeSet("PlatformTorsoHeadColModel");
	RobotNodeSetPtr rns2 = robot->getRobotNodeSet("RightHandColModel");
	if (rns1 && rns2)
	{
		int id1;
		int id2;
		Eigen::Vector3f p1;
		Eigen::Vector3f p2;
		float d = rns1->getCollisionChecker()->calculateDistance(rns1,rns2,p1,p2,&id1,&id2);
		//cout << "#### dist:" << d << ", ";
		Eigen::Matrix4f obstDistPos1 = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f obstDistPos2 = Eigen::Matrix4f::Identity();
		obstDistPos1.block(0,3,3,1) = p1;
		obstDistPos2.block(0,3,3,1) = p2;

		// transform to tcp
		Eigen::Matrix4f p1_tcp = currentRobotNodeSet->getTCP()->toLocalCoordinateSystem(obstDistPos1);
		Eigen::Matrix4f p2_tcp = currentRobotNodeSet->getTCP()->toLocalCoordinateSystem(obstDistPos2);
		Eigen::Vector3f minDistVector = p1_tcp.block(0,3,3,1) - p2_tcp.block(0,3,3,1);

		manip->setObstacleDistanceVector(minDistVector);
	} else
		cout << "no col rns?!" << endl;
	if (currentScene==0)
	{	
		// TABLE SCENE
		Eigen::Matrix4f graspGlobal = Eigen::Matrix4f::Identity();
		// get grasp
		GraspPtr g;
		if (grasps)
			g = grasps->getGrasp(UI.comboBoxGrasp->currentIndex());
		if (!g)
		{
			if (!currentRobotNodeSet)
				return;
			graspGlobal = currentRobotNodeSet->getTCP()->getGlobalPose();
		} else
		{
			graspGlobal = g->getTcpPoseGlobal(graspObject->getGlobalPose());
		}

		if (reachSpace)
		{
			unsigned char e = reachSpace->getEntry(graspGlobal);
			if (output)
				cout << "REACH SPACE ENTRY:" << (int)e << endl;
			else
				cout << int(e) << " , ";
		}
		float q = manip->getPoseQuality();
		if (output)
			cout << "EXTENDED MANIPULABILITY:" << q << endl;
		else
			cout << q << " , ";

		if (robotPlacementIK && invReach)
		{
			float posEuler[6];
			invReach->matrix2Vector(robot->getGlobalPose(),posEuler);
			if (output)
			{
				cout << "Robot pose: ";
				for (int k=0;k<6;k++)
					cout << posEuler[k] << ",";
				cout << endl;
			}
			int e2 = robotPlacementIK->getReachGrid()->getEntry(posEuler[0],posEuler[1],posEuler[5]);
			if (output)
				cout << "REACH GRID ENTRY:" << e2 << endl;
			else
				cout << e2 << " \t;\t " ;
		}

	} else
	{
		// KITCHEN SCENE

		
		std::string trajName = UI.comboBoxTrajectory->currentText().toStdString();
		
		std::vector< Eigen::Matrix4f > traj = trajectories[trajName];
		if (traj.size()==0)
			return;
	}
	*/
}

void InvReachWindow::doIK(bool output, bool considerCollision, int minQuality)
{
    if (!invReach)
        return;
	if (!robotPlacementIK)
		createRobotPlacementIK(true);
	if (!robotPlacementTrajectoryIK)
		createRobotPlacementTrajectoryIK(false);
	reachabilityMapVisuSep->removeAllChildren();
	if (currentRobotNodeSet)
	{
		std::vector<float> jv(currentRobotNodeSet->getSize(),0.0f);
		currentRobotNodeSet->setJointValues(jv);
	}

	// setup collision detection
	if (environment && robot && robot->hasRobotNodeSet(rnsNameCollisionDetection))
	{
		if (considerCollision)
		{

#ifndef BEQUIET
			cout << "Setting up collision detection:" << endl;
			cout << "\t" << environment->getName() << "<->" << rnsNameCollisionDetection << endl;
			cout << "\t" << environment->getName() << "<->" << currentRobotNodeColSet->getName() << endl;
#endif
			VirtualRobot::CDManagerPtr cdm(new CDManager());
			RobotNodeSetPtr rnsCol = robot->getRobotNodeSet(rnsNameCollisionDetection);
			cdm->addCollisionModelPair(environment,rnsCol);
			cdm->addCollisionModelPair(environment,currentRobotNodeColSet);
			if (robotPlacementIK)
				robotPlacementIK->collisionDetection(cdm);
			if (robotPlacementTrajectoryIK)
				robotPlacementTrajectoryIK->collisionDetection(cdm);
		} else
		{
			robotPlacementIK->collisionDetection(VirtualRobot::CDManagerPtr());
		}
	} 
#ifdef BEQUIET
	if (robotPlacementIK)
		robotPlacementIK->setVerbose(false);
	if (robotPlacementTrajectoryIK)
		robotPlacementTrajectoryIK->setVerbose(false);
#endif

	clock_t startTime,endTime;
	bool res;


	if (currentScene==0)
	{	
		// TABLE SCENE
		Eigen::Matrix4f graspGlobal = getTargetPose();

		startTime = clock();
		res = robotPlacementIK->solve(graspGlobal,VirtualRobot::IKSolver::All,500,minQuality);
		endTime = clock();
		reachGrid = robotPlacementIK->getReachGrid();
		int timeMS = (int)(endTime-startTime);
		if (output)
		{
			cout << "Time for searching a potential robot base pose (ms):" << timeMS << endl;
			robotPlacementIK->getReachGrid()->print();
			robotPlacementIK->printPerformanceMeasure();
		}
		InverseReachabilityCoinVisualization::buildReachMapVisu(reachabilityMapVisuSep,UI.checkBoxReachabilityMapVisu->isChecked(),invReach,reachGrid,reachableFootVisuSep,footLRVisuSep);
	} else
	{
		// KITCHEN SCENE
		
		//std::string trajName = UI.comboBoxTrajectory->currentText().toStdString();
		std::string trajName = UI.comboBoxTrajectory->currentText().toLocal8Bit().data();
		if (output)
			cout << "Using trajectory " << trajName << endl;
		std::vector< Eigen::Matrix4f > traj = trajectories[trajName];
		if (traj.size()==0)
			return;

		/*int bestEntry = int (float(reachGrid->getMaxEntry()) * 0.75f);
		if (bestEntry<1)
			bestEntry = 1;
		int minEntry = 1;
		int maxLoops = 200;*/

		startTime = clock();
		//res = robotPlacementIK->solve(graspGlobal,VirtualRobot::IKSolver::All,500);
		//res = robotPlacementIK->searchRobotPose(reachGrid,minEntry,bestEntry,maxLoops);
		trajectorySolution = robotPlacementTrajectoryIK->solve(traj,VirtualRobot::IKSolver::All,500);
		
		endTime = clock();
		res = trajectorySolution.valid;
		reachGrid = robotPlacementTrajectoryIK->getReachGrid();
		int timeMS = (int)(endTime-startTime);
		if (output)
		{
			cout << "Time for searching a potential robot base pose (ms):" << timeMS << endl;
			robotPlacementTrajectoryIK->getReachGrid()->print();
			robotPlacementTrajectoryIK->printPerformanceMeasure();
		}
		InverseReachabilityCoinVisualization::buildReachMapVisu(reachabilityMapVisuSep,UI.checkBoxReachabilityMapVisu->isChecked(),invReach,reachGrid,reachableFootVisuSep,footLRVisuSep);
	}
	
	if (res)
	{
        if (output)
		    cout << "IK Success" << endl;
	} else
		cout << "IK FAILED !!!" << endl;    
	

}

void InvReachWindow::trajectorySolutionUpdate()
{
	if (!trajectorySolution.valid || trajectorySolution.jsTrajectory.size()==0)
		return;
	float pos = float(UI.horizontalSliderTrajSol->sliderPosition()) / float(UI.horizontalSliderTrajSol->maximum());
	cout << "Trajectory pos: " << pos << endl;
	if (trajectorySolution.jsTrajectory[0].rns != reachSpace->getNodeSet()->getName())
	{
		VR_ERROR << trajectorySolution.jsTrajectory[0].rns << " != " << reachSpace->getNodeSet()->getName() << endl;
		return;
	}

	VirtualRobot::TrajectoryPtr tr(new VirtualRobot::Trajectory(reachSpace->getNodeSet()));
	size_t nrJoints = trajectorySolution.jsTrajectory[0].jointValues.size();
	for (size_t i=0;i<trajectorySolution.jsTrajectory.size();i++)
	{
		Eigen::VectorXf trPoint(nrJoints);
		for (size_t j=0;j<nrJoints;j++)
		{
			trPoint(j) = trajectorySolution.jsTrajectory[i].jointValues[j];
		}
		tr->addPoint(trPoint);
	}
	//tr->print();
	Eigen::VectorXf resPos(reachSpace->getNodeSet()->getSize());
	tr->interpolate(pos,resPos);
	reachSpace->getNodeSet()->setJointValues(resPos);
}

float InvReachWindow::getBaseHeight()
{
	if (reachSpace && reachSpace->getBaseNode())
		return reachSpace->getBaseNode()->getGlobalPose()(2,3);
	return 0;
}

Eigen::Matrix4f InvReachWindow::getTargetPose()
{
	Eigen::Matrix4f p = Eigen::Matrix4f::Identity();
	if (graspObject)
		p = graspObject->getGlobalPose();
/*
#ifdef ATLAS
	// move the target pose up, since we consider the TorsoJoint as base

	float z = 928.0f;
	//if (reachSpace && reachSpace->getBaseNode())
	//	z = reachSpace->getBaseNode()->getGlobalPose()(2,3);

	p(2,3) += z;
#endif*/
	/*
	GraspPtr g;
	if (grasps && graspObject)
		g = grasps->getGrasp(UI.comboBoxGrasp->currentIndex());
	if (!g)
	{
		if (!currentRobotNodeSet)
			return;
		graspGlobal = currentRobotNodeSet->getTCP()->getGlobalPose();
	} else
	{
		graspGlobal = g->getTcpPoseGlobal(graspObject->getGlobalPose());
	}*/

		//Eigen::Matrix4f graspGlobal = g->getTcpPoseGlobal(graspObject->getGlobalPose());

	return p;
}
