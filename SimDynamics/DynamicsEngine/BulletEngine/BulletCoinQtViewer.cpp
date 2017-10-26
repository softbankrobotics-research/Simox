#include "BulletCoinQtViewer.h"
/*
#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include "Inventor/actions/SoBoxHighlightRenderAction.h"
#include <Inventor/nodes/SoSelection.h>
*/

#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationNode.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>



using namespace VirtualRobot;
using namespace SimoxGui;

namespace SimDynamics
{


    BulletCoinQtViewer::BulletCoinQtViewer(QWidget* parent, DynamicsWorldPtr world, int antiAliasingSteps)
        : CoinViewer(parent), warned_norealtime(false), simModeFixedTimeStep(false)
    {
        bulletTimeStepMsec = 16; // 60FPS
        bulletMaxSubSteps = 1;
        enablePhysicsUpdates = true;

        updateTimerIntervalMS = 5;

        // no mutex for standard viewer
        //engineMutexPtr.reset(new std::recursive_mutex());

        //const double TIMER_MS = 5.0f;

        SIMDYNAMICS_ASSERT(world);

        bulletEngine = std::dynamic_pointer_cast<BulletEngine>(world->getEngine());

        SIMDYNAMICS_ASSERT(bulletEngine);

        initSceneGraph(parent, antiAliasingSteps);

        /*sceneGraphRoot = new SoSeparator();
        sceneGraphRoot->ref();
        floor = new SoSeparator();
        sceneGraphRoot->addChild(floor);
        sceneGraph = new SoSelection();
        sceneGraphRoot->addChild(sceneGraph);

        //SoSelection *selection = new SoSelection();
        //sceneGraph->addChild( selection );
        viewer = NULL;*/

        // register callback
        SoSensorManager* sensor_mgr = SoDB::getSensorManager();
        timerSensor = new SoTimerSensor(timerCB, this);
        setUpdateInterval(updateTimerIntervalMS);
        sensor_mgr->insertTimerSensor(timerSensor);



        // selection cb
        //sceneGraph->addSelectionCallback(selectionCB, this);
        //sceneGraph->addDeselectionCallback(deselectionCB, this);
    }

    BulletCoinQtViewer::~BulletCoinQtViewer()
    {
        stopCB();
    }
/*
    void BulletCoinQtViewer::selectionCB(void* userdata, SoPath* path)
    {
        BulletCoinQtViewer* bulletViewer = static_cast<BulletCoinQtViewer*>(userdata);
        VR_ASSERT(bulletViewer);

        VR_INFO << "Selected object" << endl;

        bulletViewer->customSelection(path);

        bulletViewer->scheduleRedraw();
    }
    void BulletCoinQtViewer::deselectionCB(void* userdata, SoPath* path)
    {
        BulletCoinQtViewer* bulletViewer = static_cast<BulletCoinQtViewer*>(userdata);
        VR_ASSERT(bulletViewer);

        VR_INFO << "Deselected object" << endl;

        bulletViewer->customDeselection(path);

        bulletViewer->scheduleRedraw();
    }
*/
    void BulletCoinQtViewer::timerCB(void* data, SoSensor* sensor)
    {
        BulletCoinQtViewer* bulletViewer = static_cast<BulletCoinQtViewer*>(data);
        VR_ASSERT(bulletViewer);

        // now its safe to update physical information and set the models to the according poses
        bulletViewer->updatePhysics();

        // perform some custom updates if needed
        bulletViewer->customUpdate();

        bulletViewer->scheduleRedraw();

    }

/*
    void BulletCoinQtViewer::initSceneGraph(QFrame* embedViewer, VirtualRobot::VisualizationPtr scene, int antiAliasingSteps)
    {
        CoinVisualizationPtr cs = std::dynamic_pointer_cast<CoinVisualization>(scene);
        if (!cs)
            return;
        initSceneGraph(embedViewer, cs->getCoinVisualization(false), antiAliasingSteps);
    }
*/

    void BulletCoinQtViewer::initSceneGraph(QWidget* parent, int antiAliasingSteps)
    {
        setAntiAliasing(antiAliasingSteps);

        if (bulletEngine->getFloor())
        {
            // better grid visu
            Eigen::Vector3f floorPos;
            Eigen::Vector3f floorUp;
            double floorExtendMM;
            double floorDepthMM;
            bulletEngine->getFloorInfo(floorPos, floorUp, floorExtendMM, floorDepthMM);
            MathTools::Plane p(floorPos, floorUp);

            VisualizationNodePtr v = VisualizationFactory::getGlobalVisualizationFactory()->createPlane(p, floorExtendMM, 0.0f);
            std::string f1("floor");
            std::string f2("floor");
            CoinViewer::addVisualization(f1, f2, v);
            /*SoNode* n = (SoNode*)CoinVisualizationFactory::CreatePlaneVisualization(floorPos, floorUp, floorExtendMM, 0.0f);
            if (n)
            {
                floor->addChild(n);
                addedVisualizations[bulletEngine->getFloor()] = n;
            }
            //addVisualization(bulletEngine->getFloor());
            */
        }


        //viewer->setSceneGraph(sceneGraphRoot);

        /*if (antiAliasingSteps > 0)
        {
            viewer->setAntialiasing(true, antiAliasingSteps);
        }
        else
        {
            viewer->setAntialiasing(false, 0);
        }*/

        viewAll();
    }

    void BulletCoinQtViewer::scheduleRedraw()
    {
        //this->touch();
    }

    void BulletCoinQtViewer::stepPhysics()
    {
        MutexLockPtr lock = getScopedLock();

        //simple dynamics world doesn't handle fixed-time-stepping
        double ms = getDeltaTimeMicroseconds();

        if (bulletEngine)
        {
            bulletEngine->activateAllObjects(); // avoid sleeping objects

            // Commented out: This is now handled by Bullet (bulletMaxSubSteps * bulletTimeStepMsec is the maximal duration of a frame)
            /* double minFPS = 1000000.f/40.f;  // Don't use 60 Hz (cannot be reached due to Vsync)
            if (ms > minFPS) {
                VR_INFO << "Slow frame (" << ms << "us elapsed)! Limiting elapsed time (losing realtime capabilities for this frame)." << endl;
                ms = minFPS;
            } */
            if (!simModeFixedTimeStep)
            {
                if ((ms / 1000.0f) > bulletMaxSubSteps * bulletTimeStepMsec)
                {
                    if (!warned_norealtime)
                    {
                        VR_INFO << "Elapsed time (" << (ms / 1000.0f) << "ms) too long: Simulation is not running in realtime." << endl;
                        warned_norealtime = true;
                    }
                }
                else
                {
                    warned_norealtime = false;
                }

                btScalar dt1 = btScalar(ms / 1000000.0f);
                bulletEngine->stepSimulation(dt1, bulletMaxSubSteps, float(bulletTimeStepMsec) / 1000.0f);
            }
            else
            {
                // FIXED TIME STEP
                btScalar dt1 = float(bulletTimeStepMsec) / 1000.0f;

                for (int i = 0; i < bulletMaxSubSteps; i++)
                {
                    bulletEngine->stepSimulation(dt1, 1, dt1);
                }
            }

            // VR_INFO << "stepSimulation(" << dt1 << ", " << bulletMaxSubSteps << ", " << (bulletTimeStepMsec / 1000.0f) << ")" << endl;

            //optional but useful: debug drawing
            //m_dynamicsWorld->debugDrawWorld();
        }
    }


    btScalar BulletCoinQtViewer::getDeltaTimeMicroseconds()
    {
        btScalar dt = (btScalar)m_clock.getTimeMicroseconds();
        m_clock.reset();
        return dt;
    }

    /*void BulletCoinQtViewer::viewAll()
    {

        //getCamera()->viewAll(this, viewer->getViewportRegion());
        //viewer->viewAll();
    }*/

    void BulletCoinQtViewer::addSimDynamicsVisualization(RobotPtr robot, VirtualRobot::ModelLink::VisualizationType visuType)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(robot);
        removeSimDynamicsVisualization(robot);
        CoinViewer::addVisualization("robots", robot->getName(), robot->getVisualization(visuType));

        /*SoNode* n = CoinVisualizationFactory::getCoinVisualization(robot,visuType);

        if (n)
        {
            SoNode* rootNode = n;

            if (container)
            {
                container->addChild(n);
                rootNode = container;
            }

            sceneGraph->addChild(rootNode);
            sceneGraph->addSelectionCallback(selectionCB, this);
            sceneGraph->addDeselectionCallback(deselectionCB, this);
            addedSpriteRobotVisualizations[robot] = rootNode;
        }*/
    }

    void BulletCoinQtViewer::addSimDynamicsVisualization(ModelLinkPtr so, VirtualRobot::ModelLink::VisualizationType visuType)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(so);
        removeSimDynamicsVisualization(so);
        CoinViewer::addVisualization("links", so->getName(), so->getVisualization(visuType));

        /*VR_ASSERT(so);
        removeVisualization(so);
        SoNode* n = CoinVisualizationFactory::getCoinVisualization(so, visuType);

        if (n)
        {
            SoNode* rootNode = n;

            if (container)
            {
                container->addChild(n);
                rootNode = container;
            }

            sceneGraph->addChild(rootNode);
            sceneGraph->addSelectionCallback(selectionCB, this);
            sceneGraph->addDeselectionCallback(deselectionCB, this);
            addedSpriteVisualizations[so] = rootNode;
        }*/
    }

    void BulletCoinQtViewer::addSimDynamicsVisualization(DynamicsObjectPtr o, VirtualRobot::ModelLink::VisualizationType visuType)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(o);
        ModelLinkPtr so = o->getSceneObject();
        addSimDynamicsVisualization(so, visuType);
        /*VR_ASSERT(so);
        removeVisualization(o);
        SoNode* n = CoinVisualizationFactory::getCoinVisualization(so, visuType);

        if (n)
        {
            SoNode* rootNode = n;

            if (container)
            {
                container->addChild(n);
                rootNode = container;
            }

            sceneGraph->addChild(rootNode);
            sceneGraph->addSelectionCallback(selectionCB, this);
            sceneGraph->addDeselectionCallback(deselectionCB, this);
            addedVisualizations[o] = rootNode;
        }*/
    }

/*
    void BulletCoinQtViewer::addVisualization(const std::string &name, VisualizationNodePtr visu)
    {
        MutexLockPtr lock = getScopedLock();
        removeVisualization(name);
        CoinVisualizationNodePtr cn = std::dynamic_pointer_cast<CoinVisualizationNode>(visu);
        if (!visu)
            return;

        SoNode* n = cn->getCoinVisualization();

        if (n)
        {
            SoNode* rootNode = n;

            sceneGraph->addChild(rootNode);
            sceneGraph->addSelectionCallback(selectionCB, this);
            sceneGraph->addDeselectionCallback(deselectionCB, this);
            addedCustomVisualizations[name] = rootNode;
        }
    }*/

    void BulletCoinQtViewer::addStepCallback(BulletStepCallback callback, void* data)
    {
        bulletEngine->addExternalCallback(callback, data);
    }

    void BulletCoinQtViewer::addSimDynamicsVisualization(DynamicsModelPtr r, VirtualRobot::ModelLink::VisualizationType visuType)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(r);
        RobotPtr ro = r->getRobot();
        addSimDynamicsVisualization(ro, visuType);
        /*VR_ASSERT(ro);
        removeVisualization(r);

        SoNode* n = CoinVisualizationFactory::getCoinVisualization(ro, visuType);
        SoNode* rootNode = n;

        if (container)
        {
            container->addChild(n);
            rootNode = container;
        }

        sceneGraph->addChild(rootNode);
        addedRobotVisualizations[r] = rootNode;*/
    }

    void BulletCoinQtViewer::removeSimDynamicsVisualization(RobotPtr o)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(o);
        CoinViewer::removeVisualization("robots", o->getName());

        /*if (addedSpriteRobotVisualizations.find(o) != addedSpriteRobotVisualizations.end())
        {
            sceneGraph->removeChild(addedSpriteRobotVisualizations[o]);
            addedSpriteRobotVisualizations.erase(o);
        }*/
    }

    void BulletCoinQtViewer::removeSimDynamicsVisualization(ModelLinkPtr o)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(o);
        CoinViewer::removeVisualization("links", o->getName());

/*
        if (addedSpriteVisualizations.find(o) != addedSpriteVisualizations.end())
        {
            sceneGraph->removeChild(addedSpriteVisualizations[o]);
            addedSpriteVisualizations.erase(o);
        }*/
    }

    void BulletCoinQtViewer::removeSimDynamicsVisualization(DynamicsObjectPtr o)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(o && o->getSceneObject());
        CoinViewer::removeVisualization("links", o->getSceneObject()->getName());

/*        if (addedVisualizations.find(o) != addedVisualizations.end())
        {
            sceneGraph->removeChild(addedVisualizations[o]);
            addedVisualizations.erase(o);
        }*/
    }

    void BulletCoinQtViewer::removeSimDynamicsVisualization(DynamicsModelPtr r)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(r && r->getRobot());
        CoinViewer::removeVisualization("robots", r->getRobot()->getName());

        /*if (addedRobotVisualizations.find(r) != addedRobotVisualizations.end())
        {
            sceneGraph->removeChild(addedRobotVisualizations[r]);
            addedRobotVisualizations.erase(r);
        }*/
    }


   /* void BulletCoinQtViewer::removeVisualization(const std::string &name)
    {
        MutexLockPtr lock = getScopedLock();

        if (addedCustomVisualizations.find(name) != addedCustomVisualizations.end())
        {
            sceneGraph->removeChild(addedCustomVisualizations[name]);
            addedCustomVisualizations.erase(name);
        }
    }*/

    void BulletCoinQtViewer::stopCB()
    {
        MutexLockPtr lock = getScopedLock();

        if (timer)
            timer->stop();
        /*
        if (timerSensor)
        {
            SoSensorManager* sensor_mgr = SoDB::getSensorManager();
            sensor_mgr->removeTimerSensor(timerSensor);
            delete timerSensor;
            timerSensor = NULL;
        }

        if (sceneGraph)
        {
            sceneGraph->removeSelectionCallback(selectionCB, this);
            sceneGraph->removeDeselectionCallback(deselectionCB, this);
        }*/
    }

    void BulletCoinQtViewer::setBulletSimTimeStepMsec(int msec)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(msec > 0);
        bulletTimeStepMsec = msec;
    }

    void BulletCoinQtViewer::setBulletSimMaxSubSteps(int n)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(n > 0);
        bulletMaxSubSteps = n;
    }

    bool BulletCoinQtViewer::engineRunning()
    {
        return enablePhysicsUpdates;
    }

    void BulletCoinQtViewer::stopEngine()
    {
        MutexLockPtr lock = getScopedLock();
        enablePhysicsUpdates = false;
    }
    void BulletCoinQtViewer::startEngine()
    {
        MutexLockPtr lock = getScopedLock();
        enablePhysicsUpdates = true;
    }

    void BulletCoinQtViewer::updatePhysics()
    {
        if (enablePhysicsUpdates)
        {
            stepPhysics();
        }
    }

    void BulletCoinQtViewer::setSimModeRealTime()
    {
        simModeFixedTimeStep = false;
    }

    void BulletCoinQtViewer::setSimModeFixedTimeStep()
    {
        simModeFixedTimeStep = true;
    }

    void BulletCoinQtViewer::setUpdateInterval(int updateTimerIntervalMS)
    {
        this->updateTimerIntervalMS = updateTimerIntervalMS;

        if (timerSensor)
        {
            timerSensor->setInterval(SbTime(float(updateTimerIntervalMS) / 1000.0f));
        }
    }

    void BulletCoinQtViewer::setMutex(std::shared_ptr<std::recursive_mutex> engineMutexPtr)
    {
        this->engineMutexPtr = engineMutexPtr;
    }

    BulletCoinQtViewer::MutexLockPtr BulletCoinQtViewer::getScopedLock()
    {
        std::shared_ptr< std::unique_lock<std::recursive_mutex> > scoped_lock;

        if (engineMutexPtr)
        {
            scoped_lock.reset(new std::unique_lock<std::recursive_mutex>(*engineMutexPtr));
        }

        return scoped_lock;
    }

    void BulletCoinQtViewer::setAntiAliasing(int steps)
    {
        MutexLockPtr lock = getScopedLock();
        CoinViewer::setAntialiasing(true, steps);
    }

}

