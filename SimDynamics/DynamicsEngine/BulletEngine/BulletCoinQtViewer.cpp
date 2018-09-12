#include "BulletCoinQtViewer.h"

#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h>
#include <Inventor/sensors/SoTimerSensor.h>

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

        SIMDYNAMICS_ASSERT(world);

        bulletEngine = std::dynamic_pointer_cast<BulletEngine>(world->getEngine());

        SIMDYNAMICS_ASSERT(bulletEngine);

        initSceneGraph(parent, antiAliasingSteps);

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

            VisualizationPtr v = VisualizationFactory::getInstance()->createPlane(p, floorExtendMM);
            CoinViewer::addVisualization(v, "floor");
        }

        viewAll();
    }

    void BulletCoinQtViewer::scheduleRedraw()
    {

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

    void BulletCoinQtViewer::addSimDynamicsVisualization(RobotPtr robot, VirtualRobot::ModelLink::VisualizationType visuType)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(robot);
        removeSimDynamicsVisualization(robot);
        robotVisus[robot->getName()] = robot->getVisualization(visuType);
        CoinViewer::addVisualization(robotVisus[robot->getName()], "robots");
    }

    void BulletCoinQtViewer::addSimDynamicsVisualization(ModelLinkPtr so, VirtualRobot::ModelLink::VisualizationType visuType)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(so);
        removeSimDynamicsVisualization(so);
        linkVisus[so->getName()] = so->getVisualization(visuType);
        CoinViewer::addVisualization(linkVisus[so->getName()], "links");
    }

    void BulletCoinQtViewer::addSimDynamicsVisualization(DynamicsObjectPtr o, VirtualRobot::ModelLink::VisualizationType visuType)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(o);
        ModelLinkPtr so = o->getSceneObject();
        addSimDynamicsVisualization(so, visuType);
    }

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
    }

    void BulletCoinQtViewer::removeSimDynamicsVisualization(RobotPtr o)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(o);
        auto it = robotVisus.find(o->getName());
        if (it != robotVisus.end())
        {
            CoinViewer::removeVisualization(it->second, "robots");
            robotVisus.erase(it);
        }
    }

    void BulletCoinQtViewer::removeSimDynamicsVisualization(ModelLinkPtr o)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(o);
        auto it = linkVisus.find(o->getName());
        if (it != linkVisus.end())
        {
            CoinViewer::removeVisualization(it->second, "links");
            linkVisus.erase(it);
        }
    }

    void BulletCoinQtViewer::removeSimDynamicsVisualization(DynamicsObjectPtr o)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(o && o->getSceneObject());
        auto it = linkVisus.find(o->getSceneObject()->getName());
        if (it != linkVisus.end())
        {
            CoinViewer::removeVisualization(it->second, "links");
            linkVisus.erase(it);
        }
    }

    void BulletCoinQtViewer::removeSimDynamicsVisualization(DynamicsModelPtr r)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(r && r->getRobot());
        auto it = robotVisus.find(r->getRobot()->getName());
        if (it != robotVisus.end())
        {
            CoinViewer::removeVisualization(it->second, "robots");
            robotVisus.erase(it);
        }
    }

    void BulletCoinQtViewer::stopCB()
    {
        MutexLockPtr lock = getScopedLock();

        if (timer)
            timer->stop();
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
        CoinViewer::setAntialiasing(steps);
    }

}

