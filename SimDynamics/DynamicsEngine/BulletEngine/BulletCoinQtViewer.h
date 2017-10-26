/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    SimDynamics
* @author     Nikolaus Vahrenkamp
* @copyright  2012 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/

#ifndef _SimDynamics_BulletCoinQtViewer_h_
#define _SimDynamics_BulletCoinQtViewer_h_

#include "../../SimDynamics.h"
#include "../../DynamicsWorld.h"
#include "BulletEngine.h"

#include <btBulletDynamicsCommon.h>
#include <LinearMath/btQuickprof.h>

#include <Gui/ViewerInterface.h>
#include <Gui/Coin/CoinViewer.h>

//todo: use qtimer
#include <Inventor/sensors/SoTimerSensor.h>

#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>
#if QT_VERSION >= 0x050000
#include <QtWidgets/QFrame>
#endif

namespace SimDynamics
{

    class SIMDYNAMICS_IMPORT_EXPORT BulletCoinQtViewer : public SimoxGui::CoinViewer
    {
    public:
        BulletCoinQtViewer(QWidget* parent, DynamicsWorldPtr world, int antiAliasingSteps = 0);
        virtual ~BulletCoinQtViewer();

        /*!
            In this mode, the time between two updates is measures and the engine is stepped accordingly. (standard)
            Could result in inaccurate simulation. Especially, if the steps tend to become large (e.g. on slow computers).
            Also not deterministic.
        */
        void setSimModeRealTime();

        /*!
            Sets the simulation mode to fixed time step. This mode is deterministic.
            The time step can be specified in milli seconds:
            \see setBulletSimTimeStepMsec
            In this mode bulletMaxSubsteps is used as the number of
            full simulation steps bullet performs per frame.
        */
        void setSimModeFixedTimeStep();

        /*!
            How often should the physics engine be updated.
            Standard: 5ms
            On slow computers the update interval might be longer than specified.
            \param updateTimerIntervalMS The timer interval in milliseconds.
        */
        void setUpdateInterval(int updateTimerIntervalMS);

        /*!
            Visualize dynamics object.
        */
        void addSimDynamicsVisualization(VirtualRobot::ModelPtr o,
                              VirtualRobot::ModelLink::VisualizationType visuType = VirtualRobot::ModelLink::Full
                              );
        void addSimDynamicsVisualization(VirtualRobot::ModelLinkPtr o,
                              VirtualRobot::ModelLink::VisualizationType visuType = VirtualRobot::ModelLink::Full
                              );
        void addSimDynamicsVisualization(DynamicsObjectPtr o,
                              VirtualRobot::ModelLink::VisualizationType visuType = VirtualRobot::ModelLink::Full
                              );
        void addSimDynamicsVisualization(DynamicsModelPtr r,
                              VirtualRobot::ModelLink::VisualizationType visuType = VirtualRobot::ModelLink::Full
                              );
        /*!
            Remove visualization of dynamics object.
        */
        void removeSimDynamicsVisualization(VirtualRobot::ModelPtr o);
        void removeSimDynamicsVisualization(VirtualRobot::ModelLinkPtr o);
        void removeSimDynamicsVisualization(DynamicsObjectPtr o);
        void removeSimDynamicsVisualization(DynamicsModelPtr r);

        //! Returns true, if physics engine is running. False, if paused.
        bool engineRunning();

        //! Pauses the physics engine.
        void stopEngine();

        //! Restarts the engine
        void startEngine();

        //! Only allowed when engine is paused.
        virtual void stepPhysics();

        //! Stop callbacks which update the dynamics engine. Shuts down automatic physics engine functionality!
        void stopCB();

        /*!
            Length of a simulation timestep in milliseconds (default 1/60s).
            If necessary, this timestep is applied multiple times depending on the elapsed time since the last frame (e.g. if 50ms elapsed, 3
            timesteps of 1/60s are performed), but at most maxSubSteps times per frame.
        */
        void setBulletSimTimeStepMsec(int timeStep);

        /*!
            Specifies how many sub steps should be performed at most for each render frame (default 1).
            If this value is too low, the simulation will not run in real-time if the computer is not fast enough to render enough frames to
            perform 1/timeStep time steps per second.
            If this value is too high, more time is spent on simulation, increasing visualization lagginess on a slow computer.
            In fixed timestep mode this parameter is used to specifiy how many steps per call to stepEngine()
            are actually calculated by bullet. (Good to speed up simulation)
        */
        void setBulletSimMaxSubSteps(int maxSubSteps);

        /*!
            See setBulletSimTimeStepMsec()
         */
        int getBulletSimTimeStepMsec() const
        {
            return bulletTimeStepMsec;
        }

        /*!
            See setBulletSimMaxSubSteps()
         */
        int getBulletSimMaxSubSteps() const
        {
            return bulletMaxSubSteps;
        }
        int getUpdateTimerInterval() const
        {
            return updateTimerIntervalMS;
        }

        //! If steps==0 anti aliasing is disabled
        void setAntiAliasing(int steps);

        /*!
         * Adds callback that is called each time the engine is updated.
         */
        void addStepCallback(BulletStepCallback callback, void* data);

        //! If set, all actions are protected with this mutex
        virtual void setMutex(std::shared_ptr<std::recursive_mutex> engineMutexPtr);

        typedef std::shared_ptr< std::unique_lock<std::recursive_mutex> > MutexLockPtr;

        /*!
        This lock can be used to protect data access. It locks the mutex until deletion.
        If no mutex was specified, an empty lock will be returned which does not protect the engine calls (this is the standard behavior).
        \see setMutex

        Exemplary usage:
        {
            MutexLockPtr lock = getScopedLock();
            // now the mutex is locked

            // access data
            // ...

        } // end of scope -> lock gets deleted and mutex is released automatically
        */
        MutexLockPtr getScopedLock();

    protected:

        void initSceneGraph(QWidget* parent, int antiAliasingSteps = 0);

        //checks if physics engine is enabled and performes a time step.
        virtual void updatePhysics();

        /*!
            This method is called periodically, triggered by a timer callback.
            It can be overwritten in order to perform custom updates.
            It is safe to access the scene graph.
        */
        virtual void customUpdate() {}

        /*!
            This method is called when a node has been selected by the user.
            It can be overwritten to implement custom reactions.
            It is safe to access the scene graph.
            \param path The path that was selected
        */
        virtual void customSelection(SoPath* path)
        {
            std::cout << "Selecting node " <<  path->getTail()->getTypeId().getName().getString() << endl;
        }

        virtual void customDeselection(SoPath* path)
        {
            std::cout << "Deselecting node " <<  path->getTail()->getTypeId().getName().getString() << endl;
        }

        //! Redraw
        virtual void scheduleRedraw();

        btScalar getDeltaTimeMicroseconds();

        static void timerCB(void* data, SoSensor* sensor);
        static void selectionCB(void* userdata, SoPath* path);
        static void deselectionCB(void* userdata, SoPath* path);

        SoTimerSensor* timerSensor;

        BulletEnginePtr bulletEngine;
        btClock m_clock;

        QTimer *timer;

        int bulletTimeStepMsec;
        int bulletMaxSubSteps;

        bool simModeFixedTimeStep;

        bool warned_norealtime;

        bool enablePhysicsUpdates;
        int updateTimerIntervalMS;

        std::shared_ptr <std::recursive_mutex> engineMutexPtr;
    };

    typedef std::shared_ptr<BulletCoinQtViewer> BulletCoinQtViewerPtr;

} // namespace

#endif
