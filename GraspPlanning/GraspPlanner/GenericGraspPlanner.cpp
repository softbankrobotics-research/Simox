#include "GenericGraspPlanner.h"
#include <VirtualRobot/Grasping/Grasp.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotConfig.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <iostream>
#include <sstream>
#include "../GraspQuality/GraspQualityMeasureWrenchSpace.h"
#include "../GraspQuality/GraspQualityMeasure.h"
#include "../ApproachMovementGenerator.h"

#include <chrono>
using namespace std;

namespace GraspStudio
{


    GenericGraspPlanner::GenericGraspPlanner(VirtualRobot::GraspSetPtr graspSet, GraspStudio::GraspQualityMeasurePtr graspQuality, GraspStudio::ApproachMovementGeneratorPtr approach, float minQuality, bool forceClosure)
        : GraspPlanner(graspSet), graspQuality(graspQuality), approach(approach), minQuality(minQuality), forceClosure(forceClosure)
    {
        THROW_VR_EXCEPTION_IF(!graspQuality, "NULL grasp quality...");
        THROW_VR_EXCEPTION_IF(!approach, "NULL approach...");
        THROW_VR_EXCEPTION_IF(!graspQuality->getObject(), "no object...");
        THROW_VR_EXCEPTION_IF(graspQuality->getObject() != approach->getObject(), "graspQuality and approach have to use the same object.");
        object = graspQuality->getObject();
        eef = approach->getEEF();
        THROW_VR_EXCEPTION_IF(!eef, "NULL eef in approach...");
        THROW_VR_EXCEPTION_IF(!graspSet, "NULL graspSet...");
        verbose = true;
        eval.fcCheck = forceClosure;
        eval.minQuality = minQuality;
    }

    GenericGraspPlanner::~GenericGraspPlanner()
    {
    }

    int GenericGraspPlanner::plan(int nrGrasps, int timeOutMS, VirtualRobot::SceneObjectSetPtr obstacles)
    {
        startTime = clock();
        this->timeOutMS = timeOutMS;

        int nLoop = 0;
        int nGraspsCreated = 0;

        if (verbose)
        {
            GRASPSTUDIO_INFO << ": Searching " << nrGrasps << " grasps for EEF:" << approach->getEEF()->getName() << " and object:" << graspQuality->getObject()->getName() << ".\n";
            GRASPSTUDIO_INFO << ": Approach movements are generated with " << approach->getName() << endl;
            GRASPSTUDIO_INFO << ": Grasps are evaluated with " << graspQuality->getName() << endl;
        }

        while (!timeout() && nGraspsCreated < nrGrasps)
        {
            VirtualRobot::GraspPtr g = planGrasp(obstacles);

            if (g)
            {
                if (graspSet)
                {
                    graspSet->addGrasp(g);
                }

                plannedGrasps.push_back(g);
                nGraspsCreated++;
            }

            nLoop++;
        }

        if (verbose)
        {
            GRASPSTUDIO_INFO << ": created " << nGraspsCreated << " valid grasps in " << nLoop << " loops" << endl;
        }

        return nGraspsCreated;
    }

    bool GenericGraspPlanner::moveEEFAway(const Eigen::Vector3f& approachDir, float step, int maxLoops)
    {
        VR_ASSERT(eef);
        VR_ASSERT(approach);

        VirtualRobot::SceneObjectSetPtr sos = eef->createSceneObjectSet();

        if (!sos)
        {
            return false;
        }

        int loop = 0;
        Eigen::Vector3f delta = approachDir * step;
        bool finishedContactsOK = false;
        bool finishedCollision = false;

        while (loop < maxLoops && !finishedCollision && !finishedContactsOK)
        {
            approach->openHand();
            approach->updateEEFPose(delta);

            if (eef->getCollisionChecker()->checkCollision(object->getCollisionModel(), sos))
            {
                finishedCollision = true;
                break;
            }
            auto contacts = eef->closeActors(object);
            if (contacts.size()>=2)
            {
                approach->openHand();
                finishedContactsOK = true;
                break;
            }
            loop++;
        }
        return finishedContactsOK;
    }

    VirtualRobot::GraspPtr GenericGraspPlanner::planGrasp(VirtualRobot::SceneObjectSetPtr obstacles)
    {
        auto start_time = chrono::high_resolution_clock::now();
        std::string sGraspPlanner("Simox - GraspStudio - ");
        sGraspPlanner += graspQuality->getName();
        std::string sGraspNameBase = "Grasp ";

        VirtualRobot::RobotPtr robot = approach->getEEFOriginal()->getRobot();
        VirtualRobot::RobotNodePtr tcp = eef->getTcp();

        VR_ASSERT(robot);
        VR_ASSERT(tcp);


        // GENERATE APPROACH POSE
        bool bRes = approach->setEEFToRandomApproachPose();
        if (bRes && obstacles)
        {
            // CHECK VALID APPROACH POSE
            VirtualRobot::CollisionCheckerPtr colChecker = eef->getCollisionChecker();
            VR_ASSERT(eef->getRobot());
            VR_ASSERT(obstacles);

            if (colChecker->checkCollision(eef->createSceneObjectSet(), obstacles))
            {                
                //                GRASPSTUDIO_INFO << ": Collision detected before closing fingers" << endl;
                //return VirtualRobot::GraspPtr();
                bRes = false;
            }
        }

        // CHECK CONTACTS
        if (bRes)
        {
            contacts = eef->closeActors(object);

            eef->addStaticPartContacts(object, contacts, approach->getApproachDirGlobal());

            // low number of contacts: check if it helps to move away (small object)
            if (contacts.size()<2)
            {
                VR_INFO << "Low number of contacts, retreating hand (small object)" << endl;
                if (moveEEFAway(approach->getApproachDirGlobal(),2.0f,30))
                {
                    contacts = eef->closeActors(object);
                    eef->addStaticPartContacts(object, contacts, approach->getApproachDirGlobal());
                }
            }

            if (obstacles)
            {
                VirtualRobot::CollisionCheckerPtr colChecker = eef->getCollisionChecker();
                VR_ASSERT(eef->getRobot());
                VR_ASSERT(obstacles);

                if (colChecker->checkCollision(eef->createSceneObjectSet(), obstacles))
                {
                    //              GRASPSTUDIO_INFO << ": Collision detected after closing fingers" << endl;
                    //return VirtualRobot::GraspPtr();
                    bRes = false;
                }
            }
        }

        // eval data
        eval.graspTypePower.push_back(true);
        eval.nrGraspsGenerated++;

        if (!bRes)
        {
            // result not valid due to collision
            auto end_time = chrono::high_resolution_clock::now();
            float ms = float(chrono::duration_cast<chrono::milliseconds>(end_time - start_time).count());
            eval.graspScore.push_back(0.0f);
            eval.graspValid.push_back(false);
            eval.nrGraspsInvalidCollision++;
            eval.timeGraspMS.push_back(ms);

            return VirtualRobot::GraspPtr();
        }

        if (contacts.size() < 2)
        {
            if (verbose)
            {
                GRASPSTUDIO_INFO << ": ignoring grasp hypothesis, low number of contacts" << endl;
            }
            // result not valid due to low number of contacts
            auto end_time = chrono::high_resolution_clock::now();
            float ms = float(chrono::duration_cast<chrono::milliseconds>(end_time - start_time).count());
            eval.graspScore.push_back(0.0f);
            eval.graspValid.push_back(false);
            eval.nrGraspsInvalidContacts++;
            eval.timeGraspMS.push_back(ms);
            return VirtualRobot::GraspPtr();
        }

        graspQuality->setContactPoints(contacts);
        float score = graspQuality->getGraspQuality();

        if (forceClosure && !graspQuality->isGraspForceClosure())
        {
            // not force closure
            auto end_time = chrono::high_resolution_clock::now();
            float ms = float(chrono::duration_cast<chrono::milliseconds>(end_time - start_time).count());
            eval.graspScore.push_back(0.0f);
            eval.graspValid.push_back(false);
            eval.nrGraspsInvalidFC++;
            eval.timeGraspMS.push_back(ms);
            return VirtualRobot::GraspPtr();
        }

        if (score < minQuality)
        {
            // min quality not reached
            auto end_time = chrono::high_resolution_clock::now();
            float ms = float(chrono::duration_cast<chrono::milliseconds>(end_time - start_time).count());
            eval.graspScore.push_back(score);
            eval.graspValid.push_back(false);
            eval.nrGraspsInvalidFC++;
            eval.timeGraspMS.push_back(ms);
            return VirtualRobot::GraspPtr();
        }

        // found valid grasp
        if (verbose)
        {
            GRASPSTUDIO_INFO << ": Found grasp with " << contacts.size() << " contacts, score: " << score << endl;
        }

        auto end_time = chrono::high_resolution_clock::now();
        float ms = float(chrono::duration_cast<chrono::milliseconds>(end_time - start_time).count());
        eval.graspScore.push_back(score);
        eval.graspValid.push_back(true);
        eval.nrGraspsValid++;
        // only power grasps
        eval.nrGraspsValidPower++;
        eval.timeGraspMS.push_back(ms);

        std::stringstream ss;
        ss << sGraspNameBase << (graspSet->getSize() + 1);
        std::string sGraspName = ss.str();
        Eigen::Matrix4f objP = object->getGlobalPose();
        Eigen::Matrix4f pLocal = tcp->toLocalCoordinateSystem(objP);
        VirtualRobot::GraspPtr g(new VirtualRobot::Grasp(sGraspName, robot->getType(), eef->getName(), pLocal, sGraspPlanner, score));
        // set joint config
        VirtualRobot::RobotConfigPtr config = eef->getConfiguration();
        std::map< std::string, float > configValues = config->getRobotNodeJointValueMap();
        g->setConfiguration(configValues);
        return g;
    }

    VirtualRobot::EndEffector::ContactInfoVector GenericGraspPlanner::getContacts() const
    {
        return contacts;
    }

    void GenericGraspPlanner::setParameters(float minQuality, bool forceClosure)
    {
        this->minQuality = minQuality;
        this->forceClosure = forceClosure;
    }


    bool GenericGraspPlanner::timeout()
    {
        if (timeOutMS <= 0)
        {
            return false;
        }

        clock_t endTime = clock();
        int timeMS = (int)(((float)(endTime - startTime) / (float)CLOCKS_PER_SEC) * 1000.0);
        return (timeMS > timeOutMS);
    }

} // namespace
