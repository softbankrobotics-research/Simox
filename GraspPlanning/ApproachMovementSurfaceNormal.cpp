#include "ApproachMovementSurfaceNormal.h"
#include <VirtualRobot/Model/Model.h>
#include <VirtualRobot/Model/Model.h>
#include <VirtualRobot/Model/Nodes/ModelNode.h>
#include <VirtualRobot/EndEffector/EndEffector.h>
#include <VirtualRobot/Tools/MathTools.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <VirtualRobot/CollisionDetection/CollisionChecker.h>

#include <cstdio>
#include <cstring>
#include <iostream>

using namespace std;

namespace GraspPlanning
{

    ApproachMovementSurfaceNormal::ApproachMovementSurfaceNormal(VirtualRobot::ModelPtr object, VirtualRobot::EndEffectorPtr eef, const std::string& graspPreshape, float maxRandDist)
        : ApproachMovementGenerator(object, eef, graspPreshape)
    {
        name = "ApproachMovementSurfaceNormal";
        randomDistanceMax = maxRandDist;
    }

    ApproachMovementSurfaceNormal::~ApproachMovementSurfaceNormal()
    {
    }

    bool ApproachMovementSurfaceNormal::getPositionOnObject(Eigen::Vector3f& storePos, Eigen::Vector3f& storeApproachDir)
    {
        if (!object || objectModel->faces.size() == 0)
        {
            return false;
        }

        int nRandFace = rand() % objectModel->faces.size();
        int nVert1 = (objectModel->faces[nRandFace]).id1;
        int nVert2 = (objectModel->faces[nRandFace]).id2;
        int nVert3 = (objectModel->faces[nRandFace]).id3;

        storePos = VirtualRobot::MathTools::randomPointInTriangle(objectModel->vertices[nVert1], objectModel->vertices[nVert2], objectModel->vertices[nVert3]);
        //storePos = (objectModel->vertices[nVert1] + objectModel->vertices[nVert2] + objectModel->vertices[nVert3]) / 3.0f;
        /*position(0) = (objectModel->vertices[nVert1].x + objectModel->vertices[nVert2].x + objectModel->vertices[nVert3].x) / 3.0f;
        position(1) = (objectModel->vertices[nVert1].y + objectModel->vertices[nVert2].y + objectModel->vertices[nVert3].y) / 3.0f;
        position(2) = (objectModel->vertices[nVert1].z + objectModel->vertices[nVert2].z + objectModel->vertices[nVert3].z) / 3.0f;*/

        storeApproachDir = (objectModel->faces[nRandFace]).normal;
        return true;
    }

    Eigen::Matrix4f ApproachMovementSurfaceNormal::createNewApproachPose()
    {
        // store current pose
        Eigen::Matrix4f pose = getEEFPose();
        openHand();
        Eigen::Vector3f position;
        Eigen::Vector3f approachDir;

        if (!getPositionOnObject(position, approachDir))
        {
            GRASPPLANNING_ERROR << "no position on object?!" << endl;
            return pose;
        }

        this->aporachDirGlobal = approachDir;

        // set new pose
        setEEFToApproachPose(position,approachDir);


        // move away until valid
        moveEEFAway(approachDir, 1.0f);

        Eigen::Matrix4f poseB = getEEFPose();


        // check if a random distance is wanted
        if (randomDistanceMax > 0)
        {
            float d = (float)(rand() % 10000) * 0.0001f * randomDistanceMax;
            Eigen::Vector3f delta = approachDir * d;
            updateEEFPose(delta);

            if (!eef_cloned->getCollisionChecker()->checkCollision(object, eef->createLinkSet()))
            {
                poseB = getEEFPose();
            } // else remain at original pose

        }


        // restore original pose
        setEEFPose(pose);

        return poseB;
    }

    bool ApproachMovementSurfaceNormal::setEEFToApproachPose(const Eigen::Vector3f& position, const Eigen::Vector3f& approachDir)
    {
        VirtualRobot::CoordinatePtr graspNode = eef_cloned->getGCP();

        // current pose
        //Eigen::Matrix4f pose = graspNode->getGlobalPose();

        // target pose
        Eigen::Matrix4f poseFinal = Eigen::Matrix4f::Identity();

        // position
        poseFinal.block(0, 3, 3, 1) = position;

        //target orientation
        Eigen::Vector3f z = approachDir;

        while (z.norm() < 1e-10)
        {
            z.setRandom();
        }

        z.normalize();
        z *= -1.0f;

        Eigen::Vector3f y;
        Eigen::Vector3f x;

        // create a random rotation around approach vector
        bool bSuccess = false;
        int loop = 0;

        while (!bSuccess)
        {
            loop++;

            if (loop > 1000)
            {
                VR_ERROR << "INTERNAL ERROR, aborting..." << endl;
                return false;
            }

            //random y dir vector
            y.setRandom();

            if (y.norm() < 1e-8)
            {
                continue;
            }

            y.normalize();

            x = y.cross(z);

            if (x.norm() < 1e-8)
            {
                continue;
            }

            x.normalize();


            // now recalculate y again to obtain a correct base for a right handed coord system
            y = z.cross(x);

            if (y.norm() < 1e-8)
            {
                continue;
            }

            y.normalize();

            bSuccess = true;
        }

        poseFinal.block(0, 0, 3, 1) = x;
        poseFinal.block(0, 1, 3, 1) = y;
        poseFinal.block(0, 2, 3, 1) = z;

        setEEFPose(poseFinal);
        return true;
    }

    void ApproachMovementSurfaceNormal::moveEEFAway(const Eigen::Vector3f& approachDir, float step, int maxLoops)
    {
        VirtualRobot::LinkSetPtr sos = eef_cloned->createLinkSet();

        if (!sos)
        {
            return;
        }

        int loop = 0;
        Eigen::Vector3f delta = approachDir * step;

        while (loop < maxLoops && eef_cloned->getCollisionChecker()->checkCollision(object->getLinks().at(0), sos))
        {
            updateEEFPose(delta);
            loop++;
        }
    }
}
