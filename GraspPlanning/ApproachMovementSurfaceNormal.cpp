#include "ApproachMovementSurfaceNormal.h"
#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/EndEffector/EndEffector.h>
#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/SceneObjectSet.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <VirtualRobot/CollisionDetection/CollisionChecker.h>
#include <VirtualRobot/RobotConfig.h>

#include <cstdio>
#include <cstring>
#include <iostream>


class RobotConfig;

using namespace std;
using namespace VirtualRobot;


namespace GraspStudio
{

    ApproachMovementSurfaceNormal::ApproachMovementSurfaceNormal(VirtualRobot::SceneObjectPtr object, VirtualRobot::EndEffectorPtr eef,
            const std::string& graspPreshape, float maxRandDist, bool regardFaceAreas)
        : ApproachMovementGenerator(object, eef, graspPreshape),
          randomDistanceMax(maxRandDist),
          regardFaceAreas(regardFaceAreas)
    {
        name = "ApproachMovementSurfaceNormal";
        
        if (regardFaceAreas)
        {
            initFaceAreas();
        }
    }

    ApproachMovementSurfaceNormal::~ApproachMovementSurfaceNormal()
    = default;

    bool ApproachMovementSurfaceNormal::getPositionOnObject(Eigen::Vector3f& storePos, Eigen::Vector3f& storeApproachDir)
    {
        if (!object || objectModel->faces.size() == 0)
        {
            return false;
        }

        std::size_t faceIndex = 0;
        if (!regardFaceAreas)
        {
            faceIndex = static_cast<std::size_t>(rand()) % objectModel->faces.size();
        }
        else
        {
            float ticket = distrib(randomEngine);
            float currentTotal = 0;
            
            // find face that owns the ticket (i.e. face i with total[i-1] < ticket < total[i])
            std::size_t i = 0;
            for (i = 0; i < faceSizes.size() && currentTotal < ticket; ++i)
            {
                currentTotal += faceSizes[i];
            }
            faceIndex = i;
        }

        std::size_t nVert1 = (objectModel->faces[faceIndex]).id1;
        std::size_t nVert2 = (objectModel->faces[faceIndex]).id2;
        std::size_t nVert3 = (objectModel->faces[faceIndex]).id3;

        storePos = VirtualRobot::MathTools::randomPointInTriangle(objectModel->vertices[nVert1],
                                                                  objectModel->vertices[nVert2],
                                                                  objectModel->vertices[nVert3]);
        
        //storePos = (objectModel->vertices[nVert1] + objectModel->vertices[nVert2] + objectModel->vertices[nVert3]) / 3.0f;
        /*position(0) = (objectModel->vertices[nVert1].x + objectModel->vertices[nVert2].x + objectModel->vertices[nVert3].x) / 3.0f;
          position(1) = (objectModel->vertices[nVert1].y + objectModel->vertices[nVert2].y + objectModel->vertices[nVert3].y) / 3.0f;
          position(2) = (objectModel->vertices[nVert1].z + objectModel->vertices[nVert2].z + objectModel->vertices[nVert3].z) / 3.0f;*/
        
        storeApproachDir = (objectModel->faces[faceIndex]).normal;
        
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
            GRASPSTUDIO_ERROR << "no position on object?!" << endl;
            return pose;
        }

        this->approachDirGlobal = approachDir;

        // set new pose
        setEEFToApproachPose(position,approachDir);


        // move away until valid
        moveEEFAway(approachDir, 1.0f);

        Eigen::Matrix4f poseB = getEEFPose();


        // check if a random distance is wanted
        if (randomDistanceMax > 0)
        {
            float d = static_cast<float>(rand() % 10000) * 0.0001f * randomDistanceMax;
            Eigen::Vector3f delta = approachDir * d;
            updateEEFPose(delta);

            if (!eef_cloned->getCollisionChecker()->checkCollision(object, eef->createSceneObjectSet()))
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
        VirtualRobot::RobotNodePtr graspNode = eef_cloned->getGCP();

        // current pose
        //Eigen::Matrix4f pose = graspNode->getGlobalPose();

        // target pose
        Eigen::Matrix4f poseFinal = Eigen::Matrix4f::Identity();

        // position
        poseFinal.block(0, 3, 3, 1) = position;

        //target orientation
        Eigen::Vector3f z = approachDir;

        while (z.norm() < 1e-10f)
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

            if (y.norm() < 1e-8f)
            {
                continue;
            }

            y.normalize();

            x = y.cross(z);

            if (x.norm() < 1e-8f)
            {
                continue;
            }

            x.normalize();


            // now recalculate y again to obtain a correct base for a right handed coord system
            y = z.cross(x);

            if (y.norm() < 1e-8f)
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
        VirtualRobot::SceneObjectSetPtr sos = eef_cloned->createSceneObjectSet();

        if (!sos)
        {
            return;
        }

        int loop = 0;
        Eigen::Vector3f delta = approachDir * step;

        while (loop < maxLoops && eef_cloned->getCollisionChecker()->checkCollision(object->getCollisionModel(), sos))
        {
            updateEEFPose(delta);
            loop++;
        }
    }

    Eigen::Matrix4f ApproachMovementSurfaceNormal::getEEFPose()
    {
        RobotNodePtr tcp;
        if (!graspPreshape.empty() && eef_cloned->hasPreshape(graspPreshape) && eef_cloned->getPreshape(graspPreshape)->getTCP())
            tcp = eef_cloned->getPreshape(graspPreshape)->getTCP();
        else
            tcp = eef_cloned->getGCP();
        return tcp->getGlobalPose();
    }

    bool ApproachMovementSurfaceNormal::setEEFPose(const Eigen::Matrix4f& pose)
    {
        RobotNodePtr tcp;
        if (!graspPreshape.empty() && eef_cloned->hasPreshape(graspPreshape) && eef_cloned->getPreshape(graspPreshape)->getTCP())
            tcp = eef_cloned->getPreshape(graspPreshape)->getTCP();
        else
            tcp = eef_cloned->getGCP();
        eefRobot->setGlobalPoseForRobotNode(tcp, pose);
        return true;
    }
    
    void ApproachMovementSurfaceNormal::initFaceAreas()
    {
        faceSizesTotal = 0;
        for (const auto& face : objectModel->faces)
        {
            float area = MathTools::getTriangleArea(objectModel->vertices[face.id1],
                    objectModel->vertices[face.id2], objectModel->vertices[face.id3]);
            
            faceSizes.push_back(area);
            faceSizesTotal += area;
        }
        
        std::random_device rd;
        randomEngine = std::default_random_engine(rd());
        distrib = std::uniform_real_distribution<float>(0, faceSizesTotal);
    }
}
