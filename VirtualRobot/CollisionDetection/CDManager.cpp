
#include "CDManager.h"

#include <iostream>
#include <set>
#include <cfloat>
#include "../Robot.h"


using namespace std;

//#define CCM_DEBUG

namespace VirtualRobot
{

    CDManager::CDManager(CollisionCheckerPtr colChecker)
    {
        if (colChecker == nullptr)
        {
            this->colChecker = VirtualRobot::CollisionChecker::getGlobalCollisionChecker();
        }
        else
        {
            this->colChecker = colChecker;
        }
    }

    CDManager::~CDManager() = default;

    void CDManager::addCollisionModel(SceneObjectSetPtr m)
    {
        if (m)
        {
            if (m->getCollisionChecker() != colChecker)
            {
                VR_WARNING << "CollisionModel of SceneObjectSet '" << m->getName() << "' is linked to a different instance of collision checker than this CollisionManager..." << endl;
            }

            for (const auto& colModel : colModels)
            {
                if (m != colModel)
                {
                    addCollisionModelPair(colModel, m);
                }
            }

            if (!_hasSceneObjectSet(m))
            {
                colModels.push_back(m);
            }
        }
    }

    void CDManager::addCollisionModel(SceneObjectPtr m)
    {
        if (m)
        {
            VirtualRobot::SceneObjectSetPtr cms(new VirtualRobot::SceneObjectSet(m->getName() + "Set", colChecker));
            cms->addSceneObject(m);
            addCollisionModel(cms);
        }
    }

    bool CDManager::isInCollision(SceneObjectSetPtr m)
    {
        if (!m || !colChecker)
        {
            VR_WARNING << " NULL data..." << endl;
            return false;
        }

        // check all colmodels
        for (const auto& colModel : colModels)
        {
            if (m != colModel)
            {
                if (colChecker->checkCollision(colModel, m))
                {
                    return true;
                }
            }
        }

        // if here -> no collision
        return false;
    }


    float CDManager::getDistance(SceneObjectSetPtr m)
    {
        float minDist = FLT_MAX;
        float tmp;

        if (!m || !colChecker)
        {
            VR_WARNING << " NULL data..." << endl;
            return 0.0f;
        }

        // check all colmodels
        for (const auto& colModel : colModels)
        {
            if (m != colModel)
            {
                tmp = (float)colChecker->calculateDistance(colModel, m);

                if (tmp < minDist)
                {
                    minDist = tmp;
                }
            }
        }

        return minDist;
    }



    float CDManager::getDistance(SceneObjectSetPtr m, std::vector<SceneObjectSetPtr>& sets)
    {
        float minDist = FLT_MAX;

        for (const auto& set : sets)
        {
            float tmp = (float)colChecker->calculateDistance(m, set);

            if (tmp < minDist)
            {
                minDist = tmp;
            }
        }

        return minDist;
    }


    float CDManager::getDistance()
    {
        float minDist = FLT_MAX;
        float tmp;

        if (!colChecker)
        {
            return -1.0f;
        }

        std::map< SceneObjectSetPtr, std::vector<SceneObjectSetPtr> >::iterator i = colModelPairs.begin();

        while (i != colModelPairs.end())
        {
            tmp = getDistance(i->first, i->second);

            if (tmp < minDist)
            {
                minDist = tmp;
            }

            i++;
        }

        return minDist;
    }

    float CDManager::getDistance(SceneObjectSetPtr m, std::vector<SceneObjectSetPtr>& sets, Eigen::Vector3f& P1, Eigen::Vector3f& P2, int& trID1, int& trID2)
    {
        float minDist = FLT_MAX;
        Eigen::Vector3f _P1;
        Eigen::Vector3f _P2;
        int _trID1;
        int _trID2;

        for (const auto& set : sets)
        {
            float tmp = (float)colChecker->calculateDistance(m, set, _P1, _P2, &_trID1, &_trID2);

            if (tmp < minDist)
            {
                minDist = tmp;
                trID1 = _trID1;
                P1[0] = _P1[0];
                P1[1] = _P1[1];
                P1[2] = _P1[2];
                trID2 = _trID2;
                P2[0] = _P2[0];
                P2[1] = _P2[1];
                P2[2] = _P2[2];
            }
        }

        return minDist;
    }

    float CDManager::getDistance(Eigen::Vector3f& P1, Eigen::Vector3f& P2, int& trID1, int& trID2)
    {
        float minDist = FLT_MAX;
        float tmp;
        Eigen::Vector3f _P1;
        Eigen::Vector3f _P2;
        int _trID1;
        int _trID2;

        if (!colChecker)
        {
            return -1.0f;
        }


        std::map< SceneObjectSetPtr, std::vector<SceneObjectSetPtr> >::iterator i = colModelPairs.begin();

        while (i != colModelPairs.end())
        {
            tmp = getDistance(i->first, i->second, _P1, _P2, _trID1, _trID2);

            if (tmp < minDist)
            {
                minDist = tmp;
                trID1 = _trID1;
                P1[0] = _P1[0];
                P1[1] = _P1[1];
                P1[2] = _P1[2];
                trID2 = _trID2;
                P2[0] = _P2[0];
                P2[1] = _P2[1];
                P2[2] = _P2[2];
            }

            i++;
        }

        return minDist;
    }

    float CDManager::getDistance(SceneObjectSetPtr m, Eigen::Vector3f& P1, Eigen::Vector3f& P2, int& trID1, int& trID2)
    {
        float minDist = FLT_MAX;
        float tmp;
        Eigen::Vector3f _P1;
        Eigen::Vector3f _P2;
        int _trID1;
        int _trID2;

        if (!colChecker || !m)
        {
            return -1.0f;
        }

        for (const auto& colModel : colModels)
        {
            tmp = (float)colChecker->calculateDistance(m, colModel, _P1, _P2, &_trID1, &_trID2);

            if (tmp < minDist)
            {
                minDist = tmp;
                trID1 = _trID1;
                P1[0] = _P1[0];
                P1[1] = _P1[1];
                P1[2] = _P1[2];
                trID2 = _trID2;
                P2[0] = _P2[0];
                P2[1] = _P2[1];
                P2[2] = _P2[2];
            }
        }

        return minDist;
    }



    bool CDManager::isInCollision(SceneObjectSetPtr m, std::vector<SceneObjectSetPtr>& sets)
    {
        for (const auto& set : sets)
        {
            if (colChecker->checkCollision(m, set))
            {
                return true;
            }
        }

        return false;
    }

    bool CDManager::isInCollision()
    {
        if (!colChecker)
        {
            return false;
        }

        std::map<SceneObjectSetPtr, std::vector<SceneObjectSetPtr>  >::iterator i = colModelPairs.begin();

        while (i != colModelPairs.end())
        {
            if (isInCollision(i->first, i->second))
            {
                return true;
            }

            i++;
        }

        return false;
    }


    std::vector<SceneObjectSetPtr> CDManager::getSceneObjectSets()
    {
        return colModels;
    }

    CollisionCheckerPtr CDManager::getCollisionChecker()
    {
        return colChecker;
    }

    bool CDManager::hasSceneObjectSet(SceneObjectSetPtr m)
    {
        for (const auto& colModel : colModels)
        {
            if (colModel == m)
            {
                return true;
            }
        }

        return false;
    }

    bool CDManager::_hasSceneObjectSet(SceneObjectSetPtr m)
    {
        for (auto& colModel : colModels)
        {
            if (colModel == m)
            {
                return true;
            }

            if (m->getSize() == 1 && colModel->getSize() == 1 &&  colModel->getSceneObject(0) == m->getSceneObject(0))
            {
                return true;
            }
        }

        return false;
    }

    bool CDManager::hasSceneObject(SceneObjectPtr m)
    {
        for (auto& colModel : colModels)
        {
            if (colModel->getSize() == 1 &&  colModel->getSceneObject(0) == m)
            {
                return true;
            }
        }

        return false;
    }

    void CDManager::addCollisionModelPair(SceneObjectSetPtr m1, SceneObjectSetPtr m2)
    {
        if (!m1 || !m2)
        {
            return;
        }

        if (!_hasSceneObjectSet(m1))
        {
            colModels.push_back(m1);
        }

        if (!_hasSceneObjectSet(m2))
        {
            colModels.push_back(m2);
        }

        colModelPairs[m1].push_back(m2);
    }

    void CDManager::addCollisionModelPair(SceneObjectPtr m1, SceneObjectSetPtr m2)
    {
        if (!m1 || !m2)
        {
            return;
        }

        VirtualRobot::SceneObjectSetPtr cms(new VirtualRobot::SceneObjectSet("", colChecker));
        cms->addSceneObject(m1);
        addCollisionModelPair(cms, m2);
    }

    void CDManager::addCollisionModelPair(SceneObjectPtr m1, SceneObjectPtr m2)
    {
        if (!m1 || !m2)
        {
            return;
        }

        VirtualRobot::SceneObjectSetPtr cms(new VirtualRobot::SceneObjectSet("", colChecker));
        cms->addSceneObject(m1);
        VirtualRobot::SceneObjectSetPtr cms2(new VirtualRobot::SceneObjectSet("", colChecker));
        cms2->addSceneObject(m2);
        addCollisionModelPair(cms, cms2);

    }

}
