
#include "CollisionChecker.h"
#include "CollisionModel.h"
#include "../Model/ModelNodeSet.h"
#include "../Model/Nodes/ModelNode.h"
#include "../Model/Model.h"
#include "../VirtualRobotException.h"

#include <cfloat>

#include <Eigen/Core>
#include <Eigen/Geometry>


#if defined(VR_COLLISION_DETECTION_PQP)
#define COL_CHECKER_IMPL CollisionCheckerPQP
#else
#define COL_CHECKER_IMPL CollisionCheckerDummy
#endif




namespace VirtualRobot
{

    namespace
    {
		std::mutex mutex;
    }

    VIRTUAL_ROBOT_IMPORT_EXPORT CollisionCheckerPtr CollisionChecker::__globalCollisionChecker;

    CollisionCheckerPtr CollisionChecker::getGlobalCollisionChecker()
    {
        if (true)
        {
            std::lock_guard<std::mutex> lock(mutex);

            if (!__globalCollisionChecker)
            {
                __globalCollisionChecker.reset(new CollisionChecker());
            }
        }

        return __globalCollisionChecker;
    }

    //----------------------------------------------------------------------
    // class CollisionChecker constructor
    //----------------------------------------------------------------------
    CollisionChecker::CollisionChecker()
    {
        initialized = true;
        debugOutput = false; // ENABLES OUTPUT OF COLLISION MODEL TRIANGLES
        automaticSizeCheck = true;

        collisionCheckerImplementation.reset(new COL_CHECKER_IMPL());
    }

    //----------------------------------------------------------------------
    // class CollisionChecker destructor
    //----------------------------------------------------------------------
    CollisionChecker::~CollisionChecker()
    {
    }

    float CollisionChecker::calculateDistance(const std::vector<CollisionModelPtr>& colModels1, const std::vector<CollisionModelPtr>& colModels2, Eigen::Vector3f& P1, Eigen::Vector3f& P2, int* trID1, int* trID2)
    {
        if (colModels1.size() == 0 || colModels2.size() == 0)
        {
            VR_WARNING << "no internal data..." << endl;
            return -1.0f;
        }

        float fResult = FLT_MAX;
        Eigen::Vector3f v1;
        Eigen::Vector3f v2;
        int trID1_r;
        int trID2_r;
        auto it1 = colModels1.begin();

        while (it1 != colModels1.end())
        {
            auto it2 = colModels2.begin();

            while (it2 != colModels2.end())
            {
                float fRes = calculateDistance(*it1, *it2, v1, v2, &trID1_r, &trID2_r);

                if (fRes <= fResult)
                {
                    fResult = fRes;
                    P1 = v1;
                    P2 = v2;

                    if (trID1)
                    {
                        *trID1 = trID1_r;
                    }

                    if (trID2)
                    {
                        *trID2 = trID2_r;
                    }
                }

                it2++;
            }

            it1++;
        }

        return fResult;
    }

    float CollisionChecker::calculateDistance(const CollisionModelPtr& model1, const std::vector<CollisionModelPtr>& colModels, Eigen::Vector3f& P1, Eigen::Vector3f& P2, int* trID1, int* trID2)
    {
        if (colModels.size() == 0)
        {
            VR_WARNING << "no internal data..." << endl;
            return -1.0f;
        }

        float fResult = FLT_MAX;
        Eigen::Vector3f v1;
        Eigen::Vector3f v2;
        int trID1_r;
        int trID2_r;
        auto it = colModels.begin();

        while (it != colModels.end())
        {
            float fRes = calculateDistance(model1, *it, v1, v2, &trID1_r, &trID2_r);

            if (fRes <= fResult)
            {
                fResult = fRes;
                P1 = v1;
                P2 = v2;

                if (trID1)
                {
                    *trID1 = trID1_r;
                }

                if (trID2)
                {
                    *trID2 = trID2_r;
                }
            }

            it++;
        }

        return fResult;
    }

    float CollisionChecker::calculateDistance(const CollisionModelPtr& model1, const CollisionModelPtr& model2, Eigen::Vector3f& P1, Eigen::Vector3f& P2, int* trID1, int* trID2)
    {
        VR_ASSERT(model1 && model2);
        VR_ASSERT_MESSAGE(model1->getCollisionChecker() == model2->getCollisionChecker(), "Collision models are linked to different Collision Checker instances");
        VR_ASSERT_MESSAGE(model1->getCollisionChecker() == shared_from_this(), "Collision models are linked to different Collision Checker instances");
        VR_ASSERT(isInitialized());

        return collisionCheckerImplementation->calculateDistance(model1, model2, P1, P2, trID1, trID2);
    }



    bool CollisionChecker::checkCollision(const std::vector<CollisionModelPtr>& model1, const CollisionModelPtr& model2)
    {
        if (model1.size() == 0)
        {
            VR_WARNING << "no internal data..." << endl;
            return false;
        }

        auto it1 = model1.begin();

        while (it1 != model1.end())
        {
            if (checkCollision(*it1, model2))
            {
                return true;
            }

            it1++;
        }

        return false;
    }

    bool CollisionChecker::checkCollision(const std::vector<CollisionModelPtr>& vColModels1, const std::vector<CollisionModelPtr>& vColModels2)
    {
        if (vColModels1.size() == 0 || vColModels2.size() == 0)
        {
            VR_WARNING << "no internal data..." << endl;
            return false;
        }

        auto it1 = vColModels1.begin();

        while (it1 != vColModels1.end())
        {
            auto it2 = vColModels2.begin();

            while (it2 != vColModels2.end())
            {
                if (checkCollision(*it1, *it2))
                {
                    return true;
                }

                it2++;
            }

            it1++;
        }

        return false;
    }

    bool CollisionChecker::checkCollision(const CollisionModelPtr& model1, const CollisionModelPtr& model2)
    {
        VR_ASSERT(model1 && model2);
        VR_ASSERT_MESSAGE(model1->getCollisionChecker() == model2->getCollisionChecker(), "Collision models are linked to different Collision Checker instances");
        VR_ASSERT_MESSAGE(model1->getCollisionChecker() == shared_from_this(), "Collision models are linked to different Collision Checker instances");
        VR_ASSERT(isInitialized());

        return collisionCheckerImplementation->checkCollision(model1, model2);//, storeContact);
    }


    void CollisionChecker::setAutomaticSizeCheck(bool checkSizeOnColModelCreation)
    {
        automaticSizeCheck = checkSizeOnColModelCreation;
        collisionCheckerImplementation->setAutomaticSizeCheck(automaticSizeCheck);
    }

    bool CollisionChecker::IsSupported_CollisionDetection()
    {
        return COL_CHECKER_IMPL::IsSupported_CollisionDetection();
    }

    bool CollisionChecker::IsSupported_ContinuousCollisionDetection()
    {
        return COL_CHECKER_IMPL::IsSupported_ContinuousCollisionDetection();
    }

    bool CollisionChecker::IsSupported_DistanceCalculations()
    {
        return COL_CHECKER_IMPL::IsSupported_DistanceCalculations();
    }

    bool CollisionChecker::IsSupported_Multithreading_Threadsafe()
    {
        return COL_CHECKER_IMPL::IsSupported_Multithreading_Threadsafe();
    }

    bool CollisionChecker::IsSupported_Multithreading_MultipleColCheckers()
    {
        return COL_CHECKER_IMPL::IsSupported_Multithreading_MultipleColCheckers();
    }

    void CollisionChecker::getContacts(const MathTools::Plane& p, const CollisionModelPtr& colModel, std::vector< MathTools::ContactPoint >& storeContatcs, float maxDist /*= 1.0f*/)
    {
        THROW_VR_EXCEPTION_IF(!colModel, "NULl data");

        // first check if plane hits bounding box
        BoundingBox bbox = colModel->getBoundingBox(false);
        // enlarge bbox by maxDist
        bbox.enlarge(Eigen::Vector3f(maxDist, maxDist, maxDist));
        std::vector <Eigen::Vector3f> ptsBB = bbox.getPoints();

        for (size_t i = 0; i < ptsBB.size(); i++)
        {
            ptsBB[i] = MathTools::transformPosition(ptsBB[i], colModel->getGlobalPose());
        }

        BoundingBox bboxGlobal(ptsBB);

        if (!bboxGlobal.planeGoesThrough(p))
        {
            // plane is not going through bounding box
            return;
        }

        // bbox was hit, test all points...
        std::vector< Eigen::Vector3f > pts = colModel->getModelVeticesGlobal();

        for (std::vector< Eigen::Vector3f >::iterator i = pts.begin(); i != pts.end(); i++)
        {
            if (MathTools::getDistancePointPlane(*i, p) <= maxDist)
            {
                MathTools::ContactPoint contact;
                contact.n = p.n;
                contact.p = *i;
                storeContatcs.push_back(contact);
            }
        }
    }

    /*
    bool CollisionChecker::checkContinuousCollision( CollisionModelPtr model1, SbMatrix &mGoalPose1, CollisionModelPtr model2, SbMatrix &mGoalPose2, float &fStoreTOC )
    {
        return collisionCheckerImplementation->checkContinuousCollision(model1,mGoalPose1,model2,mGoalPose2, fStoreTOC);
    }
    */

} // namespace VirtualRobot

