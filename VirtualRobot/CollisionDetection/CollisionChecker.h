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
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_CollisionChecker_h_
#define _VirtualRobot_CollisionChecker_h_

#include "../VirtualRobotImportExport.h"
#include "../Model/Model.h"
#include "../Model/ModelSet.h"
#include "../Tools/MathTools.h"

#include "../Model/Nodes/ModelLink.h"
#include "../Model/Model.h"
#include "../Model/LinkSet.h"

#include <string>
#include <vector>

#if defined(VR_COLLISION_DETECTION_PQP)
#include "PQP/CollisionCheckerPQP.h"
#else
#include "Dummy/CollisionCheckerDummy.h"
#endif

#include <boost/preprocessor.hpp>


namespace VirtualRobot
{

    /*!
        A CollisionChecker is an instance that handles collision and distance queries.
        Internally the requests are passed to the underlying engine (e.g. the PQP library).
        All objects that should be considered for collision detection (called CollisionModels)
        must be registered here. Usually the objects take care of registering on their own.

        When collision detection should not be performed in parallel, the global CollisionChecker
        singleton can be used. It can be retrieved with CollisionChecker::getGlobalCollisionChecker().


    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT CollisionChecker : public std::enable_shared_from_this<CollisionChecker>
    {
    public:

        CollisionChecker();
        virtual ~CollisionChecker();

        /*!
            Returns distance of the collision models.
            Returns -1.0 if no distance calculation lib was specified (-> Dummy Col Checker)
        */
        // To add a new type for distance checking implement the getCollisionModel function for this type
        template<typename T1, typename T2>
        inline float calculateDistance(const T1& m1, const T2& m2)
        {
            return calculateDistanceP(getCollisionModel(m1), getCollisionModel(m2), tmpV1, tmpV2, nullptr, nullptr);
        }
        template<typename T1, typename T2>
        inline float calculateDistance(const T1& m1, const T2& m2, Eigen::Vector3f& P1, Eigen::Vector3f& P2, int* trID1 = nullptr, int* trID2 = nullptr)
        {
            return calculateDistanceP(getCollisionModel(m1), getCollisionModel(m2), P1, P2, trID1, trID2);
        }

    private:
        inline float calculateDistanceP(const std::vector<CollisionModelPtr>& m1, const std::vector<CollisionModelPtr>& m2)
        {
            return calculateDistanceP(m1, m2, tmpV1, tmpV2, nullptr, nullptr);
        }
        inline float calculateDistanceP(const std::vector<CollisionModelPtr>& m1, const CollisionModelPtr& m2)
        {
            return calculateDistanceP(m1, m2, tmpV1, tmpV2, nullptr, nullptr);
        }
        inline float calculateDistanceP(const CollisionModelPtr& m1, const std::vector<CollisionModelPtr>& m2)
        {
            return calculateDistanceP(m1, m2, tmpV1, tmpV2, nullptr, nullptr);
        }
        inline float calculateDistanceP(const CollisionModelPtr& m1, const CollisionModelPtr& m2)
        {
            return calculateDistanceP(m1, m2, tmpV1, tmpV2, nullptr, nullptr);
        }

        virtual float calculateDistanceP(const std::vector<CollisionModelPtr>& m1, const std::vector<CollisionModelPtr>& m2, Eigen::Vector3f& P1, Eigen::Vector3f& P2, int* trID1 = nullptr, int* trID2 = nullptr);
        virtual float calculateDistanceP(const CollisionModelPtr& m1, const std::vector<CollisionModelPtr>& m2, Eigen::Vector3f& P1, Eigen::Vector3f& P2, int* trID1 = nullptr, int* trID2 = nullptr);
        inline float calculateDistanceP(const std::vector<CollisionModelPtr>& m1, const CollisionModelPtr& m2, Eigen::Vector3f& P1, Eigen::Vector3f& P2, int* trID1 = nullptr, int* trID2 = nullptr)
        {
            return calculateDistanceP(m2, m1, P1, P2, trID1, trID2);
        }
        virtual float calculateDistanceP(const CollisionModelPtr& m1, const CollisionModelPtr& m2, Eigen::Vector3f& P1, Eigen::Vector3f& P2, int* trID1 = nullptr, int* trID2 = nullptr);

    public:
        /*!
            Test if the two models are colliding.
            Returns true on collision.
        */
        // To add a new type for collision checking implement the getCollisionModel function for this type
        template<typename T1, typename T2>
        inline bool checkCollision(const T1& m1, const T2& m2)
        {
            return checkCollisionP(getCollisionModel(m1), getCollisionModel(m2));
        }

    private:
        virtual bool checkCollisionP(const std::vector<CollisionModelPtr>& model1, const std::vector<CollisionModelPtr>& model2);
        virtual bool checkCollisionP(const std::vector<CollisionModelPtr>& model1, const CollisionModelPtr& model2);
        inline bool checkCollisionP(const CollisionModelPtr& model1, const std::vector<CollisionModelPtr>& model2)
        {
            return checkCollisionP(model2, model1);
        }
        virtual bool checkCollisionP(const CollisionModelPtr& model1, const CollisionModelPtr& model2); //, Eigen::Vector3f *storeContact = nullptr);

    public:
        /*!
            Store all vertices of colModel whose distance to p is smaller than maxDist.
        */
        virtual void getContacts(const MathTools::Plane& p, const CollisionModelPtr& colModel, std::vector< MathTools::ContactPoint >& storeContatcs, float maxDist = 1.0f);

        /*!
            If continuous collision detection (CCD) is supported, this method can be used to detect collisions on the path
            from the current pose of the collision models to the goal poses.
            true -> collision (then the time of contact [0..1] is stored to fStoreTOC)
        */
        //bool CheckContinuousCollision (CollisionModelPtr model1, SbMatrix &mGoalPose1, CollisionModelPtr model2, SbMatrix &mGoalPose2, float &fSToreTOC);


        inline bool isInitialized()
        {
            return initialized;
        }


        /*!
        Activates / Deactivates the automatic size check on col model creation.
        The size check can be useful when the UNITS definitions in 3d files result in different scalings of the 3D models.
        (Standard: true)
        */
        void setAutomaticSizeCheck(bool checkSizeOnColModelCreation);

        void enableDebugOutput(bool e)
        {
            debugOutput = e;
            collisionCheckerImplementation->enableDebugOutput(e);
        }
        bool debugOutput;

        /*!
            Does the underlying collision detection library support discrete collision detection.
        */
        static bool IsSupported_CollisionDetection();

        /*!
            Does the underlying collision detection library support continuous collision detection.
        */
        static bool IsSupported_ContinuousCollisionDetection();

        /*!
            Does the underlying collision detection library support distance calculations.
        */
        static bool IsSupported_DistanceCalculations();

        /*!
            Does the underlying collision detection library support threadsafe access.
            E.g. multiple threads query the collision checker asynchronously.
        */
        static bool IsSupported_Multithreading_Threadsafe();

        /*!
            Does the underlying collision detection library support multiple instances of the collision checker.
            E.g. one per thread.
        */
        static bool IsSupported_Multithreading_MultipleColCheckers();

#if defined(VR_COLLISION_DETECTION_PQP)
        std::shared_ptr<CollisionCheckerPQP> getCollisionCheckerImplementation()
        {
            return collisionCheckerImplementation;
        }
#else
        std::shared_ptr<CollisionCheckerDummy> getCollisionCheckerImplementation()
        {
            return collisionCheckerImplementation;
        }
#endif
        //! This is the global collision checker singleton
        static CollisionCheckerPtr getGlobalCollisionChecker();

    private:
        inline CollisionModelPtr getCollisionModel(const CollisionModelPtr& c)
        {
            VR_ASSERT(m);
            return c;
        }
        inline std::vector<CollisionModelPtr> getCollisionModel(const std::vector<CollisionModelPtr>& c)
        {
            return c;
        }

        inline CollisionModelPtr getCollisionModel(const ModelNodePtr& m)
        {
            VR_ASSERT(m);
            if (!ModelNode::checkNodeOfType(m, ModelNode::ModelNodeType::Link))
                return CollisionModelPtr();
            return std::static_pointer_cast<ModelLink>(m)->getCollisionModel();
        }

        inline std::vector<CollisionModelPtr> getCollisionModel(const ModelPtr& m)
        {
            VR_ASSERT(m);
            return m->getCollisionModels();
        }
        inline std::vector<CollisionModelPtr> getCollisionModel(const ModelSetPtr& m)
        {
            VR_ASSERT(m);
            return getCollisionModel(m->getModels());
        }
        inline std::vector<CollisionModelPtr> getCollisionModel(const LinkSetPtr& m)
        {
            VR_ASSERT(m);
            return m->getCollisionModels();
        }

        inline std::vector<CollisionModelPtr> getCollisionModel(const std::vector<ModelPtr>& m)
        {
            std::vector<CollisionModelPtr> mVec;
            for (auto mTmp : m)
            {
                VR_ASSERT(mTmp);
                std::vector<CollisionModelPtr> tmp = mTmp->getCollisionModels();
                mVec.insert(mVec.end(), tmp.begin(), tmp.end());
            }
            return mVec;
        }
        inline std::vector<CollisionModelPtr> getCollisionModel(const std::vector<ModelNodePtr>& m)
        {
            std::vector<CollisionModelPtr> mVec;
            for (auto mTmp : m)
            {
                VR_ASSERT(mTmp);
                if (!ModelNode::checkNodeOfType(mTmp, ModelNode::ModelNodeType::Link))
                    continue;
                mVec.push_back(std::static_pointer_cast<ModelLink>(mTmp)->getCollisionModel());
            }
            return mVec;
        }

        bool initialized;
        bool automaticSizeCheck;

        static CollisionCheckerPtr __globalCollisionChecker;

        Eigen::Vector3f tmpV1;
        Eigen::Vector3f tmpV2;

#if defined(VR_COLLISION_DETECTION_PQP)
        std::shared_ptr<CollisionCheckerPQP> collisionCheckerImplementation;
#else
        std::shared_ptr<CollisionCheckerDummy> collisionCheckerImplementation;
#endif
    };

} // namespace VirtualRobot

#endif // _VirtualRobot_CollisionChecker_h_
