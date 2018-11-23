
#pragma once

#include "../../VirtualRobotImportExport.h"
#include "../CollisionCheckerImplementation.h"
#include <string>
#include <vector>

class CollisionModel;

namespace VirtualRobot
{

    class VIRTUAL_ROBOT_IMPORT_EXPORT CollisionCheckerDummy : public CollisionCheckerImplementation
    {
    public:
        friend class CollisionChecker;

        CollisionCheckerDummy();
        ~CollisionCheckerDummy() override;

        /*! Returns distance of the collision models.
        Collision detected if result is zero.
        Returns -1.0 if no distance calculation lib was specified (e.g. VR_COLLISION_DETECTION_PQP)
        */
        virtual float calculateDistance(const CollisionModelPtr& model1, const CollisionModelPtr& model2, Eigen::Vector3f& P1, Eigen::Vector3f& P2, int* trID1, int* trID2) override;
        //! tests if the two models are colliding
        virtual bool checkCollision(const CollisionModelPtr& model1, const CollisionModelPtr& model2) override;

        /*!
        If continuous collision detection (CCD) is supported, this method can be used to detect collisions on the path
        from the current pose of the collision models to the goal poses.
        true -> collision
        */
        //bool CheckContinuousCollision (CollisionModelPtr model1, SbMatrix &mGoalPose1, CollisionModelPtr model2, SbMatrix &mGoalPose2, float &fStoreTOC);

        /*!
        Does the underlying collision detection library support discrete collision detection.
        */
        static bool IsSupported_CollisionDetection()
        {
            return false;
        }

        /*!
        Does the underlying collision detection library support continuous collision detection.
        */
        static bool IsSupported_ContinuousCollisionDetection()
        {
            return false;
        }

        /*!
        Does the underlying collision detection library support distance calculations.
        */
        static bool IsSupported_DistanceCalculations()
        {
            return false;
        }

        /*!
        Does the underlying collision detection library support threadsafe access.
        E.g. multiple threads query the collision checker asynchronously.
        */
        static bool IsSupported_Multithreading_Threadsafe()
        {
            return false;
        }

        /*!
        Does the underlying collision detection library support multiple instances of the collision checker.
        E.g. one per thread.
        */
        static bool IsSupported_Multithreading_MultipleColCheckers()
        {
            return false;
        }

        void setAutomaticSizeCheck(bool checkSizeOnColModelCreation) override;
        void enableDebugOutput(bool e) override;
    };

}

