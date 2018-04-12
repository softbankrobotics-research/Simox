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
#ifndef _SimDynamics_BulletObject_h_
#define _SimDynamics_BulletObject_h_

#include "../DynamicsObject.h"
#include "SimoxMotionState.h"

#include "btBulletDynamicsCommon.h"

namespace SimDynamics
{
    class SIMDYNAMICS_IMPORT_EXPORT BulletObject : public DynamicsObject
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
            Constructor
        */
        BulletObject(VirtualRobot::SceneObjectPtr o);

        /*!
        */
        ~BulletObject() override;


        boost::shared_ptr<btRigidBody> getRigidBody();


        /*!
            Set world position [MM].
        */
        void setPosition(const Eigen::Vector3f& posMM) override;

        /*!
            Set world pose [mm].
        */
        void setPose(const Eigen::Matrix4f& pose) override;

        Eigen::Vector3f getCom()
        {
            return com;
        }

        Eigen::Vector3f getLinearVelocity() override;
        Eigen::Vector3f getAngularVelocity() override;

        void setLinearVelocity(const Eigen::Vector3f& vel) override;
        void setAngularVelocity(const Eigen::Vector3f& vel) override;

        //! This is the world pose which is set by bullet
        Eigen::Matrix4f getComGlobal();

        /*!
         * \brief applyForce Applies an external force on this object. The force is applied at the CoM position.
         * \param force The force to apply (value with respect to one second). The force will be deleted after one simulation step.
         */
        void applyForce(const Eigen::Vector3f& force) override;

        /*!
         * \brief applyTorque Applies an external torque on this object. The torque is applied at the CoM position.
         * \param torque The torque to apply (value with respect to one second). The torque will be deleted after one simulation step.
         */
        void applyTorque(const Eigen::Vector3f& torque) override;

        void setSimType(VirtualRobot::SceneObject::Physics::SimulationType s) override;

        /*!
         * \brief activate If object is sleeping, we can activate it here.
         */
        void activate() override;

        //! All object's sizes are scaled by this factor for bullet. (Small objects (<5cm) do not work well with bullet).
        static float ScaleFactor;
        //! All object's masses are scaled by this factor. (Heavy objects do not work well with motors.)
        static float MassFactor;

    protected:

        void setPoseIntern(const Eigen::Matrix4f& pose);
        btCollisionShape* getShapeFromPrimitive(VirtualRobot::Primitive::PrimitivePtr primitive);

        btConvexHullShape* createConvexHullShape(VirtualRobot::TriMeshModelPtr trimesh);

        boost::shared_ptr<btRigidBody> rigidBody;
        boost::shared_ptr<btCollisionShape> collisionShape; // bullet collision shape

        Eigen::Vector3f com; // com offset of trimesh

        btScalar btMargin;
        SimoxMotionState* motionState;

    };

    typedef boost::shared_ptr<BulletObject> BulletObjectPtr;

} // namespace SimDynamics

#endif // _SimDynamics_BulletObject_h_
