#include "ModelLink.h"
#include "../CollisionDetection/CollisionModel.h"
#include "../Visualization/VisualizationNode.h"
#include "../Visualization/TriMeshModel.h"
#include "../VirtualRobotException.h"


namespace VirtualRobot
{
    ModelLink::Physics::Physics() : localCoM(Eigen::Vector3f::Zero()),
                                    massKg(0.0f),
                                    comLocation(eVisuBBoxCenter),
                                    inertiaMatrix(Eigen::Matrix3f::Identity()),
                                    simType(eUnknown),
                                    ignoreCollisions()
    {
    }

    std::string ModelLink::Physics::getString(ModelLink::Physics::SimulationType s) const
    {
        std::string r;

        switch (s)
        {
            case eStatic:
                r = "Static";
                break;

            case eKinematic:
                r = "Kinematic";
                break;

            case eDynamic:
                r = "Dynamic";
                break;

            default:
                r = "Unknown";
        }

        return r;
    }

    void ModelLink::Physics::print() const
    {
        // TODO
    }

    bool ModelLink::Physics::isSet()
    {
        return (simType != eUnknown || massKg != 0.0f || comLocation != eVisuBBoxCenter || !inertiaMatrix.isIdentity() || ignoreCollisions.size() > 0);
    }

    std::string ModelLink::Physics::toXML(int tabs)
    {
        // TODO
        return std::string();
    }

    ModelLink::Physics ModelLink::Physics::scale(float scaling) const
    {
        THROW_VR_EXCEPTION_IF(scaling <= 0, "Scaling must be > 0");
        Physics res = *this;
        res.localCoM *= scaling;
        return res;
    }
}

namespace VirtualRobot
{
    ModelLink::ModelLink(const ModelWeakPtr& model,
                         const std::string& name,
                         Eigen::Matrix4f& localTransformation,
                         const VisualizationNodePtr& visualization,
                         const CollisionModelPtr& collisionModel,
                         const ModelLink::Physics& p,
                         const CollisionCheckerPtr& colChecker) : ModelNode(model, name, localTransformation),
                                                           visualizationModel(visualization),
                                                           collisionModel(collisionModel),
                                                           physics(p),
                                                           collisionChecker(colChecker)
    {
        visualizationModel->setGlobalPose(getGlobalPose());
        collisionModel->setGlobalPose(getGlobalPose());
    }

    ModelLink::~ModelLink()
    {
    }

    void ModelLink::initialize(const ModelNodePtr& parent, const std::vector<ModelNodePtr>& children)
    {
        ModelNode::initialize(parent, children);

        if (!initializePhysics())
        {
            VR_ERROR << "Could not initialize physics." << std::endl;
        }
    }

    ModelNode::ModelNodeType ModelLink::getType() const
    {
        return ModelNode::ModelNodeType::Link;
    }

    CollisionModelPtr ModelLink::getCollisionModel() const
    {
        ReadLockPtr r = getModel()->getReadLock();
        return collisionModel;
    }

    void ModelLink::setCollisionModel(const CollisionModelPtr& colModel, bool keepUpdateVisualization)
    {
        WriteLockPtr w = getModel()->getWriteLock();
        if (keepUpdateVisualization && collisionModel)
        {
            colModel->setUpdateVisualization(collisionModel->getUpdateVisualizationStatus());
        }

        collisionModel = colModel;

        if (collisionModel)
        {
            collisionModel->setGlobalPose(getGlobalPose());
        }

        //TODO: update physics?
    }

    CollisionCheckerPtr ModelLink::getCollisionChecker() const
    {
        ReadLockPtr r = getModel()->getReadLock();
        return collisionChecker;
    }

    void ModelLink::setVisualization(const VisualizationNodePtr& visualization, bool keepUpdateVisualization)
    {
        WriteLockPtr w = getModel()->getWriteLock();
        if (keepUpdateVisualization && visualizationModel)
        {
            visualization->setUpdateVisualization(visualizationModel->getUpdateVisualizationStatus());
        }

        visualizationModel = visualization;

        if (visualizationModel)
        {
            visualizationModel->setGlobalPose(getGlobalPose());
        }

        //TODO: update physics?
    }

    VisualizationNodePtr ModelLink::getVisualization(ModelLink::VisualizationType visuType)
    {
        ReadLockPtr r = getModel()->getReadLock();
        if (visuType == ModelLink::VisualizationType::Full)
        {
            return visualizationModel;
        }
        else
        {
            if (collisionModel)
            {
                if (visuType == ModelLink::VisualizationType::Collision)
                {
                    return collisionModel->getVisualization();
                }
                else
                {
                    return collisionModel->getModelDataVisualization();
                }
            }
            else
            {
                return VisualizationNodePtr();
            }
        }
    }

    int ModelLink::getNumFaces(bool collisionModel)
    {
        ReadLockPtr r = getModel()->getReadLock();
        if (collisionModel)
        {
            if (this->collisionModel)
            {
                return this->collisionModel->getNumFaces();
            }
            else
            {
                return 0;
            }
        }
        else
        {
            if (visualizationModel)
            {
                return visualizationModel->getNumFaces();
            }
            else
            {
                return 0;
            }
        }
    }

    ModelLink::Physics ModelLink::getPhysics() const
    {
        ReadLockPtr r = getModel()->getReadLock();
        return physics;
    }

    ModelLink::Physics::SimulationType ModelLink::getSimulationType() const
    {
        ReadLockPtr r = getModel()->getReadLock();
        return physics.simType;
    }

    void ModelLink::setSimulationType(ModelLink::Physics::SimulationType s)
    {
        WriteLockPtr w = getModel()->getWriteLock();
        physics.simType = s;
    }

    std::vector<std::string> ModelLink::getIgnoredCollisionModels() const
    {
        ReadLockPtr r = getModel()->getReadLock();
        return physics.ignoreCollisions;
    }

    Eigen::Vector3f ModelLink::getCoMLocal() const
    {
        ReadLockPtr r = getModel()->getReadLock();
        return physics.localCoM;
    }

    Eigen::Vector3f ModelLink::getCoMGlobal() const
    {
        return toGlobalCoordinateSystemVec(getCoMLocal());
    }

    float ModelLink::getMass() const
    {
        ReadLockPtr r = getModel()->getReadLock();
        return physics.massKg;
    }

    void ModelLink::setMass(float m)
    {
        WriteLockPtr w = getModel()->getWriteLock();
        physics.massKg = m;
    }

    Eigen::Matrix3f ModelLink::getInertiaMatrix() const
    {
        ReadLockPtr r = getModel()->getReadLock();
        return physics.inertiaMatrix;
    }
    
    Eigen::Matrix3f ModelLink::getInertiaMatrix(const Eigen::Vector3f &shift)
    {
        Eigen::Matrix3f skew;
        skew << 0        , -shift(2), +shift(1),
                +shift(2), 0        , -shift(0),
                -shift(1), +shift(0),0;
        return getInertiaMatrix() + getMass() * skew.transpose()*skew;
    }

    Eigen::Matrix3f ModelLink::getInertiaMatrix(const Eigen::Vector3f& shift, const Eigen::Matrix3f& rotation)
    {
        return rotation*getInertiaMatrix(shift)*rotation.transpose();
    }
    Eigen::Matrix3f ModelLink::getInertiaMatrix(const Eigen::Matrix4f& transform)
    {
        return getInertiaMatrix(transform.block<3,1>(0,3), transform.block<3,3>(0,0));
    }

    float ModelLink::getFriction()
    {
        return physics.friction;
    }

    void ModelLink::setFriction(float friction)
    {
        physics.friction = friction;
    }


    void ModelLink::setInertiaMatrix(const Eigen::Matrix3f& im)
    {
        WriteLockPtr w = getModel()->getWriteLock();
        physics.inertiaMatrix = im;
    }

    bool ModelLink::initializePhysics()
    {
        // check if physics node's CoM location has to be calculated
        if (physics.comLocation == ModelLink::Physics::eVisuBBoxCenter)
        {
            if (visualizationModel || collisionModel)
            {
                TriMeshModelPtr tm;
                // since globalPose and visualization's global pose may differ, we transform com to local coord system (globalpose)
                Eigen::Matrix4f posVisu;

                if (collisionModel)
                {
                    tm = collisionModel->getTriMeshModel();
                    posVisu = collisionModel->getGlobalPose();
                }
                else
                {
                    tm = visualizationModel->getTriMeshModel();
                    posVisu = visualizationModel->getGlobalPose();
                }

                if (!tm)
                {
                    VR_WARNING << "Could not create trimeshmodel for CoM computation, setting CoM to local position (0/0/0)" << endl;
                }
                else
                {
                    Eigen::Vector3f minS, maxS;
                    tm->getSize(minS, maxS);
                    physics.localCoM = minS + (maxS - minS) * 0.5f;

                    // trimeshmodel is without any pose transformations, so apply visu global pose
                    Eigen::Matrix4f visuComGlobal;
                    visuComGlobal.setIdentity();
                    visuComGlobal.block(0, 3, 3, 1) = physics.localCoM;
                    visuComGlobal = posVisu * visuComGlobal;

                    // transform to this object's global pose (this might be a different one, e.g. when postJointTransformations are considered)
                    Eigen::Matrix4f comLocal = toLocalCoordinateSystem(visuComGlobal);
                    physics.localCoM = comLocal.block(0, 3, 3, 1);
                }
            }
        }

        // check for inertia matrix determination
        if (physics.inertiaMatrix.isZero())
        {
            if (physics.massKg <= 0)
            {
                // standard box
                physics.inertiaMatrix.setIdentity();
            }
            else
            {
                TriMeshModelPtr tm;
                // since globalPose and visualization's global pose may differ, we transform com to local coord system (globalpose)
                Eigen::Matrix4f posVisu;

                if (collisionModel)
                {
                    tm = collisionModel->getTriMeshModel();
                    posVisu = collisionModel->getGlobalPose();
                }
                else if (visualizationModel)
                {
                    tm = visualizationModel->getTriMeshModel();
                    posVisu = visualizationModel->getGlobalPose();
                }

                if (!tm)
                {
                    // standard box
                    physics.inertiaMatrix.setIdentity();
                    physics.inertiaMatrix *= 0.01f; // 10 cm bbox
                    physics.inertiaMatrix *= physics.massKg;
                }
                else
                {
                    Eigen::Vector3f minS, maxS;
                    tm->getSize(minS, maxS);
                    MathTools::OOBB bbox(minS, maxS, posVisu);

                    Eigen::Matrix4f coordCOM = getGlobalPose();
                    coordCOM.block(0, 3, 3, 1) = physics.localCoM;
                    // get bbox in local coord system
                    bbox.changeCoordSystem(coordCOM);
                    Eigen::Vector3f l = bbox.maxBB - bbox.minBB;
                    l *= 0.001f; // mm -> m

                    physics.inertiaMatrix.setZero();
                    physics.inertiaMatrix(0, 0) = (l(1) * l(1) + l(2) * l(2)) / 12.0f;
                    physics.inertiaMatrix(1, 1) = (l(0) * l(0) + l(2) * l(2)) / 12.0f;
                    physics.inertiaMatrix(2, 2) = (l(0) * l(0) + l(1) * l(1)) / 12.0f;

                    float mass = physics.massKg;
                    physics.inertiaMatrix *= mass;
                }
            }
        }

        return true;
    }

    void ModelLink::updatePoseInternally(bool updateChildren, bool updateAttachments)
    {
        if (visualizationModel && visualizationModel->getUpdateVisualizationStatus())
        {
            visualizationModel->setGlobalPose(getGlobalPose());
        }

        if (collisionModel && collisionModel->getUpdateVisualizationStatus())
        {
            collisionModel->setGlobalPose(getGlobalPose());
        }

        ModelNode::updatePoseInternally(updateChildren, updateAttachments);
    }
}
