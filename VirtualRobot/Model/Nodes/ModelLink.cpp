#include "ModelLink.h"
#include "../../CollisionDetection/CollisionChecker.h"
#include "../../CollisionDetection/CollisionModel.h"
#include "../../Visualization/Visualization.h"
#include "../../Visualization/TriMeshModel.h"
#include "../../VirtualRobotException.h"
#include "../../XML/BaseIO.h"
#include "Attachments/ModelNodeAttachment.h"

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

    ModelLink::Physics::Physics(const Eigen::Vector3f &localCoM, float massKg, float friction, ModelLink::Physics::CoMLocation comLocation, const Eigen::Matrix3f &inertiaMatrix, ModelLink::Physics::SimulationType simType, const std::vector<std::string> &ignoreCollisions) :
        localCoM(localCoM),
        massKg(massKg),
        friction(friction),
        comLocation(comLocation),
        inertiaMatrix(inertiaMatrix),
        simType(simType),
        ignoreCollisions(ignoreCollisions)
    {}

    ModelLink::Physics::SimulationType ModelLink::Physics::string2SimulationType(const std::string &s)
    {
        if (s == "Static")
        {
            return eStatic;
        }
        else if (s == "Kinematic")
        {
            return eKinematic;
        }
        else if (s == "Dynamic")
        {
            return eDynamic;
        }
        else
        {
            return eUnknown;
        }
    }

    std::string ModelLink::Physics::simulationType2String(ModelLink::Physics::SimulationType s)
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
        std::string ta;
        std::stringstream ss;

        for (int i = 0; i < tabs; i++)
        {
            ta += "\t";
        }

        ss << ta << "<Physics>\n";

        if (simType != eUnknown)
        {
            ss << ta << "\t<SimulationType value='" << simulationType2String(simType) << "'/>\n";
        }

        ss << ta << "\t<Mass unit='kg' value='" << massKg << "'/>\n";
        ss << ta << "\t<CoM location=";

        if (comLocation == eVisuBBoxCenter)
        {
            ss << "'VisualizationBBoxCenter'/>\n";
        }
        else
        {
            ss << "'Custom' x='" << localCoM(0) << "' y='" << localCoM(1) << "' z='" << localCoM(2) << "'/>\n";
        }

        ss << ta << "\t<InertiaMatrix>\n";
        ss << MathTools::getTransformXMLString(inertiaMatrix, tabs + 2, true);
        ss << ta << "\t</InertiaMatrix>\n";

        for (size_t i = 0; i < ignoreCollisions.size(); i++)
        {
            ss << ta << "\t<IgnoreCollisions name='" << ignoreCollisions[i] << "'/>\n";
        }

        ss << ta << "</Physics>\n";
        return ss.str();
    }

    ModelLink::Physics ModelLink::Physics::scale(float scaling) const
    {
        THROW_VR_EXCEPTION_IF(scaling <= 0, "Scaling must be > 0");
        Physics res = *this;
        res.localCoM *= scaling;
        return res;
    }

    ModelLink::Physics ModelLink::Physics::clone(float scaling) const
    {
        return scale(scaling);
    }

    ModelLink::Physics &ModelLink::Physics::operator=(ModelLink::Physics other)
    {
        inertiaMatrix = other.inertiaMatrix;
        localCoM = other.localCoM;
        ignoreCollisions = other.ignoreCollisions;
        return *this;
    }
}

namespace VirtualRobot
{
    ModelLink::ModelLink(const ModelWeakPtr& model,
                         const std::string& name,
                         const Eigen::Matrix4f& localTransformation,
                         const VisualizationPtr& visualization,
                         const CollisionModelPtr& collisionModel,
                         const ModelLink::Physics& p,
                         const CollisionCheckerPtr& colChecker) : ModelNode(model, name, localTransformation),
                                                           visualizationModel(visualization),
                                                           collisionModel(collisionModel),
                                                           physics(p),
                                                           collisionChecker(colChecker)
    {
        if (visualizationModel)
			visualizationModel->setGlobalPose(getGlobalPose());
        if (collisionModel)
			collisionModel->setGlobalPose(getGlobalPose());
        if (!collisionChecker)
        {
            if (collisionModel)
                collisionChecker = collisionModel->getCollisionChecker();
            if (!collisionChecker)
                collisionChecker = CollisionChecker::getGlobalCollisionChecker();
        }
        initializePhysics();
    }

    ModelLink::~ModelLink()
    {
    }


    ModelNode::NodeType ModelLink::getType() const
    {
        return ModelNode::NodeType::Link;
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

    void ModelLink::setVisualization(const VisualizationPtr& visualization, bool keepUpdateVisualization)
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

    VisualizationPtr ModelLink::getVisualization(ModelLink::VisualizationType visuType) const
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
                return VisualizationPtr();
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

    ModelNodePtr ModelLink::_clone(ModelPtr newModel, float scaling)
    {
        VisualizationPtr clonedVisu;
        if (visualizationModel)
        {
            clonedVisu = visualizationModel->clone();
            clonedVisu->scale(Eigen::Vector3f::Constant(scaling));
        }
        CollisionModelPtr clonedCol;
        if (collisionModel)
            clonedCol = collisionModel->clone(newModel->getCollisionChecker(), scaling);
        Physics clonedPhysics = physics.clone(scaling);
        ModelLinkPtr result(new ModelLink(newModel, getName(), this->getLocalTransformation(), clonedVisu, clonedCol, clonedPhysics, newModel->getCollisionChecker()));
        return result;
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

    std::string ModelLink::toXML(const std::string &basePath, const std::string &modelPathRelative, bool storeAttachments)
    {
        std::stringstream ss;
        std::string pre = "\t";
        std::string pre2 = "\t\t";
        ss << pre << "<ModelNode name='" << getName() << "'>\n";
        if (!this->getLocalTransformation().isIdentity())
        {
            ss << pre2 << "<Transform>" << endl;
            ss << BaseIO::toXML(getLocalTransformation(), "\t\t\t");
            ss << pre2 << "</Transform>" << endl;
        }

        if (physics.isSet())
        {
            ss << physics.toXML(2);
        }

        boost::filesystem::path pBase(basePath);

        if (visualizationModel && visualizationModel->getTriMeshModel() && visualizationModel->getTriMeshModel()->faces.size() > 0)
        {
              std::string visuFile = visualizationModel->getFilename();//getFilenameReplacementVisuModel();

              boost::filesystem::path pModel(modelPathRelative);
              boost::filesystem::path modelDirComplete = boost::filesystem::operator/(pBase, pModel);
              boost::filesystem::path fn(visuFile);
              boost::filesystem::path modelFileComplete = boost::filesystem::operator/(modelDirComplete, fn.filename());

              ss << visualizationModel->toXML(pBase.string(), modelFileComplete.string(), 2);
        }

        if (collisionModel && collisionModel->getTriMeshModel() && collisionModel->getTriMeshModel()->faces.size() > 0)
        {
              std::string colFile;
              if (collisionModel->getVisualization())
              {
                  colFile = collisionModel->getVisualization()->getFilename();//getFilenameReplacementColModel();
              }
              boost::filesystem::path pModel(modelPathRelative);
              boost::filesystem::path modelDirComplete = boost::filesystem::operator/(pBase, pModel);
              boost::filesystem::path fn(colFile);
              boost::filesystem::path modelFileComplete = boost::filesystem::operator/(modelDirComplete, fn.filename());
              // stl fiels are saved as wrl, adapt filename (todo: generalize this procedure...)
              boost::filesystem::path extension = modelFileComplete.extension();
              if (extension != ".iv" && extension!=".wrl")
              {
                  modelFileComplete = boost::filesystem::operator/(modelDirComplete, fn.stem());
                  modelFileComplete += ".wrl";
              }
              ss << collisionModel->toXML(pBase.string(), modelFileComplete.string(), 2);
        }

        if (storeAttachments)
        {
            for (auto &a:getAttachments())
            {
                ss << a->toXML(basePath, modelPathRelative);
            }
        }

        std::vector<ModelNodePtr> children = this->getChildNodes();
        for (size_t i = 0; i < children.size(); i++)
        {
                ss << pre2 << "<Child name='" << children[i]->getName() << "'/>" << endl;
        }

        ss << pre << "</ModelNode>\n";
        return ss.str();
    }

    bool ModelLink::saveModelFiles(const std::string& modelPath)
    {
        bool res = true;

        if (visualizationModel && visualizationModel->getTriMeshModel() && visualizationModel->getTriMeshModel()->faces.size() > 0)
        {
            std::string newFilename;

            /*if (replaceFilenames)
            {
                newFilename = getFilenameReplacementVisuModel();
                this->visualizationModel->setFilename(newFilename, false);
            }
            else
            {*/
                std::string fnV = visualizationModel->getFilename();
                boost::filesystem::path fn(fnV);
                boost::filesystem::path filnameNoPath = fn.filename();
                newFilename = filnameNoPath.string();
            //}

            res = res & visualizationModel->saveModel(modelPath, newFilename);
        }

        if (collisionModel && collisionModel->getTriMeshModel() && collisionModel->getTriMeshModel()->faces.size() > 0)
        {
            // check if we need to replace the filename (also in case the trimesh model is stored!)
            std::string newFilename;

            /*if (replaceFilenames || !collisionModel->getVisualization())
            {
                newFilename = getFilenameReplacementColModel();
                collisionModel->getVisualization()->setFilename(newFilename, false);
            }
            else
            {*/
            if (collisionModel->getVisualization())
            {
                std::string fnV = collisionModel->getVisualization()->getFilename();
                boost::filesystem::path fn(fnV);
                boost::filesystem::path filnameNoPath = fn.filename();
                newFilename = filnameNoPath.string();
            }

            res = res & collisionModel->saveModel(modelPath, newFilename);
        }

        return res;
    }

}
