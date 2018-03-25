
#include "CollisionModel.h"
#include "CollisionChecker.h"
#include "../Visualization/TriMeshModel.h"
#include "../Visualization/Visualization.h"
#include "../XML/BaseIO.h"
#include "../VirtualRobotException.h"
#include <algorithm>



namespace VirtualRobot
{

    CollisionModel::CollisionModel(const VisualizationPtr &visu, const std::string& name, CollisionCheckerPtr colChecker, int id, float margin)
    {
        globalPose = Eigen::Matrix4f::Identity();
        this->id = id;

        this->name = name;
        this->margin = margin;
        this->colChecker = colChecker;

        if (!this->colChecker)
        {
            this->colChecker = CollisionChecker::getGlobalCollisionChecker();
        }

        if (!this->colChecker)
        {
            VR_WARNING << "no col checker..." << endl;
        }

        updateVisualization = true;
        setVisualization(visu);
    }

    CollisionModel::CollisionModel(const VisualizationPtr &visu, const std::string &name, CollisionCheckerPtr colChecker, int id, InternalCollisionModelPtr collisionModel)
    {
        margin = 0.0;
        globalPose = Eigen::Matrix4f::Identity();
        this->id = id;

        this->name = name;

        this->colChecker = colChecker;

        if (!this->colChecker)
        {
            this->colChecker = CollisionChecker::getGlobalCollisionChecker();
        }

        if (!this->colChecker)
        {
            VR_WARNING << "no col checker..." << endl;
        }

        updateVisualization = true;
        if(!collisionModel)
            VR_WARNING << "internal collision model is NULL for " << name << endl;
        collisionModelImplementation = std::dynamic_pointer_cast<InternalCollisionModel>(collisionModel->clone(false));
        VR_ASSERT(collisionModelImplementation->getPQPModel());
        setVisualization(visu);
    }


    CollisionModel::~CollisionModel()
    {
//        destroyData();
    }


    void CollisionModel::destroyData()
    {
    }

    float CollisionModel::getMargin() const
    {
        return margin;
    }

    void CollisionModel::inflateModel(float value)
    {
        if(std::abs(margin - value) < 0.01f || (origVisualization && !model))
        {
            visualization = origVisualization->clone();
            visualization->shrinkFatten(value);
            model = visualization->getTriMeshModel();
            if (model)
            {
                bbox = model->boundingBox;
            }


#if defined(VR_COLLISION_DETECTION_PQP)
            collisionModelImplementation.reset(new CollisionModelPQP(model, colChecker, id));
#else
            collisionModelImplementation.reset(new CollisionModelDummy(colChecker));
#endif
        }
        if(!origVisualization)
            margin = 0.0;
        else
            margin = value;
    }


    std::string CollisionModel::getName()
    {
        return name;
    }

    void CollisionModel::setGlobalPose(const Eigen::Matrix4f& m)
    {
        globalPose = m;
        collisionModelImplementation->setGlobalPose(m);

        if (visualization && updateVisualization)
        {
            visualization->setGlobalPose(m);
        }
    }

    VirtualRobot::CollisionModelPtr CollisionModel::clone(const CollisionCheckerPtr &colChecker, float scaling, bool deepVisuMesh)
    {
        VisualizationPtr visuOrigNew;

        if(origVisualization)
        {
            visuOrigNew = origVisualization->clone();
            visuOrigNew->scale(Eigen::Vector3f::Constant(scaling));
        }

        std::string nameNew = name;
        int idNew = id;

        CollisionModelPtr p;
        if(deepVisuMesh || !this->collisionModelImplementation)
            p.reset(new CollisionModel(visuOrigNew, nameNew, colChecker, idNew, margin));
        else
        {
            p.reset(new CollisionModel(visuOrigNew, nameNew, colChecker, idNew, this->collisionModelImplementation));
            if(visualization)
            {
                p->visualization = visualization->clone();
                p->visualization->scale(Eigen::Vector3f::Constant(scaling));
                p->margin = margin;
            }
            else
            {
                std::stringstream ss;
                ss << "Fix this " << __FILE__ << ":" << __LINE__;
                THROW_VR_EXCEPTION(ss.str());
//                p->origVisualization->clone(deepVisuMesh, scaling);
//                p->inflateModel(margin);
            }

        }
        p->setGlobalPose(getGlobalPose());
        p->setUpdateVisualization(getUpdateVisualizationStatus());
        return p;
    }

    void CollisionModel::setVisualization(const VisualizationPtr visu)
    {
        visualization = visu;
        origVisualization = visu;
        model.reset();
        bbox.clear();

        inflateModel(margin); // updates the model
    }

    int CollisionModel::getId()
    {
        return id;
    }

    void CollisionModel::setUpdateVisualization(bool enable)
    {
        updateVisualization = enable;
    }

    bool CollisionModel::getUpdateVisualizationStatus()
    {
        return updateVisualization;
    }

    VisualizationPtr CollisionModel::getVisualization()
    {
        return visualization;
    }

    void CollisionModel::print()
    {
        collisionModelImplementation->print();

        if (visualization)
        {
            visualization->print();
        }
    }

    int CollisionModel::getNumFaces()
    {
        if (!visualization)
        {
            return 0;
        }

        return visualization->getNumFaces();

    }

    std::vector< Eigen::Vector3f > CollisionModel::getModelVeticesGlobal()
    {
        std::vector< Eigen::Vector3f > result;
        TriMeshModelPtr model = collisionModelImplementation->getTriMeshModel();

        if (!model)
        {
            return result;
        }

        Eigen::Matrix4f t;
        t.setIdentity();

        for (std::vector<Eigen::Vector3f >::iterator i = model->vertices.begin(); i != model->vertices.end(); i++)
        {
            t.block(0, 3, 3, 1) = *i;
            t = globalPose * t;
            result.push_back(t.block(0, 3, 3, 1));
        }

        return result;
    }

    BoundingBox CollisionModel::getBoundingBox(bool global /*= true*/)
    {
        if (global)
        {

            std::vector<Eigen::Vector3f> pts = bbox.getPoints();

            for (size_t i = 0; i < pts.size(); i++)
            {
                pts[i] = MathTools::transformPosition(pts[i], globalPose);
            }

            BoundingBox result(pts);
            return result;
        }

        return bbox;
    }

    VirtualRobot::VisualizationPtr CollisionModel::getModelDataVisualization()
    {
        if (!modelVisualization && visualization)
        {
            TriMeshModelPtr model = collisionModelImplementation->getTriMeshModel();

            if (model)
            {
                VisualizationFactoryPtr visualizationFactory = VisualizationFactory::getInstance();

                if (visualizationFactory)
                {
                    modelVisualization = model->getVisualization(true);
                    modelVisualization->setGlobalPose(globalPose);
                }
            }
        }

        return modelVisualization;
    }

    std::string CollisionModel::toXML(const std::string& basePath, const std::string& filename, int tabs)
    {
        std::stringstream ss;
        std::string t = "\t";
        std::string pre = "";

        for (int i = 0; i < tabs; i++)
        {
            pre += "\t";
        }

        ss << pre << "<CollisionModel";

        if (getVisualization()->usedBoundingBoxVisu())
        {
            ss << " BoundingBox='true'";
        }

        ss << ">\n";

        std::string fileType = VisualizationFactory::getInstance()->getVisualizationType();

        if (!filename.empty())
        {
            std::string tmpFilename = filename;
            BaseIO::makeRelativePath(basePath, tmpFilename);
            ss << pre << t
               << "<File type='" << fileType << "'>"
               << tmpFilename
               << "</File>\n";
        }

        if (visualization && visualization->getPrimitives().size() != 0)
        {
            auto primitives = visualization->getPrimitives();
            ss << pre << "\t<Primitives>\n";
            std::vector<Primitive::PrimitivePtr>::const_iterator it;

            for (it = primitives.begin(); it != primitives.end(); it++)
            {
                ss << (*it)->toXMLString(tabs + 1);
            }

            ss << pre << "\t</Primitives>\n";
        }

        ss << pre << "</CollisionModel>\n";
        return ss.str();
    }

    std::string CollisionModel::toXML(const std::string& basePath, int tabs)
    {
        std::string collisionFilename;

        if (visualization)
        {
            collisionFilename = visualization->getFilename();
        }
        else if (modelVisualization)
        {
            collisionFilename = modelVisualization->getFilename();
        }

        boost::filesystem::path fn(collisionFilename);
        return toXML(basePath, fn.string(), tabs);

    }

    VirtualRobot::CollisionModelPtr CollisionModel::CreateUnitedCollisionModel(const std::vector<CollisionModelPtr>& colModels)
    {
        VR_ASSERT(colModels.size() > 0);
        CollisionCheckerPtr colChecker = colModels[0]->getCollisionChecker();
        std::vector<VisualizationPtr> visus;

        for (size_t i = 0; i < colModels.size(); i++)
        {
            VisualizationPtr v = colModels[i]->getVisualization();

            if (v)
            {
                visus.push_back(v);
            }

            VR_ASSERT(colModels[i]->getCollisionChecker() == colChecker);
        }

        if (visus.size() == 0)
        {
            return CollisionModelPtr();
        }

        VisualizationPtr vc = VisualizationFactory::getInstance()->createVisualisationSet(visus);
        return CollisionModelPtr(new CollisionModel(vc, "", colChecker));
    }

    bool CollisionModel::saveModel(const std::string& modelPath, const std::string& filename)
    {
        if (visualization)
        {
            return visualization->saveModel(modelPath, filename);
        }

        if (modelVisualization)
        {
            return modelVisualization->saveModel(modelPath, filename);
        }

        return true; // no model given
    }

    void CollisionModel::scale(Eigen::Vector3f& scaleFactor)
    {
        if (model)
        {
            TriMeshModelPtr modelScaled = model->clone(scaleFactor);
            bbox = modelScaled->boundingBox;
#if defined(VR_COLLISION_DETECTION_PQP)
            collisionModelImplementation.reset(new CollisionModelPQP(modelScaled, colChecker, id));
#else
            collisionModelImplementation.reset(new CollisionModelDummy(colChecker));
#endif
        }

        if (visualization)
        {
            visualization->scale(scaleFactor);
        }
    }

    /*
    void CollisionModel::GetAABB( SbBox3f& store_aabb )
    {
        if (!m_pIVModel)
            return;
        SbViewportRegion vpreg;
        SoGetBoundingBoxAction bboxAction(vpreg);
        store_aabb.makeEmpty();
        bboxAction.apply(m_pIVModel);
        store_aabb.extendBy(bboxAction.getBoundingBox());
    }

    void CollisionModel::GetOOBB(SbXfBox3f& store_oobb)
    {
        if (!m_pIVModel)
            return;
        SbViewportRegion vpreg;
        SoGetBoundingBoxAction bboxAction(vpreg);
        bboxAction.apply(m_pIVModel);
        store_oobb = bboxAction.getXfBoundingBox();
    }
    */


} // namespace VirtualRobot

