
#include "CollisionModel.h"
#include "CollisionChecker.h"
#include "../Visualization/TriMeshModel.h"
#include "../Visualization/VisualizationNode.h"
#include "../XML/BaseIO.h"
#include <algorithm>



namespace VirtualRobot
{

    CollisionModel::CollisionModel(VisualizationNodePtr visu, const std::string& name, CollisionCheckerPtr colChecker, int id, float margin)
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

    CollisionModel::CollisionModel(VisualizationNodePtr visu, const std::string &name, CollisionCheckerPtr colChecker, int id, InternalCollisionModelPtr collisionModel)
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
        collisionModelImplementation = boost::dynamic_pointer_cast<InternalCollisionModel>(collisionModel->clone(false));
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
        float diff = std::abs(margin - value);
        if(diff > 0.01f || (origVisualization && !model))
        {
            visualization = origVisualization->clone(true);
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
            BOOST_ASSERT(origVisualization);
            origVisualization->setGlobalPose(m);
        }
    }

    VirtualRobot::CollisionModelPtr CollisionModel::clone(CollisionCheckerPtr colChecker, float scaling, bool deepVisuMesh)
    {
        VisualizationNodePtr visuOrigNew;

        if(origVisualization)
            visuOrigNew = origVisualization->clone(deepVisuMesh, scaling);

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
                p->visualization = visualization->clone(false, scaling);
                p->margin = margin;
            }
            else
            {                
                p->inflateModel(margin);
            }

        }
        p->setGlobalPose(getGlobalPose());
        p->setUpdateVisualization(getUpdateVisualizationStatus());
        return p;
    }

    void CollisionModel::setVisualization(const VisualizationNodePtr visu)
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

    VisualizationNodePtr CollisionModel::getVisualization()
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

        for (auto & vertice : model->vertices)
        {
            t.block(0, 3, 3, 1) = vertice;
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

    VirtualRobot::VisualizationNodePtr CollisionModel::getModelDataVisualization()
    {
        if (!modelVisualization && visualization)
        {
            TriMeshModelPtr model = collisionModelImplementation->getTriMeshModel();

            if (model)
            {
                std::string type = visualization->getType();
                VisualizationFactoryPtr visualizationFactory = VisualizationFactory::fromName(type, NULL);

                if (visualizationFactory)
                {
                    modelVisualization = visualizationFactory->createTriMeshModelVisualization(model, true, globalPose);
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

        std::string fileType("unknown");

        if (visualization)
        {
            fileType = visualization->getType();
        }
        else if (modelVisualization)
        {
            fileType = modelVisualization->getType();
        }

        if (!filename.empty())
        {
            std::string tmpFilename = filename;
            BaseIO::makeRelativePath(basePath, tmpFilename);
            ss << pre << t
               << "<File type='" << fileType << "'>"
               << tmpFilename
               << "</File>\n";
        }

        if (visualization && visualization->primitives.size() != 0)
        {
            ss << pre << "\t<Primitives>\n";
            std::vector<Primitive::PrimitivePtr>::const_iterator it;

            for (it = visualization->primitives.begin(); it != visualization->primitives.end(); it++)
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

        std::filesystem::path fn(collisionFilename);
        return toXML(basePath, fn.string(), tabs);

    }

    VirtualRobot::CollisionModelPtr CollisionModel::CreateUnitedCollisionModel(const std::vector<CollisionModelPtr>& colModels)
    {
        VR_ASSERT(colModels.size() > 0);
        CollisionCheckerPtr colChecker = colModels[0]->getCollisionChecker();
        std::vector<VisualizationNodePtr> visus;

        for (const auto & colModel : colModels)
        {
            VisualizationNodePtr v = colModel->getVisualization();

            if (v)
            {
                visus.push_back(v);
            }

            VR_ASSERT(colModel->getCollisionChecker() == colChecker);
        }

        if (visus.size() == 0)
        {
            return CollisionModelPtr();
        }

        VisualizationNodePtr vc = VisualizationNode::CreateUnitedVisualization(visus);
        return CollisionModelPtr(new CollisionModel(vc, "", colChecker));
    }

    bool CollisionModel::saveModel(const std::string& modelPath, std::string& filename)
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

