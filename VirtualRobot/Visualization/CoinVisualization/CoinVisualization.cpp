/**
* @package    VirtualRobot
* @author     Manfred Kroehnert, Nikolaus Vahrenkamp, Adrian Knobloch
* @copyright  2010, 2017 Manfred Kroehnert, Nikolaus Vahrenkamp, Adrian Knobloch
*/

#include "CoinVisualization.h"
#include "CoinVisualizationFactory.h"
#include "../../Tools/MathTools.h"
#include "../TriMeshModel.h"
#include "../../VirtualRobotException.h"
#include "../../XML/BaseIO.h"

#include <Inventor/SoPrimitiveVertex.h>
#include <Inventor/SbLinear.h>
#include <Inventor/nodes/SoShape.h>
#include <Inventor/nodes/SoNode.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoScale.h>
#include <Inventor/nodes/SoFile.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/actions/SoCallbackAction.h>
#include <Inventor/actions/SoLineHighlightRenderAction.h>
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/actions/SoToVRML2Action.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/VRMLnodes/SoVRMLGroup.h>

namespace VirtualRobot
{

    CoinVisualization::CoinVisualization(SoNode *visuNode) :
        Visualization(),
        mainNode(new SoSeparator),
        transformNode(new SoTransform),
        scaleNode(new SoScale),
        materialNode(new SoMaterial),
        materialNodeNone(new SoSeparator),
        drawStyleNode(new SoDrawStyle),
        visualizationNode(visuNode),
        filename(""),
        boundingBox(false)
    {
        if (!visualizationNode)
        {
            visualizationNode = new SoSeparator(); // create dummy node
        }

        mainNode->ref();
        transformNode->ref();
        scaleNode->ref();
        materialNode->ref();
        materialNodeNone->ref();
        drawStyleNode->ref();
        visualizationNode->ref();

        mainNode->addChild(transformNode);
        mainNode->addChild(scaleNode);
        mainNode->addChild(materialNodeNone);
        mainNode->addChild(drawStyleNode);
        mainNode->addChild(visualizationNode);

        setGlobalPose(Eigen::Matrix4f::Identity());
        setVisible(true);
        setUpdateVisualization(true);
        setStyle(DrawStyle::normal);
        setColor(Color::None());
        setSelected(false);
        createTriMeshModel();
    }

    CoinVisualization::~CoinVisualization()
    {
        mainNode->removeAllChildren();

        visualizationNode->unref();
        drawStyleNode->unref();
        materialNodeNone->unref();
        materialNode->unref();
        scaleNode->unref();
        transformNode->unref();
        mainNode->unref();
    }

    SoNode *CoinVisualization::getMainNode() const
    {
        return mainNode;
    }

    void CoinVisualization::setVisualization(SoNode *newVisu)
    {
        if (newVisu)
        {
            mainNode->removeChild(visualizationNode);
            visualizationNode->unref();
            visualizationNode = newVisu;
            visualizationNode->ref();
            mainNode->addChild(visualizationNode);
        }
    }

    SoNode *CoinVisualization::getCoinVisualization() const
    {
        return visualizationNode;
    }

    void CoinVisualization::setGlobalPose(const Eigen::Matrix4f &m)
    {
        if (!getUpdateVisualizationStatus())
        {
            return;
        }

        Visualization::setGlobalPose(m);
        transformNode->translation.setValue(m(0, 3), m(1, 3), m(2, 3));
        MathTools::Quaternion q = MathTools::eigen4f2quat(m);
        transformNode->rotation.setValue(q.x, q.y, q.z, q.w);
        for (auto& f : poseChangedCallbacks)
        {
            f.second(m);
        }
    }

    size_t CoinVisualization::addPoseChangedCallback(std::function<void (const Eigen::Matrix4f &)> f)
    {
        static unsigned int id = 0;
        poseChangedCallbacks[id] = f;
        return id++;
    }

    void CoinVisualization::removePoseChangedCallback(size_t id)
    {
        auto it = poseChangedCallbacks.find(id);
        if (it != poseChangedCallbacks.end())
        {
            poseChangedCallbacks.erase(it);
        }
    }

    void CoinVisualization::setVisible(bool showVisualization)
    {
        if (!getUpdateVisualizationStatus())
        {
            return;
        }

        auto index = mainNode->findChild(visualizationNode);
        if (showVisualization && index < 0)
        {
            mainNode->addChild(visualizationNode);
        }
        else if (!showVisualization && index >= 0)
        {
            mainNode->removeChild(index);
        }
    }

    bool CoinVisualization::isVisible() const
    {
        return mainNode->findChild(visualizationNode) >= 0;
    }

    void CoinVisualization::setUpdateVisualization(bool enable)
    {
        updateVisualization = enable;
    }

    bool CoinVisualization::getUpdateVisualizationStatus() const
    {
        return updateVisualization;
    }

    void CoinVisualization::setStyle(Visualization::DrawStyle s)
    {
        if (!getUpdateVisualizationStatus())
        {
            return;
        }

        style = s;
        switch (style)
        {
        case normal:
            drawStyleNode->style =  SoDrawStyle::FILLED;
            break;
        case wireframe:
            drawStyleNode->style =  SoDrawStyle::LINES;
            break;
        }
    }

    Visualization::DrawStyle CoinVisualization::getStyle() const
    {
        return style;
    }

    void CoinVisualization::setColor(const Visualization::Color &c)
    {
        if (!getUpdateVisualizationStatus())
        {
            return;
        }

        if (c.isNone())
        {
            setMaterial(MaterialPtr(new NoneMaterial));
        }
        else if (c.isTransparencyOnly())
        {
            PhongMaterialPtr m = std::dynamic_pointer_cast<PhongMaterial>(material);
            if (!m)
            {
                m.reset(new PhongMaterial);
            }
            m->transparency = c.transparency;
            setMaterial(m);
            materialNode->ambientColor.setIgnored(true);
            materialNode->diffuseColor.setIgnored(true);
            materialNode->specularColor.setIgnored(true);
            materialNode->emissiveColor.setIgnored(true);
            materialNode->shininess.setIgnored(true);
            materialNode->transparency.setIgnored(false);
            materialNode->setOverride(true);
        }
        else
        {
            PhongMaterialPtr m = std::dynamic_pointer_cast<PhongMaterial>(material);
            if (!m)
            {
                m.reset(new PhongMaterial);
            }
            m->diffuse = c;
            m->ambient = c;
            m->transparency = c.transparency;
            setMaterial(m);
        }
    }

    Visualization::Color CoinVisualization::getColor() const
    {
        PhongMaterialPtr m = std::dynamic_pointer_cast<PhongMaterial>(material);
        if (!m)
        {
            return Color::None();
        }
        if (materialNode->diffuseColor.isIgnored())
        {
            return Color::Transparency(m->transparency);
        }
        return m->diffuse;
    }

    void CoinVisualization::setMaterial(const Visualization::MaterialPtr &material)
    {
        if (!getUpdateVisualizationStatus())
        {
            return;
        }

        this->material = material;
        auto mNone = std::dynamic_pointer_cast<NoneMaterial>(material);
        if (mNone)
        {
            auto index = mainNode->findChild(materialNode);
            if (index >= 0)
            {
                mainNode->replaceChild(index, materialNodeNone);
            }
        }
        else
        {
            auto index = mainNode->findChild(materialNodeNone);
            if (index >= 0)
            {
                mainNode->replaceChild(index, materialNode);
            }
            PhongMaterialPtr mNone = std::dynamic_pointer_cast<PhongMaterial>(material);
            if (mNone)
            {
                materialNode->ambientColor.setValue(mNone->ambient.r, mNone->ambient.g, mNone->ambient.b);
                materialNode->diffuseColor.setValue(mNone->diffuse.r, mNone->diffuse.g, mNone->diffuse.b);
                materialNode->specularColor.setValue(mNone->specular.r, mNone->specular.g, mNone->specular.b);
                materialNode->transparency.setValue(mNone->transparency);

                materialNode->ambientColor.setIgnored(false);
                materialNode->diffuseColor.setIgnored(false);
                materialNode->specularColor.setIgnored(false);
                materialNode->emissiveColor.setIgnored(false);
                materialNode->shininess.setIgnored(false);
                materialNode->transparency.setIgnored(false);
                materialNode->setOverride(true);
            }
        }
    }

    Visualization::MaterialPtr CoinVisualization::getMaterial() const
    {
        return material;
    }

    void CoinVisualization::setSelected(bool selected)
    {
        VR_ERROR << "NYI" << std::endl;
    }

    bool CoinVisualization::isSelected() const
    {
        VR_ERROR << "NYI" << std::endl;
        return false;
    }

    size_t CoinVisualization::addSelectionChangedCallback(std::function<void (bool)> f)
    {
        VR_ERROR << "NYI" << std::endl;
        return 0;
    }

    void CoinVisualization::removeSelectionChangedCallback(size_t id)
    {
        VR_ERROR << "NYI" << std::endl;
    }

    void CoinVisualization::scale(const Eigen::Vector3f &s)
    {
        THROW_VR_EXCEPTION_IF(s.x() <= 0 || s.y() <= 0 || s.z() <= 0, "Scaling must be >0");
        if (!getUpdateVisualizationStatus())
        {
            return;
        }

        float x, y, z;
        scaleNode->scaleFactor.getValue().getValue(x, y, z);
        scaleNode->scaleFactor.setValue(x*s.x(), y*s.y(), z*s.z());
        createTriMeshModel();
    }

    Eigen::Vector3f CoinVisualization::getScalingFactor() const
    {
        float x, y, z;
        scaleNode->scaleFactor.getValue().getValue(x, y, z);
        return Eigen::Vector3f(x, y, z);
    }

    void CoinVisualization::shrinkFatten(float offset)
    {
        if (!getUpdateVisualizationStatus())
        {
            return;
        }

        if(offset != 0.0f)
        {
            createTriMeshModel();
            getTriMeshModel()->mergeVertices();
            getTriMeshModel()->fattenShrink(offset);

            auto visuNode = CoinVisualizationFactory::createTriMeshModelCoin(getTriMeshModel());
            setVisualization(visuNode);
        }
    }

    void CoinVisualization::_addManipulator(Visualization::ManipulatorType t)
    {
        VR_ERROR << "NYI" << std::endl;
    }

    void CoinVisualization::_removeManipulator(Visualization::ManipulatorType t)
    {
        VR_ERROR << "NYI" << std::endl;
    }

    void CoinVisualization::_removeAllManipulators()
    {
        VR_ERROR << "NYI" << std::endl;
    }

    bool CoinVisualization::hasManipulator(Visualization::ManipulatorType t) const
    {
        VR_ERROR << "NYI" << std::endl;
        return false;
    }

    std::vector<Visualization::ManipulatorType> CoinVisualization::getAddedManipulatorTypes() const
    {
        VR_ERROR << "NYI" << std::endl;
        return std::vector<ManipulatorType>();
    }

    std::vector<Primitive::PrimitivePtr> CoinVisualization::getPrimitives() const
    {
        return primitives;
    }

    void CoinVisualization::setFilename(const std::string &filename, bool boundingBox)
    {
        this->filename = filename;
        this->boundingBox = boundingBox;
    }

    std::string CoinVisualization::getFilename() const
    {
        return filename;
    }

    bool CoinVisualization::usedBoundingBoxVisu() const
    {
        return boundingBox;
    }

    BoundingBox CoinVisualization::getBoundingBox() const
    {
        VirtualRobot::BoundingBox bbox = getTriMeshModel()->boundingBox;
        bbox.transform(this->getGlobalPose());
        return bbox;
    }

    TriMeshModelPtr CoinVisualization::getTriMeshModel() const
    {
        VR_ASSERT(triMeshModel);
        return triMeshModel;
    }

    void CoinVisualization::createTriMeshModel()
    {
        THROW_VR_EXCEPTION_IF(!visualizationNode, "CoinVisualizationNode::createTriMeshModel(): no Coin model present!");

        if (triMeshModel)
        {
            triMeshModel->clear();
        }
        else
        {
            triMeshModel.reset(new TriMeshModel());
        }

        SoCallbackAction ca;
        ca.addTriangleCallback(SoShape::getClassTypeId(), &CoinVisualization::InventorTriangleCB, triMeshModel.get());
        ca.apply(getMainNode());
    }

    int CoinVisualization::getNumFaces() const
    {
        return static_cast<int>(getTriMeshModel()->faces.size());
    }

    VisualizationPtr CoinVisualization::clone() const
    {
        SoNode* deepCopiedNode = copyNode(visualizationNode);
        VR_ASSERT(deepCopiedNode);
        CoinVisualizationPtr p(new CoinVisualization(deepCopiedNode));
        p->setGlobalPose(this->getGlobalPose());
        p->setVisible(this->isVisible());
        p->setStyle(this->getStyle());
        p->setMaterial(this->getMaterial());
        p->scale(this->getScalingFactor());
        p->primitives = this->primitives;
        p->setUpdateVisualization(this->getUpdateVisualizationStatus());

        return p;
    }

    void CoinVisualization::print() const
    {
        cout << "  CoinVisualization: ";

        if (getTriMeshModel())
        {
            Eigen::Vector3f mi;
            Eigen::Vector3f ma;

            if (getTriMeshModel()->faces.size() > 0)
            {
                cout << getTriMeshModel()->faces.size() << " triangles" << endl;// Extend: " << ma[0]-mi[0] << ", " << ma[1] - mi[1] << ", " << ma[2] - mi[2] << endl;
                getTriMeshModel()->getSize(mi, ma);
                cout << "    Min point: (" << mi[0] << "," << mi[1] << "," << mi[2] << ")" << endl;
                cout << "    Max point: (" << ma[0] << "," << ma[1] << "," << ma[2] << ")" << endl;
            }
            else
            {
                cout << "No model" << endl;
            }

        }
        else
        {
            cout << "No model" << endl;
        }
    }

    std::string CoinVisualization::toXML(const std::string &basePath, int tabs) const
    {
        VR_ERROR << "NYI" << std::endl;
        return "";
    }

    std::string CoinVisualization::toXML(const std::string &basePath, const std::string &filename, int tabs) const
    {
        VR_ERROR << "NYI" << std::endl;
        return "";
    }

    bool CoinVisualization::saveModel(const std::string &modelPath, const std::string &filename)
    {
        std::string outFile = filename;
        bool vrml = true; // may be changed later according to file extension

        boost::filesystem::path completePath(modelPath);
        boost::filesystem::path fn(outFile);

        if (!boost::filesystem::is_directory(completePath))
        {
            if (!boost::filesystem::create_directories(completePath))
            {
                VR_ERROR << "Could not create model dir  " << completePath.string() << endl;
                return false;
            }
        }

        boost::filesystem::path completeFile = boost::filesystem::operator/(completePath, fn);

        SoOutput* so = new SoOutput();

        if (!so->openFile(completeFile.string().c_str()))
        {
            VR_ERROR << "Could not open file " << completeFile.string() << " for writing." << endl;
        }

        boost::filesystem::path extension = completeFile.extension();
        std::string extStr = extension.string();
        BaseIO::getLowerCase(extStr);

        if (extStr == ".iv")
        {
            vrml = false;
        }
        else
        {
            vrml = true;
        }


        SoGroup* n = new SoGroup;
        n->ref();
        n->addChild(getCoinVisualization());
        SoGroup* newVisu = convertSoFileChildren(n);
        newVisu->ref();

        if (vrml)
        {
            SoToVRML2Action tovrml2;
            tovrml2.apply(newVisu);
            SoVRMLGroup* newroot = tovrml2.getVRML2SceneGraph();
            newroot->ref();
            so->setHeaderString("#VRML V2.0 utf8");
            SoWriteAction wra(so);
            wra.apply(newroot);
            newroot->unref();
        }
        else
        {
            SoWriteAction wa(so);
            wa.apply(newVisu);
        }

        so->closeFile();

        newVisu->unref();
        n->unref();

        return true;
    }

    void CoinVisualization::InventorTriangleCB(void *data, SoCallbackAction *action, const SoPrimitiveVertex *v1, const SoPrimitiveVertex *v2, const SoPrimitiveVertex *v3)
    {
        TriMeshModel* triangleMeshModel  = static_cast<TriMeshModel*>(data);
        if (!triangleMeshModel)
        {
            VR_INFO << ": Internal error, NULL data" << endl;
            return;
        }

        SbMatrix mm = action->getModelMatrix();
        SbMatrix scale;
        scale.setScale(1000.0f); // simox operates in mm, coin3d in m
        mm = mm.multRight(scale);
        SbVec3f triangle[3];
        mm.multVecMatrix(v1->getPoint(), triangle[0]);
        mm.multVecMatrix(v2->getPoint(), triangle[1]);
        mm.multVecMatrix(v3->getPoint(), triangle[2]);
        SbVec3f normal[3];
        /*mm.multVecMatrix(v1->getNormal(), normal[0]);
                mm.multVecMatrix(v2->getNormal(), normal[1]);
                mm.multVecMatrix(v3->getNormal(), normal[2]);*/
        mm.multDirMatrix(v1->getNormal(), normal[0]);
        mm.multDirMatrix(v2->getNormal(), normal[1]);
        mm.multDirMatrix(v3->getNormal(), normal[2]);

        normal[0] = (normal[0] + normal[1] + normal[2]) / 3.0f;


        // read out vertices
        Eigen::Vector3f a, b, c, n;
        a << triangle[0][0], triangle[0][1], triangle[0][2];
        b << triangle[1][0], triangle[1][1], triangle[1][2];
        c << triangle[2][0], triangle[2][1], triangle[2][2];
        n << normal[0][0], normal[0][1], normal[0][2];

        // add new triangle to the model
        triangleMeshModel->addTriangleWithFace(a, b, c, n);
    }

    SoGroup *CoinVisualization::convertSoFileChildren(SoGroup *orig)
    {
        if (!orig)
        {
            return new SoGroup;
        }

        SoGroup* storeResult;

        if (orig->getTypeId() == SoSeparator::getClassTypeId())
        {
            storeResult = new SoSeparator;
        }
        else
        {
            storeResult = new SoGroup;
        }

        storeResult->ref();

        if (orig->getTypeId().isDerivedFrom(SoGroup::getClassTypeId()))
        {
            // process group node
            for (int i = 0; i < orig->getNumChildren(); i++)
            {
                SoNode* n1 = orig->getChild(i);

                if (n1->getTypeId().isDerivedFrom(SoGroup::getClassTypeId()))
                {
                    // convert group
                    SoGroup* n2 = (SoGroup*)n1;
                    SoGroup* gr1 = convertSoFileChildren(n2);
                    storeResult->addChild(gr1);
                }
                else if (n1->getTypeId() == SoFile::getClassTypeId())
                {
                    // really load file!!
                    SoFile* fn = (SoFile*)n1;
                    SoGroup* fileChildren;
                    fileChildren = fn->copyChildren();
                    storeResult->addChild(fileChildren);
                }
                else
                {
                    // just copy child node
                    storeResult->addChild(n1);
                }
            }
        }

        storeResult->unrefNoDelete();
        return storeResult;
    }

    SoNode *CoinVisualization::copyNode(SoNode *n)
    {
        if (!n)
        {
            return NULL;
        }

        bool copyImages = true;
        std::vector<SoSFImage*> changedImages;

        if (copyImages)
        {
            // find all SoTexture2 nodes
            SoSearchAction search;
            search.setType(SoTexture2::getClassTypeId());
            search.setInterest(SoSearchAction::ALL);
            search.setSearchingAll(TRUE);
            search.apply(n);
            SoPathList& list = search.getPaths();

            //VR_INFO << "copy: copying " <<  list.getLength() << " textures" << std::endl;

            // set their images to not default to copy the contents
            for (int i = 0; i < list.getLength(); i++)
            {
                SoFullPath* path = (SoFullPath*) list[i];
                assert(path->getTail()->isOfType(SoTexture2::getClassTypeId()));
                SoSFImage* image = &((SoTexture2*)path->getTail())->image;

                if (image->isDefault() == TRUE)
                {
                    ((SoTexture2*)path->getTail())->image.setDefault(FALSE);
                    changedImages.push_back(image);
                }
            }

        }

        // the actual copy operation
        SoNode* result = n->copy(TRUE);

        // reset the changed ones back
        for (std::vector<SoSFImage*>::iterator it = changedImages.begin();
             it != changedImages.end(); it++)
        {
            (*it)->setDefault(TRUE);
        }

        return result;
    }

} // namespace VirtualRobot
