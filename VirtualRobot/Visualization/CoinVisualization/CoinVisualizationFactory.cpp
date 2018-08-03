/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @author     Manfred Kroehnert
* @author     Adiran Knobloch
* @copyright  2010,2011,2017 Nikolaus Vahrenkamp, Manfred Kroehnert, Adrian Knobloch
*/
#include "CoinVisualizationFactory.h"

#ifdef SIMOX_USE_SOQT
#include <Inventor/Qt/SoQt.h>
#endif
#include <Inventor/nodes/SoCube.h>
#include <Inventor/SoDB.h>
#include <Inventor/nodes/SoLineSet.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/SbVec3f.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoCylinder.h>
#include <Inventor/nodes/SoComplexity.h>
#include <Inventor/nodes/SoTriangleStripSet.h>
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoFaceSet.h>
#include <Inventor/nodes/SoTextureCoordinate2.h>
#include <Inventor/nodes/SoTexture2.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoCone.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/VRMLnodes/SoVRMLBillboard.h>
#include <Inventor/nodes/SoAsciiText.h>
#include <Inventor/nodes/SoScale.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/SoInput.h>
#include <Inventor/sensors/SoDataSensor.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/VRMLnodes/SoVRMLImageTexture.h>
#include <Inventor/VRMLnodes/SoVRMLAppearance.h>

#include "../TriMeshModel.h"
#include "../../Tools/RuntimeEnvironment.h"
#include "../../Workspace/WorkspaceRepresentation.h"
#include "../../Import/MeshImport/STLReader.h"
#include "../../XML/BaseIO.h"

namespace VirtualRobot
{
    void CoinVisualizationFactory::init(int &argc, char *argv[], const std::string &appName)
    {
        if (!SoDB::isInitialized())
        {
            SoDB::init();
        }
    #ifdef SIMOX_USE_SOQT
        SoQt::init(argc, argv, appName.c_str());
    #endif
        // enable Coin3D extension: transparent settings without color
        (void)coin_setenv("COIN_SEPARATE_DIFFUSE_TRANSPARENCY_OVERRIDE", "1", TRUE);

        // neded since some Linux gfx drivers seem to report zero max pbuffer size -> offscreen renderer is disabled
        (void)coin_setenv("COIN_OFFSCREENRENDERER_TILEWIDTH", "2048", TRUE);
        (void)coin_setenv("COIN_OFFSCREENRENDERER_TILEHEIGHT", "2048", TRUE);

        // we can enable coin debug messages
        //(void)coin_setenv("COIN_DEBUG_SOOFFSCREENRENDERER", "1", TRUE);
        //(void)coin_setenv("COIN_DEBUG_GLGLUE", "1", TRUE);
    }

    VisualizationPtr CoinVisualizationFactory::createVisualizationFromFile(const std::string &filename, bool boundingBox) const
    {
        // passing an empty string to SoInput and trying to open it aborts the program
        if (filename.empty())
        {
            std::cerr <<  "No filename given" << std::endl;
            return createVisualization();
        }

        // check for STL file (.stl, .stla, .stlb)
        if (filename.length() >= 4)
        {
            std::string ending = filename.substr(filename.length() - 4, 4);
            BaseIO::getLowerCase(ending);

            if (ending == ".stl" || ending == "stla" || ending == "stlb")
            {
                return createVisualizationFromSTLFile(filename, boundingBox);
            }
        }

        return createVisualizationFromCoin3DFile(filename, boundingBox);
    }

    VisualizationPtr CoinVisualizationFactory::createVisualizationFromFile(const std::ifstream &ifs, bool boundingBox) const
    {
        // passing an empty string to SoInput and trying to open it aborts the program
        if (!ifs)
        {
            std::cerr <<  "Filestream not valid" << std::endl;
            return createVisualization();
        }

        // try to open the given file
        std::ostringstream oss;
        oss << ifs.rdbuf();

        if (!ifs && !ifs.eof())
        {
            std::cerr <<  "Error reading filestream " << std::endl;
            return createVisualization();
        }

        std::string contents(oss.str());
        if (contents.empty())
        {
            std::cerr << "Error file is empty" << std::endl;
            return createVisualization();
        }

        SoInput stringInput;
        stringInput.setBuffer(const_cast<char*>(contents.c_str()), contents.size());
        return createVisualizationFromSoInput(stringInput, boundingBox);
    }

    VisualizationPtr CoinVisualizationFactory::createVisualizationFromCoin3DFile(const std::string &filename, bool boundingBox) const
    {
        // try to open the given file
        SoInput fileInput;

        if (!fileInput.openFile(filename.c_str()))
        {
            std::cerr <<  "Cannot open file " << filename << std::endl;
            return createVisualization();
        }

        auto visu = createVisualizationFromSoInput(fileInput, boundingBox);

        fileInput.closeFile();
        visu->setFilename(filename, boundingBox);

        return visu;
    }

    VisualizationPtr CoinVisualizationFactory::createVisualizationFromSTLFile(const std::string &filename, bool boundingBox) const
    {
        // try to read from file
        TriMeshModelPtr t(new TriMeshModel());
        STLReaderPtr r(new STLReader());
        r->setScaling(1000.0f); // mm
        bool readOK = r->read(filename, t);

        if (!readOK)
        {
            VR_ERROR << "Could not read stl file " << filename << endl;
            return createVisualization();
        }

        auto visu = createTriMeshModel(t);
        visu->setFilename(filename, boundingBox);

        return visu;
    }

    VisualizationPtr CoinVisualizationFactory::createVisualizationFromSoInput(SoInput &soInput, bool boundingBox, bool freeDuplicateTextures) const
    {
        // read the contents of the file
        SoNode* coinVisualization = SoDB::readAll(&soInput);

        // check if the visualization was read
        if (!coinVisualization)
        {
            std::cerr <<  "Problem reading model from SoInput: "  << soInput.getCurFileName() << std::endl;
            return createVisualization();
        }
        VisualizationPtr visu(new CoinVisualization(coinVisualization));

        if (boundingBox)
        {
            return visu->getBoundingBox().getVisualization();
        }

        if(freeDuplicateTextures)
        {
            boost::filesystem::path p(soInput.getCurFileName());
            boost::filesystem::path dir = p.parent_path();

            RemoveDuplicateTextures(coinVisualization, dir.string());
        }
        return visu;
    }

    VisualizationSetPtr CoinVisualizationFactory::createVisualisationSet(const std::vector<VisualizationPtr> &visualizations) const
    {
        return VisualizationSetPtr(new CoinVisualizationSet(visualizations));
    }

    VisualizationPtr CoinVisualizationFactory::createBox(float width, float height, float depth) const
    {
        SoCube* c = new SoCube();
        c->width = width;
        c->height = height;
        c->depth = depth;

        return VisualizationPtr(new CoinVisualization(c));
    }

    VisualizationPtr CoinVisualizationFactory::createLine(const Eigen::Vector3f &from, const Eigen::Vector3f &to, float width) const
    {
        SoSeparator* s = new SoSeparator;
        s->ref();

        SoDrawStyle* lineSolutionStyle = new SoDrawStyle();
        lineSolutionStyle->lineWidth.setValue(width);
        s->addChild(lineSolutionStyle);

        float x = from[0];
        float y = from[1];
        float z = from[2];
        float x2 = to[0];
        float y2 = to[1];
        float z2 = to[2];

        SbVec3f points[2];
        points[0].setValue(x, y, z);
        points[1].setValue(x2, y2, z2);

        SoCoordinate3* coordinate3 = new SoCoordinate3;
        coordinate3->point.set1Value(0, points[0]);
        coordinate3->point.set1Value(1, points[1]);
        s->addChild(coordinate3);

        SoLineSet* lineSet = new SoLineSet;
        lineSet->numVertices.setValue(2);
        lineSet->startIndex.setValue(0);
        s->addChild(lineSet);
        s->unrefNoDelete();
        return VisualizationPtr(new CoinVisualization(s));
    }

    VisualizationPtr CoinVisualizationFactory::createSphere(float radius) const
    {
        SoSphere* s = new SoSphere();
        s->radius = radius;

        return VisualizationPtr(new CoinVisualization(s));
    }

    VisualizationPtr CoinVisualizationFactory::createCircle(float radius, float circleCompletion, float width, size_t numberOfCircleParts) const
    {
        SoSeparator* s = new SoSeparator();
        s->ref();

        SoCoordinate3* coordinate3 = new SoCoordinate3;
        circleCompletion = std::min<float>(1.0f, circleCompletion);
        circleCompletion = std::max<float>(-1.0f, circleCompletion);
        for (size_t i = 0; i < numberOfCircleParts; ++i)
        {
            SbVec3f point;
            float angle0 = static_cast<float>(i)/static_cast<float>(numberOfCircleParts) * 2 * M_PI * circleCompletion;
            float x0 = radius * cos(angle0);
            float y0 = radius * sin(angle0);

            point.setValue(x0,y0,0);
            coordinate3->point.set1Value(i, point);
        }
        s->addChild(coordinate3);

        SoDrawStyle* lineSolutionStyle = new SoDrawStyle();
        lineSolutionStyle->lineWidth.setValue(width);
        s->addChild(lineSolutionStyle);

        SoLineSet* lineSet = new SoLineSet;
        lineSet->numVertices.setValue(numberOfCircleParts);
        lineSet->startIndex.setValue(0);
        s->addChild(lineSet);
        s->unrefNoDelete();

        return VisualizationPtr(new CoinVisualization(s));
    }

    VisualizationPtr CoinVisualizationFactory::createCircleArrow(float radius, float tubeRadius, float completion, int sides, int rings) const
    {
        // create torus
        VR_ASSERT_MESSAGE(rings >= 4, "You need to pass in atleast 4 rings for a torus");
        VR_ASSERT_MESSAGE(sides >= 4, "You need to pass in atleast 4 sides for a torus");
        completion = std::min<float>(1.0f, completion);
        completion = std::max<float>(-1.0f, completion);
        int sign = completion >=0?1:-1;
        float torusCompletion = completion - 1.0f/rings*sign;
        if(torusCompletion * sign < 0)
            torusCompletion = 0;
        auto torus = createTorus(radius, tubeRadius, torusCompletion);

        // create Arrow
        float angle0 = (float)(rings-2)/rings * 2 * M_PI * completion;
        float x0 = radius * cos(angle0);
        float y0 = radius * sin(angle0);
        float angle1 = (float)(rings-1)/rings * 2 * M_PI * completion;


        auto arrow = createArrow(Eigen::Vector3f::UnitY()*sign, 0, tubeRadius);
        Eigen::Matrix4f gp = MathTools::axisangle2eigen4f(Eigen::Vector3f(0, 0, 1), angle1);
        gp(0, 3) = x0;
        gp(1, 3) = y0;
        gp(2, 3) = 0.f;
        arrow->setGlobalPose(gp);

        return createVisualisationSet({torus, arrow});
    }

    VisualizationPtr CoinVisualizationFactory::createCylinder(float radius, float height) const
    {
        SoCylinder* c = new SoCylinder();
        c->radius = radius;
        c->height = height;
        return VisualizationPtr(new CoinVisualization(c));
    }

    VisualizationPtr CoinVisualizationFactory::createPoint(float radius) const
    {
        SoSeparator* res = new SoSeparator;
        res->ref();

        // Control complexity  of the scene's primitives
        SoComplexity* comp = new SoComplexity;
        comp->value.setValue(0.1f);
        res->addChild(comp);

        // Set shape
        SoSphere* s = new SoSphere;
        s->radius = radius;
        res->addChild(s);

        res->unrefNoDelete();
        return VisualizationPtr(new CoinVisualization(res));
    }

    VisualizationPtr CoinVisualizationFactory::createTriMeshModel(const TriMeshModelPtr &model) const
    {
        return VisualizationPtr(new CoinVisualization(createTriMeshModelCoin(model)));
    }

    SoNode *CoinVisualizationFactory::createTriMeshModelCoin(const TriMeshModelPtr &model)
    {
        VR_ASSERT(model);
        const unsigned long numStrips = model->faces.size();
        const std::vector<int32_t> numVertices(numStrips, 3);

        SoSeparator* res = new SoSeparator;
        res->ref();

        // A shape hints tells the ordering of polygons.
        // This ensures double-sided lighting.
        SoShapeHints *myHints = new SoShapeHints;
        myHints->vertexOrdering = SoShapeHints::COUNTERCLOCKWISE;
        res->addChild(myHints);

        // Define colors for the strips
        SoMaterial *myMaterials = new SoMaterial;

        SoMaterialBinding *myMaterialBinding = new SoMaterialBinding;
        myMaterialBinding->value = SoMaterialBinding::PER_VERTEX;

        // Define coordinates for vertices
        SoCoordinate3 *myCoords = new SoCoordinate3;

        // set the values for vertex color and positions
        for (size_t i=0; i<numStrips; ++i)
        {
            auto &face = model->faces[i];

            for (unsigned int vertexNum=0; vertexNum<3; vertexNum++)
            {
                unsigned int coinId = 3*i+vertexNum;

                auto vertexId = vertexNum==0 ? face.id1 : (vertexNum==1 ? face.id2 : face.id3);
                auto pos = model->vertices.at(vertexId);
                myCoords->point.set1Value(coinId, pos.x(), pos.y(), pos.z());

                auto colorId = vertexNum==0 ? face.idColor1 : (vertexNum==1 ? face.idColor2 : face.idColor3);
                if (face.idMaterial < 0 || face.idMaterial >= model->materials.size())
                {
                    if (!model->colors.empty())
                    {
                        auto& color = model->colors.at(colorId);
                        if (!color.isNone() && !color.isTransparencyOnly())
                        {
                            float r = color.r;
                            float g = color.g;
                            float b = color.b;
                            float transparency = color.transparency;
                            myMaterials->ambientColor.set1Value(coinId, r, g, b);
                            myMaterials->diffuseColor.set1Value(coinId, r, g, b);
                            myMaterials->transparency.set1Value(coinId, transparency);
                        }
                    }
                    else
                    {
                        myMaterials->ambientColor.set1Value(i, 0.5f, 0.5f, 0.5f);
                        myMaterials->diffuseColor.set1Value(i, 0.5f, 0.5f, 0.5f);
                        myMaterials->transparency.set1Value(i, 0.5f);
                    }
                }
                else
                {
                    auto& material = model->materials[face.idMaterial];
                    myMaterials->ambientColor.set1Value(coinId, material.ambient.r, material.ambient.g, material.ambient.b);
                    myMaterials->diffuseColor.set1Value(coinId, material.diffuse.r, material.diffuse.g, material.diffuse.b);
                    myMaterials->specularColor.set1Value(coinId, material.specular.r, material.specular.g, material.specular.b);
                    myMaterials->transparency.set1Value(coinId, material.transparency);
                }
            }
        }
        res->addChild(myMaterials);
        res->addChild(myMaterialBinding);
        res->addChild(myCoords);

        // Define the TriangleStripSet
        SoTriangleStripSet *myStrips = new SoTriangleStripSet;
        myStrips->numVertices.setValues(0, numStrips, numVertices.data());
        res->addChild(myStrips);

        res->unrefNoDelete();
        return res;
    }

    VisualizationPtr CoinVisualizationFactory::createPolygon(const std::vector<Eigen::Vector3f> &points) const
    {
        SoSeparator* res = new SoSeparator;
        res->ref();

        // A shape hints tells the ordering of polygons.
        // This ensures double-sided lighting.
        SoShapeHints *myHints = new SoShapeHints;
        myHints->vertexOrdering = SoShapeHints::COUNTERCLOCKWISE;
        res->addChild(myHints);

        SoCoordinate3 *myCoords = new SoCoordinate3;
        for (size_t i=0; i<points.size(); ++i)
        {
            myCoords->point.set1Value(i, points[i].x(), points[i].y(), points[i].z());
        }
        res->addChild(myCoords);

        SoFaceSet *myFaceSet = new SoFaceSet;
        myFaceSet->numVertices.set1Value(0, points.size());
        res->addChild(myFaceSet);

        res->unrefNoDelete();
        return VisualizationPtr(new CoinVisualization(res));
    }

    std::vector<Eigen::Vector3f> createPlanePoints(float extend)
    {
        std::vector<Eigen::Vector3f> points;
        points.reserve(4);
        points.emplace_back(extend/2.f, extend/2.f, 0.f);
        points.emplace_back(-extend/2.f, extend/2.f, 0.f);
        points.emplace_back(-extend/2.f, -extend/2.f, 0.f);
        points.emplace_back(extend/2.f, -extend/2.f, 0.f);
        return points;
    }


    VisualizationPtr CoinVisualizationFactory::createPlane(const Eigen::Vector3f &point, const Eigen::Vector3f &normal, float extend, const std::string &texture) const
    {
        VisualizationPtr visu = texture.empty() ? createPolygon(createPlanePoints(extend))
                                                : createTexturedPlane(extend, texture);
        auto r = MathTools::getRotation(Eigen::Vector3f::UnitZ(), normal);
        Eigen::Matrix4f gp = MathTools::quat2eigen4f(r);
        gp.block<3, 1>(0, 3) = point;
        visu->setGlobalPose(gp);
        return visu;
    }

    VisualizationPtr CoinVisualizationFactory::createTexturedPlane(float extend, const std::string &texture)
    {
        SoSeparator* res = new SoSeparator();
        res->ref();

        std::string tf = texture;
        if (tf.empty() || !RuntimeEnvironment::getDataFileAbsolute(tf))
        {
            tf  = "images/Floor.png";
            RuntimeEnvironment::getDataFileAbsolute(tf);
        }
        else
        {
            tf = texture;
        }

        float widthMosaic = extend / 500.0f;
        float depthMosaic = extend / 500.0f;

        SoMaterial* pSoMaterial = new SoMaterial;
        SoCoordinate3* pImagePlaneSoCoordinate3 = new SoCoordinate3;
        auto planePoints = createPlanePoints(extend);
        for (size_t i=0; i<planePoints.size(); ++i)
        {
            pImagePlaneSoCoordinate3->point.set1Value(i, SbVec3f(planePoints[i].x(), planePoints[i].y(), planePoints[i].z()));
        }
        SoFaceSet* pSoFaceSet = new SoFaceSet;
        pSoFaceSet->numVertices.set1Value(0, 4);
        SoTextureCoordinate2* pSoTextureCoordinate2 = new SoTextureCoordinate2;
        pSoTextureCoordinate2->point.set1Value(0, SbVec2f(widthMosaic, 0));
        pSoTextureCoordinate2->point.set1Value(1, SbVec2f(0, 0));
        pSoTextureCoordinate2->point.set1Value(2, SbVec2f(0, depthMosaic));
        pSoTextureCoordinate2->point.set1Value(3, SbVec2f(widthMosaic, depthMosaic));
        SoTextureCoordinateBinding* pSoTextureCoordinateBinding =  new SoTextureCoordinateBinding;
        pSoTextureCoordinateBinding->value.setValue(SoTextureCoordinateBinding::PER_VERTEX);
        SoTexture2* pSoTexture2 = new SoTexture2;
        pSoTexture2->filename.setValue(tf.c_str());
        pSoTexture2->wrapS = pSoTexture2->wrapT = SoTexture2::REPEAT;
        res->addChild(pSoMaterial);
        res->addChild(pSoTextureCoordinate2);
        res->addChild(pSoTextureCoordinateBinding);
        res->addChild(pSoTexture2);
        res->addChild(pImagePlaneSoCoordinate3);
        res->addChild(pSoFaceSet);

        res->unrefNoDelete();
        return VisualizationPtr(new CoinVisualization(res));
    }

    VisualizationPtr CoinVisualizationFactory::createArrow(const Eigen::Vector3f &n, float length, float width) const
    {
        Eigen::Vector3f n2 = n;
        if (n2.norm()<1e-10)
            n2 << 0,0,1;
        n2.normalize();
        float coneHeight = width * 6.0f;
        float coneBottomRadius = width * 2.5f;
        float baseLength = length - coneHeight;
        baseLength = std::max(0.0f,baseLength);

        SoSeparator* res = new SoSeparator;
        res->ref();

        auto q = MathTools::getRotation(Eigen::Vector3f::UnitY(), n);
        SoTransform* trans = new SoTransform;
        trans->translation.setValue(0, baseLength * 0.5f, 0);
        trans->rotation.setValue(q.x, q.y, q.z, q.w);
        res->addChild(trans);

        SoCylinder* c = new SoCylinder();
        c->radius = width;
        c->height = baseLength;
        res->addChild(c);

        SoTranslation* transl = new SoTranslation;
        transl->translation.setValue(0, length * 0.5f, 0);
        res->addChild(transl);

        SoCone* cone = new SoCone();
        cone->bottomRadius.setValue(coneBottomRadius);
        cone->height.setValue(coneHeight);
        res->addChild(cone);

        res->unrefNoDelete();
        return VisualizationPtr(new CoinVisualization(res));
    }

    VisualizationPtr CoinVisualizationFactory::createText(const std::string &text, bool billboard, float offsetX, float offsetY, float offsetZ) const
    {
        SoSeparator* res = new SoSeparator();
        res->ref();

        SoTranslation* moveT = new SoTranslation();
        moveT->translation.setValue(offsetX, offsetY, offsetZ);
        res->addChild(moveT);

        SoAsciiText* textNode = new SoAsciiText();
        if (billboard)
        {
            SoVRMLBillboard* bb = new SoVRMLBillboard();
            res->addChild(bb);
            bb->addChild(textNode);
        }
        else
        {
            res->addChild(textNode);
        }
        SbString text2(*text.c_str());
        text2.apply(&IVToolsHelper_ReplaceSpaceWithUnderscore);
        textNode->string.set(text2.getString());

        res->unrefNoDelete();
        return VisualizationPtr(new CoinVisualization(res));
    }

    VisualizationPtr CoinVisualizationFactory::createCone(float baseRadius, float height) const
    {
        SoCone* cone = new SoCone();
        cone->bottomRadius.setValue(baseRadius);
        cone->height.setValue(height);
        return VisualizationPtr(new CoinVisualization(cone));
    }

    VisualizationPtr CoinVisualizationFactory::createEllipse(float x, float y, float z) const
    {
        // check for min size
        float minSize = 1e-6f;

        if (x < minSize)
        {
            x = minSize;
        }

        if (y < minSize)
        {
            y = minSize;
        }

        if (z < minSize)
        {
            z = minSize;
        }

        SoSeparator* result = new SoSeparator;
        result->ref();

        // ensure good quality
        SoComplexity* c = new SoComplexity();
        c->type.setValue(SoComplexity::OBJECT_SPACE);
        c->value.setValue(1.0f);
        result->addChild(c);

        SoScale* sc1 = new SoScale;
        sc1->scaleFactor.setValue(x, y, z);
        result->addChild(sc1);

        SoSphere* sp = new SoSphere();
        sp->radius.setValue(1.0f);
        result->addChild(sp);

        result->unrefNoDelete();
        return VisualizationPtr(new CoinVisualization(result));
    }

    VisualizationPtr CoinVisualizationFactory::createContactVisualization(const EndEffector::ContactInfoVector &contacts, float frictionConeHeight, float frictionConeRadius, bool scaleAccordingToApproachDir) const
    {
        SoSeparator* res = new SoSeparator;
        res->ref();

        for (const auto& contact : contacts)
        {
            SoSeparator* contactVisu = new SoSeparator();
            res->addChild(contactVisu);

            // add gfx for contact point on object
            SoSeparator* sep = new SoSeparator();
            SoTranslation* tr = new SoTranslation();
            SoMaterial* mat = new SoMaterial();
            mat->diffuseColor.setValue(1.0f, 0, 0);
            tr->translation.setValue(contact.contactPointObstacleGlobal(0), contact.contactPointObstacleGlobal(1), contact.contactPointObstacleGlobal(2));
            SoSphere* sph = new SoSphere();
            sph->radius.setValue(1);
            sep->addChild(mat);
            sep->addChild(tr);
            sep->addChild(sph);
            contactVisu->addChild(sep);

            // add gfx for contact point on finger
            SoSeparator* sep2 = new SoSeparator();
            SoTranslation* tr2 = new SoTranslation();
            tr2->translation.setValue(contact.contactPointFingerGlobal(0), contact.contactPointFingerGlobal(1), contact.contactPointFingerGlobal(2));
            SoSphere* sph2 = new SoSphere();
            sph2->radius.setValue(1);
            sep2->addChild(tr2);
            sep2->addChild(sph2);
            contactVisu->addChild(sep2);

            Eigen::Vector3f n;
            n = contact.contactPointObstacleGlobal - contact.contactPointFingerGlobal;

            if (scaleAccordingToApproachDir && n.norm() > 1e-10)
            {
                float factor = n.dot(contact.approachDirectionGlobal) / n.norm();

                frictionConeHeight *= factor;
                frictionConeRadius *= factor;
            }

            // add gfx for approach direction
            SoSeparator* sep3 = new SoSeparator();
            SoMatrixTransform* tr3 = new SoMatrixTransform();
            SbVec3f transl3(contact.contactPointObstacleGlobal(0), contact.contactPointObstacleGlobal(1), contact.contactPointObstacleGlobal(2));
            // compute rotation
            SbVec3f rotFrom(1.0f, 0.0f, 0.0f);
            SbVec3f rotTo;
            rotTo[0] = n[0];
            rotTo[1] = n[1];
            rotTo[2] = n[2];
            SbRotation rot3(rotFrom, rotTo);

            SbVec3f sc3;
            sc3[0] = sc3[1] = sc3[2] = 1.0f;
            SbMatrix m3;
            m3.setTransform(transl3, rot3, sc3);
            tr3->matrix.setValue(m3);

            // create cone
            float fConeHeight = frictionConeHeight;
            float fConeRadius = frictionConeRadius;
            SoCone* cone3 = new SoCone();
            cone3->bottomRadius = fConeRadius;
            cone3->height = fConeHeight;
            SoSeparator* ConeSep = new SoSeparator;
            SbMatrix orientCone;
            SbMatrix orientCone2;
            orientCone.makeIdentity();
            SbVec3f orientConeA(0.0f, 0.0f, 1.0f);
            SbRotation orientConeR(orientConeA, (float)(-M_PI / 2.0f));
            orientCone.setRotate(orientConeR);
            SbVec3f coneTr(0, -fConeHeight / 2.0f, 0);
            orientCone2.setTranslate(coneTr);
            SoMatrixTransform* coneOri = new SoMatrixTransform();
            coneOri->matrix.setValue(orientCone2.multRight(orientCone));
            ConeSep->addChild(coneOri);
            ConeSep->addChild(cone3);
            // material
            SoMaterial* mat3 = new SoMaterial();
            mat3->diffuseColor.setValue(0.2f, 0.7f, 0.2f);
            mat3->ambientColor.setValue(0.2f, 0.7f, 0.2f);
            mat3->transparency.setValue(0.5f);

            sep3->addChild(mat3);
            sep3->addChild(tr3);
            sep3->addChild(ConeSep);
            contactVisu->addChild(sep3);
        }

        res->unrefNoDelete();
        return VisualizationPtr(new CoinVisualization(res));
    }

    VisualizationPtr CoinVisualizationFactory::createVisualization() const
    {
        return VisualizationPtr(new CoinVisualization(new SoSeparator));
    }

    void CoinVisualizationFactory::cleanup()
    {
        if (SoDB::isInitialized())
        {
            SoDB::finish();
        }
    }

    std::string CoinVisualizationFactory::getVisualizationType() const
    {
        return getName();
    }

    char CoinVisualizationFactory::IVToolsHelper_ReplaceSpaceWithUnderscore(char input)
    {
        if (' ' == input)
        {
            return '_';
        }
        else
        {
            return input;
        }
    }

    std::mutex CoinVisualizationFactory::globalTextureCacheMutex;
    CoinVisualizationFactory::TextureCacheMap CoinVisualizationFactory::globalTextureCache;

    std::ifstream::pos_type getFilesize(const char* filename)
    {
        std::ifstream in(filename, std::ifstream::ate | std::ifstream::binary);
        return in.tellg();
    }

    void CoinVisualizationFactory::RemoveDuplicateTextures(SoNode *node, const std::string &currentPath)
    {
        // internal class to keep track of texture removal
        struct DeleteTextureCallBack : SoDataSensor
        {
            DeleteTextureCallBack(const std::string& nodeName, const std::string& path, size_t filesize) :
                nodeName(nodeName), path(path), filesize(filesize) {}

            // SoDataSensor interface
        public:
            void dyingReference()
            {
                //std::mutex::scoped_lock lock(CoinVisualizationFactory::globalTextureCacheMutex);
                std::unique_lock<std::mutex> scoped_lock(CoinVisualizationFactory::globalTextureCacheMutex);

                CoinVisualizationFactory::globalTextureCache.erase(std::make_pair(filesize, path));
                delete this;
            }

        private:
            ~DeleteTextureCallBack(){}
            std::string nodeName;
            std::string path;
            size_t filesize;
        };


        SoSearchAction sa;
        sa.setType(SoVRMLImageTexture::getClassTypeId());
        sa.setInterest(SoSearchAction::ALL);
        sa.setSearchingAll(TRUE);
        sa.apply(node);
        SoPathList & pl = sa.getPaths();

        //std::mutex::scoped_lock lock(globalTextureCacheMutex);
        std::unique_lock<std::mutex> scoped_lock(globalTextureCacheMutex);

        for (int i = 0; i < pl.getLength(); i++) {
            SoFullPath * p = (SoFullPath*) pl[i];
            if (p->getTail()->isOfType(SoVRMLImageTexture::getClassTypeId())) {
                SoVRMLImageTexture * tex = (SoVRMLImageTexture*) p->getTail();
                for (int i = 0; i < tex->url.getNum(); ++i)
                {
                    SbName name = tex->url[i].getString();
                    std::string texturePath = currentPath + "/" + std::string(tex->url[i].getString());
                    bool exists = boost::filesystem::exists(texturePath);
                    size_t filesize = getFilesize(texturePath.c_str());
                    //                    VR_WARNING << "Texture path: " << texturePath << " file size: " << filesize  << " exists: " << exists << std::endl;

                    //              unsigned long key = (unsigned long) ((void*) name.getString());
                    auto it = globalTextureCache.find(std::make_pair(filesize,texturePath));
                    if (it == globalTextureCache.end())
                    {
                        globalTextureCache[std::make_pair(filesize,texturePath)] =  (void*)tex;
                        tex->addAuditor(new DeleteTextureCallBack(name.getString(), texturePath, filesize), SoNotRec::SENSOR);
                        //                        VR_INFO << "Found NOT in cache" << std::endl;
                    }
                    else if (it->second != (void*) tex)
                    {
                        //                        VR_INFO << "Found in cache" << std::endl;
                        SoNode * parent = p->getNodeFromTail(1);
                        if (parent->isOfType(SoVRMLAppearance::getClassTypeId()))
                        {
                            ((SoVRMLAppearance*)parent)->texture = (SoNode*) it->second;
                        }
                        else
                        {
                            // not a valid VRML2 file. Print a warning or something.
                            std::cout << "not a valid VRML2 file" << std::endl;
                        }
                    }
                }
            }
        }
    }

    std::string CoinVisualizationFactory::getName()
    {
        return "inventor";
    }
} // namespace VirtualRobot
