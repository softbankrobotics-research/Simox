
#include "CoinConvexHullVisualization.h"
#include "../../ConvexHullGenerator.h"

#include <Inventor/SoPrimitiveVertex.h>
#include <Inventor/SbLinear.h>
#include <Inventor/nodes/SoShape.h>
#include <Inventor/actions/SoCallbackAction.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/nodes/SoScale.h>
#include <Inventor/nodes/SoNormal.h>


#include <Inventor/nodes/SoComplexity.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoLineSet.h>
#include <Inventor/nodes/SoFaceSet.h>

#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoBaseColor.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoUnits.h>


namespace GraspStudio
{

    CoinConvexHullVisualization::CoinConvexHullVisualization(VirtualRobot::MathTools::ConvexHull6DPtr convHull, bool useFirst3Coords) :
        ConvexHullVisualization(convHull, useFirst3Coords)
    {
        buildVisu();
    }

    CoinConvexHullVisualization::CoinConvexHullVisualization(VirtualRobot::MathTools::ConvexHull3DPtr convHull) :
        ConvexHullVisualization(convHull)
    {
        buildVisu();
    }

    void CoinConvexHullVisualization::buildVisu()
    {
        visualization = new SoSeparator();
        visualization->ref();

        if (convHull3D)
        {
            SoSeparator* vi = createConvexHullVisualization(convHull3D);

            if (vi)
            {
                visualization->addChild(vi);
            }

        }

        if (convHull6D)
        {
            SoSeparator* vi = createConvexHullVisualization(convHull6D, useFirst3Coords);

            if (vi)
            {
                visualization->addChild(vi);
            }
        }
    }

    /**
     * If CoinConvexHullVisualization::visualization is a valid object call SoNode::unref()
     * on it.
     */
    CoinConvexHullVisualization::~CoinConvexHullVisualization()
    {
        if (visualization)
        {
            visualization->unref();
        }

    }



    /**
     * This mehtod returns the internal CoinConvexHullVisualization::visualization.
     */
    SoSeparator* CoinConvexHullVisualization::getCoinVisualization()
    {
        return visualization;
    }


    SoSeparator* CoinConvexHullVisualization::createConvexHullVisualization(VirtualRobot::MathTools::ConvexHull3DPtr& convHull)
    {
        SoSeparator* result = new SoSeparator;

        if (!convHull || convHull->vertices.size() == 0 || convHull->faces.size() == 0)
        {
            return result;
        }

        result->ref();

        SoUnits* u = new SoUnits;
        u->units = SoUnits::MILLIMETERS;
        result->addChild(u);

        SoCoordinate3* pCoords = new SoCoordinate3();
        SoFaceSet* pFaceSet = new SoFaceSet();

        int nFaces = (int)convHull->faces.size();
        int nVertices = nFaces * 3;

        // compute points and normals
        SbVec3f* pVertexArray = new SbVec3f[nVertices];
        int nVertexCount = 0;

        for (int i = 0; i < nFaces; i++)
        {
            Eigen::Vector3f &v1 = convHull->vertices.at(convHull->faces[i].id1);
            Eigen::Vector3f &v2 = convHull->vertices.at(convHull->faces[i].id2);
            Eigen::Vector3f &v3 = convHull->vertices.at(convHull->faces[i].id3);

            Eigen::Vector3f &n = convHull->faces[i].normal;

            bool bNeedFlip = ConvexHullGenerator::checkVerticeOrientation(v1, v2, v3, n);

            // COUNTER CLOCKWISE
            if (bNeedFlip)
            {
                pVertexArray[nVertexCount].setValue((float)v3(0), (float)v3(1), (float)v3(2));
            }
            else
            {
                pVertexArray[nVertexCount].setValue((float)v1(0), (float)v1(1), (float)v1(2));
            }

            nVertexCount++;
            pVertexArray[nVertexCount].setValue((float)v2(0), (float)v2(1), (float)v2(2));
            nVertexCount++;

            if (bNeedFlip)
            {
                pVertexArray[nVertexCount].setValue((float)v1(0), (float)v1(1), (float)v1(2));
            }
            else
            {
                pVertexArray[nVertexCount].setValue((float)v3(0), (float)v3(1), (float)v3(2));
            }

            nVertexCount++;
        }

        // set normals
        //pNormals->vector.setValues(0, nFaces, normalsArray);
        //result->addChild(pNormals);
        SoNormalBinding *pNormalBinding = new SoNormalBinding;
        pNormalBinding->value = SoNormalBinding::NONE;
        result->addChild(pNormalBinding);

        // set points
        pCoords->point.setValues(0, nVertices, pVertexArray);
        result->addChild(pCoords);

        // set faces
        int32_t* nNumVertices = new int32_t[nFaces];

        for (int i = 0; i < nFaces; i++)
        {
            nNumVertices[i] = 3;
        }
        pFaceSet->numVertices.setValues(0, nFaces, nNumVertices);

        pFaceSet->numVertices.setValues(0, nFaces, (const int32_t*)nNumVertices);

        result->addChild(pCoords);
        result->addChild(pFaceSet);
        //delete []pVertexArray;
        //delete []nNumVertices;
        //result->unrefNoDelete();

        return result;
    }


    SoSeparator* CoinConvexHullVisualization::createConvexHullVisualization(VirtualRobot::MathTools::ConvexHull6DPtr& convHull, bool buseFirst3Coords)
    {
        SoSeparator* result = new SoSeparator;

        if (!convHull || convHull->vertices.size() == 0 || convHull->faces.size() == 0)
        {
            return result;
        }

        result->ref();

        SoUnits* u = new SoUnits;
        u->units = SoUnits::MILLIMETERS;
        result->addChild(u);

        SoScale* sc = new SoScale();
        float fc = 400.0f;
        sc->scaleFactor.setValue(fc, fc, fc);
        result->addChild(sc);


        int nFaces = (int)convHull->faces.size();


        //Eigen::Vector3f v[6];
        // project points to 3d, then create hull of these points to visualize it
        std::vector<Eigen::Vector3f> vProjectedPoints;

        for (int i = 0; i < nFaces; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                if (buseFirst3Coords)
                {
                    //v[j] = convHull->vertices.at(convHull->faces[i].id[j]).p;
                    vProjectedPoints.push_back(convHull->vertices[ convHull->faces[i].id[j] ].p);
                }
                else
                {
                    //v[j] = convHull->vertices.at(convHull->faces[i].id[j]).n;
                    vProjectedPoints.push_back(convHull->vertices[ convHull->faces[i].id[j] ].n);
                }

            }
        }

        VirtualRobot::MathTools::ConvexHull3DPtr projectedHull = ConvexHullGenerator::CreateConvexHull(vProjectedPoints);

        if (!projectedHull)
        {
            GRASPSTUDIO_ERROR << " Could not create hull of projected points, aborting..." << endl;
            return result;
        }

        SoSeparator* hullV = createConvexHullVisualization(projectedHull);

        if (!hullV)
        {
            GRASPSTUDIO_ERROR << " Could not create visualization of projected points, aborting..." << endl;
            return result;
        }

        result->addChild(hullV);
        result->unrefNoDelete();
        return result;
    }


}
