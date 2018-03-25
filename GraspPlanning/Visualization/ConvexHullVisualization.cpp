#include "ConvexHullVisualization.h"

#include "../ConvexHullGenerator.h"

#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>


namespace GraspPlanning
{

    ConvexHullVisualization::ConvexHullVisualization(VirtualRobot::MathTools::ConvexHull6DPtr convHull, bool useFirst3Coords)
        : convHull6D(convHull),
          useFirst3Coords(useFirst3Coords)
    {
        visualization = createConvexHullVisualization(convHull6D, useFirst3Coords);
    }

    ConvexHullVisualization::ConvexHullVisualization(VirtualRobot::MathTools::ConvexHull3DPtr convHull)
        : convHull3D(convHull),
          useFirst3Coords(true)
    {
        visualization = createConvexHullVisualization(convHull3D);
    }

    VirtualRobot::VisualizationPtr ConvexHullVisualization::getVisualization()
    {
        return visualization;
    }

    VirtualRobot::VisualizationPtr ConvexHullVisualization::createConvexHullVisualization(const VirtualRobot::MathTools::ConvexHull3DPtr &convHull)
    {
        const auto visualizationFactory = VirtualRobot::VisualizationFactory::getInstance();
        if (!convHull || convHull->vertices.size() == 0 || convHull->faces.size() == 0)
        {
            return visualizationFactory->createVisualization();
        }

        VirtualRobot::TriMeshModelPtr mesh(new VirtualRobot::TriMeshModel);

        for (size_t i = 0; i < convHull->faces.size(); i++)
        {
            Eigen::Vector3f& v1 = convHull->vertices.at(convHull->faces[i].id1);
            Eigen::Vector3f& v2 = convHull->vertices.at(convHull->faces[i].id2);
            Eigen::Vector3f& v3 = convHull->vertices.at(convHull->faces[i].id3);

            Eigen::Vector3f& n = convHull->faces[i].normal;

            bool bNeedFlip = ConvexHullGenerator::checkVerticeOrientation(v1, v2, v3, n);

            // COUNTER CLOCKWISE
            if (bNeedFlip)
            {
                mesh->addTriangleWithFace(v3, v2, v1);
            }
            else
            {
                mesh->addTriangleWithFace(v1, v2, v3);
            }
        }
        return visualizationFactory->createTriMeshModel(mesh);
    }

    VirtualRobot::VisualizationPtr ConvexHullVisualization::createConvexHullVisualization(const VirtualRobot::MathTools::ConvexHull6DPtr &convHull, bool buseFirst3Coords)
    {
        const auto visualizationFactory = VirtualRobot::VisualizationFactory::getInstance();
        if (!convHull || convHull->vertices.size() == 0 || convHull->faces.size() == 0)
        {
            return visualizationFactory->createVisualization();
        }

        std::vector<Eigen::Vector3f> vProjectedPoints;
        for (size_t i = 0; i < convHull->faces.size(); i++)
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
            GRASPPLANNING_ERROR << " Could not create hull of projected points, aborting..." << endl;
            return visualizationFactory->createVisualization();
        }

        float scaleFactor = 400.0f;

        auto visu = createConvexHullVisualization(projectedHull);
        visu->scale(Eigen::Vector3f::Constant(scaleFactor));
        return visu;
    }

} // namespace GraspPlanning
