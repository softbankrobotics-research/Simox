#include "Qt3DOffscreenRenderer.h"

#include "Qt3DVisualization.h"
#include "Qt3DVisualizationSet.h"
#include "Qt3DVisualizationFactory.h"

#include <QTimer>
#include <QEventLoop>

#include <Qt3DExtras/QCuboidMesh>

namespace VirtualRobot
{
    Qt3DOffscreenRenderer::Qt3DOffscreenRenderer()
    {

    }

    Qt3DOffscreenRenderer::~Qt3DOffscreenRenderer()
    {

    }

    void Qt3DOffscreenRenderer::init(int &, char *[], const std::string &)
    {

    }

    bool Qt3DOffscreenRenderer::renderOffscreen(const Eigen::Matrix4f &camPose, const std::vector<VirtualRobot::VisualizationPtr> &scene,
                                                unsigned short width, unsigned short height,
                                                bool renderRgbImage, std::vector<unsigned char> &rgbImage,
                                                bool renderDepthImage, std::vector<float> &depthImage,
                                                bool renderPointcloud, std::vector<Eigen::Vector3f> &pointCloud,
                                                float zNear, float zFar, float vertFov, float nanValue, Visualization::Color backgroundColor) const
    {
        return false;
    }

    void Qt3DOffscreenRenderer::cleanup()
    {
    }

    std::string Qt3DOffscreenRenderer::getVisualizationType() const
    {
        return "qt3d";
    }
}
