#include "Qt3DOffscreenRenderer.h"

namespace VirtualRobot
{
    void Qt3DOffscreenRenderer::init(int &, char *[], const std::string &)
    {
    }

    bool Qt3DOffscreenRenderer::renderOffscreen(const Eigen::Matrix4f &camPose, const std::vector<VirtualRobot::VisualizationPtr> &scene,
                                                unsigned short width, unsigned short height,
                                                bool renderRgbImage, std::vector<unsigned char> &rgbImage,
                                                bool renderDepthImage, std::vector<float> &depthImage,
                                                bool renderPointcloud, std::vector<Eigen::Vector3f> &pointCloud,
                                                float zNear, float zFar, float vertFov, float nanValue) const
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
