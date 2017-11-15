#include "OffscreenRenderer.h"

namespace VirtualRobot
{
    void OffscreenRenderer::init(int &, char *[], const std::string &)
    {
    }

    bool OffscreenRenderer::renderOffscreen(const Eigen::Matrix4f&, const std::vector<VisualizationPtr>&, unsigned short, unsigned short, bool, std::vector<unsigned char>&, bool, std::vector<float>&, bool, std::vector<Eigen::Vector3f>&, float, float, float, float) const
    {
        return false;
    }

    bool OffscreenRenderer::renderOffscreenRgbImage(const Eigen::Matrix4f &cameraPose, const std::vector<VisualizationPtr> &scene, unsigned short width, unsigned short height, std::vector<unsigned char> &rgbImage, float zNear, float zFar, float vertFov, float nanValue) const
    {
        auto depthImage = std::vector<float>();
        auto pointCloud = std::vector<Eigen::Vector3f>();
        return renderOffscreen(cameraPose, scene,
                               width, height,
                               true, rgbImage,
                               false, depthImage,
                               false, pointCloud,
                               zNear, zFar, vertFov, nanValue);
    }

    bool OffscreenRenderer::renderOffscreenDepthImage(const Eigen::Matrix4f &cameraPose, const std::vector<VisualizationPtr> &scene, unsigned short width, unsigned short height, std::vector<float> &depthImage, float zNear, float zFar, float vertFov, float nanValue) const
    {
        auto rgbImage = std::vector<unsigned char>();
        auto pointCloud = std::vector<Eigen::Vector3f>();
        return renderOffscreen(cameraPose, scene,
                               width, height,
                               false, rgbImage,
                               true, depthImage,
                               false, pointCloud,
                               zNear, zFar, vertFov, nanValue);
    }

    bool OffscreenRenderer::renderOffscreenPointCloud(const Eigen::Matrix4f &cameraPose, const std::vector<VisualizationPtr> &scene, unsigned short width, unsigned short height, std::vector<Eigen::Vector3f> &pointCloud, float zNear, float zFar, float vertFov, float nanValue) const
    {
        auto rgbImage = std::vector<unsigned char>();
        auto depthImage = std::vector<float>();
        return renderOffscreen(cameraPose, scene,
                               width, height,
                               false, rgbImage,
                               false, depthImage,
                               true, pointCloud,
                               zNear, zFar, vertFov, nanValue);
    }

    void OffscreenRenderer::cleanup()
    {
    }

    OffscreenRendererPtr OffscreenRenderer::getGlobalOffscreenRenderer()
    {
        return OffscreenRenderer::first(NULL);
    }

    std::string OffscreenRenderer::getVisualizationType() const
    {
        return "dummy";
    }
}
