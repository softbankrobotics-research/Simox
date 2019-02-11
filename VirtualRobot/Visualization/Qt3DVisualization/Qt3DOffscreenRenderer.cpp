#include "Qt3DOffscreenRenderer.h"

#include <VirtualRobot/Tools/RuntimeEnvironment.h>
#include <VirtualRobot/VirtualRobot.h>

#include <Qt3DCore/QEntity>
#include <Qt3DRender/QCamera>
#include <Qt3DRender/QPointLight>
#include <Qt3DCore/QTransform>
#include <Qt3DCore/QNode>
#include <Qt3DRender/QCamera>
#include <Qt3DRender/QRenderSettings>
#include <Qt3DRender/QRenderCapture>
#include <Qt3DCore/QAspectEngine>
#include <Qt3DRender/QRenderAspect>
#include <Qt3DLogic/QLogicAspect>
#include <Qt3DRender/QLayer>
#include <Qt3DRender/QMaterial>
#include <Qt3DCore/QTransform>
#include <Qt3DExtras/QCuboidMesh>
#include <Qt3DExtras/QSphereMesh>
#include <Qt3DExtras/QPhongMaterial>

#include <QTimer>
#include <QEventLoop>

#include "OffscreenRendering/OffscreenRenderEngineShaderEffect.h"

namespace VirtualRobot
{
    Qt3DOffscreenRenderer::Qt3DOffscreenRenderer()
    {
        this->engine = new OffscreenRenderEngine(QSize(500, 500));

        //Discard first and second frame, as they sometimes contain only clear color and no content.
        Qt3DRender::QRenderCaptureReply *firstReply = engine->getRenderCapture()->requestCapture();
        Qt3DRender::QRenderCaptureReply *secondReply = engine->getRenderCapture()->requestCapture();

        delete firstReply;
        delete secondReply;
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
        //Set engine to new size
        engine->setSize(QSize(width, height));

        //Request frame
        Qt3DRender::QRenderCaptureReply *reply = engine->getRenderCapture()->requestCapture();

        //Wait for completion or 3000ms timeout event
        QTimer timer;
        QEventLoop loop;
        QObject::connect(reply, SIGNAL(completed()), &loop, SLOT(quit()));
        QObject::connect(&timer, SIGNAL(timeout()), &loop, SLOT(quit()));
        timer.start(3000);
        loop.exec();

        if(timer.isActive())
        {
            QImage result = reply->image();
            VR_ASSERT(result.width() == width);
            VR_ASSERT(result.height() == height);

            result = result.convertToFormat(QImage::Format_RGB888);
            const unsigned char* imageBuffer = result.constBits();
            const unsigned int numValues = static_cast<unsigned int>(result.byteCount());
            rgbImage.resize(numValues);
            memcpy(&rgbImage[0], imageBuffer, numValues);
            delete reply;
            return true;
        }
        else
        {
            delete reply;
            return false;
        }
    }

    void Qt3DOffscreenRenderer::cleanup()
    {
    }

    std::string Qt3DOffscreenRenderer::getVisualizationType() const
    {
        return "qt3d";
    }
}
