#include "Qt3DOffscreenRenderer.h"

#include "../CoinVisualization/CoinVisualization.h"
#include "Qt3DElement.h"

#include <QTimer>
#include <QEventLoop>

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
        // Root entity in the 3D scene.
        Qt3DCore::QEntity *rootEntity = new Qt3DCore::QEntity();

        // Set up a camera
        Qt3DRender::QCamera *cameraEntity = new Qt3DRender::QCamera(rootEntity);
        cameraEntity->lens()->setPerspectiveProjection((vertFov / M_PI) * 180.0f, static_cast<float>(width)/static_cast<float>(height), zNear, zFar);
        cameraEntity->setProjectionMatrix(QMatrix4x4(camPose.data()).transposed());

        //Add all visualizations to the scene
        for (const auto& visu : scene)
        {

            visualization_cast<Qt3DElement>(visu->clone())->getEntity()->setParent(rootEntity);
        }

        // Create the offscreen engine. This is the object which is responsible for handling the 3D scene itself.
        OffscreenEngine offscreenEngine(cameraEntity, QSize(width, height));
        // Set our scene to be rendered by the offscreen engine.
        offscreenEngine.setSceneRoot(rootEntity);

        Qt3DRender::QRenderCaptureReply *reply = offscreenEngine.getRenderCapture()->requestCapture();
        //Wait for completion or 3000ms timeout event
        QTimer timer;
        timer.setSingleShot(true);
        QEventLoop loop;
        QObject::connect(reply, SIGNAL(completed()), &loop, SLOT(quit()));
        QObject::connect(&timer, SIGNAL(timeout()), &loop, SLOT(quit()));
        timer.start(3000);
        loop.exec();

        if(timer.isActive())
        {
            QImage result = reply->image();
            result = result.convertToFormat(QImage::Format_RGB888);

            const unsigned char* imageBuffer = result.constBits();
            const unsigned int numValues = result.byteCount();
            rgbImage.resize(numValues);
            memcpy(&rgbImage[0], imageBuffer, numValues);

            delete reply;
            return true;
        }
        else
        {
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

    Qt3DOffscreenRenderer::TextureRenderTarget::TextureRenderTarget(Qt3DCore::QNode *parent,
                                             const QSize &size,
                                             Qt3DRender::QRenderTargetOutput::AttachmentPoint attatchmentPoint) :
        Qt3DRender::QRenderTarget(parent),
        size(size)
    {
        // The lifetime of the objects created here is managed
        // automatically, as they become children of this object.

        // Create a render target output for rendering colour.
        output = new Qt3DRender::QRenderTargetOutput(this);
        output->setAttachmentPoint(attatchmentPoint);

        // Create a texture to render into.
        texture = new Qt3DRender::QTexture2D(output);
        texture->setSize(size.width(), size.height());
        texture->setFormat(Qt3DRender::QAbstractTexture::RGB8_UNorm);
        texture->setMinificationFilter(Qt3DRender::QAbstractTexture::Linear);
        texture->setMagnificationFilter(Qt3DRender::QAbstractTexture::Linear);

        // Hook the texture up to our output, and the output up to this object.
        output->setTexture(texture);
        addOutput(output);

        depthTextureOutput = new Qt3DRender::QRenderTargetOutput(this);
        depthTextureOutput->setAttachmentPoint(Qt3DRender::QRenderTargetOutput::Depth);
        depthTexture = new Qt3DRender::QTexture2D(depthTextureOutput);
        depthTexture->setSize(size.width(), size.height());
        depthTexture->setFormat(Qt3DRender::QAbstractTexture::DepthFormat);
        depthTexture->setMinificationFilter(Qt3DRender::QAbstractTexture::Linear);
        depthTexture->setMagnificationFilter(Qt3DRender::QAbstractTexture::Linear);
        depthTexture->setComparisonFunction(Qt3DRender::QAbstractTexture::CompareLessEqual);
        depthTexture->setComparisonMode(Qt3DRender::QAbstractTexture::CompareRefToTexture);
        // Hook up the depth texture
        depthTextureOutput->setTexture(depthTexture);
        addOutput(depthTextureOutput);
    }

    void Qt3DOffscreenRenderer::TextureRenderTarget::setSize(const QSize &size)
    {
        this->size = size;
        texture->setSize(size.width(), size.height());
    }

    Qt3DRender::QTexture2D* Qt3DOffscreenRenderer::TextureRenderTarget::getTexture()
    {
        return texture;
    }



    Qt3DOffscreenRenderer::OffscreenSurfaceFrameGraph::OffscreenSurfaceFrameGraph(Qt3DCore::QNode* parent, Qt3DRender::QCamera *camera, const QSize &size) :
        Qt3DRender::QRenderSurfaceSelector(parent),
        camera(camera)
    {
        // Firstly, create the offscreen surface. This will take the place
        // of a QWindow, allowing us to render our scene without one.
        offscreenSurface = new QOffscreenSurface();
        offscreenSurface->setFormat(QSurfaceFormat::defaultFormat());
        offscreenSurface->create();

        // Hook it up to the frame graph.
        setSurface(offscreenSurface);
        setExternalRenderTargetSize(size);

        // Create a texture to render into. This acts as the buffer that
        // holds the rendered image.
        renderTargetSelector = new Qt3DRender::QRenderTargetSelector(this);
        textureTarget = new TextureRenderTarget(renderTargetSelector, size);
        renderTargetSelector->setTarget(textureTarget);

        // Create a node used for clearing the required buffers.
        clearBuffers = new Qt3DRender::QClearBuffers(renderTargetSelector);
        clearBuffers->setClearColor(QColor(100, 100, 100, 255));
        clearBuffers->setBuffers(Qt3DRender::QClearBuffers::ColorDepthBuffer);

        // Create a viewport node. The viewport here just covers the entire render area.
        viewport = new Qt3DRender::QViewport(renderTargetSelector);
        viewport->setNormalizedRect(QRectF(0.0, 0.0, 1.0, 1.0));

        // Create a camera selector node, and tell it to use the camera we've ben given.
        cameraSelector = new Qt3DRender::QCameraSelector(viewport);
        cameraSelector->setCamera(camera);
    }

    void Qt3DOffscreenRenderer::OffscreenSurfaceFrameGraph::setSize(const QSize &size)
    {
        textureTarget->setSize(size);
        setExternalRenderTargetSize(size);
    }

    Qt3DCore::QNode *Qt3DOffscreenRenderer::OffscreenSurfaceFrameGraph::getRenderTargetSelector()
    {
        return renderTargetSelector;
    }

    Qt3DOffscreenRenderer::OffscreenEngine::OffscreenEngine(Qt3DRender::QCamera *camera, const QSize &size)
    {
        sceneRoot = nullptr;

        // Set up the engine and the aspects that we want to use.
        aspectEngine = new Qt3DCore::QAspectEngine();
        renderAspect = new Qt3DRender::QRenderAspect(Qt3DRender::QRenderAspect::Threaded); // Only threaded mode seems to work right now.
        logicAspect = new Qt3DLogic::QLogicAspect();

        aspectEngine->registerAspect(renderAspect);
        aspectEngine->registerAspect(logicAspect);

        // Create the root entity of the engine.
        // This is not the same as the 3D scene root: the QRenderSettings
        // component must be held by the root of the QEntity tree,
        // so it is added to this one. The 3D scene is added as a subtree later,
        // in setSceneRoot().
        Qt3DCore::QEntityPtr root(new Qt3DCore::QEntity());
        renderSettings = new Qt3DRender::QRenderSettings(root.data());
        root->addComponent(renderSettings);

        // Create the offscreen frame graph, which will manage all of the resources required
        // for rendering without a QWindow.
        offscreenFrameGraph = new OffscreenSurfaceFrameGraph(renderSettings, camera, size);

        // Set this frame graph to be in use.
        renderSettings->setActiveFrameGraph(offscreenFrameGraph);

        // Add a render capture node to the frame graph.
        // This is set as the next child of the render target selector node,
        // so that the capture will be taken from the specified render target
        // once all other rendering operations have taken place.
        renderCapture = new Qt3DRender::QRenderCapture(offscreenFrameGraph->getRenderTargetSelector());

        // Set the root entity of the engine. This causes the engine to begin running.
        aspectEngine->setRootEntity(root);
    }

    Qt3DOffscreenRenderer::OffscreenEngine::~OffscreenEngine()
    {
        // Setting a null root entity shuts down the engine.
        aspectEngine->setRootEntity(Qt3DCore::QEntityPtr());

        // Not sure if the following is strictly required, as it may
        // happen automatically when the engine is destroyed.
        aspectEngine->unregisterAspect(logicAspect);
        aspectEngine->unregisterAspect(renderAspect);
        delete logicAspect;
        delete renderAspect;

        delete aspectEngine;
    }

    void Qt3DOffscreenRenderer::OffscreenEngine::setSceneRoot(Qt3DCore::QNode *sceneRoot)
    {
        // Make sure any existing scene root is unparented.
        if ( this->sceneRoot )
        {
            this->sceneRoot->setParent(static_cast<Qt3DCore::QNode*>(nullptr));
        }

        // Parent the incoming scene root to our current root entity.
        this->sceneRoot = sceneRoot;
        this->sceneRoot->setParent(aspectEngine->rootEntity().data());
    }

    Qt3DRender::QRenderCapture* Qt3DOffscreenRenderer::OffscreenEngine::getRenderCapture()
    {
        return renderCapture;
    }

    void Qt3DOffscreenRenderer::OffscreenEngine::setSize(const QSize &size)
    {
        offscreenFrameGraph->setSize(size);
    }
}
