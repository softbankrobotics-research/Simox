#include "OffscreenRenderEngineFrameGraph.h"


VirtualRobot::OffscreenRenderEngineFrameGraph::OffscreenRenderEngineFrameGraph(Qt3DCore::QNode* parent, Qt3DRender::QCamera *camera, const QSize &size) :
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
    clearBuffers->setClearColor(QColor(255, 255, 255, 255));
    clearBuffers->setBuffers(Qt3DRender::QClearBuffers::ColorDepthBuffer);

    // Create a camera selector node, and tell it to use the camera we've been given.
    cameraSelector = new Qt3DRender::QCameraSelector(renderTargetSelector);
    cameraSelector->setCamera(camera);

    // Create a viewport node.
    viewportLeft = new Qt3DRender::QViewport(cameraSelector);
    viewportLeft->setNormalizedRect(QRectF(0.0, 0.0, 0.5, 1.0));

    viewportLeftLayer = new Qt3DRender::QLayerFilter(viewportLeft);

    // Create a viewport node.
    viewportRight = new Qt3DRender::QViewport(cameraSelector);
    viewportRight->setNormalizedRect(QRectF(0.5, 0.0, 0.5, 1.0));

    viewportRightLayer = new Qt3DRender::QLayerFilter(viewportRight);
}

void VirtualRobot::OffscreenRenderEngineFrameGraph::setSize(const QSize &size)
{
    textureTarget->setSize(size);
    setExternalRenderTargetSize(size);
}

Qt3DCore::QNode *VirtualRobot::OffscreenRenderEngineFrameGraph::getRenderTargetSelector()
{
    return renderTargetSelector;
}

void VirtualRobot::OffscreenRenderEngineFrameGraph::setViewportLeftLayer(Qt3DRender::QLayer *layer)
{
    viewportLeftLayer->addLayer(layer);
}

void VirtualRobot::OffscreenRenderEngineFrameGraph::setViewportRightLayer(Qt3DRender::QLayer *layer)
{
    viewportRightLayer->addLayer(layer);
}

VirtualRobot::OffscreenRenderEngineFrameGraph::TextureRenderTarget::TextureRenderTarget(Qt3DCore::QNode *parent,
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

Qt3DRender::QTexture2D* VirtualRobot::OffscreenRenderEngineFrameGraph::TextureRenderTarget::getTexture()
{
    return texture;
}

void VirtualRobot::OffscreenRenderEngineFrameGraph::TextureRenderTarget::setSize(const QSize &size)
{
    this->size = size;
    texture->setSize(size.width(), size.height());
    depthTexture->setSize(size.width(), size.height());
}
