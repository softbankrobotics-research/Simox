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
        this->rootEntity = new Qt3DCore::QEntity();
        this->cameraEntity = new Qt3DRender::QCamera(rootEntity);

        auto lightEntity = new Qt3DCore::QEntity(rootEntity);
        auto light = new Qt3DRender::QPointLight(lightEntity);
        light->setIntensity(1.2f);
        lightEntity->addComponent(light);
        auto lightTransform = new Qt3DCore::QTransform(lightEntity);
        lightTransform->setTranslation(this->cameraEntity->position());
        QObject::connect(this->cameraEntity, &Qt3DRender::QCamera::positionChanged, lightTransform, &Qt3DCore::QTransform::setTranslation);
        lightEntity->addComponent(lightTransform);


        this->offscreenEngine = new Qt3DOffscreenRenderer::OffscreenEngine(cameraEntity, QSize(640, 480));
        this->offscreenEngine->setSceneRoot(rootEntity);
    }

    Qt3DOffscreenRenderer::~Qt3DOffscreenRenderer()
    {
        delete this->offscreenEngine;
        delete this->cameraEntity;
        delete this->rootEntity;
    }

    void Qt3DOffscreenRenderer::init(int &, char *[], const std::string &)
    {
        //Render once, because first frame is always emtpy clear buffer
        auto reply = render();
        delete reply;
    }

    Qt3DRender::QRenderCaptureReply *Qt3DOffscreenRenderer::render() const
    {
        Qt3DRender::QRenderCaptureReply *reply = this->offscreenEngine->getRenderCapture()->requestCapture();
        //Wait for completion or 3000ms timeout event
        QTimer timer;
        QEventLoop loop;
        QObject::connect(reply, SIGNAL(completed()), &loop, SLOT(quit()));
        QObject::connect(&timer, SIGNAL(timeout()), &loop, SLOT(quit()));
        timer.start(3000);
        loop.exec();

        if(timer.isActive())
        {
            return reply;
        }
        else
        {
            delete reply;
            return nullptr;
        }
    }

    void Qt3DOffscreenRenderer::addToSceneGraph(const VisualizationPtr &visu) const
    {
        VirtualRobot::VisualizationSetPtr set = std::dynamic_pointer_cast<VisualizationSet>(visu);
        if (set)
        {
            for (const auto& v : set->getVisualizations())
            {
                addToSceneGraph(v);
            }
        }
        else
        {
            //Qt3DVisualizationPtr visu2 = std::static_pointer_cast<Qt3DVisualization>(visu);
            //visu2->getEntity()->setParent(this->rootEntity);
            //auto clonedVisu = visu->clone();
            std::static_pointer_cast<Qt3DVisualization>(visu)->getEntity()->setParent(rootEntity);
        }
    }

    bool Qt3DOffscreenRenderer::renderOffscreen(const Eigen::Matrix4f &camPose, const std::vector<VirtualRobot::VisualizationPtr> &scene,
                                                unsigned short width, unsigned short height,
                                                bool renderRgbImage, std::vector<unsigned char> &rgbImage,
                                                bool renderDepthImage, std::vector<float> &depthImage,
                                                bool renderPointcloud, std::vector<Eigen::Vector3f> &pointCloud,
                                                float zNear, float zFar, float vertFov, float nanValue, Visualization::Color backgroundColor) const
    {
        this->offscreenEngine->setSize(QSize(width, height));

        this->cameraEntity->lens()->setPerspectiveProjection((vertFov / M_PI) * 180.0f, static_cast<float>(width)/static_cast<float>(height), zNear, zFar);
        //cameraEntity->setProjectionMatrix(QMatrix4x4(camPose.data()).transposed());
        this->cameraEntity->setPosition(QVector3D(800.0f, 1000.0f, 2000.0f));
        this->cameraEntity->setViewCenter(QVector3D(0.0f, 0.0f, 0.0f));

        /*//Remove all visulizations from current scene
        for(auto child : rootEntity->children())
        {
            child->setParent(nullptr);
        }*/

        //Add all visualizations to the scene
        for (const auto& visu : scene)
        {
            addToSceneGraph(visu);
        }

        Qt3DRender::QRenderCaptureReply *reply = render();
        if(reply)
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
        clearBuffers->setClearColor(QColor(255, 255, 255, 255));
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

    Qt3DOffscreenRenderer::GBuffer::GBuffer(Qt3DCore::QNode *parent)
        : Qt3DRender::QRenderTarget(parent)
    {
        const Qt3DRender::QAbstractTexture::TextureFormat formats[AttachmentsCount] = {
            Qt3DRender::QAbstractTexture::RGBA32F,
            // We use RGBA32F for the following two instead of a more fitting format because
            // OpenGL vendors might not support other formats
            Qt3DRender::QAbstractTexture::RGBA32F,
            Qt3DRender::QAbstractTexture::RGBA32F,
            Qt3DRender::QAbstractTexture::D32F
        };

        const Qt3DRender::QRenderTargetOutput::AttachmentPoint attachmentPoints[AttachmentsCount] = {
            Qt3DRender::QRenderTargetOutput::Color0,
            Qt3DRender::QRenderTargetOutput::Color1,
            Qt3DRender::QRenderTargetOutput::Color2,
            Qt3DRender::QRenderTargetOutput::Depth
        };

        for (int i = 0; i < AttachmentsCount; i++) {
            Qt3DRender::QRenderTargetOutput *output = new Qt3DRender::QRenderTargetOutput(this);

            m_textures[i] = new Qt3DRender::QTexture2D();
            m_textures[i]->setFormat(formats[i]);
            m_textures[i]->setWidth(1024);
            m_textures[i]->setHeight(1024);Qt3DOffscreenRenderer::GBuffer::
            m_textures[i]->setGenerateMipMaps(false);
            m_textures[i]->setWrapMode(Qt3DRender::QTextureWrapMode(Qt3DRender::QTextureWrapMode::ClampToEdge));
            m_textures[i]->setMinificationFilter(Qt3DRender::QAbstractTexture::Linear);
            m_textures[i]->setMagnificationFilter(Qt3DRender::QAbstractTexture::Linear);

            output->setTexture(m_textures[i]);
            output->setAttachmentPoint(attachmentPoints[i]);
            addOutput(output);
        }
    }

    Qt3DRender::QAbstractTexture *Qt3DOffscreenRenderer::GBuffer::colorTexture() const
    {
        return m_textures[Color];
    }

    Qt3DRender::QAbstractTexture *Qt3DOffscreenRenderer::GBuffer::positionTexture() const
    {
        return m_textures[Position];
    }

    Qt3DRender::QAbstractTexture *Qt3DOffscreenRenderer::GBuffer::normalTexture() const
    {
        return m_textures[Normal];
    }

    Qt3DRender::QAbstractTexture *Qt3DOffscreenRenderer::GBuffer::depthTexture() const
    {
        return m_textures[Depth];
    }

    Qt3DOffscreenRenderer::FinalShaderEffect::FinalShaderEffect(Qt3DCore::QNode *parent)
        : Qt3DRender::QEffect(parent)
        , m_gl3Technique(new Qt3DRender::QTechnique())
        , m_gl2Technique(new Qt3DRender::QTechnique())
        , m_gl2Pass(new Qt3DRender::QRenderPass())
        , m_gl3Pass(new Qt3DRender::QRenderPass())
        , m_passCriterion(new Qt3DRender::QFilterKey(this))
    {
        m_gl3Technique->graphicsApiFilter()->setApi(Qt3DRender::QGraphicsApiFilter::OpenGL);
        m_gl3Technique->graphicsApiFilter()->setMajorVersion(3);
        m_gl3Technique->graphicsApiFilter()->setMinorVersion(1);
        m_gl3Technique->graphicsApiFilter()->setProfile(Qt3DRender::QGraphicsApiFilter::CoreProfile);

        m_gl2Technique->graphicsApiFilter()->setApi(Qt3DRender::QGraphicsApiFilter::OpenGL);
        m_gl2Technique->graphicsApiFilter()->setMajorVersion(2);
        m_gl2Technique->graphicsApiFilter()->setMinorVersion(0);
        m_gl2Technique->graphicsApiFilter()->setProfile(Qt3DRender::QGraphicsApiFilter::NoProfile);

        m_passCriterion->setName(QStringLiteral("pass"));
        m_passCriterion->setValue(QStringLiteral("final"));

        Qt3DRender::QShaderProgram *gl3Shader = new Qt3DRender::QShaderProgram();
        gl3Shader->setVertexShaderCode(gl3Shader->loadSource(QUrl(QStringLiteral("qrc:/final_gl3.vert"))));
        gl3Shader->setFragmentShaderCode(gl3Shader->loadSource(QUrl(QStringLiteral("qrc:/final_gl3.frag"))));

        m_gl3Pass->addFilterKey(m_passCriterion);
        m_gl3Pass->setShaderProgram(gl3Shader);
        m_gl3Technique->addRenderPass(m_gl3Pass);

        Qt3DRender::QShaderProgram *gl2Shader = new Qt3DRender::QShaderProgram();
        gl2Shader->setVertexShaderCode(gl2Shader->loadSource(QUrl(QStringLiteral("qrc:/final_gl2.vert"))));
        gl2Shader->setFragmentShaderCode(gl2Shader->loadSource(QUrl(QStringLiteral("qrc:/final_gl2.frag"))));

        m_gl2Pass->addFilterKey(m_passCriterion);
        m_gl2Pass->setShaderProgram(gl2Shader);
        m_gl2Technique->addRenderPass(m_gl2Pass);

        addTechnique(m_gl3Technique);
        addTechnique(m_gl2Technique);
    }

    QList<Qt3DRender::QFilterKey *> Qt3DOffscreenRenderer::FinalShaderEffect::passCriteria() const
    {
        return QList<Qt3DRender::QFilterKey *>() << m_passCriterion;
    }

    Qt3DOffscreenRenderer::GBufferShaderEffect::GBufferShaderEffect(Qt3DCore::QNode *parent)
        : Qt3DRender::QEffect(parent)
        , m_gl3Technique(new Qt3DRender::QTechnique())
        , m_gl2Technique(new Qt3DRender::QTechnique())
        , m_gl2Pass(new Qt3DRender::QRenderPass())
        , m_gl3Pass(new Qt3DRender::QRenderPass())
        , m_passCriterion(new Qt3DRender::QFilterKey(this))
    {

        m_gl3Technique->graphicsApiFilter()->setProfile(Qt3DRender::QGraphicsApiFilter::CoreProfile);
        m_gl3Technique->graphicsApiFilter()->setApi(Qt3DRender::QGraphicsApiFilter::OpenGL);
        m_gl3Technique->graphicsApiFilter()->setMajorVersion(3);
        m_gl3Technique->graphicsApiFilter()->setMinorVersion(1);

        m_gl2Technique->graphicsApiFilter()->setApi(Qt3DRender::QGraphicsApiFilter::OpenGL);
        m_gl2Technique->graphicsApiFilter()->setMajorVersion(2);
        m_gl2Technique->graphicsApiFilter()->setMinorVersion(0);
        m_gl2Technique->graphicsApiFilter()->setProfile(Qt3DRender::QGraphicsApiFilter::NoProfile);


        m_passCriterion->setName(QStringLiteral("pass"));
        m_passCriterion->setValue(QStringLiteral("geometry"));

        Qt3DRender::QShaderProgram *gl3Shader = new Qt3DRender::QShaderProgram();
        gl3Shader->setVertexShaderCode(Qt3DRender::QShaderProgram::loadSource(QUrl(QStringLiteral("qrc:/geometry_gl3.vert"))));
        gl3Shader->setFragmentShaderCode(Qt3DRender::QShaderProgram::loadSource(QUrl(QStringLiteral("qrc:/geometry_gl3.frag"))));

        m_gl3Pass->addFilterKey(m_passCriterion);
        m_gl3Pass->setShaderProgram(gl3Shader);
        m_gl3Technique->addRenderPass(m_gl3Pass);

        Qt3DRender::QShaderProgram *gl2Shader = new Qt3DRender::QShaderProgram();
        gl2Shader->setVertexShaderCode(Qt3DRender::QShaderProgram::loadSource(QUrl(QStringLiteral("qrc:/geometry_gl2.vert"))));
        gl2Shader->setFragmentShaderCode(Qt3DRender::QShaderProgram::loadSource(QUrl(QStringLiteral("qrc:/geometry_gl2.frag"))));

        m_gl2Pass->addFilterKey(m_passCriterion);
        m_gl2Pass->setShaderProgram(gl2Shader);
        m_gl2Technique->addRenderPass(m_gl2Pass);

        addTechnique(m_gl3Technique);
        addTechnique(m_gl2Technique);
    }

    QList<Qt3DRender::QFilterKey *> Qt3DOffscreenRenderer::GBufferShaderEffect::passCriteria() const
    {
        return QList<Qt3DRender::QFilterKey *>() << m_passCriterion;
    }

    Qt3DOffscreenRenderer::OffscreenSurfaceDepthFrameGraph::OffscreenSurfaceDepthFrameGraph(Qt3DCore::QNode *parent, Qt3DRender::QCamera *camera, const QSize &size)
        : Qt3DRender::QRenderSurfaceSelector(parent)
    {
        viewport = new Qt3DRender::QViewport(this);
        sceneFilter = new Qt3DRender::QLayerFilter(viewport);
        gBufferTargetSelector = new Qt3DRender::QRenderTargetSelector(sceneFilter);
        clearGBuffer = new Qt3DRender::QClearBuffers(gBufferTargetSelector);
        geometryPassFilter = new Qt3DRender::QRenderPassFilter(clearGBuffer);
        sceneCameraSelector = new Qt3DRender::QCameraSelector(geometryPassFilter);
        renderTargetSelector = new Qt3DRender::QRenderTargetSelector(viewport);
        textureTarget = new TextureRenderTarget(renderTargetSelector, size);
        screenQuadFilter = new Qt3DRender::QLayerFilter(textureTarget);
        clearScreenQuad = new Qt3DRender::QClearBuffers(screenQuadFilter);
        finalPassFilter = new Qt3DRender::QRenderPassFilter(clearScreenQuad);
        sizeParameter = new Qt3DRender::QParameter(QStringLiteral("winSize"), size);
        gBuffer = new GBuffer(this);

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
        renderTargetSelector->setTarget(textureTarget);

        clearGBuffer->setBuffers(Qt3DRender::QClearBuffers::ColorDepthBuffer);
        clearScreenQuad->setBuffers(Qt3DRender::QClearBuffers::ColorDepthBuffer);
        clearScreenQuad->setClearColor(QColor(255, 255, 255));
        gBufferTargetSelector->setTarget(gBuffer);

        finalPassFilter->addParameter(new Qt3DRender::QParameter(QStringLiteral("position"), gBuffer->positionTexture()));
        finalPassFilter->addParameter(new Qt3DRender::QParameter(QStringLiteral("normal"), gBuffer->normalTexture()));
        finalPassFilter->addParameter(new Qt3DRender::QParameter(QStringLiteral("color"), gBuffer->colorTexture()));
        finalPassFilter->addParameter(new Qt3DRender::QParameter(QStringLiteral("depth"), gBuffer->depthTexture()));

        finalPassFilter->addParameter(sizeParameter);

        sceneCameraSelector->setCamera(camera);
    }

    void Qt3DOffscreenRenderer::OffscreenSurfaceDepthFrameGraph::setSize(const QSize &size)
    {
        textureTarget->setSize(size);
        setExternalRenderTargetSize(size);
        sizeParameter->setValue(size);
    }

    Qt3DCore::QNode *Qt3DOffscreenRenderer::OffscreenSurfaceDepthFrameGraph::getRenderTargetSelector()
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

    Qt3DOffscreenRenderer::DeferredOffscreenEngine::DeferredOffscreenEngine(Qt3DRender::QCamera *camera, const QSize &size)
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
        offscreenFrameGraph = new OffscreenSurfaceDepthFrameGraph(renderSettings, camera, size);

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

    Qt3DOffscreenRenderer::DeferredOffscreenEngine::~DeferredOffscreenEngine()
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

    void Qt3DOffscreenRenderer::DeferredOffscreenEngine::setSceneRoot(Qt3DCore::QNode *sceneRoot)
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

    Qt3DRender::QRenderCapture* Qt3DOffscreenRenderer::DeferredOffscreenEngine::getRenderCapture()
    {
        return renderCapture;
    }

    void Qt3DOffscreenRenderer::DeferredOffscreenEngine::setSize(const QSize &size)
    {
        offscreenFrameGraph->setSize(size);
    }
}
