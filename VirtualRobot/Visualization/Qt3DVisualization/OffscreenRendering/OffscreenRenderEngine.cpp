#include "OffscreenRenderEngine.h"

#include <VirtualRobot/Tools/RuntimeEnvironment.h>


#include <Qt3DExtras/QCuboidMesh>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DRender/QMaterial>

VirtualRobot::OffscreenRenderEngine::OffscreenRenderEngine(const QSize &size)
{
    // Set up internal nodes (camera, layers, effects, etc.)
    this->sceneRoot = nullptr;
    this->rootEntity = new Qt3DCore::QEntity();

    // Set up a camera to point at the content of the scene.
    this->cameraEntity = new Qt3DRender::QCamera(this->rootEntity);
    this->cameraEntity->lens()->setPerspectiveProjection(45.0f, static_cast<float>(size.width())/static_cast<float>(size.height()), 10.0f, 100000.0f);
    this->cameraEntity->setPosition(QVector3D(0.0f, 1500.0f, 1200.0f));
    this->cameraEntity->setViewCenter(QVector3D(0.0f, 0.0f, 0.0f));

    // Set up a light to illuminate the shapes.
    this->lightEntity = new Qt3DCore::QEntity(this->rootEntity);
    this->light = new Qt3DRender::QPointLight(this->lightEntity);
    this->light->setColor("white");
    this->light->setIntensity(1);
    this->lightEntity->addComponent(this->light);
    this->lightTransform = new Qt3DCore::QTransform(this->lightEntity);
    this->lightTransform->setTranslation(QVector3D(0.0f, 1500.0f, 1200.0f));
    this->lightEntity->addComponent(this->lightTransform);

    this->phongLayer = new Qt3DRender::QLayer(this->rootEntity);
    this->shaderLayer = new Qt3DRender::QLayer(this->rootEntity);

    std::string gl3VertexShaderLocation("shader/offscreenrenderer/depth_encoder.vert");
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(gl3VertexShaderLocation);
    std::string gl3FragmentShaderLocation("shader/offscreenrenderer/depth_encoder.frag");
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(gl3FragmentShaderLocation);

    this->depthEncodingShader= new OffscreenRenderEngineShaderEffect(QString("depthEncodingShader")
                                                                                            , QUrl("file:" + QString::fromStdString(gl3VertexShaderLocation))
                                                                                            , QUrl("file:" + QString::fromStdString(gl3FragmentShaderLocation))
                                                                                            , this->rootEntity);
    this->lightEntity->addComponent(this->phongLayer);

    {
        Qt3DCore::QEntity *cube = new Qt3DCore::QEntity(rootEntity);

        Qt3DExtras::QCuboidMesh *cubeGeometry = new Qt3DExtras::QCuboidMesh(cube);
        cubeGeometry->setXExtent(500.0f);
        cubeGeometry->setYExtent(500.0f);
        cubeGeometry->setZExtent(500.0f);

        Qt3DCore::QTransform *cubeTransform = new Qt3DCore::QTransform(cube);
        cubeTransform->setTranslation(QVector3D(0.0f, 240.0f, 0.0f));

        Qt3DExtras::QPhongMaterial *cubeMaterial = new Qt3DExtras::QPhongMaterial(cube);

        cube->addComponent(cubeMaterial);
        cube->addComponent(cubeTransform);
        cube->addComponent(cubeGeometry);
        cube->addComponent(phongLayer);
    }

    {
        Qt3DCore::QEntity *cube = new Qt3DCore::QEntity(rootEntity);

        Qt3DExtras::QCuboidMesh *cubeGeometry = new Qt3DExtras::QCuboidMesh(cube);
        cubeGeometry->setXExtent(500.0f);
        cubeGeometry->setYExtent(500.0f);
        cubeGeometry->setZExtent(500.0f);

        Qt3DCore::QTransform *cubeTransform = new Qt3DCore::QTransform(cube);
        cubeTransform->setTranslation(QVector3D(0.0f, 240.0f, 0.0f));

        Qt3DRender::QMaterial *cubeMaterial = new Qt3DRender::QMaterial(cube);
        cubeMaterial->setEffect(depthEncodingShader);

        cube->addComponent(cubeMaterial);
        cube->addComponent(cubeTransform);
        cube->addComponent(cubeGeometry);
        cube->addComponent(shaderLayer);
    }

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
    offscreenRenderEngineFrameGraph = new OffscreenRenderEngineFrameGraph(renderSettings, this->cameraEntity, size);

    // Set this frame graph to be in use.
    renderSettings->setActiveFrameGraph(offscreenRenderEngineFrameGraph);

    // Add a render capture node to the frame graph.
    // This is set as the next child of the render target selector node,
    // so that the capture will be taken from the specified render target
    // once all other rendering operations have taken place.
    renderCapture = new Qt3DRender::QRenderCapture(offscreenRenderEngineFrameGraph->getRenderTargetSelector());

    // Set the root entity of the engine. This causes the engine to begin running.
    aspectEngine->setRootEntity(root);

    offscreenRenderEngineFrameGraph->setViewportLeftLayer(this->phongLayer);
    offscreenRenderEngineFrameGraph->setViewportRightLayer(this->shaderLayer);
    this->setSceneRoot(rootEntity);
}

VirtualRobot::OffscreenRenderEngine::~OffscreenRenderEngine()
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

void VirtualRobot::OffscreenRenderEngine::setSceneRoot(Qt3DCore::QNode *sceneRoot)
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

Qt3DRender::QRenderCapture* VirtualRobot::OffscreenRenderEngine::getRenderCapture()
{
    return renderCapture;
}

void VirtualRobot::OffscreenRenderEngine::setSize(const QSize &size)
{
    offscreenRenderEngineFrameGraph->setSize(size);
}
