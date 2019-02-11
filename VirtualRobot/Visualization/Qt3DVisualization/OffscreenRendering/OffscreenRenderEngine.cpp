#include "OffscreenRenderEngine.h"

#include <VirtualRobot/Tools/RuntimeEnvironment.h>

#include <VirtualRobot/Visualization/VisualizationSet.h>

#include <Qt3DExtras/QCuboidMesh>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DRender/QMaterial>

VirtualRobot::OffscreenRenderEngine::OffscreenRenderEngine(const QSize &size) : visualizations(std::vector<VirtualRobot::Qt3DVisualizationPtr>())
{
    // Set up internal nodes (camera, layers, effects, etc.)
    this->rootEntity = new Qt3DCore::QEntity();
    this->sceneRoot = new Qt3DCore::QNode(rootEntity);

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

    this->rootEntity->setParent(aspectEngine->rootEntity().data());
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

void VirtualRobot::OffscreenRenderEngine::addSceneContent(const std::vector<VirtualRobot::VisualizationPtr> &scene)
{
    for(auto visualization : scene)
    {
        addSceneContent(visualization);
    }

    //Discard next two frames, as they do not contain updated scene content (FUTURE FIX IN QT3D?).
    Qt3DRender::QRenderCaptureReply *firstReply = this->getRenderCapture()->requestCapture();
    Qt3DRender::QRenderCaptureReply *secondReply = this->getRenderCapture()->requestCapture();

    delete firstReply;
    delete secondReply;
}


void VirtualRobot::OffscreenRenderEngine::addSceneContent(const VirtualRobot::VisualizationPtr &visualization)
{
    VirtualRobot::VisualizationSetPtr set = std::dynamic_pointer_cast<VirtualRobot::VisualizationSet>(visualization);
    if (set)
    {
        for (const auto& setVisualization : set->getVisualizations())
        {
            addSceneContent(setVisualization);
        }
    }
    else
    {
        auto castedVisualization = std::static_pointer_cast<Qt3DVisualization>(visualization);

        auto phongVisuClone = std::static_pointer_cast<Qt3DVisualization>(visualization->clone());
        auto shaderVisuClone = std::static_pointer_cast<Qt3DVisualization>(visualization->clone());
        visualizations.push_back(phongVisuClone);
        visualizations.push_back(shaderVisuClone);

        // For phong, add phong layer and we are good to go
        phongVisuClone->getEntity()->addComponent(this->phongLayer);
        phongVisuClone->getEntity()->setParent(this->sceneRoot);

        // For shader, strip all materials and add shader
        shaderVisuClone->getEntity()->addComponent(this->shaderLayer);
        shaderVisuClone->getEntity()->setParent(this->sceneRoot);

        //Remove all materials
        for(auto component : shaderVisuClone->getEntity()->components())
        {
            if (qobject_cast<Qt3DRender::QMaterial *>(component))
            {
                shaderVisuClone->getEntity()->removeComponent(component);
            }
        }

        //Add shader material
        Qt3DRender::QMaterial *shaderMaterial = new Qt3DRender::QMaterial(shaderVisuClone->getEntity());
        shaderMaterial->setEffect(this->depthEncodingShader);
        shaderVisuClone->getEntity()->addComponent(shaderMaterial);
    }
}


void VirtualRobot::OffscreenRenderEngine::clearSceneContent()
{
    // Make sure any existing scene content element is unparented.
    for(auto visualization : visualizations)
    {
        visualization->getEntity()->setParent(static_cast<Qt3DCore::QNode*>(nullptr));
    }

    VR_ASSERT(sceneRoot->childNodes().size() == 0);

    //Discard next two frames, as they do not contain updated scene content (FUTURE FIX IN QT3D?).
    Qt3DRender::QRenderCaptureReply *firstReply = this->getRenderCapture()->requestCapture();
    Qt3DRender::QRenderCaptureReply *secondReply = this->getRenderCapture()->requestCapture();

    delete firstReply;
    delete secondReply;
}

Qt3DRender::QRenderCapture* VirtualRobot::OffscreenRenderEngine::getRenderCapture()
{
    return renderCapture;
}

void VirtualRobot::OffscreenRenderEngine::setSize(const QSize &size)
{
    offscreenRenderEngineFrameGraph->setSize(size);
}
