/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    VirtualRobot
* @author     Philipp Schmidt
* @copyright  2019 Philipp Schmidt
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_OffscreenRenderEngine_h_
#define _VirtualRobot_OffscreenRenderEngine_h_

#include "OffscreenRenderEngineFrameGraph.h"
#include "OffscreenRenderEngineShaderEffect.h"

#include <VirtualRobot/Visualization/Qt3DVisualization/Qt3DVisualization.h>

#include <QSize>

#include <Qt3DCore/QNode>
#include <Qt3DCore/QEntity>
#include <Qt3DRender/QPointLight>
#include <Qt3DCore/QTransform>
#include <Qt3DRender/QLayer>
#include <Qt3DRender/QCamera>
#include <Qt3DRender/QRenderSettings>
#include <Qt3DRender/QRenderCapture>
#include <Qt3DCore/QAspectEngine>
#include <Qt3DRender/QRenderAspect>
#include <Qt3DLogic/QLogicAspect>

// Offscreen Implementation: credits to https://github.com/florianblume/Qt3D-OffscreenRenderer

// The OffscreenEngine brings together various Qt3D classes that are required in order to
// perform basic scene rendering. Of these, the most important for this project is the OffscreenRenderEngineFrameGraph.
namespace VirtualRobot
{
    class OffscreenRenderEngine
    {
    public:
        OffscreenRenderEngine(const QSize &size);
        ~OffscreenRenderEngine();

        void addSceneContent(const std::vector<VirtualRobot::VisualizationPtr> &scene);
        void clearSceneContent();
        Qt3DRender::QRenderCapture *getRenderCapture();
        void setSize(const QSize &size);

    private:
        void addSceneContent(const VirtualRobot::VisualizationPtr &visualization);
        std::vector<VirtualRobot::Qt3DVisualizationPtr> visualizations;

        // We need all of the following in order to render a scene:
        Qt3DCore::QAspectEngine *aspectEngine;                              // The aspect engine, which holds the scene and related aspects.
        Qt3DRender::QRenderAspect *renderAspect;                            // The render aspect, which deals with rendering the scene.
        Qt3DLogic::QLogicAspect *logicAspect;                               // The logic aspect, which runs jobs to do with synchronising frames.
        Qt3DRender::QRenderSettings *renderSettings;                        // The render settings, which control the general rendering behaviour.
        Qt3DRender::QRenderCapture *renderCapture;                          // The render capture node, which is appended to the frame graph.
        OffscreenRenderEngineFrameGraph *offscreenRenderEngineFrameGraph;   // The frame graph, which allows rendering to an offscreen surface.
        Qt3DCore::QNode *sceneRoot;                                         // The scene root, which becomes a child of the engine's root entity.

        // Internal nodes (root node, camera, light, necessary shaders, layers, etc.)
        Qt3DCore::QEntity *rootEntity;
        Qt3DRender::QCamera *cameraEntity;
        Qt3DCore::QEntity *lightEntity;
        Qt3DRender::QPointLight *light;
        Qt3DCore::QTransform *lightTransform;
        Qt3DRender::QLayer *phongLayer;
        Qt3DRender::QLayer *shaderLayer;
        OffscreenRenderEngineShaderEffect* depthEncodingShader;
    };
}

#endif // _VirtualRobot_OffscreenRenderEngine_h_
