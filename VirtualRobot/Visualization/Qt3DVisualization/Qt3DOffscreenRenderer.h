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
* @copyright  2018 Philipp Schmidt
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_Qt3DOffscreenRenderer_h_
#define _VirtualRobot_Qt3DOffscreenRenderer_h_

#include "../OffscreenRenderer.h"


#include <Qt3DRender/QRenderTarget>
#include <Qt3DRender/QRenderTargetOutput>
#include <Qt3DRender/QTexture>
#include <QOffscreenSurface>
#include <Qt3DRender/QRenderSurfaceSelector>
#include <Qt3DRender/QRenderTargetSelector>
#include <Qt3DRender/QRenderPassFilter>
#include <Qt3DRender/QEffect>
#include <Qt3DRender/QTechnique>
#include <Qt3DRender/QLayerFilter>
#include <Qt3DRender/QViewport>
#include <Qt3DRender/QClearBuffers>
#include <Qt3DRender/QCameraSelector>
#include <Qt3DRender/QCamera>
#include <Qt3DCore/QEntity>
#include <Qt3DCore/QNode>
#include <Qt3DRender/QRenderSettings>
#include <Qt3DRender/QRenderCapture>
#include <Qt3DCore/QAspectEngine>
#include <Qt3DRender/QRenderAspect>
#include <Qt3DLogic/QLogicAspect>


namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT Qt3DOffscreenRenderer  : public OffscreenRenderer
    {
    public:
        Qt3DOffscreenRenderer();
        virtual ~Qt3DOffscreenRenderer();
        virtual void init(int &/*argc*/, char* /*argv*/[], const std::string &/*appName*/) override;

        virtual bool renderOffscreen(const Eigen::Matrix4f& cameraPose, const std::vector<VisualizationPtr>& scene,
            unsigned short width, unsigned short height,
            bool renderRgbImage, std::vector<unsigned char>& rgbImage,
            bool renderDepthImage, std::vector<float>& depthImage,
            bool renderPointcloud, std::vector<Eigen::Vector3f>& pointCloud,
            float zNear=10.f, float zFar=100000.f, float vertFov = M_PI/4, float nanValue = NAN,
            VirtualRobot::Visualization::Color backgroundColor = VirtualRobot::Visualization::Color::None()) const override;

        /*!
        Here, a manual cleanup can be called, visualization engine access may not be possible after calling this method.
        Usually no need to call cleanup explicitly, since cleanup is performed automatically at application exit.
        */
        virtual void cleanup() override;

        /**
         * A dynamicly bound version of getName().
         * @return the visualization type that is supported by this factory.
         */
        virtual std::string getVisualizationType() const override;

    private:
        class TextureRenderTarget : public Qt3DRender::QRenderTarget
        {
        public:
            TextureRenderTarget(Qt3DCore::QNode *parent = nullptr,
                                const QSize &size = QSize(500, 500),
                                Qt3DRender::QRenderTargetOutput::AttachmentPoint attatchmentPoint
                                                        = Qt3DRender::QRenderTargetOutput::Color0);

            void setSize(const QSize &size);
            QSize getSize() { return size; }
            Qt3DRender::QTexture2D* getTexture();

        private:
            QSize size;
            Qt3DRender::QRenderTargetOutput *output;
            Qt3DRender::QTexture2D *texture;
            // To enable depth testing
            Qt3DRender::QRenderTargetOutput *depthTextureOutput;
            Qt3DRender::QTexture2D *depthTexture;
        };

        class OffscreenSurfaceFrameGraph : public Qt3DRender::QRenderSurfaceSelector
        {
        public:
            OffscreenSurfaceFrameGraph(Qt3DCore::QNode* parent = nullptr, Qt3DRender::QCamera *camera = nullptr, const QSize &size = QSize(500, 500));

            void setSize(const QSize &size);
            Qt3DCore::QNode *getRenderTargetSelector();

        private:
            TextureRenderTarget *textureTarget;
            QOffscreenSurface *offscreenSurface;
            Qt3DRender::QRenderTargetSelector *renderTargetSelector;
            Qt3DRender::QViewport *viewport;
            Qt3DRender::QClearBuffers *clearBuffers;
            Qt3DRender::QCameraSelector *cameraSelector;
            Qt3DRender::QCamera *camera;
        };

        class GBuffer : public Qt3DRender::QRenderTarget
        {
        public:
            explicit GBuffer(Qt3DCore::QNode *parent = 0);

            enum Attachments {
                Color = 0,
                Position,
                Normal,
                Depth,
                AttachmentsCount
            };

            Qt3DRender::QAbstractTexture *colorTexture() const;
            Qt3DRender::QAbstractTexture *positionTexture() const;
            Qt3DRender::QAbstractTexture *normalTexture() const;
            Qt3DRender::QAbstractTexture *depthTexture() const;

        private:
            Qt3DRender::QAbstractTexture *m_textures[AttachmentsCount];
        };

        class FinalShaderEffect : public Qt3DRender::QEffect
        {
        public:
            explicit FinalShaderEffect(Qt3DCore::QNode *parent = 0);

            QList<Qt3DRender::QFilterKey *> passCriteria() const;
            inline Qt3DRender::QTechnique *gl3Technique() const { return m_gl3Technique; }
            inline Qt3DRender::QTechnique *gl2Technique() const { return m_gl2Technique; }

        private :
            Qt3DRender::QTechnique *m_gl3Technique;
            Qt3DRender::QTechnique *m_gl2Technique;
            Qt3DRender::QRenderPass *m_gl2Pass;
            Qt3DRender::QRenderPass *m_gl3Pass;
            Qt3DRender::QFilterKey *m_passCriterion;
        };

        class GBufferShaderEffect : public Qt3DRender::QEffect
        {
        public:
            explicit GBufferShaderEffect(Qt3DCore::QNode *parent = 0);

            QList<Qt3DRender::QFilterKey *> passCriteria() const;

        private:
            Qt3DRender::QTechnique *m_gl3Technique;
            Qt3DRender::QTechnique *m_gl2Technique;
            Qt3DRender::QRenderPass *m_gl2Pass;
            Qt3DRender::QRenderPass *m_gl3Pass;
            Qt3DRender::QFilterKey *m_passCriterion;
        };

        class OffscreenSurfaceDepthFrameGraph : public Qt3DRender::QRenderSurfaceSelector
        {
        public:
            OffscreenSurfaceDepthFrameGraph(Qt3DCore::QNode* parent = nullptr, Qt3DRender::QCamera *camera = nullptr, const QSize &size = QSize(500, 500));

            void setSize(const QSize &size);
            Qt3DCore::QNode *getRenderTargetSelector();

        private:
            Qt3DRender::QViewport *viewport;
            Qt3DRender::QLayerFilter *sceneFilter;
            Qt3DRender::QRenderTargetSelector *gBufferTargetSelector;
            Qt3DRender::QClearBuffers *clearGBuffer;
            Qt3DRender::QRenderPassFilter *geometryPassFilter;
            Qt3DRender::QCameraSelector *sceneCameraSelector;
            Qt3DRender::QLayerFilter *screenQuadFilter;
            Qt3DRender::QClearBuffers *clearScreenQuad;
            Qt3DRender::QRenderPassFilter *finalPassFilter;
            GBuffer *gBuffer;
            Qt3DRender::QParameter *sizeParameter;

            TextureRenderTarget *textureTarget;
            QOffscreenSurface *offscreenSurface;
            Qt3DRender::QRenderTargetSelector *renderTargetSelector;
        };

        class OffscreenEngine
        {
        public:
            OffscreenEngine(Qt3DRender::QCamera *camera, const QSize &size);
            ~OffscreenEngine();

            void setSceneRoot(Qt3DCore::QNode *sceneRoot);
            Qt3DRender::QRenderCapture *getRenderCapture();
            void setSize(const QSize &size);

        private:
            // We need all of the following in order to render a scene:
            Qt3DCore::QAspectEngine *aspectEngine;              // The aspect engine, which holds the scene and related aspects.
            Qt3DRender::QRenderAspect *renderAspect;            // The render aspect, which deals with rendering the scene.
            Qt3DLogic::QLogicAspect *logicAspect;               // The logic aspect, which runs jobs to do with synchronising frames.
            Qt3DRender::QRenderSettings *renderSettings;        // The render settings, which control the general rendering behaviour.
            Qt3DRender::QRenderCapture *renderCapture;          // The render capture node, which is appended to the frame graph.
            OffscreenSurfaceFrameGraph *offscreenFrameGraph;    // The frame graph, which allows rendering to an offscreen surface.
            Qt3DCore::QNode *sceneRoot;                         // The scene root, which becomes a child of the engine's root entity.
        };

        class DeferredOffscreenEngine
        {
        public:
            DeferredOffscreenEngine(Qt3DRender::QCamera *camera, const QSize &size);
            ~DeferredOffscreenEngine();

            void setSceneRoot(Qt3DCore::QNode *sceneRoot);
            Qt3DRender::QRenderCapture *getRenderCapture();
            void setSize(const QSize &size);

        private:
            // We need all of the following in order to render a scene:
            Qt3DCore::QAspectEngine *aspectEngine;              // The aspect engine, which holds the scene and related aspects.
            Qt3DRender::QRenderAspect *renderAspect;            // The render aspect, which deals with rendering the scene.
            Qt3DLogic::QLogicAspect *logicAspect;               // The logic aspect, which runs jobs to do with synchronising frames.
            Qt3DRender::QRenderSettings *renderSettings;        // The render settings, which control the general rendering behaviour.
            Qt3DRender::QRenderCapture *renderCapture;          // The render capture node, which is appended to the frame graph.
            OffscreenSurfaceDepthFrameGraph *offscreenFrameGraph;    // The frame graph, which allows rendering to an offscreen surface.
            Qt3DCore::QNode *sceneRoot;                         // The scene root, which becomes a child of the engine's root entity.
        };

        Qt3DCore::QEntity *rootEntity;
        Qt3DRender::QCamera *cameraEntity;
        OffscreenEngine *offscreenEngine;
        DeferredOffscreenEngine *deferredOffscreenEngine;

        Qt3DRender::QRenderCaptureReply *render() const;
        void addToSceneGraph(const VisualizationPtr &visu) const;
    };
}

#endif // _VirtualRobot_Qt3DOffscreenRenderer_h_
