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
#ifndef _VirtualRobot_OffscreenRenderEngineFrameGraph_h_
#define _VirtualRobot_OffscreenRenderEngineFrameGraph_h_

#include <QOffscreenSurface>
#include <Qt3DRender/QRenderSurfaceSelector>
#include <Qt3DRender/QRenderTargetSelector>
#include <Qt3DRender/QViewport>
#include <Qt3DRender/QClearBuffers>
#include <Qt3DRender/QCameraSelector>
#include <Qt3DRender/QCamera>
#include <Qt3DRender/QLayerFilter>

#include <Qt3DRender/QRenderTarget>
#include <Qt3DRender/QRenderTargetOutput>
#include <Qt3DRender/QTexture>

// Offscreen Implementation: credits to https://github.com/florianblume/Qt3D-OffscreenRenderer

// The OffscreenSurfaceFrameGraph class is where the magic happens.
// It is responsible for rendering the scene to an "offscreen" surface (ie. a texture),
// rather than directly to a QWindow. This means that the render contents can be
// taken and used within other QWidgets.
namespace VirtualRobot
{
    class OffscreenRenderEngineFrameGraph : public Qt3DRender::QRenderSurfaceSelector
    {
    public:
        OffscreenRenderEngineFrameGraph(Qt3DCore::QNode* parent = nullptr, Qt3DRender::QCamera *camera = nullptr, const QSize &size = QSize(500, 500));

        void setSize(const QSize &size);
        Qt3DCore::QNode *getRenderTargetSelector();

        void setViewportLeftLayer(Qt3DRender::QLayer *layer);
        void setViewportRightLayer(Qt3DRender::QLayer *layer);

    private:
        // Encapsulates a 2D texture that a frame graph can render into.
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

    private:
        TextureRenderTarget *textureTarget;
        QOffscreenSurface *offscreenSurface;
        Qt3DRender::QRenderTargetSelector *renderTargetSelector;
        Qt3DRender::QViewport *viewportLeft;
        Qt3DRender::QLayerFilter *viewportLeftLayer;
        Qt3DRender::QViewport *viewportRight;
        Qt3DRender::QLayerFilter *viewportRightLayer;
        Qt3DRender::QClearBuffers *clearBuffers;
        Qt3DRender::QCameraSelector *cameraSelector;
        Qt3DRender::QCamera *camera;
    };
}

#endif // _VirtualRobot_OffscreenRenderEngineFrameGraph_h_
