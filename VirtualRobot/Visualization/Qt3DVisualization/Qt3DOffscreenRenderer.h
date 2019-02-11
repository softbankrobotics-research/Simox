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

#include "OffscreenRendering/OffscreenRenderEngine.h"

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
        OffscreenRenderEngine* engine;
    };
}

#endif // _VirtualRobot_Qt3DOffscreenRenderer_h_
