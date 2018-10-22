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

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT Qt3DOffscreenRenderer  : public OffscreenRenderer
    {
    public:
        Qt3DOffscreenRenderer() = default;
        virtual ~Qt3DOffscreenRenderer() = default;

        virtual void init(int &/*argc*/, char* /*argv*/[], const std::string &/*appName*/) override;

        virtual bool renderOffscreen(const Eigen::Matrix4f& cameraPose, const std::vector<VisualizationPtr>& scene,
            unsigned short width, unsigned short height,
            bool renderRgbImage, std::vector<unsigned char>& rgbImage,
            bool renderDepthImage, std::vector<float>& depthImage,
            bool renderPointcloud, std::vector<Eigen::Vector3f>& pointCloud,
            float zNear=10.f, float zFar=100000.f, float vertFov = M_PI/4, float nanValue = NAN) const override;

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
    };
}

#endif // _VirtualRobot_Qt3DOffscreenRenderer_h_
