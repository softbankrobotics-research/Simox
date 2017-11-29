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
* @author     Adrian Knobloch
* @copyright  2017 Adrian Knobloch
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_CoinOffscreenRenderer_h_
#define _VirtualRobot_CoinOffscreenRenderer_h_

#include "../OffscreenRenderer.h"

#include <Eigen/Core>
#include <string>

class SbMatrix;
namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT CoinOffscreenRenderer  : public OffscreenRenderer
    {
    public:
        CoinOffscreenRenderer() = default;
        virtual ~CoinOffscreenRenderer() = default;

        virtual void init(int &/*argc*/, char* /*argv*/[], const std::string &/*appName*/) override;

        /*!
         * \brief Renders the given scene from the given cam position and outputs (optionally) the rgb image, depth image and point cloud.
         * \param cameraPose The Pose of the camera.
         * \param scene All visualizations to render into the images.
         * \param width The used image width. (>0)
         * \param height The used image height. (>0)
         * \param renderRgbImage Whether to output the rgb image.
         * \param rgbImage The rgb image's output parameter.
         * \param renderDepthImgae Whether to output the depth image.
         * \param depthImage The depth image's output parameter.
         * \param renderPointcloud Whether to output the point cloud.
         * \param pointCloud The pointcloud's output parameter.
         * \param zNear The near plane's distance.
         * \param zFar The far plane's distance
         * \param vertFov The fov in rad. (vertical)
         * \param nanValue All values above the zFar value will be mapped on this value (usually nan or 0)
         * \return true on success
         */
        virtual bool renderOffscreen
            (
                const Eigen::Matrix4f& cameraPose, const std::vector<VisualizationPtr>& scene,
                unsigned short width, unsigned short height,
                bool renderRgbImage, std::vector<unsigned char>& rgbImage,
                bool renderDepthImage, std::vector<float>& depthImage,
                bool renderPointcloud, std::vector<Eigen::Vector3f>& pointCloud,
                float zNear=10.f, float zFar=100000.f, float vertFov = M_PI/4, float nanValue = NAN
                ) const override;

        //virtual bool renderOffscreenRgbImage
        //    (
        //        const Eigen::Matrix4f& cameraPose, const std::vector<VisualizationPtr>& scene,
        //        unsigned short width, unsigned short height,
        //        std::vector<unsigned char>& rgbImage,
        //        float zNear=10.f, float zFar=100000.f, float vertFov = M_PI/4, float nanValue = NAN
        //        ) const;
        //virtual bool renderOffscreenDepthImage
        //    (
        //        const Eigen::Matrix4f& cameraPose, const std::vector<VisualizationPtr>& scene,
        //        unsigned short width, unsigned short height,
        //        std::vector<float>& depthImage,
        //        float zNear=10.f, float zFar=100000.f, float vertFov = M_PI/4, float nanValue = NAN
        //        ) const;
        //virtual bool renderOffscreenPointCloud
        //    (
        //        const Eigen::Matrix4f& cameraPose, const std::vector<VisualizationPtr>& scene,
        //        unsigned short width, unsigned short height,
        //        std::vector<Eigen::Vector3f>& pointCloud,
        //        float zNear=10.f, float zFar=100000.f, float vertFov = M_PI/4, float nanValue = NAN
        //        ) const;

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

    protected:
        SbMatrix getSbMatrix(const Eigen::Matrix4f& m) const;

    private:
        /**
         * @brief Used for a Coin3d callback to store the zBuffer.
         * @see renderOffscreen
         */
        struct DepthRenderData
        {
            float* buffer;
            unsigned int w;
            unsigned int h;
        };
        static void getZBuffer(void* userdata);
    };
} // namespace VirtualRobot

#endif // _VirtualRobot_CoinOffscreenRenderer_h_
