#include "CoinOffscreenRenderer.h"

#include "../CoinVisualization/CoinVisualization.h"

#include <Inventor/SoOffscreenRenderer.h>
#include <Inventor/SbViewportRegion.h>
#include <Inventor/SbMatrix.h>
#include <Inventor/actions/SoGLRenderAction.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/nodes/SoDirectionalLight.h>

#ifdef WIN32
/* gl.h assumes windows.h is already included */
// avoid std::min, std::max errors
#  define NOMINMAX
#  include <winsock2.h>
#  include <windows.h>
#endif
#include <GL/gl.h>

namespace VirtualRobot
{
    void CoinOffscreenRenderer::init(int &, char *[], const std::string &)
    {
    }

    bool CoinOffscreenRenderer::renderOffscreen(const Eigen::Matrix4f &camPose, const std::vector<VirtualRobot::VisualizationPtr> &scene,
                                                unsigned short width, unsigned short height,
                                                bool renderRgbImage, std::vector<unsigned char> &rgbImage,
                                                bool renderDepthImage, std::vector<float> &depthImage,
                                                bool renderPointcloud, std::vector<Eigen::Vector3f> &pointCloud,
                                                float zNear, float zFar, float vertFov, float nanValue) const
    {
        SoOffscreenRenderer offscreenRenderer{SbViewportRegion{static_cast<short>(width), static_cast<short>(height)}};
        offscreenRenderer.setComponents(SoOffscreenRenderer::RGB);
        offscreenRenderer.setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
        //check input
        if (scene.empty())
        {
            VR_ERROR << "No scene to render..." << endl;
            return false;
        }
        //setup
        const bool calculateDepth = renderDepthImage || renderPointcloud;
        const unsigned int numPixel=width*height;
        //required to get the zBuffer
        DepthRenderData userdata;
        std::vector<float> zBuffer;
        if(calculateDepth)
        {
            if(renderDepthImage)
            {
                //we can overwrite the depth image. maybe it already has enough mem
                std::swap(zBuffer,depthImage);
            }
            zBuffer.resize(numPixel);

            userdata.h = height;
            userdata.w = width;
            userdata.buffer = zBuffer.data();

            offscreenRenderer.getGLRenderAction()->setPassCallback(getZBuffer, static_cast<void*>(&userdata));
            ///TODO find way to only render rgb once
            offscreenRenderer.getGLRenderAction()->setNumPasses(2);
        }

        //render
        // add all to a inventor scene graph
        SoSeparator* root = new SoSeparator;
        root->ref();
        root->addChild(new SoDirectionalLight);

        //setup cam
        float sc = 0.001f;//cam has to be in m but values are in mm. (if the cam is set to mm with a unit node the robot visu disapperars)
        SoPerspectiveCamera* camInMeters = new SoPerspectiveCamera;
        Eigen::Vector3f camPos = camPose.block<3, 1>(0, 3);
        camInMeters->position.setValue(camPos[0]*sc, camPos[1]*sc, camPos[2]*sc);
        SbRotation align(SbVec3f(1, 0, 0), (float)(M_PI)); // first align from  default direction -z to +z by rotating with 180 degree around x axis
        SbRotation align2(SbVec3f(0, 0, 1), (float)(-M_PI / 2.0)); // align up vector by rotating with -90 degree around z axis
        SbRotation trans(getSbMatrix(camPose)); // get rotation from global pose
        camInMeters->orientation.setValue(align2 * align * trans); // perform total transformation
        camInMeters->nearDistance.setValue(sc*zNear);
        camInMeters->farDistance.setValue(sc*zFar);
        camInMeters->heightAngle.setValue(vertFov);
        camInMeters->aspectRatio.setValue(static_cast<float>(width)/static_cast<float>(height));

        root->addChild(camInMeters);

        SoSeparator* sceneSep = new SoSeparator;
        root->addChild(sceneSep);
        for (const auto& visu : scene)
        {
            sceneSep->addChild(visualization_cast<CoinElement>(visu)->getMainNode());
        }

        static bool renderErrorPrinted = false;
        const bool renderOk=offscreenRenderer.render(root);


        if (!renderOk)
        {
            if (!renderErrorPrinted)
            {
                VR_ERROR << "Rendering not successful! This error is printed only once." << std::endl;
                renderErrorPrinted = true;
            }
            return false;
        }

        root->unref();
        //rgb
        if(renderRgbImage)
        {
            const unsigned char* glBuffer = offscreenRenderer.getBuffer();
            const unsigned int numValues = numPixel*3;
            rgbImage.resize(numValues);
            std::copy(glBuffer, glBuffer+ numValues, rgbImage.begin());
        }
        //per pixel
        if(!calculateDepth)
        {
            return true;
        }
        if(renderPointcloud)
        {
            pointCloud.resize(numPixel);
        }

        const float focalLength = static_cast<float>(height) / (2 * std::tan(vertFov / 2));

        assert(0<=height);
        for(unsigned int y=0;y<static_cast<std::size_t>(height);++y)
        {
            assert(0<=width);
            for(unsigned int x=0;x<static_cast<std::size_t>(width);++x)
            {
                const unsigned int pixelIndex = x+width*y;
                const float bufferVal = zBuffer.at(pixelIndex);
                /*
                // projection matrix (https://www.opengl.org/sdk/docs/man2/xhtml/glFrustum.xml)
                // [Sx  0 Kx  0]
                // [ 0 Sy Ky  0]
                // [ 0  0  A  B]
                // [ 0  0 -1  0]
                //
                // right = width/2, left = -right;
                // top = height/2, bottom = -top;
                // Sx = (2*zNear)/(right-left) => (2*zNear)/width
                // Sy = (2*zNear)/(top-bottom) => (2*zNear)/height
                // Kx = (right + left)/(right - left) => 0
                // Ky = (top + bottom)/(top - bottom) => 0
                // A  = (zFar + zNear)/(zNear - zFar)
                // B  = (2 * zFar * zNear)/(zNear - zFar)
                //
                // horizontalFOV = 2 * atan(width/(2*focalLength))
                // verticalFOV = 2 * atan(height/(2*focalLength))
                //
                // formulars from:
                // https://www.opengl.org/discussion_boards/showthread.php/197819-simulate-depth-sensor?p=1280162
                */

                assert(bufferVal >= -1e-6);
                assert(bufferVal < 1 + 1e-6);

                //bufferVal = (Zndc+1)/2 => zNDC = 2*bufferVal-1
                assert(std::abs(2 * bufferVal - 1) < 1 + 1e-6);
                /*
                //zNDC = zClip/wClip
                //zClip = zEye*A+B
                //wClip = -zEye
                //=> zNDC = (zEye*A+B)/-zEye = -(A+B/zEye)
                //=> 2*bufferVal-1 = -A -B/zEye
                //=> 2*bufferVal-1 +A = -B/zEye
                //=> zEye = -B/(2*bufferVal-1 +A) = B/(1-A-2*bufferVal)
                // A = (zFar + zNear)/(zNear - zFar);
                // B = (2 * zFar * zNear)/(zNear - zFar);
                // => zEye = (2 * zFar * zNear)/((zNear - zFar)(1-(zFar + zNear)/(zNear - zFar)-2*bufferVal))
                // => zEye = (2 * zFar * zNear)/(zNear - zFar -(zFar+zNear) -2*bufferVal*(zNear-zFar))
                // => zEye = (2 * zFar * zNear)/( - 2*zFar -2*bufferVal*(zNear-zFar))
                // => zEye = -(zFar * zNear)/(zFar +bufferVal*(zNear-zFar))
                */

                const float zEye = -(zFar * zNear) / (zFar + bufferVal * (zNear - zFar));

                assert(zEye < 1e-6);
                assert(std::abs(zEye) < zFar + 1e-6);
                assert(std::abs(zEye) > zNear - 1e-6);

                //the cam is at (x,y)=(0,0) => shift x and y to image center
                const float xShifted = static_cast<float>(x) - static_cast<float>(width ) / 2.f;
                const float yShifted = static_cast<float>(y) - static_cast<float>(height) / 2.f;
                const float xEye = xShifted / focalLength * (zEye);
                const float yEye = yShifted / focalLength * (zEye);

                if(renderDepthImage)
                {
                    if(-zEye < zFar)
                    {
                        zBuffer.at(pixelIndex) = std::sqrt(xEye * xEye + yEye * yEye + zEye * zEye);
                    }
                    else
                    {
                        zBuffer.at(pixelIndex) = nanValue;
                    }
                }

                if(renderPointcloud)
                {
                    //the cam looks along -z => rotate aroud y 180°
                    auto& point = pointCloud.at(pixelIndex);
                    if(-zEye < zFar)
                    {
                        point[0] = -xEye;
                        point[1] =  yEye;
                        point[2] = -zEye;
                    }
                    else
                    {
                        point[0] = nanValue;
                        point[1] = nanValue;
                        point[2] = nanValue;
                    }
                }
            }
        }

        if(renderDepthImage)
        {
            depthImage = std::move(zBuffer);
        }
        return true;
    }

    void CoinOffscreenRenderer::cleanup()
    {
    }

    std::string CoinOffscreenRenderer::getVisualizationType() const
    {
        return "inventor";
    }

    SbMatrix CoinOffscreenRenderer::getSbMatrix(const Eigen::Matrix4f &m) const
    {
        SbMatrix res(reinterpret_cast<const SbMat*>(m.data()));
        return res;
    }

    void CoinOffscreenRenderer::getZBuffer(void *userdata)
    {
        DepthRenderData* ud = reinterpret_cast<DepthRenderData*>(userdata);
        try
        {
            glReadPixels(0, 0, ud->w, ud->h, GL_DEPTH_COMPONENT, GL_FLOAT, ud->buffer);
        }
        catch (...)
        {
            VR_ERROR << "renderOffscreenRgbDepthPointcloud: Error while reading the z-buffer (the read data may be partial)";
        }
    }
}
