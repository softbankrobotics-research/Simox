/*!
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
* @author     Manfred Kroehnert, Nikolaus Vahrenkamp, Adrian Knobloch
* @copyright  2010, 2011, 2017 Manfred Kroehnert, Nikolaus Vahrenkamp, Adrian Knobloch
*             GNU Lesser General Public License
*
*/
#pragma once

#include "../VisualizationFactory.h"
#include "CoinVisualization.h"
#include "CoinVisualizationSet.h"

class SoInput;
namespace VirtualRobot
{
    /*!
        A Coin3D based implementation of a VisualizationFactory.
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT CoinVisualizationFactory  : public VisualizationFactory
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        CoinVisualizationFactory() = default;
        virtual ~CoinVisualizationFactory() = default;

        /*!
            Initialises SoDB and SoQt.
            Sets the COIN_SEPARATE_DIFFUSE_TRANSPARENCY_OVERRIDE environment variable to enable a Coin3D transparency extension.
        */
        virtual void init(int &argc, char* argv[], const std::string &appName) override;

        //virtual VisualizationPtr createVisualizationFromPrimitives(const std::vector<Primitive::PrimitivePtr> &primitives, bool boundingBox) const override;
        virtual VisualizationPtr createVisualizationFromFile(const std::string& filename, bool boundingBox) const override;
        virtual VisualizationPtr createVisualizationFromFile(const std::ifstream &ifs, bool boundingBox) const override;
    protected:
        VisualizationPtr createVisualizationFromCoin3DFile(const std::string& filename, bool boundingBox) const;

        VisualizationPtr createVisualizationFromSTLFile(const std::string& filename, bool boundingBox) const;
        VisualizationPtr createVisualizationFromSoInput(SoInput& soInput, bool boundingBox, bool freeDuplicateTextures = true) const;
    public:
        virtual VisualizationSetPtr createVisualisationSet(const std::vector<VisualizationPtr> &visualizations = std::vector<VisualizationPtr>()) const override;
        virtual SelectionGroupPtr createSelectionGroup() const override;
        virtual VisualizationPtr createBox(float width, float height, float depth) const override;
        virtual VisualizationPtr createLine(const Eigen::Vector3f &from, const Eigen::Vector3f &to, float width) const override;
        virtual VisualizationPtr createLine(const Eigen::Matrix4f &from, const Eigen::Matrix4f &to, float width) const override;
        //virtual VisualizationSetPtr createLineSet(const std::vector<Eigen::Vector3f> &from, const std::vector<Eigen::Vector3f> &to, float width) const override;
        //virtual VisualizationSetPtr createLineSet(const std::vector<Eigen::Matrix4f> &from, const std::vector<Eigen::Matrix4f> &to, float width) const override;
        virtual VisualizationPtr createSphere(float radius) const override;
        virtual VisualizationPtr createCircle(float radius, float circleCompletion, float width, size_t numberOfCircleParts) const override;
        virtual VisualizationPtr createTorus(float radius, float tubeRadius, float completion = 1.0f, int sides = 8, int rings = 30) const override;
        virtual VisualizationPtr createCircleArrow(float radius, float tubeRadius, float completion, int sides, int rings) const override;
        virtual VisualizationPtr createCylinder(float radius, float height) const override;
        //virtual VisualizationPtr createCoordSystem(std::string *text, float axisLength, float axisSize, int nrOfBlocks) const override;
        virtual VisualizationPtr createPoint(float radius) const override;
        //virtual VisualizationSetPtr createPointCloud(const std::vector<Eigen::Matrix4f> & points, float radius) const override;
        //virtual VisualizationSetPtr createPointCloud(const std::vector<Eigen::Vector3f> & points, float radius) const override;
        virtual VisualizationPtr createTriMeshModel(const TriMeshModelPtr &model) const override;
        static SoNode* createTriMeshModelCoin(const TriMeshModelPtr &model);
        virtual VisualizationPtr createPolygon(const std::vector<Eigen::Vector3f> &points) const override;
        virtual VisualizationPtr createPlane(const Eigen::Vector3f &point, const Eigen::Vector3f &normal, float extend, const std::string& texture) const override;
        static VisualizationPtr createTexturedPlane(float extend, const std::string& texture);
        virtual VisualizationPtr createArrow(const Eigen::Vector3f &n, float length, float width) const override;
        virtual VisualizationPtr createText(const std::string &text, bool billboard, float offsetX, float offsetY, float offsetZ) const override;
        virtual VisualizationPtr createCone(float baseRadius, float height) const override;
        virtual VisualizationPtr createEllipse(float x, float y, float z) const override;
        virtual VisualizationPtr createContactVisualization(const EndEffector::ContactInfoVector &contacts, float frictionConeHeight, float frictionConeRadius, bool scaleAccordingToApproachDir) const override;
        virtual VisualizationPtr createVisualization() const override;

        /*!
            Here, a manual cleanup can be called, no Coin3D access possible after this.
            Usually no need to call cleanup explicitly, since cleanup is performed automatically at application exit.
        */
        virtual void cleanup() override;

        virtual std::string getVisualizationType() const override;

    protected:
        static inline char IVToolsHelper_ReplaceSpaceWithUnderscore(char input);
        static void RemoveDuplicateTextures(SoNode* node, const std::string &currentPath);

        // AbstractFactoryMethod
    public:
        static std::string getName();

        typedef std::map<std::pair<size_t, std::string>, void*> TextureCacheMap;

    private:
        static VisualizationPtr createVisualizationFromSoNode(SoNode* node);
        static std::mutex globalTextureCacheMutex;
        static TextureCacheMap globalTextureCache;
    };

    typedef std::shared_ptr<CoinVisualizationFactory> CoinVisualizationFactoryPtr;


} // namespace VirtualRobot

