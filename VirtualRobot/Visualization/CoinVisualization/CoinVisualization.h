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
* @author     Manfred Kroehnert, Nikolaus Vahrenkamp, Adrian Knobloch
* @copyright  2010, 2011, 2017 Manfred Kroehnert, Nikolaus Vahrenkamp, ADrian Knobloch
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_CoinVisualization_h_
#define _VirtualRobot_CoinVisualization_h_

#include "../../Model/Model.h"
#include "../Visualization.h"
#include "CoinElement.h"

class SoNode;
class SoGroup;
class SoDrawStyle;
class SoTransform;
class SoMaterial;
class SoSeparator;
class SoScale;
class SoCallbackAction;
class SoPrimitiveVertex;

namespace VirtualRobot
{
    class TriMeshModel;

    class VIRTUAL_ROBOT_IMPORT_EXPORT CoinVisualization : public Visualization, public CoinElement
    {
        friend class CoinVisualizationFactory;
        friend class CoinVisualizationSet;
    protected:
        CoinVisualization(SoNode* visuNode);
    public:
        virtual ~CoinVisualization();

        virtual SoNode* getMainNode() const override;
    protected:
        /*!
            Replace current visualization of this node.
            Be careful: any former grabbed trimeshmodels do no longer represent the new datastructure!
        */
        virtual void setVisualization(SoNode* newVisu);
        SoNode* getCoinVisualization() const;

    public:
        virtual void setGlobalPose(const Eigen::Matrix4f &m) override;
        //virtual void applyDisplacement(const Eigen::Matrix4f &dp) override;
        virtual size_t addPoseChangedCallback(std::function<void (const Eigen::Matrix4f &)> f) override;
        virtual void removePoseChangedCallback(size_t id) override;

        virtual void setVisible(bool showVisualization) override;
        virtual bool isVisible() const override;

        virtual void setUpdateVisualization(bool enable) override;
        virtual bool getUpdateVisualizationStatus() const override;

        virtual void setStyle(DrawStyle s) override;
        virtual DrawStyle getStyle() const override;

        // needed to only override transparency
        // (void)coin_setenv("COIN_SEPARATE_DIFFUSE_TRANSPARENCY_OVERRIDE", "1", TRUE);
        virtual void setColor(const Color &c) override;
        virtual Color getColor() const override;

        virtual void setMaterial(const MaterialPtr &material) override;
        virtual MaterialPtr getMaterial() const override;

        virtual void setSelected(bool selected) override;
        virtual bool isSelected() const override;
        virtual size_t addSelectionChangedCallback(std::function<void (bool)> f) override;
        virtual void removeSelectionChangedCallback(size_t id) override;

        virtual void scale(const Eigen::Vector3f &s) override;
        Eigen::Vector3f getScalingFactor() const;

        virtual void shrinkFatten(float offset) override;

    protected:
        virtual void _addManipulator(ManipulatorType t) override;
        virtual void _removeManipulator(ManipulatorType t) override;
        virtual void _removeAllManipulators() override;
    public:
        virtual bool hasManipulator(ManipulatorType t) const override;
        virtual std::vector<ManipulatorType> getAddedManipulatorTypes() const override;

        virtual std::vector<Primitive::PrimitivePtr> getPrimitives() const override;

        virtual void setFilename(const std::string &filename, bool boundingBox) override;
        virtual std::string getFilename() const override;
        virtual bool usedBoundingBoxVisu() const override;

        virtual BoundingBox getBoundingBox() const override;

        virtual TriMeshModelPtr getTriMeshModel() const override;

        virtual int getNumFaces() const override;

        virtual VisualizationPtr clone() const override;

        virtual void print() const override;

        virtual std::string toXML(const std::string &basePath, int tabs) const override;
        virtual std::string toXML(const std::string &basePath, const std::string &filename, int tabs) const override;

        virtual bool saveModel(const std::string &modelPath, const std::string &filename) override;

    protected:
        /**
         * This method constructs an instance of TriMeshModel and stores it in
         * CoinVisualizationNode::triMeshModel.
         * If CoinVisualizationMode::visualization is invalid VirtualRobotException
         * is thrown.
         * Otherwise CoinVisualizationNode::InventorTriangleCB() is called on the
         * Inventor graph stored in CoinVisualizationNode::visualization.
         */
        void createTriMeshModel();
        //virtual void setIsInVisualizationSet(bool inSet) override;

        static void InventorTriangleCB(void* data, SoCallbackAction* action,
                                       const SoPrimitiveVertex* v1,
                                       const SoPrimitiveVertex* v2,
                                       const SoPrimitiveVertex* v3);
        static SoGroup* convertSoFileChildren(SoGroup* orig);
        static SoNode* copyNode(SoNode* n);

        SoSeparator* mainNode;
        SoTransform* transformNode;
        SoScale* scaleNode;
        SoMaterial* materialNode;
        SoNode* materialNodeNone;
        SoDrawStyle* drawStyleNode;
        SoNode* visualizationNode;

        bool updateVisualization;
        DrawStyle style;
        MaterialPtr material;
        std::string filename; //!< if the visualization was build from a file, the filename is stored here
        bool boundingBox; //!< Indicates, if the bounding box model was used
        std::vector<Primitive::PrimitivePtr> primitives;
        TriMeshModelPtr triMeshModel;
        std::map<unsigned int, std::function<void(const Eigen::Matrix4f&)>> poseChangedCallbacks;
    };

    typedef std::shared_ptr<CoinVisualization> CoinVisualizationPtr;

} // namespace VirtualRobot

#endif // _VirtualRobot_CoinVisualization_h_
