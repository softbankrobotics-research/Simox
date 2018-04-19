#pragma once

#include "collada.h"
#include <Inventor/nodes/SoSeparator.h>

namespace Collada
{
    struct InventorRobotNode : ColladaRobotNode
    {
        SoSeparator* visualization;
        SoSeparator* collisionModel;
        SoSeparator* root;
        SoGroup* preJointTransformation;
        InventorRobotNode(SoSeparator* root);
        InventorRobotNode();
        ~InventorRobotNode();
        void visualizeBoundingBox();
        void initialize() override;
    private:
        bool m_bOwn;
    };

    // Adds a <instance_geometry_node> directly to a SoSeparator.
    void addGeometry(SoSeparator* separator, const pugi::xml_node& node);

    struct InventorWalker : ColladaWalker
    {
        InventorWalker(StructureMap _structureMap, XmlMap physicsMap, SoSeparator* _root) : ColladaWalker(_structureMap, physicsMap), root(_root) {}
        bool for_each(pugi::xml_node& node) override;

        SoSeparator* root;
        std::vector<SoSeparator*> stack;
        ColladaRobotNodeSet parents;

    };

    /// Only used for stand-alone Inventor viewer (e.g. for use with SoQtExaminarViewer)
    class InventorRobot : public ColladaRobot
    {
    private:
    public:
        InventorRobot(SoSeparator* _root) : root(_root) {}
        ColladaRobotNodePtr robotNodeFactory() override
        {
            return ColladaRobotNodePtr(new InventorRobotNode(root));
        }
        ColladaWalkerPtr visualSceneWalkerFactory(StructureMap structureMap, XmlMap physicsMap) override
        {
            return ColladaWalkerPtr(new InventorWalker(structureMap, physicsMap, root));
        }
        void addCollisionModel(ColladaRobotNodePtr robotNode, pugi::xml_node shapeNode) override;
    protected:
        SoSeparator* root;
        InventorRobot() {};
    };
}//namespace

