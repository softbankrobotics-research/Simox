/**
* @package    VirtualRobot
* @author     Peter Kaiser
* @copyright  2018 Peter Kaiser
*/

#include "RobotNodeMetaFactory.h"
#include "RobotNode.h"
#include "RobotNodeMeta.h"


namespace VirtualRobot
{

    RobotNodeMetaFactory::RobotNodeMetaFactory()
    {
    }


    RobotNodeMetaFactory::~RobotNodeMetaFactory()
    {
    }


    /**
     * This method creates a VirtualRobot::RobotNodeMeta.
     *
     * \return instance of VirtualRobot::RobotNodeMeta.
     */
    RobotNodePtr RobotNodeMetaFactory::createRobotNode(RobotPtr robot, const std::string& nodeName, VisualizationNodePtr visualizationModel, CollisionModelPtr collisionModel, float limitLow, float limitHigh, float jointValueOffset, const Eigen::Matrix4f& preJointTransform, const Eigen::Vector3f& /*axis*/, const Eigen::Vector3f& translationDirection, const SceneObject::Physics& p, RobotNode::RobotNodeType rntype) const
    {
        RobotNodePtr robotNode(new RobotNodeMeta(robot, nodeName, limitLow, limitHigh, preJointTransform, visualizationModel, collisionModel, jointValueOffset, p, CollisionCheckerPtr(), rntype));

        return robotNode;
    }


    /**
     * This method creates a VirtualRobot::RobotNodeMeta from DH parameters.
     *
     * \return instance of VirtualRobot::RobotNodeMeta.
     */
    RobotNodePtr RobotNodeMetaFactory::createRobotNodeDH(RobotPtr robot, const std::string& nodeName, VisualizationNodePtr visualizationModel, CollisionModelPtr collisionModel, float limitLow, float limitHigh, float jointValueOffset, const DHParameter& dhParameters, const SceneObject::Physics& p, RobotNode::RobotNodeType rntype) const
    {
        RobotNodePtr robotNode(new RobotNodeMeta(robot, nodeName, limitLow, limitHigh, dhParameters.aMM(), dhParameters.dMM(), dhParameters.alphaRadian(), dhParameters.thetaRadian(), visualizationModel, collisionModel, jointValueOffset, p, CollisionCheckerPtr(), rntype));

        return robotNode;
    }


    /**
     * register this class in the super class factory
     */
    RobotNodeFactory::SubClassRegistry RobotNodeMetaFactory::registry(RobotNodeMetaFactory::getName(), &RobotNodeMetaFactory::createInstance);


    /**
     * \return "meta"
     */
    std::string RobotNodeMetaFactory::getName()
    {
        return "meta";
    }


    /**
     * \return new instance of RobotNodeMetaFactory.
     */
    boost::shared_ptr<RobotNodeFactory> RobotNodeMetaFactory::createInstance(void*)
    {
        boost::shared_ptr<RobotNodeMetaFactory> metaNodeFactory(new RobotNodeMetaFactory());
        return metaNodeFactory;
    }

} // namespace VirtualRobot
