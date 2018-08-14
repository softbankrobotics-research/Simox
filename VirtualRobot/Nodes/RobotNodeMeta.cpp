
#include "RobotNodeMeta.h"
#include "../Robot.h"
#include <cmath>
#include <algorithm>

#include <Eigen/Geometry>
#include "../VirtualRobotException.h"

namespace VirtualRobot
{

    RobotNodeMeta::RobotNodeMeta(RobotWeakPtr rob,
                                           const std::string& name,
                                           float jointLimitLo,
                                           float jointLimitHi,
                                           const Eigen::Matrix4f& preJointTransform,
                                           VisualizationNodePtr visualization,
                                           CollisionModelPtr collisionModel,
                                           float jointValueOffset,
                                           const SceneObject::Physics& p,
                                           CollisionCheckerPtr colChecker,
                                           RobotNodeType type
                                          ) : RobotNode(rob, name, jointLimitLo, jointLimitHi, visualization, collisionModel, jointValueOffset, p, colChecker, type)
    {
        initialized = false;
        visuScaling = false;
        visuScaleFactor << 1.0f, 1.0f, 1.0f;
        optionalDHParameter.isSet = false;
        this->localTransformation = preJointTransform;
        checkValidRobotNodeType();
    }

    RobotNodeMeta::RobotNodeMeta(RobotWeakPtr rob,
                                           const std::string& name,
                                           float jointLimitLo,
                                           float jointLimitHi,
                                           float a, float d, float alpha, float theta,
                                           VisualizationNodePtr visualization,
                                           CollisionModelPtr collisionModel,
                                           float jointValueOffset,
                                           const SceneObject::Physics& p,
                                           CollisionCheckerPtr colChecker,
                                           RobotNodeType type
                                          ) : RobotNode(rob, name, jointLimitLo, jointLimitHi, visualization, collisionModel, jointValueOffset, p, colChecker, type)
    {
        initialized = false;
        optionalDHParameter.isSet = true;
        optionalDHParameter.setAInMM(a);
        optionalDHParameter.setDInMM(d);
        optionalDHParameter.setAlphaRadian(alpha, true);
        optionalDHParameter.setThetaRadian(theta, true);

        // compute DH transformation matrices
        Eigen::Matrix4f RotTheta = Eigen::Matrix4f::Identity();
        RotTheta(0, 0) = cos(theta);
        RotTheta(0, 1) = -sin(theta);
        RotTheta(1, 0) = sin(theta);
        RotTheta(1, 1) = cos(theta);
        Eigen::Matrix4f TransD = Eigen::Matrix4f::Identity();
        TransD(2, 3) = d;
        Eigen::Matrix4f TransA = Eigen::Matrix4f::Identity();
        TransA(0, 3) = a;
        Eigen::Matrix4f RotAlpha = Eigen::Matrix4f::Identity();
        RotAlpha(1, 1) = cos(alpha);
        RotAlpha(1, 2) = -sin(alpha);
        RotAlpha(2, 1) = sin(alpha);
        RotAlpha(2, 2) = cos(alpha);

        // fixed rotation around theta
        this->localTransformation = RotTheta * TransD * TransA * RotAlpha;

        checkValidRobotNodeType();
    }

    RobotNodeMeta::~RobotNodeMeta()
    {
    }


    bool RobotNodeMeta::initialize(SceneObjectPtr parent, const std::vector<SceneObjectPtr>& children)
    {
        return RobotNode::initialize(parent, children) && compileDependencyFunctions();
    }

    void RobotNodeMeta::setDependencies(const std::map<std::string, std::string> dependency_functions)
    {
        dependencyFunctions = dependency_functions;
        if (initialized)
        {
            compileDependencyFunctions();
        }
    }

    void RobotNodeMeta::print(bool printChildren, bool printDecoration) const
    {
        ReadLockPtr lock = getRobot()->getReadLock();

        if (printDecoration)
        {
            cout << "******** RobotNodeMeta ********" << endl;
        }

        RobotNode::print(false, false);

        cout << "* VisuScaling: ";

        if (visuScaling)
        {
            cout << visuScaleFactor[0] << ", " << visuScaleFactor[1] << "," << visuScaleFactor[2] << endl;
        }
        else
        {
            cout << "disabled" << endl;
        }

        if (printDecoration)
        {
            cout << "******** End RobotNodeMeta ********" << endl;
        }


        std::vector< SceneObjectPtr > children = this->getChildren();

        if (printChildren)
        {
            std::for_each(children.begin(), children.end(), boost::bind(&SceneObject::print, _1, true, true));
        }
    }

    bool RobotNodeMeta::isMetaJoint() const
    {
        return true;
    }

    void RobotNodeMeta::setJointValueNoUpdate(float q)
    {
        // Set own joint value
        RobotNode::setJointValueNoUpdate(q);

        dependencySymbolTable.get_variable("x")->ref() = q;

        // Set dependent joint values
        for (auto& dep : compiledDependencyFunctions)
        {
            dep.first->setJointValueNoUpdate(dep.second.value());
        }
    }

    RobotNodePtr RobotNodeMeta::_clone(const RobotPtr newRobot, const VisualizationNodePtr visualizationModel, const CollisionModelPtr collisionModel, CollisionCheckerPtr colChecker, float scaling)
    {
        boost::shared_ptr<RobotNodeMeta> result;
        ReadLockPtr lock = getRobot()->getReadLock();
        Physics p = physics.scale(scaling);

        if (optionalDHParameter.isSet)
        {
            result.reset(new RobotNodeMeta(newRobot, name, jointLimitLo, jointLimitHi, optionalDHParameter.aMM()*scaling, optionalDHParameter.dMM()*scaling, optionalDHParameter.alphaRadian(), optionalDHParameter.thetaRadian(), visualizationModel, collisionModel, jointValueOffset, p, colChecker, nodeType));
        }
        else
        {
            Eigen::Matrix4f lt = getLocalTransformation();
            lt.block(0, 3, 3, 1) *= scaling;
            result.reset(new RobotNodeMeta(newRobot, name, jointLimitLo, jointLimitHi, lt, visualizationModel, collisionModel, jointValueOffset, p, colChecker, nodeType));
        }

        result->setDependencies(dependencyFunctions);
        return result;
    }


    void RobotNodeMeta::checkValidRobotNodeType()
    {
        RobotNode::checkValidRobotNodeType();
        THROW_VR_EXCEPTION_IF(nodeType == Body || nodeType == Transform, "RobotNodeMeta must be a JointNode or a GenericNode");
    }

    std::string RobotNodeMeta::_toXML(const std::string& /*modelPath*/)
    {
#if 0
        std::stringstream ss;
        ss << "\t\t<Joint type='prismatic'>" << endl;
        ss << "\t\t\t<limits lo='" << jointLimitLo << "' hi='" << jointLimitHi << "' units='mm'/>" << endl;
        ss << "\t\t\t<MaxAcceleration value='" << maxAcceleration << "'/>" << endl;
        ss << "\t\t\t<MaxVelocity value='" << maxVelocity << "'/>" << endl;
        ss << "\t\t\t<MaxTorque value='" << maxTorque << "'/>" << endl;

        std::map< std::string, float >::iterator propIt = propagatedJointValues.begin();

        while (propIt != propagatedJointValues.end())
        {
            ss << "\t\t\t<PropagateJointValue name='" << propIt->first << "' factor='" << propIt->second << "'/>" << endl;
            propIt++;
        }

        ss << "\t\t</Joint>" << endl;
        return ss.str();
#endif
        return "";
    }

    bool RobotNodeMeta::compileDependencyFunctions()
    {
        std::vector<RobotNodePtr> c;
        collectAllRobotNodes(c);

        compiledDependencyFunctions.clear();

        dependencySymbolTable.create_variable("x");
        dependencySymbolTable.add_constants();

        for (auto& dep : dependencyFunctions)
        {
            RobotNodePtr node;
            for (auto& n : c)
            {
                if (n->getName() == dep.first)
                {
                    node = n;
                    break;
                }
            }

            if (!node)
            {
                VR_WARNING << getName() << " defines dependency function for unknown child '" << dep.first << "'";
                return false;
            }

            exprtk::expression<float> expression;
            expression.register_symbol_table(dependencySymbolTable);

            exprtk::parser<float> parser;
            parser.compile(dep.second, expression);

            compiledDependencyFunctions[node] = expression;
        }

        return true;
    }

} // namespace
