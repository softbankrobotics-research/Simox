/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @author     Manfred Kroehnert
* @copyright  2011 Manfred Kroehnert
*/

#include "EndEffectorActor.h"
#include "../VirtualRobotException.h"
#include "../Model/Nodes/ModelNode.h"
#include "../Model/Nodes/ModelJoint.h"
#include "../Model/Model.h"
#include "../Model/ModelConfig.h"
#include "../Model/Frame.h"
#include "../CollisionDetection/CollisionChecker.h"
#include "../CollisionDetection/CollisionModel.h"
#include "EndEffector.h"
#include "../Model/ModelNodeSet.h"


namespace VirtualRobot
{

    EndEffectorActor::EndEffectorActor(const std::string& name, const std::vector< ActorDefinition >& a, CollisionCheckerPtr colChecker) :
        name(name),
        actors(a)
    {
        this->colChecker = colChecker;

        if (!this->colChecker)
        {
            this->colChecker = CollisionChecker::getGlobalCollisionChecker();
        }
    }

    std::vector<EndEffectorActor::ActorDefinition> EndEffectorActor::getDefinition()
    {
        return actors;
    }

    EndEffectorActorPtr EndEffectorActor::clone(RobotPtr newRobot)
    {
        if (!newRobot)
        {
            VR_ERROR << "Attempting to clone EndEffectorActor for invalid robot";
            return EndEffectorActorPtr();
        }

        std::vector<ActorDefinition> newDef;

        for (unsigned int i = 0; i < actors.size(); i++)
        {
            ActorDefinition a;
            a.colMode = actors[i].colMode;
            a.directionAndSpeed = actors[i].directionAndSpeed;
            a.robotNode = newRobot->getModelNode(actors[i].robotNode->getName());
            newDef.push_back(a);
        }

        return EndEffectorActorPtr(new EndEffectorActor(name, newDef, newRobot->getCollisionChecker()));
    }

    std::string EndEffectorActor::getName()
    {
        return name;
    }

    bool EndEffectorActor::moveActor(float angle)
    {
        if (actors.size() == 0)
        {
            return true;
        }

        RobotPtr robot = actors[0].robotNode->getModel();
        VR_ASSERT(robot);
        bool res = true;

        for (std::vector<ActorDefinition>::iterator n = actors.begin(); n != actors.end(); n++)
        {
            ModelJointPtr j = std::dynamic_pointer_cast<ModelJoint>(n->robotNode);

            if (!j)
                continue;

            float v = j->getJointValue() + angle * n->directionAndSpeed;

            if (v <= j->getJointLimitHigh() && v >= j->getJointLimitLow())
            {
                robot->setJointValue(j, v);
                res = false;
            }
        }

        return res;
    }

    bool EndEffectorActor::moveActorCheckCollision(EndEffectorPtr eef, EndEffector::ContactInfoVector& storeContacts, std::vector<ModelLinkPtr> obstacles, float angle)
    {
        VR_ASSERT(eef);
        RobotPtr robot = eef->getRobot();
        VR_ASSERT(robot);
        bool res = true;
        std::vector<EndEffectorActorPtr> eefActors;
        eef->getActors(eefActors);
        std::vector<ModelLinkPtr> eefStatic;
        eef->getStatics(eefStatic);
        EndEffector::ContactInfoVector newContacts;

        for (std::vector<ActorDefinition>::iterator n = actors.begin(); n != actors.end(); n++)
        {
            ModelJointPtr j = std::dynamic_pointer_cast<ModelJoint>(n->robotNode);

            if (!j)
                continue;
            float oldV =  j->getJointValue();
            float v = oldV + angle * n->directionAndSpeed;

            if (v <= j->getJointLimitHigh() && v >= j->getJointLimitLow())
            {
                robot->setJointValue(j, v);
                //n->robotNode->setJointValue(v);

                // check collision
                bool collision = false;

                // obstacles (store contacts)
                if ((/*n->colMode!=eNone &&*/ obstacles.size()>0 && isColliding(eef, obstacles, newContacts)))
                {
                    collision = true;
                }

                // actors (don't store contacts)
                if (!collision)
                {
                    for (std::vector<EndEffectorActorPtr>::iterator a = eefActors.begin(); a != eefActors.end(); a++)
                    {
                        // Don't check for collisions with the actor itself (don't store contacts)
                        if (((*a)->getName() != name) && isColliding(*a))   //isColliding(eef,*a,newContacts) )
                        {
                            collision = true;
                        }
                    }
                }

                // static (don't store contacts)
                if (!collision)
                {
                    for (std::vector<ModelLinkPtr>::iterator node = eefStatic.begin(); node != eefStatic.end(); node++)
                    {
                        //SceneObjectPtr so = std::dynamic_pointer_cast<SceneObject>(*node);

                        //(don't store contacts)
                        //if( isColliding(eef,so,newContacts,eStatic) )
                        if (isColliding(*node, eStatic))
                        {
                            collision = true;
                        }
                    }
                }

                if (!collision)
                {
                    res = false;
                }
                else
                {
                    // reset last position
                    //n->robotNode->setJointValue(oldV);
                    robot->setJointValue(j, oldV);
                }
            }
        }

        // update contacts
        for (size_t i = 0; i < newContacts.size(); i++)
        {
            // check for double entries (this may happen since we move all actors to the end and may detecting contacts multiple times)
            bool doubleEntry = false;

            for (size_t j = 0; j < storeContacts.size(); j++)
            {
                if (storeContacts[j].robotNode == newContacts[i].robotNode && storeContacts[j].obstacle == newContacts[i].obstacle)
                {
                    doubleEntry = true;
                    break;
                }
            }

            if (!doubleEntry)
            {
                int id1, id2;
                newContacts[i].distance = colChecker->calculateDistance(newContacts[i].robotNode->getCollisionModel(), newContacts[i].obstacle->getCollisionModel(), newContacts[i].contactPointFingerGlobal, newContacts[i].contactPointObstacleGlobal, &id1, &id2);
                newContacts[i].contactPointFingerLocal = newContacts[i].obstacle->toLocalCoordinateSystemVec(newContacts[i].contactPointFingerGlobal);
                newContacts[i].contactPointObstacleLocal = newContacts[i].obstacle->toLocalCoordinateSystemVec(newContacts[i].contactPointObstacleGlobal);

                // compute approach direction
                // todo: this could be done more elegantly (Jacobian)
                RobotConfigPtr config = getConfiguration();
                Eigen::Vector3f contGlobal1 = newContacts[i].contactPointFingerGlobal;
                Eigen::Vector3f contFinger = newContacts[i].robotNode->toLocalCoordinateSystemVec(contGlobal1);
                this->moveActor(angle);
                Eigen::Vector3f contGlobal2 = newContacts[i].robotNode->toGlobalCoordinateSystemVec(contFinger);
                newContacts[i].approachDirectionGlobal = contGlobal2 - contGlobal1;
                newContacts[i].approachDirectionGlobal.normalize();
                robot->setJointValues(config);

                storeContacts.push_back(newContacts[i]);
            }
        }

        return res;
    }


    bool EndEffectorActor::isColliding(EndEffectorPtr eef, std::vector<ModelLinkPtr> obstacles, EndEffector::ContactInfoVector& storeContacts, CollisionMode checkColMode)
    {
        //std::vector<CollisionModelPtr> colModels = obstacles->getCollisionModels();
        //Eigen::Vector3f contact;
        bool col = false;

        for (std::vector<ActorDefinition>::iterator n = actors.begin(); n != actors.end(); n++)
        {
            ModelLinkPtr l = std::dynamic_pointer_cast<ModelLink>(n->robotNode);
            if (!l || !l->getCollisionModel())
                continue;
            for (auto o : obstacles)//= colModels.begin(); o != colModels.end(); o++)
            {
                if ((n->colMode & checkColMode) &&
                    colChecker->checkCollision(l->getCollisionModel(), o->getCollisionModel()))
                {

                    col = true;
                    // create contact info
                    EndEffector::ContactInfo ci;
                    ci.eef = eef;
                    ci.actor = shared_from_this();
                    ci.robotNode = l;
                    ci.obstacle = o;

                    // todo: maybe not needed here: we are in collision, distance makes no sense...
                    // later the distance is calculated anyway (with slightly opened actors)
                    int id1, id2;
                    ci.distance = colChecker->calculateDistance(l->getCollisionModel(), o->getCollisionModel(), ci.contactPointFingerGlobal, ci.contactPointObstacleGlobal, &id1, &id2);
                    ci.contactPointFingerLocal = ci.obstacle->toLocalCoordinateSystemVec(ci.contactPointFingerGlobal);
                    ci.contactPointObstacleLocal = ci.obstacle->toLocalCoordinateSystemVec(ci.contactPointObstacleGlobal);

                    storeContacts.push_back(ci);
                }
            }
        }

        return col;
    }

    bool EndEffectorActor::isColliding(LinkSetPtr obstacles,  CollisionMode checkColMode)
    {
        for (std::vector<ActorDefinition>::iterator n = actors.begin(); n != actors.end(); n++)
        {
            ModelLinkPtr l = std::dynamic_pointer_cast<ModelLink>(n->robotNode);
            if (!l || !l->getCollisionModel())
                continue;
            if ((n->colMode & checkColMode) && l->getCollisionModel() && colChecker->checkCollision(l->getCollisionModel(), obstacles->getCollisionModels()))
            {
                return true;
            }
        }

        return false;
    }

    bool EndEffectorActor::isColliding(ModelLinkPtr obstacle, CollisionMode checkColMode)
    {
        if (!obstacle || !obstacle->getCollisionModel())
        {
            return false;
        }

        for (std::vector<ActorDefinition>::iterator n = actors.begin(); n != actors.end(); n++)
        {
            ModelLinkPtr l = std::dynamic_pointer_cast<ModelLink>(n->robotNode);
            if (!l || !l->getCollisionModel())
                continue;
            if ((n->colMode & checkColMode) && l->getCollisionModel() && colChecker->checkCollision(l->getCollisionModel(), obstacle->getCollisionModel()))
            {
                return true;
            }
        }

        return false;
    }

    bool EndEffectorActor::isColliding(EndEffectorActorPtr obstacle)
    {
        for (std::vector<ActorDefinition>::iterator n = actors.begin(); n != actors.end(); n++)
        {
            ModelLinkPtr l = std::dynamic_pointer_cast<ModelLink>(n->robotNode);
            if (!l || !l->getCollisionModel())
                continue;

            if ((n->colMode & eActors) && obstacle->isColliding(l))
            {
                return true;
            }
        }

        return false;
    }

    bool EndEffectorActor::isColliding(EndEffectorPtr obstacle)
    {
        std::vector<EndEffectorActorPtr> obstacleActors;
        obstacle->getActors(obstacleActors);

        std::vector<ModelLinkPtr> obstacleStatics;
        obstacle->getStatics(obstacleStatics);

        for (std::vector<EndEffectorActorPtr>::iterator actor = obstacleActors.begin(); actor != obstacleActors.end(); actor++)
        {
            // Don't check for collisions with the actor itself
            if (((*actor)->getName() != name) && isColliding(*actor))
            {
                return true;
            }
        }

        for (auto node = obstacleStatics.begin(); node != obstacleStatics.end(); node++)
        {
            if (isColliding(*node, EndEffectorActor::eStatic))
            {
                return true;
            }
        }

        return false;
    }

    bool EndEffectorActor::isColliding(EndEffectorPtr eef, EndEffectorPtr obstacle, EndEffector::ContactInfoVector& storeContacts)
    {
        std::vector<EndEffectorActorPtr> obstacleActors;
        obstacle->getActors(obstacleActors);

        std::vector<ModelLinkPtr> obstacleStatics;
        obstacle->getStatics(obstacleStatics);

        for (std::vector<EndEffectorActorPtr>::iterator actor = obstacleActors.begin(); actor != obstacleActors.end(); actor++)
        {
            // Don't check for collisions with the actor itself
            if (((*actor)->getName() != name) && isColliding(eef, *actor, storeContacts))
            {
                return true;
            }
        }

        for (auto node = obstacleStatics.begin(); node != obstacleStatics.end(); node++)
        {
            if (isColliding(eef, *node, storeContacts, EndEffectorActor::eStatic))
            {
                return true;
            }
        }

        return false;
    }

    bool EndEffectorActor::isColliding(EndEffectorPtr eef, EndEffectorActorPtr obstacle, EndEffector::ContactInfoVector& storeContacts)
    {
        for (std::vector<ActorDefinition>::iterator n = actors.begin(); n != actors.end(); n++)
        {
            ModelLinkPtr l = std::dynamic_pointer_cast<ModelLink>(n->robotNode);
            if (!l || !l->getCollisionModel())
                continue;

            if ((n->colMode & eActors) && obstacle->isColliding(eef, l, storeContacts))
            {
                return true;
            }
        }

        return false;

    }

    bool EndEffectorActor::isColliding(EndEffectorPtr eef, ModelLinkPtr obstacle, EndEffector::ContactInfoVector& storeContacts, CollisionMode checkColMode /*= EndEffectorActor::eAll*/)
    {
        if (!obstacle)
        {
            return false;
        }

        //Eigen::Vector3f contact;
        bool col = false;

        for (std::vector<ActorDefinition>::iterator n = actors.begin(); n != actors.end(); n++)
        {
            ModelLinkPtr l = std::dynamic_pointer_cast<ModelLink>(n->robotNode);
            if (!l || !l->getCollisionModel())
                continue;
            if ((n->colMode & checkColMode) &&
                colChecker->checkCollision(l->getCollisionModel(), obstacle->getCollisionModel()))
            {
                col = true;
                // create contact info
                EndEffector::ContactInfo ci;
                ci.eef = eef;
                ci.actor = shared_from_this();
                ci.robotNode = l;
                ci.obstacle = obstacle;

                // todo: not needed here, later we calculate the distance with opened actors...
                int id1, id2;
                ci.distance = colChecker->calculateDistance(ci.robotNode->getCollisionModel(), ci.obstacle->getCollisionModel(), ci.contactPointFingerGlobal, ci.contactPointObstacleGlobal, &id1, &id2);
                ci.contactPointFingerLocal = ci.obstacle->toLocalCoordinateSystemVec(ci.contactPointFingerGlobal);
                ci.contactPointObstacleLocal = ci.obstacle->toLocalCoordinateSystemVec(ci.contactPointObstacleGlobal);


                storeContacts.push_back(ci);
            }
        }

        return col;
    }


    std::vector< ModelNodePtr > EndEffectorActor::getModelNodes() const
    {
        std::vector< ModelNodePtr > res;

        for (auto n = actors.begin(); n != actors.end(); n++)
        {
            res.push_back(n->robotNode);
        }

        return res;
    }

    std::vector< ModelLinkPtr > EndEffectorActor::getLinks() const
    {
        std::vector< ModelLinkPtr > res;

        for (auto n = actors.begin(); n != actors.end(); n++)
        {
            ModelLinkPtr l = std::dynamic_pointer_cast<ModelLink>(n->robotNode);
            if (l)
                res.push_back(l);
        }
        return res;
    }

    std::vector< ModelJointPtr > EndEffectorActor::getJoints() const
    {
        std::vector< ModelJointPtr > res;
        for (auto n = actors.begin(); n != actors.end(); n++)
        {
            ModelJointPtr l = std::dynamic_pointer_cast<ModelJoint>(n->robotNode);
            if (l)
                res.push_back(l);
        }
        return res;
    }

    void EndEffectorActor::print()
    {
        cout << " ****" << endl;
        cout << " ** Name:" << name << endl;

        for (size_t i = 0; i < actors.size(); i++)
        {
            cout << " *** RobotNode: " << actors[i].robotNode->getName() << ", Direction/Speed:" << actors[i].directionAndSpeed << endl;
            //actors[i].robotNode->print();
        }

        cout << " ****" << endl;
    }

    bool EndEffectorActor::hasNode(RobotNodePtr node)
    {
        std::vector<ActorDefinition>::iterator iS = actors.begin();

        while (iS != actors.end())
        {
            if (iS->robotNode == node)
            {
                return true;
            }

            iS++;
        }

        return false;
    }

    bool EndEffectorActor::nodesSufficient(std::vector<RobotNodePtr> nodes) const
    {
        std::vector<ActorDefinition>::const_iterator i = actors.begin();

        while (i != actors.end())
        {
            std::vector<RobotNodePtr>::const_iterator j = nodes.begin();
            bool ok = false;

            while (j != nodes.end())
            {
                if (i->robotNode->getName() == (*j)->getName())
                {
                    ok = true;
                    break;
                }

                j++;
            }

            if (!ok)
            {
                return false;
            }

            i++;
        }

        return true;
    }

    float EndEffectorActor::getApproximatedLength()
    {
        BoundingBox bb_all;

        for (size_t j = 0; j < actors.size(); j++)
        {
            ModelLinkPtr l = std::dynamic_pointer_cast<ModelLink>(actors[j].robotNode);
            if (l && l->getCollisionModel())
            {
                BoundingBox bb = l->getCollisionModel()->getBoundingBox();
                bb_all.addPoint(bb.getMin());
                bb_all.addPoint(bb.getMax());
            }
        }

        Eigen::Vector3f d = bb_all.getMax() - bb_all.getMin();
        return d.norm();
    }

    VirtualRobot::RobotConfigPtr EndEffectorActor::getConfiguration()
    {
        if (actors.size() == 0 || !actors[0].robotNode)
        {
            return VirtualRobot::RobotConfigPtr();
        }

        std::vector< RobotConfig::Configuration > c;

        for (size_t i = 0; i < actors.size(); i++)
        {
            ModelJointPtr l = std::dynamic_pointer_cast<ModelJoint>(actors[i].robotNode);
            if (!l)
                continue;

            RobotConfig::Configuration e;
            e.name = l->getName();
            e.value = l->getJointValue();
            c.push_back(e);
        }

        RobotConfigPtr res(new RobotConfig(actors[0].robotNode->getModel(), name, c));
        return res;
    }

    std::string EndEffectorActor::toXML(int ident /*= 1*/)
    {
        std::stringstream ss;
        std::string t = "\t";
        std::string pre = "";

        for (int i = 0; i < ident; i++)
        {
            pre += t;
        }

        std::string tt = pre + t;
        std::string ttt = tt + t;
        ss << pre << "<Actor name='" << name << "'>" << endl;

        for (size_t i = 0; i < actors.size(); i++)
        {
            ss << tt << "<Node name='" << actors[i].robotNode->getName() << "' ";

            if (actors[i].colMode == eNone)
            {
                ss << "ConsiderCollisions='None' ";
            }

            if (actors[i].colMode == eAll)
            {
                ss << "ConsiderCollisions='All' ";
            }
            else
            {
                if (actors[i].colMode & eActors)
                {
                    ss << "ConsiderCollisions='Actors' ";
                }

                if (actors[i].colMode & eStatic)
                {
                    ss << "ConsiderCollisions='Static' ";
                }
            }

            ss << "Direction='" << actors[i].directionAndSpeed << "'/>" << endl;
        }

        ss << pre << "</Actor>" << endl;
        return ss.str();
    }



} // namespace VirtualRobot
