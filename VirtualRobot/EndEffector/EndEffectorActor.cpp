/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @author     Manfred Kroehnert
* @copyright  2011 Manfred Kroehnert
*/

#include "EndEffectorActor.h"
#include "../VirtualRobotException.h"
#include "../Nodes/RobotNode.h"
#include "../Robot.h"
#include "../RobotConfig.h"
#include "../SceneObjectSet.h"
#include "../CollisionDetection/CollisionChecker.h"
#include "EndEffector.h"
#include "../SceneObjectSet.h"


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

        for (auto& actor : actors)
        {
            ActorDefinition a;
            a.colMode = actor.colMode;
            a.directionAndSpeed = actor.directionAndSpeed;
            a.robotNode = newRobot->getRobotNode(actor.robotNode->getName());
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

        RobotPtr robot = actors[0].robotNode->getRobot();
        VR_ASSERT(robot);
        bool res = true;

        for (auto& actor : actors)
        {
            float v = actor.robotNode->getJointValue() + angle * actor.directionAndSpeed;

            if (v <= actor.robotNode->getJointLimitHi() && v >= actor.robotNode->getJointLimitLo())
            {
                robot->setJointValue(actor.robotNode, v);
                //n->robotNode->setJointValue(v);
                res = false;
            }
        }

        return res;
    }

    bool EndEffectorActor::moveActorCheckCollision(EndEffectorPtr eef, EndEffector::ContactInfoVector& storeContacts, SceneObjectSetPtr obstacles /*= SceneObjectSetPtr()*/, float angle /*= 0.02*/)
    {
        VR_ASSERT(eef);
        RobotPtr robot = eef->getRobot();
        VR_ASSERT(robot);
        //bool res = true;
        std::vector<EndEffectorActorPtr> eefActors;
        eef->getActors(eefActors);
        std::vector<RobotNodePtr> eefStatic;
        eef->getStatics(eefStatic);
        EndEffector::ContactInfoVector newContacts;

        enum ActorStatus
        {
            eFinished,
            eCollision,
            eMoving
        };
        std::map<RobotNodePtr, ActorStatus> actorStatus;



        for (auto& actor : actors)
        {
            float oldV =  actor.robotNode->getJointValue();
            float v = oldV + angle * actor.directionAndSpeed;

            actorStatus[actor.robotNode] = eMoving;

            if (v <= actor.robotNode->getJointLimitHi() && v >= actor.robotNode->getJointLimitLo())
            {
                robot->setJointValue(actor.robotNode, v);
                //n->robotNode->setJointValue(v);

                // check collision
                bool collision = false;

                // obstacles (store contacts)
                if ((/*n->colMode!=eNone &&*/ obstacles && isColliding(eef, obstacles, newContacts)))
                {
                    collision = true;
                }

                // actors (don't store contacts)
                if (!collision)
                {
                    for (auto& eefActor : eefActors)
                    {
                        // Don't check for collisions with the actor itself (don't store contacts)
                        if ((eefActor->getName() != name) && isColliding(eefActor))   //isColliding(eef,*a,newContacts) )
                        {
                            collision = true;
                        }
                    }
                }

                // static (don't store contacts)
                if (!collision)
                {
                    for (auto& node : eefStatic)
                    {
                        SceneObjectPtr so = boost::dynamic_pointer_cast<SceneObject>(node);

                        //(don't store contacts)
                        //if( isColliding(eef,so,newContacts,eStatic) )
                        if (isColliding(so, eStatic))
                        {
                            collision = true;
                        }
                    }
                }

                if (!collision)
                {
                    //res = false;
                }
                else
                {
                    // reset last position
                    //n->robotNode->setJointValue(oldV);
                    robot->setJointValue(actor.robotNode, oldV);

                    actorStatus[actor.robotNode] = eCollision;
                }
            }
            else
            {
                actorStatus[actor.robotNode] = eFinished;
            }
        }

        // update contacts
        for (auto& newContact : newContacts)
        {
            // check for double entries (this may happen since we move all actors to the end and may detecting contacts multiple times)
            bool doubleEntry = false;

            for (auto& storeContact : storeContacts)
            {
                if (storeContact.robotNode == newContact.robotNode && storeContact.obstacle == newContact.obstacle)
                {
                    doubleEntry = true;
                    break;
                }
            }

            if (!doubleEntry)
            {
                int id1, id2;
                newContact.distance = colChecker->calculateDistance(newContact.robotNode->getCollisionModel(), newContact.obstacle->getCollisionModel(), newContact.contactPointFingerGlobal, newContact.contactPointObstacleGlobal, &id1, &id2);
                newContact.contactPointFingerLocal = newContact.obstacle->toLocalCoordinateSystemVec(newContact.contactPointFingerGlobal);
                newContact.contactPointObstacleLocal = newContact.obstacle->toLocalCoordinateSystemVec(newContact.contactPointObstacleGlobal);

                // compute approach direction
                // todo: this could be done more elegantly (Jacobian)
                RobotConfigPtr config = getConfiguration();
                Eigen::Vector3f contGlobal1 = newContact.contactPointFingerGlobal;
                Eigen::Vector3f contFinger = newContact.robotNode->toLocalCoordinateSystemVec(contGlobal1);
                this->moveActor(angle);
                Eigen::Vector3f contGlobal2 = newContact.robotNode->toGlobalCoordinateSystemVec(contFinger);
                newContact.approachDirectionGlobal = contGlobal2 - contGlobal1;
                newContact.approachDirectionGlobal.normalize();
                robot->setJointValues(config);

                storeContacts.push_back(newContact);
            }
        }

        // check what we should return
        bool res = true;
        for (auto& actor : actors)
        {
            // if at least one actor is not in collision and not at its joint limits, we are still in the process of closing the fingers
            if (actorStatus[actor.robotNode] == eMoving)
            {
                res = false;
            }
        }

        return res;
    }


    bool EndEffectorActor::isColliding(EndEffectorPtr eef, SceneObjectSetPtr obstacles, EndEffector::ContactInfoVector& storeContacts, CollisionMode checkColMode)
    {
        std::vector<SceneObjectPtr> colModels = obstacles->getSceneObjects();
        //Eigen::Vector3f contact;
        bool col = false;

        for (auto& actor : actors)
        {
            for (auto& colModel : colModels)
            {

                if ((actor.colMode & checkColMode) &&
                    (colModel->getCollisionModel()) &&
                    actor.robotNode->getCollisionModel() &&
                    colChecker->checkCollision(actor.robotNode->getCollisionModel(), colModel->getCollisionModel()))
                {

                    col = true;
                    // create contact info
                    EndEffector::ContactInfo ci;
                    ci.eef = eef;
                    ci.actor = shared_from_this();
                    ci.robotNode = actor.robotNode;
                    ci.obstacle = colModel;

                    // todo: maybe not needed here: we are in collision, distance makes no sense...
                    // later the distance is calculated anyway (with slightly opened actors)
                    int id1, id2;
                    ci.distance = colChecker->calculateDistance(ci.robotNode->getCollisionModel(), ci.obstacle->getCollisionModel(), ci.contactPointFingerGlobal, ci.contactPointObstacleGlobal, &id1, &id2);
                    ci.contactPointFingerLocal = ci.obstacle->toLocalCoordinateSystemVec(ci.contactPointFingerGlobal);
                    ci.contactPointObstacleLocal = ci.obstacle->toLocalCoordinateSystemVec(ci.contactPointObstacleGlobal);

                    storeContacts.push_back(ci);
                }
            }
        }

        return col;
    }

    bool EndEffectorActor::isColliding(SceneObjectSetPtr obstacles,  CollisionMode checkColMode)
    {
        for (auto& actor : actors)
        {
            if ((actor.colMode & checkColMode) && actor.robotNode->getCollisionModel() && colChecker->checkCollision(actor.robotNode->getCollisionModel(), obstacles))
            {
                return true;
            }
        }

        return false;
    }

    bool EndEffectorActor::isColliding(SceneObjectPtr obstacle, CollisionMode checkColMode)
    {
        if (!obstacle || !obstacle->getCollisionModel())
        {
            return false;
        }

        for (auto& actor : actors)
        {
            if ((actor.colMode & checkColMode) && actor.robotNode->getCollisionModel() && colChecker->checkCollision(actor.robotNode->getCollisionModel(), obstacle->getCollisionModel()))
            {
                return true;
            }
        }

        return false;
    }

    bool EndEffectorActor::isColliding(EndEffectorActorPtr obstacle)
    {
        for (auto& actor : actors)
        {
            SceneObjectPtr so = boost::dynamic_pointer_cast<SceneObject>(actor.robotNode);

            if ((actor.colMode & eActors) && obstacle->isColliding(so))
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

        std::vector<RobotNodePtr> obstacleStatics;
        obstacle->getStatics(obstacleStatics);

        for (auto& obstacleActor : obstacleActors)
        {
            // Don't check for collisions with the actor itself
            if ((obstacleActor->getName() != name) && isColliding(obstacleActor))
            {
                return true;
            }
        }

        for (auto& obstacleStatic : obstacleStatics)
        {
            SceneObjectPtr so = boost::dynamic_pointer_cast<SceneObject>(obstacleStatic);

            if (isColliding(so, EndEffectorActor::eStatic))
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

        std::vector<RobotNodePtr> obstacleStatics;
        obstacle->getStatics(obstacleStatics);

        for (auto& obstacleActor : obstacleActors)
        {
            // Don't check for collisions with the actor itself
            if ((obstacleActor->getName() != name) && isColliding(eef, obstacleActor, storeContacts))
            {
                return true;
            }
        }

        for (auto& obstacleStatic : obstacleStatics)
        {
            SceneObjectPtr so = boost::dynamic_pointer_cast<SceneObject>(obstacleStatic);

            if (isColliding(eef, so, storeContacts, EndEffectorActor::eStatic))
            {
                return true;
            }
        }

        return false;
    }

    bool EndEffectorActor::isColliding(EndEffectorPtr eef, EndEffectorActorPtr obstacle, EndEffector::ContactInfoVector& storeContacts)
    {
        for (auto& actor : actors)
        {
            SceneObjectPtr so = boost::dynamic_pointer_cast<SceneObject>(actor.robotNode);

            if ((actor.colMode & eActors) && obstacle->isColliding(eef, so, storeContacts))
            {
                return true;
            }
        }

        return false;

    }

    bool EndEffectorActor::isColliding(EndEffectorPtr eef, SceneObjectPtr obstacle, EndEffector::ContactInfoVector& storeContacts, CollisionMode checkColMode /*= EndEffectorActor::eAll*/)
    {
        if (!obstacle || !obstacle->getCollisionModel())
        {
            return false;
        }

        //Eigen::Vector3f contact;
        bool col = false;

        for (auto& actor : actors)
        {

            if ((actor.colMode & checkColMode) &&
                actor.robotNode->getCollisionModel() &&
                colChecker->checkCollision(actor.robotNode->getCollisionModel(), obstacle->getCollisionModel()))
            {
                col = true;
                // create contact info
                EndEffector::ContactInfo ci;
                ci.eef = eef;
                ci.actor = shared_from_this();
                ci.robotNode = actor.robotNode;
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





    std::vector< RobotNodePtr > EndEffectorActor::getRobotNodes()
    {
        std::vector< RobotNodePtr > res;

        for (auto& actor : actors)
        {
            res.push_back(actor.robotNode);
        }

        return res;
    }

    void EndEffectorActor::print()
    {
        cout << " ****" << endl;
        cout << " ** Name:" << name << endl;

        for (auto& actor : actors)
        {
            cout << " *** RobotNode: " << actor.robotNode->getName() << ", Direction/Speed:" << actor.directionAndSpeed << endl;
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

        for (auto& actor : actors)
        {
            if (actor.robotNode->getCollisionModel())
            {
                BoundingBox bb = actor.robotNode->getCollisionModel()->getBoundingBox();
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

        for (auto& actor : actors)
        {
            RobotConfig::Configuration e;
            e.name = actor.robotNode->getName();
            e.value = actor.robotNode->getJointValue();
            c.push_back(e);
        }

        RobotConfigPtr res(new RobotConfig(actors[0].robotNode->getRobot(), name, c));
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

        for (auto& actor : actors)
        {
            ss << tt << "<Node name='" << actor.robotNode->getName() << "' ";

            if (actor.colMode == eNone)
            {
                ss << "ConsiderCollisions='None' ";
            }

            if (actor.colMode == eAll)
            {
                ss << "ConsiderCollisions='All' ";
            }
            else
            {
                if (actor.colMode & eActors)
                {
                    ss << "ConsiderCollisions='Actors' ";
                }

                if (actor.colMode & eStatic)
                {
                    ss << "ConsiderCollisions='Static' ";
                }
            }

            ss << "Direction='" << actor.directionAndSpeed << "'/>" << endl;
        }

        ss << pre << "</Actor>" << endl;
        return ss.str();
    }

    bool EndEffectorActor::isAtHiLimit() const
    {
        for (const auto& actor : actors)
        {
            const float v = actor.robotNode->getJointValue();
            const auto [min, max] =
                std::minmax(
                    actor.robotNode->getJointLimitHi(),
                    actor.robotNode->getJointLimitLo()
                );
            if (actor.directionAndSpeed > 0)
            {
                if (v < max)
                {
                    return false;
                }
            }
            else
            {
                if (v > min)
                {
                    return false;
                }
            }
        }
        return true;
    }

    bool EndEffectorActor::isAtLoLimit() const
    {
        for (const auto& actor : actors)
        {
            const float v = actor.robotNode->getJointValue();
            const auto [min, max] =
                std::minmax(
                    actor.robotNode->getJointLimitHi(),
                    actor.robotNode->getJointLimitLo()
                );
            if (actor.directionAndSpeed > 0)
            {
                if (v > min)
                {
                    return false;
                }
            }
            else
            {
                if (v < max)
                {
                    return false;
                }
            }
        }
        return true;
    }

} // namespace VirtualRobot
