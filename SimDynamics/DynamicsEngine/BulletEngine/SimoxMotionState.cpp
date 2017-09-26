
#include "SimoxMotionState.h"
#include "BulletEngine.h"
#include "BulletRobot.h"
#include "../../DynamicsWorld.h"
#include "../../../VirtualRobot/Model/Nodes/ModelJoint.h"



using namespace VirtualRobot;

namespace SimDynamics
{


    SimoxMotionState::SimoxMotionState(VirtualRobot::ModelLinkPtr sceneObject)
        : btDefaultMotionState()
    {
		VR_ASSERT(sceneObject);
        this->sceneObject = sceneObject;
        initalGlobalPose.setIdentity();

        if (sceneObject)
        {
            initalGlobalPose = sceneObject->getGlobalPose();
        }

        _transform.setIdentity();
        _graphicsTransfrom.setIdentity();
        _comOffset.setIdentity();
        Eigen::Vector3f com = sceneObject->getCoMLocal();

        if (sceneObject)
        {
            // localcom is given in coord sytsem of robotnode (== endpoint of internal transformation)
            // we need com in visualization coord system (== without postjointtransform)

            Eigen::Matrix4f t;
            t.setIdentity();
            t.block(0, 3, 3, 1) = sceneObject->getCoMGlobal();
            t = sceneObject->getGlobalPose().inverse() * t;
            com = t.block(0, 3, 3, 1);
        }

        _setCOM(com);
        setGlobalPose(initalGlobalPose);
    }

    SimoxMotionState::~SimoxMotionState()
    {

    }

    void SimoxMotionState::setWorldTransform(const btTransform& worldPose)
    {
        // Check callbacks
        if (callbacks.size() > 0)
        {
            std::vector<SimoxMotionStateCallback*>::iterator it;

            for (it = callbacks.begin(); it != callbacks.end(); it++)
            {
                (*it)->poseChanged(worldPose);
            }
        }

        // _transform is the Bullet pose, used in getWorldTransform().
        _transform = worldPose; // com position
#ifdef _DEBUG

        if (boost::math::isnan(_transform.getOrigin().x()) || boost::math::isnan(_transform.getOrigin().y()) || boost::math::isnan(_transform.getOrigin().z()))
        {
            VR_ERROR << "NAN transform!!!" << endl;
        }

#endif
        m_graphicsWorldTrans = _transform; // this is used for debug drawing
        _graphicsTransfrom = _transform;
        //_graphicsTransfrom.getOrigin();// -= _comOffset.getOrigin(); // com adjusted

        setGlobalPoseSimox(BulletEngine::getPoseEigen(_graphicsTransfrom));
    }

    void SimoxMotionState::getWorldTransform(btTransform& worldTrans) const
    {
        worldTrans = _transform;
    }

    void SimoxMotionState::setGlobalPoseSimox(const Eigen::Matrix4f& worldPose)
    {
        if (!sceneObject)
        {
            return;
        }

        // inv com as matrix4f
        Eigen::Matrix4f comLocal;
        comLocal.setIdentity();
        comLocal.block(0, 3, 3, 1) = -com;

        // worldPose -> local visualization frame
        /*Eigen::Matrix4f localPose = sceneObject->getGlobalPoseVisualization().inverse() * worldPose;

        // com as matrix4f
        Eigen::Matrix4f comLocal;
        comLocal.setIdentity();
        comLocal.block(0,3,3,1) = -com;

        //Eigen::Matrix4f localPoseAdjusted =  localPose * comLocal;
        //Eigen::Matrix4f localPoseAdjusted =  comLocal * localPose;

        //Eigen::Matrix4f localPoseAdjusted =  localPose;
        //localPoseAdjusted.block(0,3,3,1) -= com;

        Eigen::Matrix4f comTrafo = Eigen::Matrix4f::Identity();
        comTrafo.block(0,3,3,1) = -com;
        Eigen::Matrix4f localPoseAdjusted =  localPose * comTrafo;
        //localPoseAdjusted.block(0,3,3,1) -= com;

        Eigen::Matrix4f resPose = sceneObject->getGlobalPoseVisualization() * localPoseAdjusted;
        */
        Eigen::Matrix4f resPose = worldPose * comLocal;


        // get joint angle
		ModelLinkPtr rn = sceneObject;
        DynamicsWorldPtr w = DynamicsWorld::GetWorld();
        DynamicsModelPtr dr = w->getEngine()->getRobot(rn->getModel());
        BulletRobotPtr bdr = std::dynamic_pointer_cast<BulletRobot>(dr);

        if (bdr)
        {
            // check if robotnode is connected with a joint
            std::vector<BulletRobot::LinkInfo> links = bdr->getLinks(rn);

            // update all involved joint values
            for (size_t i = 0; i < links.size(); i++)
            {
                if (links[i].nodeJoint)
                {
                    float ja = float(bdr->getJointAngle(links[i].nodeJoint));
                    // we can update the joint value via an RobotNodeActuator
                    //RobotNodeActuatorPtr rna(new RobotNodeActuator(links[i].nodeJoint));
					//rna->updateJointAngle(ja);
					links[i].nodeJoint->setJointValueNoUpdate(ja);
                }
            }

            // we assume that all models are handled by Bullet, so we do not need to update children
            //robotNodeActuator->updateVisualizationPose(resPose, false);
        } 
    }

    void SimoxMotionState::setGlobalPose(const Eigen::Matrix4f& pose)
    {
        initalGlobalPose = pose;
        /* convert to local coord system, apply comoffset and convert back*/
        Eigen::Matrix4f poseLocal = sceneObject->getGlobalPose().inverse() * pose;
        poseLocal.block(0, 3, 3, 1) += com;
        Eigen::Matrix4f poseGlobal = sceneObject->getGlobalPose() * poseLocal;
        m_startWorldTrans = BulletEngine::getPoseBullet(poseGlobal);
        //m_startWorldTrans.getOrigin() -= _comOffset.getOrigin();
        updateTransform();
    }



    std::vector<SimoxMotionStateCallback*> SimoxMotionState::getCallbacks()
    {
        return callbacks;
    }

    void SimoxMotionState::_setCOM(const Eigen::Vector3f& com)
    {
        this->com = com;
        Eigen::Matrix4f comM = Eigen::Matrix4f::Identity();
        comM.block(0, 3, 3, 1) = -com;
        _comOffset = BulletEngine::getPoseBullet(comM);
    }

    void SimoxMotionState::setCOM(const Eigen::Vector3f& com)
    {
        _setCOM(com);
        updateTransform();
    }

    Eigen::Vector3f SimoxMotionState::getCOM() const
    {
        return com;
    }

    void SimoxMotionState::updateTransform()
    {
        //Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
        //m.block(0,3,3,1) = com;
        setWorldTransform(m_startWorldTrans);
    }

}
