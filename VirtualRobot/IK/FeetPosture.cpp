#include "FeetPosture.h"
#include <VirtualRobot/Model/Model.h>
#include <VirtualRobot/Model/JointSet.h>
#include <VirtualRobot/Model/LinkSet.h>
#include <VirtualRobot/CollisionDetection/CollisionChecker.h>

using namespace std;

namespace VirtualRobot
{


    FeetPosture::FeetPosture(JointSetPtr leftLeg,
							 JointSetPtr rightLeg,
							 LinkSetPtr leftLegCol,
							 LinkSetPtr rightLegCol,
							 Eigen::Matrix4f& transformationLeftToRightFoot,
                             ModelNodePtr baseNode,
                             FramePtr leftTCP,
							 FramePtr rightTCP,
							 JointSetPtr rnsLeft2RightFoot)
        : leftLeg(leftLeg), rightLeg(rightLeg), leftLegCol(leftLegCol), rightLegCol(rightLegCol), left2Right(rnsLeft2RightFoot), transformationLeftToRightFoot(transformationLeftToRightFoot), baseNode(baseNode)
    {
        VR_ASSERT(this->leftLeg);
        VR_ASSERT(this->rightLeg);
        VR_ASSERT(this->leftLeg->getModel() == this->rightLeg->getModel());
        this->leftTCP = leftTCP;

        if (!leftTCP)
        {
            this->leftTCP = leftLeg->getTCP();
        }

        this->rightTCP = rightTCP;

        if (!rightTCP)
        {
            this->rightTCP = rightLeg->getTCP();
        }

        VR_ASSERT(this->leftTCP);
        VR_ASSERT(this->rightTCP);
        VR_ASSERT(baseNode);
    }

    FeetPosture::~FeetPosture()
    {

    }

	JointSetPtr FeetPosture::getLeftLeg()
    {
        return leftLeg;
    }

	JointSetPtr FeetPosture::getRightLeg()
    {
        return rightLeg;
    }

	FramePtr FeetPosture::getLeftTCP()
    {
        return leftTCP;
    }

	FramePtr FeetPosture::getRightTCP()
    {
        return rightTCP;
    }

    ModelNodePtr FeetPosture::getBaseNode()
    {
        return baseNode;
    }

    Eigen::Matrix4f FeetPosture::getTransformationLeftToRightFoot()
    {
        return transformationLeftToRightFoot;
    }

    ModelPtr FeetPosture::getRobot()
    {
        return leftLeg->getModel();
    }

    void FeetPosture::print()
    {
        cout << "FeetPosture:" << endl;
        cout << "* Left leg:" << leftLeg->getName() << endl;
        cout << "* Left leg collision model:" << leftLegCol->getName() << endl;
        cout << "* Left Foot/TCP:" << leftTCP->getName() << endl;
        cout << "* Right leg:" << rightLeg->getName() << endl;
        cout << "* Right leg collision model:" << rightLegCol->getName() << endl;
        cout << "* Right Foot/TCP:" << rightTCP->getName() << endl;
        cout << "* Base Node: " << baseNode->getName() << endl;

        if (left2Right)
        {
            cout << "* RNS Left->Right foot: " << left2Right->getName() << endl;
        }
        else
        {
            cout << "* RNS Left->Right foot: <not set>" << endl;
        }
    }

    void FeetPosture::setCollisionCheck(LinkSetPtr leftColModel, LinkSetPtr rightColModel)
    {
        VR_ASSERT(leftLegCol && rightLegCol);
        leftLegCol = leftColModel;
        rightLegCol = rightColModel;
        VR_ASSERT(leftLegCol->getCollisionChecker() == rightLegCol->getCollisionChecker());
    }

    bool FeetPosture::icCurrentLegConfigCollisionFree()
    {
        VR_ASSERT(leftLegCol && rightLegCol);
        return !leftLegCol->getCollisionChecker()->checkCollision(leftLegCol, rightLegCol);
    }

    LinkSetPtr FeetPosture::getLeftLegCol()
    {
        return leftLegCol;
    }

	LinkSetPtr FeetPosture::getRightLegCol()
    {
        return rightLegCol;
    }

    JointSetPtr FeetPosture::getRNSLeft2RightFoot()
    {
        return left2Right;
    }

    void FeetPosture::setRNSLeft2RightFoot(JointSetPtr rns)
    {
        left2Right = rns;
    }

} // namespace VirtualRobot
