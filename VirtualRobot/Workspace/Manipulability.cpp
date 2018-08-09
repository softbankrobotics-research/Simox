#include "Manipulability.h"
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Model/Model.h>
#include <VirtualRobot/Model/JointSet.h>
#include <VirtualRobot/Model/Nodes/ModelJoint.h>
#include <VirtualRobot/Model/Nodes/ModelLink.h>
#include <VirtualRobot/CollisionDetection/CollisionChecker.h>
#include <VirtualRobot/Model/ManipulationObject.h>
#include <VirtualRobot/Grasping/Grasp.h>
#include <VirtualRobot/Grasping/GraspSet.h>
#include <VirtualRobot/IK/PoseQualityExtendedManipulability.h>
#include <VirtualRobot/XML/ModelIO.h>
#include <VirtualRobot/Random.h>

#include <fstream>
#include <cmath>
#include <float.h>
#include <limits.h>
#include <algorithm>
#include <thread>

// if enabled, not the global manipulability is considered, but the manipulability in terms of moving upwards
//#define MANIPULABILITY_USE_UPRIGHT_DIRECTION

namespace VirtualRobot
{


    Manipulability::Manipulability(RobotPtr robot) : WorkspaceRepresentation(robot)
    {
        type = "Manipulability";
        maxManip = 1.0f;
        measureName = "<not set>";
        considerJL = false;
        considerSelfDist = false;
        selfDistAlpha = 40.0f;
        selfDistBeta = 1.0f;
    }



    void Manipulability::addPose(const Eigen::Matrix4f& pose, PoseQualityMeasurementPtr qualMeasure)
    {
        addPose(pose, qualMeasure, selfDistStatic, selfDistDynamic);
    }

    void Manipulability::addPose(const Eigen::Matrix4f& pose, PoseQualityMeasurementPtr qualMeasure, RobotNodeSetPtr selfDistSt, RobotNodeSetPtr selfDistDyn)
    {
        Eigen::Matrix4f p = pose;
        toLocal(p);

        float x[6];

        matrix2Vector(p, x);
        //MathTools::eigen4f2rpy(p,x);

        // check for achieved values
        for (int i = 0; i < 6; i++)
        {
            if (x[i] < achievedMinValues[i])
            {
                achievedMinValues[i] = x[i];
            }

            if (x[i] > achievedMaxValues[i])
            {
                achievedMaxValues[i] = x[i];
            }
        }

        // get voxels
        unsigned int v[6];

        if (getVoxelFromPose(x, v))
        {
            float m = getCurrentManipulability(qualMeasure, selfDistSt, selfDistDyn);
            float mSc = m / maxManip;

            if (mSc > 1)
            {
                if (mSc > 1.05)
                {
                    VR_WARNING << "Manipulability is larger than max value. Current Manip:" << m << ", maxManip:" << maxManip << ", percent:" << mSc << endl;
                }

                mSc = 1.0f;
            }

            if (m < 0)
            {
                mSc = 0;
            }

            unsigned char e = (unsigned char)(mSc * (float)UCHAR_MAX + 0.5f);

            //cout<<"m = "<<m<<endl;
            //cout<<"mSc = "<<mSc<<endl;
            //cout<<"e = "<<int(e)<<endl;

            // add at least 1, since the pose is reachable
            if (e == 0)
            {
                e = 1;
            }

            if (e > data->get(v))
            {
                data->setDatum(v, e);
            }
        }

        buildUpLoops++;
    }

    void Manipulability::addPose(const Eigen::Matrix4f& pose)
    {
        addPose(pose, measure);
    }

    float Manipulability::getCurrentManipulability(PoseQualityMeasurementPtr qualMeasure, RobotNodeSetPtr selfDistSt, RobotNodeSetPtr selfDistDyn)
    {
        if (!qualMeasure)
        {
            return 0.0f;
        }

        if (!selfDistSt)
            selfDistSt = selfDistStatic;
        if (!selfDistDyn)
            selfDistDyn = selfDistDynamic;

        if (considerSelfDist && selfDistStatic && selfDistDynamic)
        {
            int id1;
            int id2;
            Eigen::Vector3f p1;
            Eigen::Vector3f p2;
            /*float d =*/ selfDistStatic->getCollisionChecker()->calculateDistance(selfDistStatic, selfDistDynamic, p1, p2, &id1, &id2);
            //cout << "#### dist:" << d << ", ";
            Eigen::Matrix4f obstDistPos1 = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f obstDistPos2 = Eigen::Matrix4f::Identity();
            obstDistPos1.block(0, 3, 3, 1) = p1;
            obstDistPos2.block(0, 3, 3, 1) = p2;

            // transform to tcp
            Eigen::Matrix4f p1_tcp = tcpNode->toLocalCoordinateSystem(obstDistPos1);
            Eigen::Matrix4f p2_tcp = tcpNode->toLocalCoordinateSystem(obstDistPos2);
            Eigen::Vector3f minDistVector = p1_tcp.block(0, 3, 3, 1) - p2_tcp.block(0, 3, 3, 1);

            qualMeasure->setObstacleDistanceVector(minDistVector);

        }

#ifdef MANIPULABILITY_USE_UPRIGHT_DIRECTION
        // check for upright manipulability
        Eigen::VectorXf p(6);
        p.setZero();
        p(2) = 1.0f;
        return qualMeasure->getPoseQuality(p);
#endif
        return qualMeasure->getPoseQuality();
    }

    bool Manipulability::customStringRead(std::ifstream& file, std::string& res)
    {
        int length;
        file.read((char*)&length, sizeof(int));

        if (length <= 0)
        {
            VR_WARNING << "Bad string length: " << length << std::endl;
        }
        else
        {
            char* data = new char[length + 1];
            file.read(data, length);
            data[length] = '\0';
            res = data;
            delete[] data;
            return true;
        }

        return false;
    }

    bool Manipulability::customLoad(std::ifstream& file)
    {
        std::string res;
        //bool lOK = readString(res,file);
        bool lOK = false;
        //int length = read<int>(file);
        lOK = FileIO::readString(res, file);

        if (!lOK)
        {
            VR_ERROR << "Could not get manip measure name from file?!" << endl;
            return false;
        }

        if (measure && (res != measure->getName()))
        {
            VR_WARNING << "Different manipulability measure implementations!" << endl;
            cout << "Manip File :" << res << endl;
            cout << "Instance:" << measure->getName() << endl;
            cout << "-> This may cause problems if you intend to extend the manipulability representation." << endl;
            cout << "-> Otherwise you can ignore this warning." << endl;
        }

        measureName = res;

        if (!measure && measureName == PoseQualityManipulability::getTypeName())
        {
            VR_INFO << "Creating manipulability measure" << endl;
            measure.reset(new PoseQualityManipulability(nodeSet));
        }
        else if (!measure && measureName == PoseQualityExtendedManipulability::getTypeName())
        {
            VR_INFO << "Creating extended manipulability measure" << endl;
            measure.reset(new PoseQualityExtendedManipulability(nodeSet));
        }

        //int cjl = read<int>(file);
        int cjl = (int)(FileIO::read<ioIntTypeRead>(file));

        //file.read((char *)&cjl, sizeof(int));
        if (cjl == 0)
        {
            considerJL = false;
        }
        else
        {
            considerJL = true;
        }

        file.read((char*)&maxManip, sizeof(float));

        considerSelfDist = false;
        selfDistStatic.reset();
        selfDistDynamic.reset();

        if (versionMajor>2 || (versionMajor == 2 && versionMinor >= 3))
        {
            // read self collision data
            cjl = (int)(FileIO::read<ioIntTypeRead>(file));;

            //file.read((char *)&cjl, sizeof(int));
            if (cjl == 0)
            {
                considerSelfDist = false;
            }
            else
            {
                considerSelfDist = true;
            }

            std::string selfDist1, selfDist2;
            bool sd1 = FileIO::readString(selfDist1, file); //customStringRead(file,selfDist1);
            bool sd2 = FileIO::readString(selfDist2, file); //customStringRead(file,selfDist2);

            if (!sd1 || !sd2)
            {
                VR_ERROR << "Could not get rns for self dist name from file?!" << endl;
                return false;
            }

            if (considerSelfDist)
            {
                selfDistStatic = robot->getLinkSet(selfDist1);
                selfDistDynamic = robot->getLinkSet(selfDist2);

                if (!selfDistStatic)
                {
                    VR_ERROR << " No rns with name " << selfDist1 << " found..." << endl;
                    considerSelfDist = false;
                    selfDistDynamic.reset();
                }

                if (!selfDistDynamic)
                {
                    VR_ERROR << " No rns with name " << selfDist2 << " found..." << endl;
                    considerSelfDist = false;
                    selfDistStatic.reset();
                }
            }

        }

        // since 2.9: alpha and beta for self dist
        if (versionMajor>2 || (versionMajor == 2 && versionMinor >= 9))
        {
            selfDistAlpha = FileIO::read<float>(file);
            selfDistBeta = FileIO::read<float>(file);
        }

        // init self dist
        if (considerSelfDist)
        {
            PoseQualityExtendedManipulabilityPtr pqm = std::dynamic_pointer_cast<PoseQualityExtendedManipulability>(measure);
            if (pqm)
            {
                VR_INFO << "Setting up self dist, alpha:" << selfDistAlpha << ", beta:" << selfDistBeta << endl;
                pqm->considerObstacles(true, selfDistAlpha, selfDistBeta);
            }
        }

        return true;
    }


    bool Manipulability::customSave(std::ofstream& file)
    {
        FileIO::writeString(file, measureName);

        int cjl = 0;

        if (considerJL)
        {
            cjl = 1;
        }

        FileIO::write<ioIntTypeWrite>(file, (ioIntTypeWrite)(cjl));

        FileIO::write<float>(file, maxManip);

        // write self collision data
        cjl = 0;

        if (considerSelfDist)
        {
            cjl = 1;
        }

        FileIO::write<ioIntTypeWrite>(file, (ioIntTypeWrite)(cjl));

        std::string selfDistRNS1 = "<not set>";

        if (selfDistStatic && considerSelfDist)
        {
            selfDistRNS1 = selfDistStatic->getName();
        }

        std::string selfDistRNS2 = "<not set>";

        if (selfDistDynamic && considerSelfDist)
        {
            selfDistRNS2 = selfDistDynamic->getName();
        }

        FileIO::writeString(file, selfDistRNS1);

        FileIO::writeString(file, selfDistRNS2);

        // since 2.9: alpha and beta for self dist
        float selfDistA = 0.0f;
        float selfDistB = 0.0f;
        PoseQualityExtendedManipulabilityPtr pqm = std::dynamic_pointer_cast<PoseQualityExtendedManipulability>(measure);
        if (pqm)
        {
            pqm->getSelfDistParameters(selfDistA, selfDistB);
        }
        FileIO::write<float>(file, selfDistA);
        FileIO::write<float>(file, selfDistB);

        return true;
    }


    void Manipulability::setManipulabilityMeasure(PoseQualityMeasurementPtr m)
    {
        measure = m;

        if (measure)
        {
            measureName = measure->getName();
            considerJL = measure->consideringJointLimits();
        }
        else
        {
            measureName = "<not set>";
            considerJL = false;
        }
    }

    PoseQualityMeasurementPtr Manipulability::getManipulabilityMeasure()
    {
        return measure;
    }

    float Manipulability::measureCurrentPose()
    {
        return getCurrentManipulability(measure);
    }


    std::string Manipulability::getMeasureName() const
    {
        return measureName;
    }


    bool Manipulability::consideringJointLimits() const
    {
        return considerJL;
    }


    void Manipulability::customPrint()
    {
        cout << "Manipulability Measure: " << measureName << endl;
        cout << "Considered Joint Limits:";

        if (considerJL)
        {
            cout << " yes" << endl;
        }
        else
        {
            cout << " no" << endl;
        }

        cout << "Maximal manipulability (as defined on construction):" << maxManip << endl;
        cout << "Considered Self-Distance:";

        if (considerSelfDist && selfDistStatic && selfDistDynamic)
        {
            cout << " yes" << endl;
            cout << " - Self Dist Col Model Static:" << selfDistStatic->getName() << endl;
            cout << " - Self Dist Col Model Dynamic:" << selfDistDynamic->getName() << endl;
        }
        else
        {
            cout << " no" << endl;
        }
    }

    void Manipulability::customInitialize()
    {
        if (!measure)
        {
            VR_INFO << "Creating manipulability measure" << endl;
            measure.reset(new PoseQualityManipulability(nodeSet));
            measureName = measure->getName();
        }
    }


    void Manipulability::setMaxManipulability(float maximalManip)
    {
        maxManip = maximalManip;
    }


    float Manipulability::getManipulabilityAtPose(const Eigen::Matrix4f& globalPose)
    {
        if (!data)
        {
            VR_ERROR << "NULL DATA" << endl;
            return 0.0f;
        }

        // get voxels
        unsigned int v[6];

        if (getVoxelFromPose(globalPose, v)) // assumes global_pose, transformation to  basenode is done
        {
            unsigned char e = data->get(v);
            float ef = (float)e / 255.0f;
            return ef * maxManip;
        }

        // position is outside WorkspaceRepresentation data
        return 0.0f;
    }


    bool Manipulability::checkForParameters(JointSetPtr nodeSet, float steps, float storeMinBounds[6], float storeMaxBounds[6], float& storeMaxManipulability, RobotNodePtr baseNode /*= RobotNodePtr()*/, FramePtr tcpNode /*= RobotNodePtr()*/)
    {
        if (!WorkspaceRepresentation::checkForParameters(nodeSet, steps, storeMinBounds, storeMaxBounds, baseNode, tcpNode))
        {
            return false;
        }

        if (!measure)
        {
            VR_WARNING << "No manipulability measure given?!" << endl;
            storeMaxManipulability = 1.0f;
            return true;
        }

        Eigen::VectorXf c;
        nodeSet->getJointValues(c);
        bool visuSate = robot->getUpdateVisualization();
        robot->setUpdateVisualization(false);

        storeMaxManipulability = 0.0f;

        for (int i = 0; i < steps; i++)
        {
            setRobotNodesToRandomConfig(nodeSet, false);
#ifdef MANIPULABILITY_USE_UPRIGHT_DIRECTION
            // check for upright manipulability
            Eigen::VectorXf p(6);
            p.setZero();
            p(2) = 1.0f;
            float q = measure->getPoseQuality(p);
#else
//            float q = measure->getPoseQuality();
            float q = getCurrentManipulability(measure);
#endif

            if (q > storeMaxManipulability)
            {
                storeMaxManipulability = q;
            }
        }

        nodeSet->setJointValues(c);
        robot->setUpdateVisualization(visuSate);

        if (storeMaxManipulability == 0.0f)
        {
            VR_ERROR << "Maximum manipulability == 0 ??" << endl;
            storeMaxManipulability = 1.0f;
            return false;
        }

        // assume a slightly higher max value
        storeMaxManipulability *= 1.1f;
        return true;
    }


    std::vector< Manipulability::ManipulabiliyGrasp > Manipulability::analyseGrasps(GraspSetPtr grasps, ManipulationObjectPtr object)
    {
        THROW_VR_EXCEPTION_IF(!object, "no object");
        THROW_VR_EXCEPTION_IF(!grasps, "no grasps");

        std::vector< Manipulability::ManipulabiliyGrasp > result;

        for (unsigned int i = 0; i < grasps->getSize(); i++)
        {
            GraspPtr g = grasps->getGrasp(i);
            Eigen::Matrix4f m = g->getTcpPoseGlobal(object->getGlobalPose());
            float ma = getManipulabilityAtPose(m);

            if (ma > 0)
            {
                ManipulabiliyGrasp e;
                e.manipulability = ma;
                e.grasp = g;
                result.push_back(e);
            }
        }

        std::sort(result.begin(), result.end());
        std::reverse(result.begin(), result.end());

        return result;
    }


    Manipulability::ManipulabiliyGrasp Manipulability::analyseGrasp(GraspPtr grasp, ManipulationObjectPtr object)
    {
        THROW_VR_EXCEPTION_IF(!object, "no object");
        THROW_VR_EXCEPTION_IF(!grasp, "no grasp");

        Eigen::Matrix4f m = grasp->getTcpPoseGlobal(object->getGlobalPose());
        ManipulabiliyGrasp e;
        e.manipulability = getManipulabilityAtPose(m);
        e.grasp = grasp;
        return e;
    }


    void Manipulability::initSelfDistanceCheck(LinkSetPtr staticModel, LinkSetPtr dynamicModel)
    {
        considerSelfDist = (staticModel && dynamicModel);
        selfDistStatic = staticModel;
        selfDistDynamic = dynamicModel;
    }


    bool Manipulability::smooth(unsigned int minNeighbors)
    {
        if (!data)
        {
            return false;
        }

        if (minNeighbors <= 0)
        {
            minNeighbors = 1;
        }

        // copy data
        WorkspaceDataPtr newData(data->clone());

        int s = 1;

        for (int a = s; a < (int)data->getSize(0) - s; a++)
        {
            cout << "#";

            for (int b = s; b < (int)data->getSize(1) - s; b++)
            {
                for (int c = s; c < (int)data->getSize(2) - s; c++)
                {
                    for (int d = s; d < (int)data->getSize(3) - s; d++)
                    {
                        for (int e = s; e < (int)data->getSize(4) - s; e++)
                        {
                            for (int f = s; f < (int)data->getSize(5) - s; f++)
                            {

                                //unsigned int posOrigTr;
                                //unsigned int posOrigRo;
                                //data->getPos(a,b,c,d,e,f, posOrigTr, posOrigRo);

                                long v = 0;
                                long count = 0;

                                for (int a2 = -s; a2 <= s; a2++)
                                {
                                    for (int b2 = -s; b2 <= s; b2++)
                                    {
                                        for (int c2 = -s; c2 <= s; c2++)
                                        {
                                            for (int d2 = -s; d2 <= s; d2++)
                                            {
                                                for (int e2 = -s; e2 <= s; e2++)
                                                {
                                                    for (int f2 = -s; f2 <= s; f2++)
                                                    {
                                                        //unsigned int posTr,posRo;
                                                        //data->getPos(a+a2,b+b2,c+c2,d+d2,e+e2,f+f2, posTr,posRo);
                                                        long entry = (long)data->get(a + a2, b + b2, c + c2, d + d2, e + e2, f + f2);

                                                        if (entry > 0)
                                                        {
                                                            v += entry;
                                                            count++;
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }

                                if (count >= (int)(minNeighbors))
                                {
                                    unsigned char newVal = (unsigned char)((double)v / (double)count);

                                    if (newVal > 0)
                                    {
                                        newData->setDatum(a, b, c, d, e, f, newVal);
                                    }

                                    //data->data[posOrig] = (unsigned int) ((double)v / (double)count);
                                }
                            }
                        }
                    }
                }
            }
        }

        data = newData;
        return true;
    }


    GraspSetPtr Manipulability::getReachableGrasps(GraspSetPtr grasps, ManipulationObjectPtr object)
    {
        THROW_VR_EXCEPTION_IF(!object, "no object");
        THROW_VR_EXCEPTION_IF(!grasps, "no grasps");

        GraspSetPtr result(new GraspSet(grasps->getName(), grasps->getRobotType(), grasps->getEndEffector()));

        for (unsigned int i = 0; i < grasps->getSize(); i++)
        {
            Eigen::Matrix4f m = grasps->getGrasp(i)->getTcpPoseGlobal(object->getGlobalPose());

            if (isCovered(m))
            {
                result->addGrasp(grasps->getGrasp(i));
            }
        }

        return result;
    }


    void Manipulability::getSelfDistConfig(bool& storeConsiderSelfDist, LinkSetPtr& storeStatic, LinkSetPtr& storeDynamic)
    {
        storeConsiderSelfDist = considerSelfDist;
        storeStatic = selfDistStatic;
        storeDynamic = selfDistDynamic;
    }


    VirtualRobot::WorkspaceRepresentationPtr Manipulability::clone()
    {
        VirtualRobot::ManipulabilityPtr res(new Manipulability(robot));
        res->setOrientationType(this->orientationType);
        res->versionMajor = this->versionMajor;
        res->versionMinor = this->versionMinor;
        res->nodeSet = this->nodeSet;
        res->type = this->type;

        res->baseNode = this->baseNode;
        res->tcpNode = this->tcpNode;
        res->staticCollisionModel = this->staticCollisionModel;
        res->dynamicCollisionModel = this->dynamicCollisionModel;
        res->buildUpLoops = this->buildUpLoops;
        res->collisionConfigs = this->collisionConfigs;
        res->discretizeStepTranslation = this->discretizeStepTranslation;
        res->discretizeStepRotation = this->discretizeStepRotation;
        memcpy(res->minBounds, this->minBounds, sizeof(float) * 6);
        memcpy(res->maxBounds, this->maxBounds, sizeof(float) * 6);
        memcpy(res->numVoxels, this->numVoxels, sizeof(float) * 6);
        memcpy(res->achievedMinValues, this->achievedMinValues, sizeof(float) * 6);
        memcpy(res->achievedMaxValues, this->achievedMaxValues, sizeof(float) * 6);
        memcpy(res->spaceSize, this->spaceSize, sizeof(float) * 6);

        res->adjustOnOverflow = this->adjustOnOverflow;
        res->data.reset(this->data->clone());

        res->measure = this->measure;
        res->maxManip = this->maxManip;
        res->measureName = this->measureName;
        res->considerJL = this->considerJL;
        res->considerSelfDist = this->considerSelfDist;
        res->selfDistStatic = this->selfDistStatic;
        res->selfDistDynamic = this->selfDistDynamic;

        return res;
    }

    float Manipulability::getMaxManipulability()
    {
        return maxManip;
    }


    void Manipulability::addRandomTCPPoses(unsigned int loops, unsigned int numThreads, bool checkForSelfCollisions)
    {
        THROW_VR_EXCEPTION_IF(!data || !nodeSet || !tcpNode || !measure, "Workspace data not initialized");

        if (numThreads > loops)
        {
            VR_ERROR << "Number of threads can not be bigger then number of tcp poses to add.";
            return;
        }

        std::vector<std::thread> threads(numThreads);
        unsigned int numPosesPerThread = loops / numThreads; // todo

        for (size_t i = 0; i < numThreads; i++)
        {
            // each thread gets a cloned robot
            CollisionCheckerPtr cc(new CollisionChecker());
            RobotPtr clonedRobot = VirtualRobot::ModelIO::loadModel(this->robot->getFilename(), ModelIO::eCollisionModel);
            threads[i] = std::thread([=] ()
            {

                clonedRobot->setUpdateVisualization(false);
                JointSetPtr clonedNodeSet = clonedRobot->getJointSet(this->nodeSet->getName());
                FramePtr clonedTcpNode = clonedRobot->getFrame(this->tcpNode->getName());

                LinkSetPtr staticCollisionModel = this->staticCollisionModel;
                if (staticCollisionModel && clonedRobot->hasLinkSet(staticCollisionModel->getName()))
                {
                    staticCollisionModel = clonedRobot->getLinkSet(staticCollisionModel->getName());
                }

                LinkSetPtr dynamicCollisionModel = this->dynamicCollisionModel;
                if (dynamicCollisionModel && clonedRobot->hasLinkSet(dynamicCollisionModel->getName()))
                {
                    dynamicCollisionModel = clonedRobot->getLinkSet(dynamicCollisionModel->getName());
                }

                LinkSetPtr selfDistStatic = this->selfDistStatic;
                if (selfDistStatic && clonedRobot->hasLinkSet(selfDistStatic->getName()))
                {
                    selfDistStatic = clonedRobot->getLinkSet(selfDistStatic->getName());
                }

                LinkSetPtr selfDistDynamic = this->selfDistDynamic;
                if (selfDistDynamic && clonedRobot->hasLinkSet(selfDistDynamic->getName()))
                {
                    selfDistDynamic = clonedRobot->getLinkSet(selfDistDynamic->getName());
                }


                // and a cloned pose quality
                PoseQualityMeasurementPtr clonedMeasure  = this->measure->clone(clonedRobot);

                // now sample some configs and add them to the workspace data
                for (size_t j = 0; j < numPosesPerThread; j++)
                {
                    float rndValue;
                    float minJ, maxJ;
                    Eigen::VectorXf v(clonedNodeSet->getSize());
                    float maxLoops = 1000;

                    bool successfullyRandomized = false;

                    for (int k = 0; k < maxLoops; k++)
                    {
                        for (size_t l = 0; l < clonedNodeSet->getSize(); l++)
                        {
                            rndValue = RandomFloat(); // value from 0 to 1
                            minJ = nodeSet->getJoint(l)->getJointLimitLow();
                            maxJ = nodeSet->getJoint(l)->getJointLimitHigh();
                            v[l] = minJ + ((maxJ - minJ) * rndValue);
                        }

                        clonedNodeSet->setJointValues(v);

                        // check for collisions
                        if (!checkForSelfCollisions || !staticCollisionModel || !dynamicCollisionModel)
                        {
                            successfullyRandomized = true;
                            break;
                        }

                        if (!clonedRobot->getCollisionChecker()->checkCollision(staticCollisionModel, dynamicCollisionModel))
                        {
                            successfullyRandomized = true;
                            break;
                        }

                        this->collisionConfigs++;
                    }

                    if (successfullyRandomized)
                    {
                        Eigen::Matrix4f p = clonedTcpNode->getGlobalPose();
                        addPose(p, clonedMeasure, selfDistStatic, selfDistDynamic);
                    }
                    else
                    {
                        VR_WARNING << "Could not find collision-free configuration...";
                    }
                }
                VR_INFO << "Thread " << i << " finished" << std::endl;
            });
        }

        // wait for all threads to finish
        for (size_t i = 0; i < numThreads; i++)
        {
            threads[i].join();
        }
    }

} // namespace VirtualRobot
