
#include "ElasticBandProcessor.h"
#include "MotionPlanning/CSpace/CSpaceSampled.h"
#include "MotionPlanning/CSpace/CSpacePath.h"
#include <vector>
#include <time.h>
#include <math.h>

namespace Saba
{
using namespace VirtualRobot;

    ElasticBandProcessor::ElasticBandProcessor(CSpacePathPtr path,
                                               CSpaceSampledPtr cspace,
                                               VirtualRobot::RobotNodePtr node,               // the distance for this node is considered
                                               VirtualRobot::SceneObjectSetPtr obstacles,     // these obstacles are considered for path smoothing
                                               bool verbose
                                               ) : PathProcessor(path, verbose), cspace(cspace), node(node), obstacles(obstacles)
    {
        VR_ASSERT(node);
        VR_ASSERT(node->getCollisionModel());
        VR_ASSERT(path);
        VR_ASSERT(path->getCSpace());
        VR_ASSERT(path->getCSpace()->getModelNodeSet());
        VR_ASSERT(obstacles);
        VR_ASSERT(cspace);

        rns = path->getCSpace()->getModelNodeSet();
        VR_ASSERT(rns && rns->getSize()>0);
        VR_INFO << "using rns " << rns->getName() << endl;

        stopOptimization = false;
        colChecker = node->getRobot()->getCollisionChecker();

        factorCSpaceNeighborForce = 0.3f;
        factorCSpaceObstacleForce = 10000.0f;
        maxCSpaceNeighborForce = 10.0f;
        maxCSpaceObstacleForce = 10.0f;

        // ignore obstacles far away
        minObstacleDistance = 300.0f;

        weights.resize(rns->getSize());
        weights.setConstant(1.0f);
    }

    ElasticBandProcessor::~ElasticBandProcessor()
    {
    }

    bool ElasticBandProcessor::initSolution()
    {
        if (!path)
        {
            VR_ERROR << "Wrong parameters or no path to smooth..." << std::endl;
            return false;
        }

        optimizedPath = path->clone();

        if (!optimizedPath)
        {
            VR_ERROR << "Wrong parameters or no path to smooth..." << std::endl;
            return false;
        }

        ik.reset(new GenericIKSolver(rns));

        return true;
    }

    bool ElasticBandProcessor::getObstacleForce(Eigen::Vector3f& f)
    {
        Eigen::Vector3f _P1;
        Eigen::Vector3f _P2;
        int _trID1;
        int _trID2;

        if (!obstacles)
        {
            f.setZero();
            return true;
        }


        float d = (float)colChecker->calculateDistance(node->getCollisionModel(), obstacles, _P1, _P2, &_trID1, &_trID2);


        if (d>minObstacleDistance)
        {
            f.setZero();
            return true;
        }

        if (verbose)
        {
            VR_INFO << "Obstacle dist:" << d << endl;
        }
        if (d==0)
        {
            VR_ERROR << "Collision..." << endl;
            return false;
        }

        d = 1/d;


        //f = _P2 - _P1;
        f = _P1 - _P2;
        f.normalize();
        f*= d;

        return true;
    }


    bool ElasticBandProcessor::getCSpaceForce(const Eigen::Vector3f& f, Eigen::VectorXf &fc, float factor, float maxForce)
    {
        if (ik)
        {
            Eigen::MatrixXf ijac = ik->getDifferentialIK()->getPseudoInverseJacobianMatrix();
            fc = ijac.leftCols<3>() * f; // only force
        }
        else
            fc.setZero();

        fc *= factor; //factorCSpaceObstacleForce;

        if (fc.norm() > maxForce)
        {
            fc.normalize();
            fc *= maxForce;
        }

        return true;
    }
    
    
    bool ElasticBandProcessor::getWSpaceForce(const Eigen::VectorXf& fc, Eigen::Vector3f &f)
    {
        if (ik)
        {
            Eigen::MatrixXf jac = ik->getDifferentialIK()->getJacobianMatrix(rns->getTCP(),IKSolver::Position);
            f = jac * fc; // only force
        }
        else
            f.setZero();

        return true;
    }

    bool ElasticBandProcessor::getNeighborCForce(const Eigen::VectorXf& before, const Eigen::VectorXf& act, const Eigen::VectorXf& next, Eigen::VectorXf &fc)
    {
        Eigen::VectorXf f1 = (before - act);
        Eigen::VectorXf f2 = (next - act);

        fc = (f1 + f2) * 0.5f;

        fc *= factorCSpaceNeighborForce;
        float d = fc.norm();
        if (d > maxCSpaceNeighborForce)
        {
            fc.normalize();
            fc *= maxCSpaceNeighborForce;
        }

        return true;
    }

    bool ElasticBandProcessor::elasticBandLoop()
    {
         Eigen::Vector3f fObstacle;
         Eigen::VectorXf fcObstacle;
         Eigen::VectorXf fcNeighbor;
         Eigen::VectorXf fc;


        for (int i=1;i<optimizedPath->getNrOfPoints()-1;i++)
        {

            Eigen::VectorXf& before = optimizedPath->getPointRef(i-1);
            Eigen::VectorXf& act = optimizedPath->getPointRef(i);
            Eigen::VectorXf& next = optimizedPath->getPointRef(i+1);

           rns->setJointValues(act);

           if (!getObstacleForce(fObstacle))
           {
               VR_WARNING << "Could not get obstacle force " << i << endl;
               continue;
           }
           if (verbose)
               VR_INFO << "obstacle workspace force: " << fObstacle.transpose() << endl;
           if (!getCSpaceForce(fObstacle, fcObstacle, factorCSpaceObstacleForce, maxCSpaceObstacleForce))
           {
               VR_WARNING << "Could not get obstacle cspace force " << i << endl;
               continue;
           }
           if (verbose)
               VR_INFO << "obstacle c-space force: " << fcObstacle.transpose() << endl;

           getNeighborCForce(before, act, next, fcNeighbor);
           if (verbose)
           {
               Eigen::Vector3f fn;
               getWSpaceForce(fcNeighbor, fn);
               VR_INFO << "neighbor workspace force: " << fn.transpose() << endl;
               VR_INFO << "neighbor c-space force: " << fcNeighbor.transpose() << endl;
           }
           fc = fcObstacle + fcNeighbor;

           fc = fc.cwiseProduct(weights);
           if (verbose)
               VR_INFO << "resulting c-space force: " << fc.transpose() << endl;

           optimizedPath->getPointRef(i) += fc;
        }
        return true;
    }


    bool ElasticBandProcessor::checkRemoveNodes()
    {
        /*Eigen::VectorXf& before = optimizedPath->getPointRef(0);
        Eigen::VectorXf& act = optimizedPath->getPointRef(0);
        Eigen::VectorXf& next = optimizedPath->getPointRef(1);*/

        float ssd = 2 * cspace->getSamplingSize()*cspace->getSamplingSize();

        for (int i=1;i<optimizedPath->getNrOfPoints()-2;i++)
        {
            Eigen::VectorXf& before = optimizedPath->getPointRef(i-1);
            Eigen::VectorXf& act = optimizedPath->getPointRef(i);
            Eigen::VectorXf& next = optimizedPath->getPointRef(i+1);

            float d1 = cspace->calcDist2(before, act);
            float d2 = cspace->calcDist2(act, next);

            if (d1+d2 < ssd)
            {
                optimizedPath->erasePosition(i);
                before = optimizedPath->getPointRef(i-1);
                act = optimizedPath->getPointRef(i);
                next = optimizedPath->getPointRef(i+1);
            }

        }
        return true;
    }

    bool ElasticBandProcessor::checkNewNodes()
    {

        return true;
    }
    
    Eigen::Vector3f ElasticBandProcessor::getWSpacePoint(const Eigen::VectorXf& fc)
    {
		rns->setJointValues(fc);
        Eigen::Vector3f r = rns->getTCP()->getGlobalPose().block(0,3,3,1);
        return r;
	}

    void ElasticBandProcessor::getForces(unsigned int i, Eigen::Vector3f &internalForce, Eigen::Vector3f &externalForce)
    {
        Eigen::VectorXf fcExt;
        if (!optimizedPath || optimizedPath->getNrOfPoints()<=i)
        {
            VR_WARNING << "no path or wrong index" << endl;
            internalForce.setZero();
            externalForce.setZero();
            return;
        }

        unsigned int a = 0;
        if (i>0)
            a=i-1;
        unsigned int b = i;
        unsigned int c = i;
        if (i<optimizedPath->getNrOfPoints()-1)
            a=i+1;

        Eigen::VectorXf& before = optimizedPath->getPointRef(a);
        Eigen::VectorXf& act = optimizedPath->getPointRef(b);
        Eigen::VectorXf& next = optimizedPath->getPointRef(c);
        Eigen::VectorXf fcNeighbor;

        rns->setJointValues(act);

        if (!getObstacleForce(externalForce))
        {
           VR_WARNING << "Could not get obstacle force" << endl;
        }

        // wee need to apply factors and max -> transform to cspace and back to wspace
        getCSpaceForce(externalForce, fcExt, factorCSpaceObstacleForce, maxCSpaceObstacleForce);
        getWSpaceForce(fcExt, externalForce);

        getNeighborCForce(before, act, next, fcNeighbor);
        if (!getWSpaceForce(fcNeighbor, internalForce))
        {
           VR_WARNING << "Could not get internal cspace force " << endl;
        }
    }



    CSpacePathPtr ElasticBandProcessor::optimize(int optimizeSteps)
    {
        initSolution();

        rns->getRobot()->setUpdateVisualization(false);

        for (int i=0;i<optimizeSteps;i++)
        {
            if (!elasticBandLoop())
            {
                VR_ERROR << "Error in loop, aborting..." << endl;
                break;
            }
            if (!checkRemoveNodes())
            {
                VR_ERROR << "Error in remove, aborting..." << endl;
                break;
            }
            if (!checkNewNodes())
            {
                VR_ERROR << "Error in add, aborting..." << endl;
                break;
            }
        }

        rns->getRobot()->setUpdateVisualization(true);


        return optimizedPath;
    }

    void ElasticBandProcessor::setWeights(Eigen::VectorXf w)
    {
        weights = w;
    }


} // namespace
