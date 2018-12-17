// **************************************************************
// Implementation of class GraspQualityMeasure
// **************************************************************
// Author: Niko Vahrenkamp
// Date: 26.10.2011
// **************************************************************


// **************************************************************
// includes
// **************************************************************

#include "GraspQualityMeasureWrenchSpaceNotNormalized.h"
#include <cmath>
#include <cstdio>
#include <cassert>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <cfloat>


using namespace std;
using namespace VirtualRobot;

namespace GraspStudio
{

    void GraspQualityMeasureWrenchSpaceNotNormalized::setContactPoints(const std::vector<VirtualRobot::MathTools::ContactPoint>& contactPoints)
    {
        GraspQualityMeasure::setContactPoints(contactPoints);
        GWSCalculated = false;
    }

    void GraspQualityMeasureWrenchSpaceNotNormalized::setContactPoints(const VirtualRobot::EndEffector::ContactInfoVector& contactPoints)
    {
        GraspQualityMeasure::setContactPoints(contactPoints);
        GWSCalculated = false;
    }


    float GraspQualityMeasureWrenchSpaceNotNormalized::getGraspQuality()
    {
        calculateGraspQuality();
        return graspQuality;
    }

    bool GraspQualityMeasureWrenchSpaceNotNormalized::isGraspForceClosure()
    {
        updateGWS();
        return isOriginInGWSHull();
    }

    void GraspQualityMeasureWrenchSpaceNotNormalized::updateGWS()
    {
        if (GWSCalculated)
        {
            return;
        }
        bool printAll = false;

        if (contactPointsM.empty())
        {
            if (verbose && printAll)
                printf("Contact points not set.\n");
            return;
        }

        //Rotate generic friction cone to align with object normals
        std::vector<VirtualRobot::MathTools::ContactPoint>::iterator objPointsIter;
        std::vector<VirtualRobot::MathTools::ContactPoint> conePoints;

        if (verbose && printAll)
        {
            cout << "GWS contact points:" << endl;
        }

        for (objPointsIter = contactPointsM.begin(); objPointsIter != contactPointsM.end(); objPointsIter++)
        {
            if (verbose && printAll)
            {
                MathTools::print((*objPointsIter));
            }

            coneGenerator->computeConePoints((*objPointsIter), conePoints);
        }

        //Generate convex hull from rotated contact friction cones
        convexHullGWS = calculateConvexHull(conePoints);
        //calculateHullCenter(convexHullGWS, convexHullCenterGWS);
        convexHullCenterGWS = convexHullGWS->center;

        if (verbose && printAll)
        {
            GRASPSTUDIO_INFO << " CENTER of GWS: " << endl;
            MathTools::print(convexHullCenterGWS);
        }

        GWSCalculated = true;
    }

    VirtualRobot::MathTools::ConvexHull6DPtr GraspQualityMeasureWrenchSpaceNotNormalized::calculateConvexHull(std::vector<VirtualRobot::MathTools::ContactPoint>& points)
    {
        bool printAll = true;

        if (verbose && printAll)
        {
            cout << "Convex hull points for wrench calculation:" << endl;
            printContacts(points);
        }

        // create wrench
        std::vector<VirtualRobot::MathTools::ContactPoint> wrenchPoints = createWrenchPoints(points, Eigen::Vector3f::Zero(), objectLength); // contact points are already moved so that com is at origin

        if (verbose && printAll)
        {
            cout << "Wrench points:" << endl;
            printContacts(wrenchPoints);
        }

        return ConvexHullGenerator::CreateConvexHull(wrenchPoints);
    }

    std::vector<VirtualRobot::MathTools::ContactPoint> GraspQualityMeasureWrenchSpaceNotNormalized::createWrenchPoints(std::vector<VirtualRobot::MathTools::ContactPoint>& points, const Eigen::Vector3f& centerOfModel, float objectLengthMM)
    {
        std::vector<VirtualRobot::MathTools::ContactPoint> result;
        std::vector<VirtualRobot::MathTools::ContactPoint>::iterator iter = points.begin();
        VirtualRobot::MathTools::ContactPoint p;
        Eigen::Vector3f normal;

        // convert objLength from mm to m
        bool convertMM2M = true;

        float factor = 1.0f;

        if (objectLengthMM != 0)
        {
            if (convertMM2M)
            {
                factor = 2.0f / (objectLengthMM / 1000.0f);    // == max distance from center ( 1 / (length/2) )
            }
            else
            {
                factor = 2.0f / objectLengthMM;    // == max distance from center ( 1 / (length/2) )
            }
        }

        VirtualRobot::MathTools::ContactPoint tmpP;

        while (iter != points.end())
        {
#ifdef INVERT_NORMALS
            /*p.p(0) = -(*iter).n(0);
            p.p(1) = -(*iter).n(1);
            p.p(2) = -(*iter).n(2);*/
            p.p = -1.0f * (iter->n);
#else
            p.p = iter->n;
#endif
            tmpP.p = iter->p - centerOfModel;
            tmpP.n = -(iter->n);

            //normal = crossProductPosNormalInv(tmpP);
            //p.n = factor * normal;
            p.n = factor * tmpP.p.cross(tmpP.n);

            result.push_back(p);
            iter++;
        }

        return result;
    }

    void GraspQualityMeasureWrenchSpaceNotNormalized::printContacts(std::vector<VirtualRobot::MathTools::ContactPoint>& points)
    {
        std::vector<VirtualRobot::MathTools::ContactPoint>::iterator iter = points.begin();

        while (iter != points.end())
        {
            cout << "# ";
            MathTools::print((*iter));
            iter++;
        }
    }

    VirtualRobot::MathTools::ContactPoint GraspQualityMeasureWrenchSpaceNotNormalized::calculateHullCenter(VirtualRobot::MathTools::ConvexHull6DPtr hull)
    {
        if (!hull)
        {
            GRASPSTUDIO_ERROR << "NULL data?!" << endl;
            return VirtualRobot::MathTools::ContactPoint();
        }

        VirtualRobot::MathTools::ContactPoint resultCenter;
        std::vector<VirtualRobot::MathTools::ContactPoint>::iterator iter;
        resultCenter.p.setZero();
        resultCenter.n.setZero();

        if (hull->vertices.size() == 0)
        {
            cout << __FUNCTION__ << ": error, no vertices..." << endl;
            return resultCenter;
        }

        for (iter = hull->vertices.begin(); iter != hull->vertices.end(); iter++)
        {
            resultCenter.p += iter->p;
            resultCenter.n += iter->n;
        }

        resultCenter.p /= (float)hull->vertices.size();
        resultCenter.n /= (float)hull->vertices.size();
        return resultCenter;
    }

    float GraspQualityMeasureWrenchSpaceNotNormalized::minDistanceToGWSHull(VirtualRobot::MathTools::ContactPoint& point)
    {
        float minDist = FLT_MAX;
        float dist[6];
        float currentDist2;
        std::vector<MathTools::TriangleFace6D>::iterator faceIter;

        for (faceIter = convexHullGWS->faces.begin(); faceIter != convexHullGWS->faces.end(); faceIter++)
        {
            VirtualRobot::MathTools::ContactPoint faceCenter;
            faceCenter.p.setZero();
            faceCenter.n.setZero();

            for (int j : faceIter->id)
            {
                faceCenter.p += (convexHullGWS->vertices)[j].p;
                faceCenter.n += (convexHullGWS->vertices)[j].n;
            }

            faceCenter.p /= 6.0f;
            faceCenter.n /= 6.0f;

            currentDist2 = 0;

            for (int j = 0; j < 3; j++)
            {
                dist[j] = (faceCenter.p(j) - point.p(j));
                dist[j + 3] = (faceCenter.n(j) - point.n(j));
                currentDist2 += dist[j] * dist[j];
                currentDist2 += dist[j + 3] * dist[j + 3];
            }

            if (currentDist2 < minDist)
            {
                minDist = currentDist2;
            }
        }

        return sqrtf(minDist);
    }
    bool GraspQualityMeasureWrenchSpaceNotNormalized::isOriginInGWSHull()
    {
        if (!GWSCalculated || !convexHullGWS)
        {
            return false;
        }

        for (const auto& face : convexHullGWS->faces)
        {
            // ignore rounding errors
            if (face.distPlaneZero > 1e-4)
            {
                return false;
            }
        }

        return true;
    }



    float GraspQualityMeasureWrenchSpaceNotNormalized::minOffset(VirtualRobot::MathTools::ConvexHull6DPtr ch)
    {
        if (!ch)
        {
            return 0.0f;
        }

        float fRes = FLT_MAX;
        int nWrongFacets = 0;

        for (auto & face : ch->faces)
        {
            const auto dist = face.distNormCenter;
            if (dist > 0)
            {
                //outside
                nWrongFacets++;
                continue;
            }
            fRes = std::max(fRes, -dist);
        }

        if (nWrongFacets > 0)
        {
            cout << __FUNCTION__ << " Warning: offset of " << nWrongFacets << " facets >0 (# of facets:" << ch->faces.size() << ")" << endl;
        }

        return fRes;
    }

    float GraspQualityMeasureWrenchSpaceNotNormalized::getVolumeGraspMeasure()
    {
        updateGWS();

        if (!convexHullGWS || convexHullGWS->vertices.size() == 0)
        {
            cout << __FUNCTION__ << "No vertices in Grasp Wrench Space?! Maybe I was not initialized correctly..." << endl;
            return 0.0;
        }
        return convexHullGWS->volume;
    }

    bool GraspQualityMeasureWrenchSpaceNotNormalized::calculateGraspQuality()
    {
        updateGWS();

        graspQuality = 0.0f;

        if (!convexHullGWS || convexHullGWS->vertices.size() == 0)
        {
            cout << __FUNCTION__ << "No vertices in Grasp Wrench Space?! Maybe I was not initialized correctly..." << endl;
            return 0.0;
        }

        float fResOffsetGWS = minOffset(convexHullGWS);

        graspQuality = fResOffsetGWS;

        if (verbose)
        {
            GRASPSTUDIO_INFO << endl;
            cout << ": GWS volume    : " << convexHullGWS->volume << endl;
            cout << ": GWS min Offset: " << fResOffsetGWS << endl;
            cout << ": GraspQuality  : " << graspQuality << endl;
        }

        return true;
    }

    std::string GraspQualityMeasureWrenchSpaceNotNormalized::getName()
    {
        return "GraspWrenchSpaceNotNormalized";
    }


    Eigen::Vector3f GraspQualityMeasureWrenchSpaceNotNormalized::crossProductPosNormalInv(const VirtualRobot::MathTools::ContactPoint& v1)
    {
        Eigen::Vector3f res;
        res(0) = v1.p(1) * (-v1.n(2)) - v1.p(2) * (-v1.n(1));
        res(1) = v1.p(2) * (-v1.n(0)) - v1.p(0) * (-v1.n(2));
        res(2) = v1.p(0) * (-v1.n(1)) - v1.p(1) * (-v1.n(0));
        return res;
    }

}
