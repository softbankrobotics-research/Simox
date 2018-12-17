/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    GraspStudio
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#pragma once

#include "../GraspStudio.h"
#include "GraspQualityMeasure.h"

#include <Eigen/Core>

namespace GraspStudio
{

    /*!
        \brief An efficient implementation of the grasp wrench space algorithm for grasp quality evaluation.

        the grasp wrench space algorithm is widely used in the context of grasp planning. By analyzing
        the grasp wrench space (GWS) of a given set of contact points, a quality score of a grasp
        can be evaluated.
        In this implementation, additionally an object specific wrench space (WS) is calculated, which
        approximatevly represents a "perfect" grasp. This object is used to normalize the quality score.
    */
    class GRASPSTUDIO_IMPORT_EXPORT GraspQualityMeasureWrenchSpaceNotNormalized : public GraspQualityMeasure
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        //inherit ctor
        using GraspQualityMeasure::GraspQualityMeasure;


        //! This method is used to compute a reference value that describes a perfect grasp
        bool calculateObjectProperties() override { return true; }

        /*!
            Returns f_max_gws
            with f_max_gws = max distance of GWS hull center to one of its facets
            -> also known as "epsilon" quality == radius of larges enclosing 6D ball
        */
        float getGraspQuality() override;

        /*!
            Volume grasp quality ratio of GWS volume
            -> also known as "v" quality
        */
        virtual float getVolumeGraspMeasure();

        /*
            Checks if wrench space origin is inside GWS-Hull
        */
        bool isGraspForceClosure() override;

        /*
            Returns the internally calculated convex hull object (GraspWrenchSpace)
            This hull depends on the contacts
        */
        virtual VirtualRobot::MathTools::ConvexHull6DPtr getConvexHullGWS()
        {
            return convexHullGWS;
        }

        void updateGWS();

        VirtualRobot::MathTools::ContactPoint getCenterGWS()
        {
            return convexHullCenterGWS;
        }

        /*!
        setup contact information
        the contact points are normalized by subtracting the COM
        the contact normals are normalize to unit length
        */
        void setContactPoints(const std::vector<VirtualRobot::MathTools::ContactPoint>& contactPoints) override;
        void setContactPoints(const VirtualRobot::EndEffector::ContactInfoVector& contactPoints) override;

        bool calculateGraspQuality() override;

        //! Returns description of this object
        std::string getName() override;


        static std::vector<VirtualRobot::MathTools::ContactPoint> createWrenchPoints(std::vector<VirtualRobot::MathTools::ContactPoint>& points, const Eigen::Vector3f& centerOfModel, float objectLengthMM);

        //! Goes through all facets of convex hull and searches the minimum distance to it's center
        static float minOffset(VirtualRobot::MathTools::ConvexHull6DPtr ch);

    protected:

        //Methods
        VirtualRobot::MathTools::ConvexHull6DPtr calculateConvexHull(std::vector<VirtualRobot::MathTools::ContactPoint>& points);
        VirtualRobot::MathTools::ContactPoint calculateHullCenter(VirtualRobot::MathTools::ConvexHull6DPtr hull);

        float minDistanceToGWSHull(VirtualRobot::MathTools::ContactPoint& point);


        bool isOriginInGWSHull();
        void printContacts(std::vector<VirtualRobot::MathTools::ContactPoint>& points);
        static Eigen::Vector3f crossProductPosNormalInv(const VirtualRobot::MathTools::ContactPoint& v1);

        //For safety
        bool GWSCalculated {false};

        //For Object and Grasp Wrench Space Calculation
        VirtualRobot::MathTools::ConvexHull6DPtr convexHullGWS;
        VirtualRobot::MathTools::ContactPoint convexHullCenterGWS;
    };
}

