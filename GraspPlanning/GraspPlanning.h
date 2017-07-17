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
* @package    GraspPlanning
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _GraspPlanning_h_
#define _GraspPlanning_h_


/*! \defgroup GraspPlanning The Grasp Planning Library
GraspPlanning offers algorithms for grasp scoring and planning covering 3D force space calculations and a full
implementation of the 6d wrench space algorithm for grasp quality measurement.
*/

#ifdef WIN32

// needed to have M_PI etc defined
#if !defined(_USE_MATH_DEFINES)
#define _USE_MATH_DEFINES
#endif

// eigen wants this on windows
#if !defined(NOMINMAX)
#define NOMINMAX
#endif

#endif

#include "VirtualRobot/Model/Model.h"
#include "VirtualRobot/VirtualRobotException.h"

#include <iostream>
#include <sstream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

#ifdef WIN32
#  include <winsock2.h>
#  include <windows.h>
#  pragma warning ( disable : 4251 )
#  if defined(simox_graspplanning_EXPORTS)
#    define GRASPPLANNING_IMPORT_EXPORT __declspec(dllexport)
#  else
#    define GRASPPLANNING_IMPORT_EXPORT __declspec(dllimport)
#  endif
#else
#  define GRASPPLANNING_IMPORT_EXPORT
#endif


namespace GraspPlanning
{
    // only valid within the GraspPlanning namespace
    using std::cout;
    using std::endl;

    class GraspQualityMeasure;
    class GraspQualityMeasureWrenchSpace;
    class ContactConeGenerator;

    class GraspQualityMeasure;
    class ApproachMovementGenerator;
    class ApproachMovementSurfaceNormal;
    class GraspPlanner;
    class GenericGraspPlanner;

    typedef std::shared_ptr<GraspQualityMeasure> GraspQualityMeasurePtr;
    typedef std::shared_ptr<GraspQualityMeasureWrenchSpace> GraspQualityMeasureWrenchSpacePtr;
    typedef std::shared_ptr<ContactConeGenerator> ContactConeGeneratorPtr;
    typedef std::shared_ptr<GraspQualityMeasure> GraspQualityMeasurePtr;
    typedef std::shared_ptr<ApproachMovementGenerator> ApproachMovementGeneratorPtr;
    typedef std::shared_ptr<ApproachMovementSurfaceNormal> ApproachMovementSurfaceNormalPtr;
    typedef std::shared_ptr<GraspPlanner> GraspPlannerPtr;
    typedef std::shared_ptr<GenericGraspPlanner> GenericGraspPlannerPtr;

#define GRASPPLANNING_INFO VR_INFO
#define GRASPPLANNING_WARNING VR_WARNING
#define GRASPPLANNING_ERROR VR_ERROR

#define THROW_GRASPPLANNING_EXCEPTION(a) THROW_VR_EXCEPTION(a)

#ifdef NDEBUG
#define GRASPPLANNING_ASSERT(a)
#define GRASPPLANNING_ASSERT_MESSAGE(a,b)

#else
#define GRASPPLANNING_ASSERT(a) if (!(a)) {cout << "ASSERT failed (" << #a <<")"<<endl; THROW_GRASPPLANNING_EXCEPTION( "ASSERT failed (" << #a << ")" )};
#define GRASPPLANNING_ASSERT_MESSAGE(a,b) if (!(a)) {cout << "ASSERT failed (" << #a <<"): "<<b<<endl; THROW_GRASPPLANNING_EXCEPTION( "ASSERT failed (" << #a << "): " << b )};
#endif

}

#endif
