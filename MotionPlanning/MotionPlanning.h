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
* @package    MotionPlanning
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _Saba_h_
#define _Saba_h_


/*! \defgroup MotionPlanning The Sampling-Based Motion Planning Library
The Sampling-based Motion Planning Library MotionPlanning offers state-of-the-art algorithms for planning collision-free motions
in high-dimensional configuration spaces. Standard approaches, as the Rapidly-exploring Random Trees (RRT), are efficiently implemented.
Furthermore motion planning algorithms for grasping and for bimanual tasks are provided, which can be used to solve plannign problems for
mobile manipulators or service and humanoid robots.
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
#  if defined(simox_motionplanning_EXPORTS)
#    define MOTIONPLANNING_IMPORT_EXPORT __declspec(dllexport)
#  else
#    define MOTIONPLANNING_IMPORT_EXPORT __declspec(dllimport)
#  endif
#else
#  define SABA_IMPORT_EXPORT
#endif


namespace MotionPlanning
{
    // only valid within the MotionPlanning namespace
    using std::cout;
    using std::endl;

    class CSpace;
    class CSpaceSampled;
    class CSpacePath;
    class CSpaceTree;
    class CSpaceNode;
    class Sampler;
    class ConfigurationConstraint;
    class Rrt;
    class MotionPlanner;
    class BiRrt;
    class GraspIkRrt;
    class GraspRrt;
    class PathProcessor;
    class ShortcutProcessor;
    class ElasticBandProcessor;
    class ApproachDiscretization;
    class PlanningThread;
    class PathProcessingThread;

    typedef std::shared_ptr<CSpace> CSpacePtr;
    typedef std::shared_ptr<CSpaceSampled> CSpaceSampledPtr;
    typedef std::shared_ptr<CSpacePath> CSpacePathPtr;
    typedef std::shared_ptr<Sampler> SamplerPtr;
    typedef std::shared_ptr<CSpaceTree> CSpaceTreePtr;
    typedef std::shared_ptr<CSpaceNode> CSpaceNodePtr;
    typedef std::shared_ptr<MotionPlanner> MotionPlannerPtr;
    typedef std::shared_ptr<Rrt> RrtPtr;
    typedef std::shared_ptr<BiRrt> BiRrtPtr;
    typedef std::shared_ptr<GraspIkRrt> GraspIkRrtPtr;
    typedef std::shared_ptr<GraspRrt> GraspRrtPtr;
    typedef std::shared_ptr<PathProcessor> PathProcessorPtr;
    typedef std::shared_ptr<ShortcutProcessor> ShortcutProcessorPtr;
    typedef std::shared_ptr<ElasticBandProcessor> ElasticBandProcessorPtr;
    typedef std::shared_ptr<ConfigurationConstraint> ConfigurationConstraintPtr;
    typedef std::shared_ptr<ApproachDiscretization> ApproachDiscretizationPtr;
    typedef std::shared_ptr<PlanningThread> PlanningThreadPtr;
    typedef std::shared_ptr<PathProcessingThread> PathProcessingThreadPtr;

#define MOTIONPLANNING_INFO VR_INFO
#define MOTIONPLANNING_WARNING VR_WARNING
#define MOTIONPLANNING_ERROR VR_ERROR

#define THROW_MOTIONPLANNING_EXCEPTION(a) THROW_VR_EXCEPTION(a)

#ifdef NDEBUG
#define MOTIONPLANNING_ASSERT(a)
#define MOTIONPLANNING_ASSERT_MESSAGE(a,b)
#else
#define MOTIONPLANNING_ASSERT(a) BOOST_ASSERT(a);
	//if (!(a)) {cout << "ASSERT failed (" << #a <<")"<<endl; THROW_SABA_EXCEPTION( "ASSERT failed (" << #a << ")" )};
#define MOTIONPLANNING_ASSERT_MESSAGE(a,b) BOOST_ASSERT_MSG(a,b);
	//if (!(a)) {cout << "ASSERT failed (" << #a <<"): "<<b<<endl; THROW_SABA_EXCEPTION( "ASSERT failed (" << #a << "): " << b )};
#endif

}

#endif // _Saba_h_
