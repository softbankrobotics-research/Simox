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
 * @author     Simon Ottenhaus (simon dot ottenhaus at kit dot edu)
 * @copyright  2018 Simon Ottenhaus
 *             GNU Lesser General Public License
 */

#pragma once

//scale intern sizes to milimeters
#define HAPTIC_EXPLORATION_SCALE 40

#ifndef Q_MOC_RUN // workaround for some bug in some QT/boost versions
#include <boost/shared_ptr.hpp>
#endif
#include <Eigen/Dense>
#include <stdexcept>
#include <vector>

template<class T> class Nullable {
public:
    Nullable(T& value)
        : defined(true), value(value)
    { }

    Nullable()
        : defined(false)
    { }

    T get()
    {
        if(!defined)
        {
            throw std::exception("Value is not set");
        }
        return value;
    }
    bool hasValue()
    {
        return defined;
    }

private:
    bool defined;
    T value;
};



namespace math{


    //typedef Eigen::Vector2f Eigen::Vector2f;
    //typedef Eigen::Vector3f Eigen::Vector3f;
    //typedef Eigen::Matrix3f Matrix3f;
    //typedef Eigen::MatrixXf MatrixXf;
    //typedef Eigen::VectorXf VectorXf;
    //typedef Nullable<float> floatOpt;
    typedef Nullable<Eigen::Vector3f> Vec3Opt;

    typedef boost::shared_ptr<class AbstractFunctionR1R2> AbstractFunctionR1R2Ptr;
    typedef boost::shared_ptr<class Line> LinePtr;
    typedef boost::shared_ptr<class LineStrip> LineStripPtr;
    typedef boost::shared_ptr<class AbstractFunctionR1R3> AbstractFunctionR1R3Ptr;
    typedef boost::shared_ptr<class AbstractFunctionR1R6> AbstractFunctionR1R6Ptr;
    typedef boost::shared_ptr<class AbstractFunctionR2R3> AbstractFunctionR2R3Ptr;
    typedef boost::shared_ptr<class AbstractFunctionR3R1> AbstractFunctionR3R1Ptr;
    typedef boost::shared_ptr<class Contact> ContactPtr;
    typedef boost::shared_ptr<class ContactList> ContactListPtr;
    typedef boost::shared_ptr<class ImplicitPlane> ImplicitPlanePtr;
    typedef boost::shared_ptr<class LineR2> LineR2Ptr;
    typedef boost::shared_ptr<class Plane> PlanePtr;
    typedef boost::shared_ptr<class Triangle> TrianglePtr;
    typedef boost::shared_ptr<class SimpleAbstractFunctionR1R3> SimpleAbstractFunctionR1R3Ptr;
    typedef boost::shared_ptr<class SimpleAbstractFunctionR1R6> SimpleAbstractFunctionR1R6Ptr;
    typedef boost::shared_ptr<class SimpleAbstractFunctionR2R3> SimpleAbstractFunctionR2R3Ptr;
    typedef boost::shared_ptr<class SimpleAbstractFunctionR3R1> SimpleAbstractFunctionR3R1Ptr;
    typedef boost::shared_ptr<class LinearInterpolatedOrientation> LinearInterpolatedOrientationPtr;
    typedef boost::shared_ptr<class AbstractFunctionR1Ori> AbstractFunctionR1OriPtr;
    typedef boost::shared_ptr<class SimpleAbstractFunctionR1Ori> SimpleAbstractFunctionR1OriPtr;
    typedef boost::shared_ptr<class CompositeFunctionR1R6> CompositeFunctionR1R6Ptr;
    typedef boost::shared_ptr<class AbstractFunctionR1R6> AbstractFunctionR1R6Ptr;
    typedef boost::shared_ptr<class ImplicitObjectModel> ImplicitObjectModelPtr;
    typedef boost::shared_ptr<class HalfSpaceObjectModel> HalfSpaceObjectModelPtr;
    typedef boost::shared_ptr<class HalfSpaceImplicitSurface3D> HalfSpaceImplicitSurface3DPtr;
    typedef boost::shared_ptr<class GaussianObjectModel> GaussianObjectModelPtr;
    typedef boost::shared_ptr<class GaussianObjectModelNormals> GaussianObjectModelNormalsPtr;
    typedef boost::shared_ptr<class GaussianImplicitSurface3D> GaussianImplicitSurface3DPtr;
    typedef boost::shared_ptr<class GaussianImplicitSurface3DNormals> GaussianImplicitSurface3DNormalsPtr;
    typedef boost::shared_ptr<class GaussianImplicitSurface3DCombined> GaussianImplicitSurface3DCombinedPtr;
    typedef boost::shared_ptr<class DataR3R1> DataR3R1Ptr;
    typedef boost::shared_ptr<class DataR3R2> DataR3R2Ptr;
    typedef boost::shared_ptr<class MarchingCubes> MarchingCubesPtr;
    typedef boost::shared_ptr<class Bezier> BezierPtr;
    typedef boost::shared_ptr<class LinearContinuedBezier> LinearContinuedBezierPtr;
    typedef boost::shared_ptr<class Primitive> PrimitivePtr;
    typedef boost::shared_ptr<struct Index3> Index3Ptr;
    typedef boost::shared_ptr<class AbstractContactFeature> AbstractContactFeaturePtr;
    typedef boost::shared_ptr<class BinContactFeature> BinContactFeaturePtr;
    template<class T> class Array3D;
    typedef boost::shared_ptr<Array3D<float>> Array3DFloatPtr;
    typedef boost::shared_ptr<Array3D<bool>> Array3DBoolPtr;
    //typedef boost::shared_ptr<Array3D<>> Array3DPtr<T>;
    typedef boost::shared_ptr<class VoronoiWeights> VoronoiWeightsPtr;
    struct WeightedFloatAverage;
    class WeightedVec3Average;

    typedef boost::shared_ptr<class Grid3D> Grid3DPtr;
    typedef boost::shared_ptr<class GridCacheFloat3> GridCacheFloat3Ptr;
    typedef boost::shared_ptr<class FeatureCluster> FeatureClusterPtr;
    typedef boost::shared_ptr<class EdgeCluster> EdgeClusterPtr;
    typedef boost::shared_ptr<class EdgePredictor> EdgePredictorPtr;
    typedef boost::shared_ptr<class EdgeTracer> EdgeTracerPtr;
    typedef boost::shared_ptr<std::vector<Eigen::Vector3f>> Vec3ListPtr;
    typedef boost::shared_ptr<class Edge> EdgePtr;
    typedef boost::shared_ptr<class EdgeFeature> EdgeFeaturePtr;


}

namespace sim{
    class Simulation;
    typedef boost::shared_ptr<Simulation> SimulationPtr;
    class HapticExplorationData;
    typedef boost::shared_ptr<HapticExplorationData> HapticExplorationDataPtr;

    namespace objects{

    class AbstractObject;
    typedef boost::shared_ptr<AbstractObject> AbstractObjectPtr;
    class ImplicitObject;
    typedef boost::shared_ptr<ImplicitObject> ImplicitObjectPtr;
    class InfiniteObject;
    typedef boost::shared_ptr<InfiniteObject> InfiniteObjectPtr;
    class CompositeObject;
    typedef boost::shared_ptr<CompositeObject> CompositeObjectPtr;
    class Sphere;
    typedef boost::shared_ptr<Sphere> SpherePtr;
    class TriangleMeshObject;
    typedef boost::shared_ptr<TriangleMeshObject> TriangleMeshObjectPtr;



    }
}

namespace explorationControllers{

    class AbstractExplorationController;
    typedef boost::shared_ptr<AbstractExplorationController> AbstractExplorationControllerPtr;
    class Heuristic;
    typedef boost::shared_ptr<Heuristic> HeuristicPtr;
    class InitialApproach;
    typedef boost::shared_ptr<InitialApproach> InitialApproachPtr;
    class LocalSearch;
    typedef boost::shared_ptr<LocalSearch> LocalSearchPtr;
    class PossibleTarget;
    typedef boost::shared_ptr<PossibleTarget> PossibleTargetPtr;
    class TrajectoryEdgeSearch;
    typedef boost::shared_ptr<TrajectoryEdgeSearch> TrajectoryEdgeSearchPtr;
    class Target;
    typedef boost::shared_ptr<Target> TargetPtr;

}











