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

#include <boost/shared_ptr.hpp>
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

    class AbstractFunctionR1R2;
    typedef boost::shared_ptr<AbstractFunctionR1R2> AbstractFunctionR1R2Ptr;
    class Line;
    typedef boost::shared_ptr<Line> LinePtr;
    class LineStrip;
    typedef boost::shared_ptr<LineStrip> LineStripPtr;
    class AbstractFunctionR1R3;
    typedef boost::shared_ptr<AbstractFunctionR1R3> AbstractFunctionR1R3Ptr;
    class AbstractFunctionR1R6;
    typedef boost::shared_ptr<AbstractFunctionR1R6> AbstractFunctionR1R6Ptr;
    class AbstractFunctionR2R3;
    typedef boost::shared_ptr<AbstractFunctionR2R3> AbstractFunctionR2R3Ptr;
    class AbstractFunctionR3R1;
    typedef boost::shared_ptr<AbstractFunctionR3R1> AbstractFunctionR3R1Ptr;
    class Contact;
    typedef boost::shared_ptr<Contact> ContactPtr;
    class ContactList;
    typedef boost::shared_ptr<ContactList> ContactListPtr;
    class ImplicitPlane;
    typedef boost::shared_ptr<ImplicitPlane> ImplicitPlanePtr;
    class LineR2;
    typedef boost::shared_ptr<LineR2> LineR2Ptr;
    class Plane;
    typedef boost::shared_ptr<Plane> PlanePtr;
    class Triangle;
    typedef boost::shared_ptr<Triangle> TrianglePtr;
    class SimpleAbstractFunctionR1R3;
    typedef boost::shared_ptr<SimpleAbstractFunctionR1R3> SimpleAbstractFunctionR1R3Ptr;
    class SimpleAbstractFunctionR1R6;
    typedef boost::shared_ptr<SimpleAbstractFunctionR1R6> SimpleAbstractFunctionR1R6Ptr;
    class SimpleAbstractFunctionR2R3;
    typedef boost::shared_ptr<SimpleAbstractFunctionR2R3> SimpleAbstractFunctionR2R3Ptr;
    class SimpleAbstractFunctionR3R1;
    typedef boost::shared_ptr<SimpleAbstractFunctionR3R1> SimpleAbstractFunctionR3R1Ptr;
    class LinearInterpolatedOrientation;
    typedef boost::shared_ptr<LinearInterpolatedOrientation> LinearInterpolatedOrientationPtr;
    class AbstractFunctionR1Ori;
    typedef boost::shared_ptr<AbstractFunctionR1Ori> AbstractFunctionR1OriPtr;
    class SimpleAbstractFunctionR1Ori;
    typedef boost::shared_ptr<SimpleAbstractFunctionR1Ori> SimpleAbstractFunctionR1OriPtr;
    class CompositeFunctionR1R6;
    typedef boost::shared_ptr<CompositeFunctionR1R6> CompositeFunctionR1R6Ptr;
    class AbstractFunctionR1R6;
    typedef boost::shared_ptr<AbstractFunctionR1R6> AbstractFunctionR1R6Ptr;
    class ImplicitObjectModel;
    typedef boost::shared_ptr<ImplicitObjectModel> ImplicitObjectModelPtr;
    class HalfSpaceObjectModel;
    typedef boost::shared_ptr<HalfSpaceObjectModel> HalfSpaceObjectModelPtr;
    class HalfSpaceImplicitSurface3D;
    typedef boost::shared_ptr<HalfSpaceImplicitSurface3D> HalfSpaceImplicitSurface3DPtr;
    class GaussianObjectModel;
    typedef boost::shared_ptr<GaussianObjectModel> GaussianObjectModelPtr;
    class GaussianObjectModelNormals;
    typedef boost::shared_ptr<GaussianObjectModelNormals> GaussianObjectModelNormalsPtr;
    class GaussianImplicitSurface3D;
    typedef boost::shared_ptr<GaussianImplicitSurface3D> GaussianImplicitSurface3DPtr;
    class GaussianImplicitSurface3DNormals;
    typedef boost::shared_ptr<GaussianImplicitSurface3DNormals> GaussianImplicitSurface3DNormalsPtr;
    class GaussianImplicitSurface3DCombined;
    typedef boost::shared_ptr<GaussianImplicitSurface3DCombined> GaussianImplicitSurface3DCombinedPtr;
    class DataR3R1;
    typedef boost::shared_ptr<DataR3R1> DataR3R1Ptr;
    class DataR3R2;
    typedef boost::shared_ptr<DataR3R2> DataR3R2Ptr;
    class MarchingCubes;
    typedef boost::shared_ptr<MarchingCubes> MarchingCubesPtr;
    class Bezier;
    typedef boost::shared_ptr<Bezier> BezierPtr;
    class LinearContinuedBezier;
    typedef boost::shared_ptr<LinearContinuedBezier> LinearContinuedBezierPtr;
    class Primitive;
    typedef boost::shared_ptr<Primitive> PrimitivePtr;
    class Index3;
    typedef boost::shared_ptr<Index3> Index3Ptr;
    class AbstractContactFeature;
    typedef boost::shared_ptr<AbstractContactFeature> AbstractContactFeaturePtr;
    class BinContactFeature;
    typedef boost::shared_ptr<BinContactFeature> BinContactFeaturePtr;
    template<class T> class Array3D;
    typedef boost::shared_ptr<Array3D<float>> Array3DFloatPtr;
    typedef boost::shared_ptr<Array3D<bool>> Array3DBoolPtr;
    //typedef boost::shared_ptr<Array3D<>> Array3DPtr<T>;
    class VoronoiWeights;
    typedef boost::shared_ptr<VoronoiWeights> VoronoiWeightsPtr;
    struct WeightedFloatAverage;
    class WeightedVec3Average;
    class Grid3D;
    typedef boost::shared_ptr<Grid3D> Grid3DPtr;
    class GridCacheFloat3;
    typedef boost::shared_ptr<GridCacheFloat3> GridCacheFloat3Ptr;
    class FeatureCluster;
    typedef boost::shared_ptr<FeatureCluster> FeatureClusterPtr;
    class EdgeCluster;
    typedef boost::shared_ptr<EdgeCluster> EdgeClusterPtr;
    class EdgePredictor;
    typedef boost::shared_ptr<EdgePredictor> EdgePredictorPtr;
    class EdgeTracer;
    typedef boost::shared_ptr<EdgeTracer> EdgeTracerPtr;
    class EdgeFeature;
    typedef boost::shared_ptr<std::vector<Eigen::Vector3f>> Vec3ListPtr;
    class Edge;
    typedef boost::shared_ptr<Edge> EdgePtr;


    typedef boost::shared_ptr<EdgeFeature> EdgeFeaturePtr;


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











