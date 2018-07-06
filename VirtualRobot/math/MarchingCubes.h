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

#include "MathForwardDefinitions.h"
#include "Array3D.h"
#include "Triangle.h"
#include <queue>


namespace math
{

 // code from https://nucleardevs.wordpress.com/2011/11/17/marching-cubes-sourcecode/
    class MarchingCubes
    {
    public:
        void Init(int size, Eigen::Vector3f center, Eigen::Vector3f start, int steps, float stepLength, GridCacheFloat3Ptr cache);
        PrimitivePtr Process(float isolevel);
        void Process(float isolevel, PrimitivePtr primitive); //TODO ref
        PrimitivePtr ProcessSingleSurfaceOptimized(float isolevel, Index3 start);
        void ProcessSingleSurfaceOptimized(float isolevel, PrimitivePtr primitive, Index3 start);
        static PrimitivePtr Calculate(Eigen::Vector3f center, Eigen::Vector3f start, int steps, float stepLength, SimpleAbstractFunctionR3R1Ptr modelPtr, float isolevel);

    private:
      struct GridCell{
          public:
              Eigen::Vector3f P[8];
      };

      typedef boost::shared_ptr<Array3D<GridCell>> Array3DGridCellPtr;
      typedef boost::shared_ptr<std::queue<Index3>> FringePtr;

      int _size;
      GridCacheFloat3Ptr Gdata;
      Array3DGridCellPtr _grids;
      Triangle _triangles[16];

      static const int _edgeTable[256];
      static const char _triTable[256][16];


      float GetVal(int x, int y, int z, int i);
      Eigen::Vector3f GetPos(int x, int y, int z, int i);
      void AddAndMark(int x, int y, int z, FringePtr fringe, Array3DBoolPtr marked);
      bool IsValidIndex(int x, int y, int z);

      int Polygonise(int x, int y, int z, float isolevel);

      static Eigen::Vector3f VertexInterp(float isolevel, Eigen::Vector3f p1, Eigen::Vector3f p2, float valp1, float valp2);

      void Build(PrimitivePtr res, int tianglesNum);

    };
}

