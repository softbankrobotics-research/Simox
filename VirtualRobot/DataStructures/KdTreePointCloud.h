/*
 * This file is part of ArmarX.
 *
 * Copyright (C) 2011-2016, High Performance Humanoid Technologies (H2T), Karlsruhe Institute of Technology (KIT), all rights reserved.
 *
 * ArmarX is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * ArmarX is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @package    ArmarX
 * @author     Mirko Waechter( mirko.waechter at kit dot edu)
 * @date       2016
 * @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
 *             GNU General Public License
 */
#pragma once

namespace VirtualRobot
{
    /**
     * Pointcloud for usage with nanoflann kd tree.
     */
    template <typename T>
    struct PointCloud
    {
        struct Point
        {
            T  x,y,z;
        };

        std::vector<Point>  pts;
        // Must return the number of data points
        inline size_t kdtree_get_point_count() const { return pts.size(); }

        // Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
        inline T kdtree_distance(const T *p1, const size_t idx_p2,size_t /*size*/) const
        {
            const T d0=p1[0]-pts[idx_p2].x;
            const T d1=p1[1]-pts[idx_p2].y;
            const T d2=p1[2]-pts[idx_p2].z;
            return d0*d0+d1*d1+d2*d2;
        }

        // Returns the dim'th component of the idx'th point in the class:
        // Since this is inlined and the "dim" argument is typically an immediate value, the
        //  "if/else's" are actually solved at compile time.
        inline T kdtree_get_pt(const size_t idx, int dim) const
        {
            if (dim==0) return pts[idx].x;
            else if (dim==1) return pts[idx].y;
            else return pts[idx].z;
        }

        // Optional bounding-box computation: return false to default to a standard bbox computation loop.
        //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
        //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
        template <class BBOX>
        bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }

    };

}
