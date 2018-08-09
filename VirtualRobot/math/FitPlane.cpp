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

#include "FitPlane.h"

using namespace math;

Plane FitPlane::Fit(const std::vector<Eigen::Vector3f>& points)
{
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for(const Eigen::Vector3f& p : points)
    {
        mean += p;
    }
    mean /= points.size();
    Eigen::MatrixXf matrix(3, points.size());
    for(size_t i = 0; i < points.size(); i++)
    {
        matrix.block<3, 1>(0, i) = points.at(i) - mean;
    }

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXf matrixU = svd.matrixU();
    Eigen::Vector3f dir1 = matrixU.block<3, 1>(0, 0);
    Eigen::Vector3f dir2 = matrixU.block<3, 1>(0, 1);
    return Plane(mean, dir1.normalized(), dir2.normalized());
}

