/*
* This file is part of ArmarX.
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
* @author     Martin Miller (martin dot miller at student dot kit dot edu)
* @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
*             GNU General Public License
*/

#ifndef math_Helpers
#define math_Helpers

#include "MathForwardDefinitions.h"

namespace math
{
    class Helpers
    {
    public:


        static Eigen::Vector3f GetOrthonormalVectors(Eigen::Vector3f vec, Eigen::Vector3f& dir1, Eigen::Vector3f& dir2);
        static float ShiftAngle0_2PI(float a);
        static void GetIndex(float t, float minT, float maxT, int count, int& i, float& f);
        static float FastDistribution(float x, float sigma2);
        static float Clamp(float min, float max, float value);
        static float Lerp(float a, float b, float f);
        static Eigen::Vector3f Lerp(Eigen::Vector3f a, Eigen::Vector3f b, float f);
        static float Lerp(float a, float b, int min, int max, int val);
        static float Angle(Eigen::Vector2f v);
        static int Sign(float x);
        static void AssertNormalized(Eigen::Vector3f vec, float epsilon = 0.05f);
        static std::vector<float> FloatRange(float start, float end, int steps);
        static std::vector<Eigen::Vector3f> VectorRangeSymmetric(float start, float end, int steps);
        static std::vector<Eigen::Vector3f> VectorRange(std::vector<float> xvals, std::vector<float> yvals, std::vector<float> zvals);
        static float SmallestAngle(Eigen::Vector3f a, Eigen::Vector3f b);
        static Eigen::Vector3f CwiseMin(Eigen::Vector3f a, Eigen::Vector3f b);
        static Eigen::Vector3f CwiseMax(Eigen::Vector3f a, Eigen::Vector3f b);
        static Eigen::Vector3f CwiseDivide(Eigen::Vector3f a, Eigen::Vector3f b);
        static Eigen::Vector3f Average(std::vector<Eigen::Vector3f> vectors);
        static void Swap(float &a,float &b);

    private:
    };
}

#endif // math_Helpers
