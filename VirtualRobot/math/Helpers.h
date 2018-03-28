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


        static Vec3 GetOrthonormalVectors(Vec3 vec, Vec3& dir1, Vec3& dir2);
        static float ShiftAngle0_2PI(float a);
        static void GetIndex(float t, float minT, float maxT, int count, int& i, float& f);
        static float FastDistribution(float x, float sigma2);
        static float Clamp(float min, float max, float value);
        static float Lerp(float a, float b, float f);
        static Vec3 Lerp(Vec3 a, Vec3 b, float f);
        static float Lerp(float a, float b, int min, int max, int val);
        static float Angle(Vec2 v);
        static int Sign(float x);
        static void AssertNormalized(Vec3 vec, float epsilon = 0.05f);
        static std::vector<float> FloatRange(float start, float end, int steps);
        static std::vector<Vec3> VectorRangeSymmetric(float start, float end, int steps);
        static std::vector<Vec3> VectorRange(std::vector<float> xvals, std::vector<float> yvals, std::vector<float> zvals);
        static float SmallestAngle(Vec3 a, Vec3 b);
        static Vec3 CwiseMin(Vec3 a, Vec3 b);
        static Vec3 CwiseMax(Vec3 a, Vec3 b);
        static Vec3 CwiseDivide(Vec3 a, Vec3 b);
        static Vec3 Average(std::vector<Vec3> vectors);
        static void Swap(float &a,float &b);

    private:
    };
}

#endif // math_Helpers
