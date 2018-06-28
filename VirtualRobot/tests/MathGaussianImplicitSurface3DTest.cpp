/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2012 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_MathGaussianImplicitSurface3DTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/math/GaussianImplicitSurface3D.h>
#include <VirtualRobot/math/Helpers.h>
#include <string>
#include <stdio.h>

BOOST_AUTO_TEST_SUITE(MathGaussianImplicitSurface3D)

using namespace math;

typedef Eigen::Vector3f Vec3;

BOOST_AUTO_TEST_CASE(testGPIS)
{
    std::vector<DataR3R2> samples;
    samples.push_back(DataR3R2(0, 0, 0, 1, 0.1f));
    samples.push_back(DataR3R2(1, 0, 0, 0, 0.1f));
    samples.push_back(DataR3R2(0, 1, 0, 0, 0.1f));
    samples.push_back(DataR3R2(-1, 0, 0, 0, 0.1f));
    samples.push_back(DataR3R2(0, -1, 0, 0, 0.1f));
    for(int i = 0; i < 10; i++)
    {
        samples.push_back(DataR3R2(Helpers::CreateVectorFromCylinderCoords(3, Helpers::Lerp(0, 2*M_PI, 0, 10, i), 0), -1, 0.1f));
    }

    GaussianImplicitSurface3D gpis(KernelWithDerivatives::Create(KernelType::Gauss));
    gpis.Calculate(samples);
    //std::cout << gpis.Get(Vec3(0,0,0)) << std::endl;

    for(const DataR3R2& s : samples)
    {
        float err = fabs(s.Value1() - gpis.Get(s.Position()));
        BOOST_CHECK_LE(err, 0.1);
        //std::cout << err << " " << s.Value1() << " " << gpis.Get(s.Position()) << std::endl;
    }

}


BOOST_AUTO_TEST_SUITE_END()
