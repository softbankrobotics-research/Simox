/**
* @package    VirtualRobot
* @author     Simon Ottenhaus
* @copyright  2018 Simon Ottenhaus
*/

#define BOOST_TEST_MODULE VirtualRobot_MathFitPlaneTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/math/FitPlane.h>
#include <string>
#include <stdio.h>

BOOST_AUTO_TEST_SUITE(MathFitPlane)

using namespace math;


BOOST_AUTO_TEST_CASE(testFitPlane)
{
    std::vector<Eigen::Vector3f> points;
    Eigen::Vector3f e1 = Eigen::Vector3f(1, 1, 0);
    Eigen::Vector3f e2 = Eigen::Vector3f(-1, 1, 0);
    points.push_back(0 * e1 + 0 * e2);
    points.push_back(1 * e1 + 0 * e2);
    points.push_back(0 * e1 + 2 * e2);
    points.push_back(3 * e1 + 4 * e2);
    Plane plane = FitPlane::Fit(points);
    for(float x = -5; x < 5; x += 0.5f)
    {
        for(float y = -5; y < 5; y += 0.5f)
        {
            float dist = plane.GetDistance(x * e1 + y * e2, AbstractFunctionR2R3::SimpleProjection);
            BOOST_CHECK_LE(dist, 0.0001f);
            //std::cout << dist << std::endl;
        }
    }
}


BOOST_AUTO_TEST_SUITE_END()
