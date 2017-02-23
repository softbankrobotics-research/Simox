/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotMathToolsTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <string>

BOOST_AUTO_TEST_SUITE(MathTools)


BOOST_AUTO_TEST_CASE(testMathToolsHopf)
{
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    const int NR_TESTS = 10000;
    for (int i=0;i<NR_TESTS;i++)
    {
        Eigen::Vector3f ax = Eigen::Vector3f::Random();
        ax.normalize();
        float ang = float(rand() % 1000 / 1000.0f * 2.0f*M_PI -M_PI);
        Eigen::Matrix3f m3 = Eigen::AngleAxisf(ang, ax).matrix();
        m.block(0, 0, 3, 3) = m3;

        VirtualRobot::MathTools::Quaternion q = VirtualRobot::MathTools::eigen4f2quat(m);
        Eigen::Vector3f h = VirtualRobot::MathTools::quat2hopf(q);

        BOOST_CHECK_LE(h(0), 2.0f*(float)M_PI);
        BOOST_CHECK_LE(h(1), 2.0f*(float)M_PI);
        BOOST_CHECK_LE(h(2), (float)M_PI*0.5f);

        BOOST_CHECK_GE(h(0), 0.0f);
        BOOST_CHECK_GE(h(1), 0.0f);
        BOOST_CHECK_GE(h(2), 0.0f);

        VirtualRobot::MathTools::Quaternion q2 = VirtualRobot::MathTools::hopf2quat(h);

        VirtualRobot::MathTools::Quaternion qDelta = VirtualRobot::MathTools::getDelta(q, q2);
        float a = (float)fabs(VirtualRobot::MathTools::getAngle(qDelta));
        BOOST_CHECK_SMALL(a, 0.01f);
        if (a>0.01)
        {
            std::cout << "H:" << h.transpose() << ", Q:" << q.w << ", " << q.x << "," << q.y << "," << q.z << " / Q2:" << q2.w << ", " << q2.x << "," << q2.y << "," << q2.z << std::endl;
        }
    }
}


BOOST_AUTO_TEST_CASE(testMathToolsHopfNeighborhood)
{
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    const int NR_TESTS = 50000;
    const float maxAngHopf = 0.01f;
    const float maxAngQuat = 2.5f*maxAngHopf;
    for (int i = 0; i<NR_TESTS; i++)
    {
        Eigen::Vector3f ax = Eigen::Vector3f::Random();
        ax.normalize();
        float ang = float(rand() % 1000 / 1000.0f * 2.0f*M_PI - M_PI);
        Eigen::Matrix3f m3 = Eigen::AngleAxisf(ang, ax).matrix();
        m.block(0, 0, 3, 3) = m3;
        VirtualRobot::MathTools::Quaternion q = VirtualRobot::MathTools::eigen4f2quat(m);
        Eigen::Vector3f h0 = VirtualRobot::MathTools::quat2hopf(q);

        ax = Eigen::Vector3f::Random();
        ax.normalize();
        ax *= maxAngHopf;

        Eigen::Vector3f h = h0 + ax;

        if (h(0) < 0)
            continue;
        //h(0) = float(2.0f*M_PI) - h(0);
        if (h(0) > float(2.0f*M_PI))
            continue;
        //h(0) = h(0) - float(2.0f*M_PI);
        if (h(1) < 0)
            continue;
        //h(1) = float(2.0f*M_PI) - h(1);
        if (h(1) > float(2.0f*M_PI))
            continue;
        //h(1) = h(1) - float(2.0f*M_PI);
        if (h(2) < 0)
            continue;
            //h(2) = float(M_PI*0.5f) - h(2);
        if (h(2) > float(M_PI*0.5f))
            continue;
            //h(2) = h(2) - float(M_PI*0.5f);


        BOOST_CHECK_GE(h(0), 0.0f);
        BOOST_CHECK_GE(h(1), 0.0f);
        BOOST_CHECK_GE(h(2), 0.0f);
        BOOST_CHECK_LE(h(0), 2.0f*(float)M_PI);
        BOOST_CHECK_LE(h(1), 2.0f*(float)M_PI);
        BOOST_CHECK_LE(h(2), (float)M_PI*0.5f);

        VirtualRobot::MathTools::Quaternion q2 = VirtualRobot::MathTools::hopf2quat(h);

        VirtualRobot::MathTools::Quaternion qDelta = VirtualRobot::MathTools::getDelta(q, q2);
        float a = (float)fabs(VirtualRobot::MathTools::getAngle(qDelta));
        BOOST_CHECK_SMALL(a, maxAngQuat);
        
        if (a>maxAngQuat)
        {
            std::cout << "H0:" << h0.transpose() << ", " << "h-new:" << h.transpose() << std::endl;
            std::cout << "Q:" << q.w << ", " << q.x << "," << q.y << "," << q.z << " / Q2:" << q2.w << ", " << q2.x << "," << q2.y << "," << q2.z << std::endl;
        }
    }
}


BOOST_AUTO_TEST_CASE(testMathToolsRPY)
{
    float r = (float)M_PI * 0.25f;
    float p = 0;
    float y = (float)M_PI * 0.5f;
    Eigen::Matrix4f m;
    BOOST_REQUIRE_NO_THROW(VirtualRobot::MathTools::rpy2eigen4f(r, p, y, m));
    float x[6];
    BOOST_REQUIRE_NO_THROW(VirtualRobot::MathTools::eigen4f2rpy(m, x));

    BOOST_CHECK_EQUAL(0, x[0]);
    BOOST_CHECK_EQUAL(0, x[1]);
    BOOST_CHECK_EQUAL(0, x[2]);
    BOOST_CHECK_EQUAL(r, x[3]);
    BOOST_CHECK_EQUAL(p, x[4]);
    BOOST_CHECK_EQUAL(y, x[5]);
}

BOOST_AUTO_TEST_CASE(testMathToolsPlane)
{
    VirtualRobot::MathTools::Plane plane = VirtualRobot::MathTools::getFloorPlane();
    Eigen::Vector3f p(100.0f, 200.0f, 300.0f);
    Eigen::Vector3f res;
    BOOST_REQUIRE_NO_THROW(res = VirtualRobot::MathTools::projectPointToPlane(p, plane));


    BOOST_CHECK_EQUAL(res(0), p(0));
    BOOST_CHECK_EQUAL(res(1), p(1));
    BOOST_CHECK_EQUAL(res(2), 0);

}


BOOST_AUTO_TEST_CASE(testMathToolsConvexHull2D)
{
    Eigen::Vector2f a(1.0f, 1.0f);
    Eigen::Vector2f b(1.0f, 2.0f);
    Eigen::Vector2f c(2.0f, 1.0f);
    Eigen::Vector2f d(2.0f, 2.0f);
    Eigen::Vector2f e(1.5f, 1.5f);
    Eigen::Vector2f f(1.2f, 1.8f);
    Eigen::Vector2f g(2.5f, 2.5f);
    Eigen::Vector2f h(-1.5f, -1.5f);
    std::vector< Eigen::Vector2f > points;
    points.push_back(a);
    points.push_back(b);
    points.push_back(c);
    points.push_back(d);
    points.push_back(e);
    points.push_back(f);

    VirtualRobot::MathTools::ConvexHull2DPtr cv;
    BOOST_REQUIRE_NO_THROW(cv = VirtualRobot::MathTools::createConvexHull2D(points));

    BOOST_REQUIRE(cv);

    bool isInsideE;
    bool isInsideF;
    bool isInsideG;
    bool isInsideH;
    BOOST_REQUIRE_NO_THROW(isInsideE = VirtualRobot::MathTools::isInside(e, cv));
    BOOST_REQUIRE_NO_THROW(isInsideF = VirtualRobot::MathTools::isInside(f, cv));
    BOOST_REQUIRE_NO_THROW(isInsideG = VirtualRobot::MathTools::isInside(g, cv));
    BOOST_REQUIRE_NO_THROW(isInsideH = VirtualRobot::MathTools::isInside(h, cv));

    BOOST_CHECK_EQUAL(isInsideE, true);
    BOOST_CHECK_EQUAL(isInsideF, true);
    BOOST_CHECK_EQUAL(isInsideG, false);
    BOOST_CHECK_EQUAL(isInsideH, false);
}


BOOST_AUTO_TEST_CASE(testMathToolsBasisChange)
{
    Eigen::VectorXf a1(3), a2(3), a3(3);
    a1 << 1, 0, 2;
    a2 << 3, 1, 0;
    a3 << 2, 1, 1;
    Eigen::VectorXf b1(3), b2(3), b3(3);
    b1 << 1, 0, 1;
    b2 << 0, 1, 1;
    b3 << 1, 1, 0;
    std::vector< Eigen::VectorXf > a;
    a.push_back(a1);
    a.push_back(a2);
    a.push_back(a3);
    std::vector< Eigen::VectorXf > b;
    b.push_back(b1);
    b.push_back(b2);
    b.push_back(b3);
    Eigen::MatrixXf T;
    BOOST_REQUIRE_NO_THROW(T = VirtualRobot::MathTools::getBasisTransformation(a, b));

    BOOST_CHECK_EQUAL(T.cols(), 3);
    BOOST_CHECK_EQUAL(T.rows(), 3);
    Eigen::Vector3f v_b;
    v_b << 2, -1, 3;
    Eigen::Vector3f v_c;
    BOOST_REQUIRE_NO_THROW(v_c = T * v_b);
    // should be (5,2,0)
    BOOST_CHECK_CLOSE(v_c(0), 5.0f, 0.1f);
    BOOST_CHECK_CLOSE(v_c(1), 2.0f, 0.1f);
    BOOST_CHECK_SMALL(v_c(2) , 0.001f);
}


BOOST_AUTO_TEST_CASE(testMathToolsSegemntPlaneIntersection)
{
    VirtualRobot::MathTools::Plane plane(Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 0, 1.0f));

    VirtualRobot::MathTools::Segment segment_noIntersect(Eigen::Vector3f(0, 0, 10.0f), Eigen::Vector3f(1.0f, 20.0f, 100.0f));
    VirtualRobot::MathTools::Segment segment_intersect(Eigen::Vector3f(0, 0, -1.0f), Eigen::Vector3f(0, 0, 1.0f));
    VirtualRobot::MathTools::Segment segment_intersect2(Eigen::Vector3f(200.0f, 30.0f, -100.0f), Eigen::Vector3f(50.0f, 200.0f, 100.0f));
    VirtualRobot::MathTools::Segment segment_parallel(Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(1.0f, 0, 0));

    Eigen::Vector3f res;
    BOOST_CHECK_EQUAL(VirtualRobot::MathTools::intersectSegmentPlane(segment_noIntersect, plane, res), VirtualRobot::MathTools::eNoIntersection);
    BOOST_CHECK_EQUAL(VirtualRobot::MathTools::intersectSegmentPlane(segment_intersect, plane, res), VirtualRobot::MathTools::eIntersection);
    // should be (0,0,0)
    BOOST_CHECK_CLOSE(res.norm(), 0.0f, 1e-6f);

    BOOST_CHECK_EQUAL(VirtualRobot::MathTools::intersectSegmentPlane(segment_intersect2, plane, res), VirtualRobot::MathTools::eIntersection);
    BOOST_CHECK_EQUAL(VirtualRobot::MathTools::intersectSegmentPlane(segment_parallel, plane, res), VirtualRobot::MathTools::eNoIntersection);
}


BOOST_AUTO_TEST_CASE(testMathToolsOOBBPlaneIntersection)
{
    Eigen::Matrix4f pose;
    pose.setIdentity();
    pose(2, 3) = 150.0f;
    VirtualRobot::MathTools::OOBB oobb(Eigen::Vector3f(-100.0f, -100.0f, -100.0f), Eigen::Vector3f(100.0f, 100.0f, 100.0f), pose);
    VirtualRobot::MathTools::Plane plane(Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 0, 1.0f));

    std::vector<Eigen::Vector3f> res;
    BOOST_CHECK_EQUAL(VirtualRobot::MathTools::intersectOOBBPlane(oobb, plane, res), VirtualRobot::MathTools::eNoIntersection);
    pose(2, 3) = -50.0f;
    oobb.pose = pose;
    BOOST_CHECK_EQUAL(VirtualRobot::MathTools::intersectOOBBPlane(oobb, plane, res), VirtualRobot::MathTools::eIntersection);
    BOOST_CHECK_EQUAL(res.size(), 4);
    BOOST_CHECK_CLOSE(res[0](0), -100.0f, 1e-6f);
    BOOST_CHECK_CLOSE(res[0](1), 100.0f, 1e-6f);
    BOOST_CHECK_CLOSE(res[0](2), 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(res[1](0), -100.0f, 1e-6f);
    BOOST_CHECK_CLOSE(res[1](1), -100.0f, 1e-6f);
    BOOST_CHECK_CLOSE(res[1](2), 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(res[2](0), 100.0f, 1e-6f);
    BOOST_CHECK_CLOSE(res[2](1), -100.0f, 1e-6f);
    BOOST_CHECK_CLOSE(res[2](2), 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(res[3](0), 100.0f, 1e-6f);
    BOOST_CHECK_CLOSE(res[3](1), 100.0f, 1e-6f);
    BOOST_CHECK_CLOSE(res[3](2), 0.0f, 1e-6f);
}

BOOST_AUTO_TEST_SUITE_END()
