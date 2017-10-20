#define BOOST_TEST_MODULE VirtualRobot_VirtualTimeOptimalTrajectoryTest

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/TimeOptimalTrajectory/TimeOptimalTrajectory.h>
#include <VirtualRobot/TimeOptimalTrajectory/Path.h>

#include <chrono>


BOOST_AUTO_TEST_SUITE(TimeOptimalTrajectory)

BOOST_AUTO_TEST_CASE(test1)
{
    Eigen::VectorXd waypoint(4);
    std::list<Eigen::VectorXd> waypoints;

    waypoint << 1424.0, 984.999694824219, 2126.0, 0.0;
    waypoints.push_back(waypoint);
    waypoint << 1423.0, 985.000244140625, 2126.0, 0.0;
    waypoints.push_back(waypoint);

    Eigen::VectorXd maxVelocities(4);
    maxVelocities << 1.3, 0.67, 0.67, 0.5;
    Eigen::VectorXd maxAccelerations(4);
    maxAccelerations << 0.00249, 0.00249, 0.00249, 0.00249;

    double maxDeviation = 100.0;
    double timeStep = 10.0; // ms

    VirtualRobot::TimeOptimalTrajectory trajectory(VirtualRobot::Path(waypoints, maxDeviation), maxVelocities, maxAccelerations, timeStep);

    BOOST_CHECK_EQUAL(trajectory.isValid(), true);
}

BOOST_AUTO_TEST_CASE(test2)
{
    Eigen::VectorXd waypoint(4);
    std::list<Eigen::VectorXd> waypoints;

    waypoint << 1427.0, 368.0, 690.0, 90.0;
    waypoints.push_back(waypoint);
    waypoint << 1427.0, 368.0, 790.0, 90.0;
    waypoints.push_back(waypoint);
    waypoint << 952.499938964844, 433.0, 1051.0, 90.0;
    waypoints.push_back(waypoint);
    waypoint << 452.5, 533.0, 1051.0, 90.0;
    waypoints.push_back(waypoint);
    waypoint << 452.5, 533.0, 951.0, 90.0;
    waypoints.push_back(waypoint);

    Eigen::VectorXd maxVelocities(4);
    maxVelocities << 1.3, 0.67, 0.67, 0.5;
    Eigen::VectorXd maxAccelerations(4);
    maxAccelerations << 0.002, 0.002, 0.002, 0.002;

    double maxDeviation = 100.0;
    double timeStep = 10.0; // ms

    VirtualRobot::TimeOptimalTrajectory trajectory(VirtualRobot::Path(waypoints, maxDeviation), maxVelocities, maxAccelerations, timeStep);

    BOOST_CHECK_EQUAL(trajectory.isValid(), true);
}

BOOST_AUTO_TEST_CASE(example)
{
    std::list<Eigen::VectorXd> waypoints;
    Eigen::VectorXd waypoint(3);
    waypoint << 0.0, 0.0, 0.0;
    waypoints.push_back(waypoint);
    waypoint << 0.0, 0.2, 1.0;
    waypoints.push_back(waypoint);
    waypoint << 0.0, 3.0, 0.5;
    waypoints.push_back(waypoint);
    waypoint << 1.1, 2.0, 0.0;
    waypoints.push_back(waypoint);
    waypoint << 1.0, 0.0, 0.0;
    waypoints.push_back(waypoint);
    waypoint << 0.0, 1.0, 0.0;
    waypoints.push_back(waypoint);
    waypoint << 0.0, 0.0, 1.0;
    waypoints.push_back(waypoint);

    Eigen::VectorXd maxAcceleration(3);
    maxAcceleration << 1.0, 1.0, 1.0;
    Eigen::VectorXd maxVelocity(3);
    maxVelocity << 1.0, 1.0, 1.0;

    double maxDeviation = 0.1;

    time_t  timev;
    time(&timev);

    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

    VirtualRobot::TimeOptimalTrajectory trajectory(VirtualRobot::Path(waypoints, maxDeviation), maxVelocity, maxAcceleration, 0.1);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    int dtime = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();


    trajectory.outputPhasePlaneTrajectory();
    if(trajectory.isValid()) {
        double duration = trajectory.getDuration();
        std::cout << "Trajectory duration: " << duration << " s" << std::endl << std::endl;
        std::cout << "Time      Position                  Velocity" << std::endl;
        for(double t = 0.0; t < duration; t += 0.1) {
            printf("%6.4f   %7.4f %7.4f %7.4f   %7.4f %7.4f %7.4f\n", t, trajectory.getPosition(t)[0], trajectory.getPosition(t)[1], trajectory.getPosition(t)[2],
                trajectory.getVelocity(t)[0], trajectory.getVelocity(t)[1], trajectory.getVelocity(t)[2]);
        }
        printf("%6.4f   %7.4f %7.4f %7.4f   %7.4f %7.4f %7.4f\n", duration, trajectory.getPosition(duration)[0], trajectory.getPosition(duration)[1], trajectory.getPosition(duration)[2],
            trajectory.getVelocity(duration)[0], trajectory.getVelocity(duration)[1], trajectory.getVelocity(duration)[2]);
    }
    else {
        std::cout << "Trajectory generation (example) failed." << std::endl;
    }

    std::cout << "Trajectory generation took: " << dtime << " microseconds" << std::endl;
    BOOST_CHECK_EQUAL(trajectory.isValid(), true);


}

BOOST_AUTO_TEST_CASE(exampleFromSimulationEasy)
{
    std::list<Eigen::VectorXd> waypoints;
    Eigen::VectorXd waypoint(3);
    waypoint << 2016.030000, 6347.610000, -0.461418;
    waypoints.push_back(waypoint);
    waypoint << 2014.690000, 6346.200000, -0.461100;
    waypoints.push_back(waypoint);
    waypoint << 2028.410000, 6332.140000, -0.475868;
    waypoints.push_back(waypoint);
    waypoint << 2029.450000, 6331.340000, -0.476883;
    waypoints.push_back(waypoint);
    waypoint << 2107.560000, 6287.490000, -0.546368;
    waypoints.push_back(waypoint);
    waypoint << 2174.690000, 6259.350000, -0.602234;
    waypoints.push_back(waypoint);
    waypoint << 3042.190000, 6404.520000, -1.118520;
    waypoints.push_back(waypoint);
    waypoint << 3424.310000, 6546.600000, -1.314360;
    waypoints.push_back(waypoint);

    Eigen::VectorXd maxAcceleration(3);
    maxAcceleration << 500.0, 500.0, 0.75;
    Eigen::VectorXd maxVelocity(3);
    maxVelocity << 500.0, 500.0, 0.5;

    double maxDeviation = 0.1;

    time_t  timev;
    time(&timev);

    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

    VirtualRobot::TimeOptimalTrajectory trajectory(VirtualRobot::Path(waypoints, maxDeviation), maxVelocity, maxAcceleration, 0.1);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    int dtime = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();


    trajectory.outputPhasePlaneTrajectory();
    if(trajectory.isValid()) {
        double duration = trajectory.getDuration();
        std::cout << "Trajectory duration: " << duration << " s" << std::endl << std::endl;
        std::cout << "Time      Position                  Velocity" << std::endl;
        for(double t = 0.0; t < duration; t += 0.1) {
            printf("%6.4f   %7.4f %7.4f %7.4f   %7.4f %7.4f %7.4f\n", t, trajectory.getPosition(t)[0], trajectory.getPosition(t)[1], trajectory.getPosition(t)[2],
                trajectory.getVelocity(t)[0], trajectory.getVelocity(t)[1], trajectory.getVelocity(t)[2]);
        }
        printf("%6.4f   %7.4f %7.4f %7.4f   %7.4f %7.4f %7.4f\n", duration, trajectory.getPosition(duration)[0], trajectory.getPosition(duration)[1], trajectory.getPosition(duration)[2],
            trajectory.getVelocity(duration)[0], trajectory.getVelocity(duration)[1], trajectory.getVelocity(duration)[2]);
    }
    else {
        std::cout << "Trajectory generation (exampleFromSimulationEasy) failed." << std::endl;
    }

    std::cout << "Trajectory generation (Simulation example easy) took: " << dtime << " microseconds" << std::endl;
    BOOST_CHECK_EQUAL(trajectory.isValid(), true);
}

BOOST_AUTO_TEST_CASE(exampleFromSimulationDifficult)
{
    std::list<Eigen::VectorXd> waypoints;
    Eigen::VectorXd waypoint(3);
    waypoint << 2016.030000, 6347.610000, -0.461418;
    waypoints.push_back(waypoint);
    waypoint << 2014.690000, 6346.200000, -0.461100;
    waypoints.push_back(waypoint);
    waypoint << 2028.410000, 6332.140000, -0.475868;
    waypoints.push_back(waypoint);
    waypoint << 2629.450000, 7231.340000, -1.476883;
    waypoints.push_back(waypoint);
    waypoint << 3107.560000, 6687.490000, -0.546368;
    waypoints.push_back(waypoint);
    waypoint << 2874.690000, 6259.350000, 1.602234;
    waypoints.push_back(waypoint);
    waypoint << 3042.190000, 6404.520000, -1.118520;
    waypoints.push_back(waypoint);
    waypoint << 3424.310000, 6546.600000, -1.314360;
    waypoints.push_back(waypoint);

    Eigen::VectorXd maxAcceleration(3);
    maxAcceleration << 500.0, 500.0, 0.75;
    Eigen::VectorXd maxVelocity(3);
    maxVelocity << 500.0, 500.0, 1.0;
    double maxDeviation = 2.0;

    time_t  timev;
    time(&timev);

    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

    VirtualRobot::TimeOptimalTrajectory trajectory(VirtualRobot::Path(waypoints, maxDeviation), maxVelocity, maxAcceleration, 0.1);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    int dtime = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();


    trajectory.outputPhasePlaneTrajectory();
    if(trajectory.isValid()) {
        double duration = trajectory.getDuration();
        std::cout << "Trajectory duration: " << duration << " s" << std::endl << std::endl;
        std::cout << "Time      Position                  Velocity" << std::endl;
        for(double t = 0.0; t < duration; t += 0.1) {
            printf("%6.4f   %7.4f %7.4f %7.4f   %7.4f %7.4f %7.4f\n", t, trajectory.getPosition(t)[0], trajectory.getPosition(t)[1], trajectory.getPosition(t)[2],
                trajectory.getVelocity(t)[0], trajectory.getVelocity(t)[1], trajectory.getVelocity(t)[2]);
        }
        printf("%6.4f   %7.4f %7.4f %7.4f   %7.4f %7.4f %7.4f\n", duration, trajectory.getPosition(duration)[0], trajectory.getPosition(duration)[1], trajectory.getPosition(duration)[2],
            trajectory.getVelocity(duration)[0], trajectory.getVelocity(duration)[1], trajectory.getVelocity(duration)[2]);
    }
    else {
        std::cout << "Trajectory generation (exampleFromSimulationDifficult) failed." << std::endl;
    }

    std::cout << "Trajectory generation (Simulation example difficult) took: " << dtime << " microseconds" << std::endl;
    BOOST_CHECK_EQUAL(trajectory.isValid(), true);
}

BOOST_AUTO_TEST_SUITE_END()
