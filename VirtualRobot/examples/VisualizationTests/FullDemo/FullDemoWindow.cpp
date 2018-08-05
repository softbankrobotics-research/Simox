#include "FullDemoWindow.h"
#include "../../VirtualRobotException.h"

#include "../../Visualization/VisualizationFactory.h"

#include <random>
#include <algorithm>
#include <iterator>
#include <functional>

FullDemoWindow::FullDemoWindow()
    : QMainWindow(NULL)
{
    setupUI();
}


FullDemoWindow::~FullDemoWindow()
{

}

Eigen::Matrix4f getPosition(int index)
{
    Eigen::Matrix4f mat;

    mat <<
         1.0f, 0.0f, 0.0f, (index / 4) * 1000.0f,
         0.0f, 1.0f, 0.0f, 0.0f,
         0.0f, 0.0f, 1.0f, (index % 4) * -1000.0f,
         0.0f, 0.0f, 0.0f, 1.0f;

    return mat;
}

void FullDemoWindow::setupUI()
{
    UI.setupUi(this);

    UI.tabWidget->setCurrentIndex(0);

    SimoxGui::ViewerFactoryPtr viewerFactory = SimoxGui::ViewerFactory::getInstance();
    THROW_VR_EXCEPTION_IF(!viewerFactory,"No viewer factory?!");
    factoryDemoViewer = viewerFactory->createViewer(UI.frameViewer1);
    viewer2 = viewerFactory->createViewer(UI.frameViewer2);

    int index = 0;

    // CUBES
    VirtualRobot::VisualizationPtr cube1 = VirtualRobot::VisualizationFactory::getInstance()->createBox(500.0f, 500.0f, 500.0f);
    cube1->setGlobalPose(getPosition(index++));
    factoryDemoViewer->addVisualization("testLayer", cube1);

    VirtualRobot::VisualizationPtr cube2 = VirtualRobot::VisualizationFactory::getInstance()->createBox(500.0f, 500.0f, 500.0f);
    cube2->setGlobalPose(getPosition(index++));
    cube2->setColor(VirtualRobot::Visualization::Color(1.0f, 0.0f, 0.0f));
    factoryDemoViewer->addVisualization("testLayer", cube2);

    VirtualRobot::VisualizationPtr cube3 = VirtualRobot::VisualizationFactory::getInstance()->createBox(500.0f, 500.0f, 500.0f);
    cube3->setGlobalPose(getPosition(index++));
    cube3->setColor(VirtualRobot::Visualization::Color(0.0f, 1.0f, 0.0f));
    factoryDemoViewer->addVisualization("testLayer", cube3);

    VirtualRobot::VisualizationPtr cube4 = VirtualRobot::VisualizationFactory::getInstance()->createBox(500.0f, 500.0f, 500.0f);
    cube4->setGlobalPose(getPosition(index++));
    cube4->setColor(VirtualRobot::Visualization::Color(0.0f, 0.0f, 1.0f));
    factoryDemoViewer->addVisualization("testLayer", cube4);

    // SPHERES
    VirtualRobot::VisualizationPtr sphere1 = VirtualRobot::VisualizationFactory::getInstance()->createSphere(250.0f);
    sphere1->setGlobalPose(getPosition(index++));
    factoryDemoViewer->addVisualization("testLayer", sphere1);

    VirtualRobot::VisualizationPtr sphere2 = VirtualRobot::VisualizationFactory::getInstance()->createSphere(250.0f);
    sphere2->setGlobalPose(getPosition(index++));
    sphere2->setColor(VirtualRobot::Visualization::Color(1.0f, 0.0f, 0.0f));
    factoryDemoViewer->addVisualization("testLayer", sphere2);

    VirtualRobot::VisualizationPtr sphere3 = VirtualRobot::VisualizationFactory::getInstance()->createSphere(250.0f);
    sphere3->setGlobalPose(getPosition(index++));
    sphere3->setColor(VirtualRobot::Visualization::Color(0.0f, 1.0f, 0.0f));
    factoryDemoViewer->addVisualization("testLayer", sphere3);

    VirtualRobot::VisualizationPtr sphere4 = VirtualRobot::VisualizationFactory::getInstance()->createSphere(250.0f);
    sphere4->setGlobalPose(getPosition(index++));
    sphere4->setColor(VirtualRobot::Visualization::Color(0.0f, 0.0f, 1.0f));
    factoryDemoViewer->addVisualization("testLayer", sphere4);

    // CIRCLES
    VirtualRobot::VisualizationPtr circle1 = VirtualRobot::VisualizationFactory::getInstance()->createCircle(250.0f, 1.0f, 3.0f);
    circle1->setGlobalPose(getPosition(index++));
    factoryDemoViewer->addVisualization("testLayer", circle1);

    VirtualRobot::VisualizationPtr circle2 = VirtualRobot::VisualizationFactory::getInstance()->createCircle(250.0f, 0.8f, 3.0f);
    circle2->setGlobalPose(getPosition(index++));
    circle2->setColor(VirtualRobot::Visualization::Color(1.0f, 0.0f, 0.0f));
    factoryDemoViewer->addVisualization("testLayer", circle2);

    VirtualRobot::VisualizationPtr circle3 = VirtualRobot::VisualizationFactory::getInstance()->createCircle(250.0f, 0.6f, 3.0f);
    circle3->setGlobalPose(getPosition(index++));
    circle3->setColor(VirtualRobot::Visualization::Color(0.0f, 1.0f, 0.0f));
    factoryDemoViewer->addVisualization("testLayer", circle3);

    VirtualRobot::VisualizationPtr circle4 = VirtualRobot::VisualizationFactory::getInstance()->createCircle(250.0f, 0.4f, 3.0f);
    circle4->setGlobalPose(getPosition(index++));
    circle4->setColor(VirtualRobot::Visualization::Color(0.0f, 0.0f, 1.0f));
    factoryDemoViewer->addVisualization("testLayer", circle4);

    // TORUS
    VirtualRobot::VisualizationPtr torus1 = VirtualRobot::VisualizationFactory::getInstance()->createTorus(250.0f, 50.0f, 1.0f);
    torus1->setGlobalPose(getPosition(index++));
    factoryDemoViewer->addVisualization("testLayer", torus1);

    VirtualRobot::VisualizationPtr torus2 = VirtualRobot::VisualizationFactory::getInstance()->createTorus(250.0f, 50.0f, 0.8f);
    torus2->setGlobalPose(getPosition(index++));
    torus2->setColor(VirtualRobot::Visualization::Color(1.0f, 0.0f, 0.0f));
    factoryDemoViewer->addVisualization("testLayer", torus2);

    VirtualRobot::VisualizationPtr torus3 = VirtualRobot::VisualizationFactory::getInstance()->createTorus(250.0f, 50.0f, 0.6f);
    torus3->setGlobalPose(getPosition(index++));
    torus3->setColor(VirtualRobot::Visualization::Color(0.0f, 1.0f, 0.0f));
    factoryDemoViewer->addVisualization("testLayer", torus3);

    VirtualRobot::VisualizationPtr torus4 = VirtualRobot::VisualizationFactory::getInstance()->createTorus(250.0f, 50.0f, 0.4f);
    torus4->setGlobalPose(getPosition(index++));
    torus4->setColor(VirtualRobot::Visualization::Color(0.0f, 0.0f, 1.0f));
    factoryDemoViewer->addVisualization("testLayer", torus4);

    // CIRCLE ARROW
    VirtualRobot::VisualizationPtr circleArrow1 = VirtualRobot::VisualizationFactory::getInstance()->createCircleArrow(250.0f, 30.0f, 1.0f);
    circleArrow1->setGlobalPose(getPosition(index++));
    factoryDemoViewer->addVisualization("testLayer", circleArrow1);

    VirtualRobot::VisualizationPtr circleArrow2 = VirtualRobot::VisualizationFactory::getInstance()->createCircleArrow(250.0f, 30.0f, 0.8f);
    circleArrow2->setGlobalPose(getPosition(index++));
    circleArrow2->setColor(VirtualRobot::Visualization::Color(1.0f, 0.0f, 0.0f));
    factoryDemoViewer->addVisualization("testLayer", circleArrow2);

    VirtualRobot::VisualizationPtr circleArrow3 = VirtualRobot::VisualizationFactory::getInstance()->createCircleArrow(250.0f, 30.0f, 0.6f);
    circleArrow3->setGlobalPose(getPosition(index++));
    circleArrow3->setColor(VirtualRobot::Visualization::Color(0.0f, 1.0f, 0.0f));
    factoryDemoViewer->addVisualization("testLayer", circleArrow3);

    VirtualRobot::VisualizationPtr circleArrow4 = VirtualRobot::VisualizationFactory::getInstance()->createCircleArrow(250.0f, 30.0f, 0.4f);
    circleArrow4->setGlobalPose(getPosition(index++));
    circleArrow4->setColor(VirtualRobot::Visualization::Color(0.0f, 0.0f, 1.0f));
    factoryDemoViewer->addVisualization("testLayer", circleArrow4);

    // CYLINDER
    VirtualRobot::VisualizationPtr cylinder1 = VirtualRobot::VisualizationFactory::getInstance()->createCylinder(250.0f, 500.0f);
    cylinder1->setGlobalPose(getPosition(index++));
    factoryDemoViewer->addVisualization("testLayer", cylinder1);

    VirtualRobot::VisualizationPtr cylinder2 = VirtualRobot::VisualizationFactory::getInstance()->createCylinder(250.0f, 500.0f);
    cylinder2->setGlobalPose(getPosition(index++));
    cylinder2->setColor(VirtualRobot::Visualization::Color(1.0f, 0.0f, 0.0f));
    factoryDemoViewer->addVisualization("testLayer", cylinder2);

    VirtualRobot::VisualizationPtr cylinder3 = VirtualRobot::VisualizationFactory::getInstance()->createCylinder(250.0f, 500.0f);
    cylinder3->setGlobalPose(getPosition(index++));
    cylinder3->setColor(VirtualRobot::Visualization::Color(0.0f, 1.0f, 0.0f));
    factoryDemoViewer->addVisualization("testLayer", cylinder3);

    VirtualRobot::VisualizationPtr cylinder4 = VirtualRobot::VisualizationFactory::getInstance()->createCylinder(250.0f, 500.0f);
    cylinder4->setGlobalPose(getPosition(index++));
    cylinder4->setColor(VirtualRobot::Visualization::Color(0.0f, 0.0f, 1.0f));
    factoryDemoViewer->addVisualization("testLayer", cylinder4);

    // COORD SYSTEM
    std::string text = "SYSTEM";

    VirtualRobot::VisualizationPtr coordSystem1 = VirtualRobot::VisualizationFactory::getInstance()->createCoordSystem(&text, 500.0f, 9.0f);
    coordSystem1->setGlobalPose(getPosition(index++));
    factoryDemoViewer->addVisualization("testLayer", coordSystem1);

    VirtualRobot::VisualizationPtr coordSystem2 = VirtualRobot::VisualizationFactory::getInstance()->createCoordSystem(&text, 500.0f, 9.0f);
    coordSystem2->setGlobalPose(getPosition(index++));
    //coordSystem2->setColor(VirtualRobot::Visualization::Color(1.0f, 0.0f, 0.0f));
    factoryDemoViewer->addVisualization("testLayer", coordSystem2);

    VirtualRobot::VisualizationPtr coordSystem3 = VirtualRobot::VisualizationFactory::getInstance()->createCoordSystem(&text, 500.0f, 9.0f);
    coordSystem3->setGlobalPose(getPosition(index++));
    //coordSystem3->setColor(VirtualRobot::Visualization::Color(0.0f, 1.0f, 0.0f));
    factoryDemoViewer->addVisualization("testLayer", coordSystem3);

    VirtualRobot::VisualizationPtr coordSystem4 = VirtualRobot::VisualizationFactory::getInstance()->createCoordSystem(&text, 500.0f, 9.0f);
    coordSystem4->setGlobalPose(getPosition(index++));
    //coordSystem4->setColor(VirtualRobot::Visualization::Color(0.0f, 0.0f, 1.0f));
    factoryDemoViewer->addVisualization("testLayer", coordSystem4);

    // POINTCLOUD
    std::random_device rnd_device;
    std::mt19937 mersenne_engine {rnd_device()};
    std::uniform_int_distribution<int> dist {-300, 300};
    auto gen = [&dist, &mersenne_engine](){
                   return Eigen::Vector3f(dist(mersenne_engine), dist(mersenne_engine), dist(mersenne_engine));
               };
    std::vector<Eigen::Vector3f> vec(600);
    std::generate(begin(vec), end(vec), gen);

    VirtualRobot::VisualizationPtr pointCloud1 = VirtualRobot::VisualizationFactory::getInstance()->createPointCloud(vec, 6.0f);
    pointCloud1->setGlobalPose(getPosition(index++));
    factoryDemoViewer->addVisualization("testLayer", pointCloud1);

    VirtualRobot::VisualizationPtr pointCloud2 = VirtualRobot::VisualizationFactory::getInstance()->createPointCloud(vec, 6.0f);
    pointCloud2->setGlobalPose(getPosition(index++));
    pointCloud2->setColor(VirtualRobot::Visualization::Color(1.0f, 0.0f, 0.0f));
    factoryDemoViewer->addVisualization("testLayer", pointCloud2);

    VirtualRobot::VisualizationPtr pointCloud3 = VirtualRobot::VisualizationFactory::getInstance()->createPointCloud(vec, 6.0f);
    pointCloud3->setGlobalPose(getPosition(index++));
    pointCloud3->setColor(VirtualRobot::Visualization::Color(0.0f, 1.0f, 0.0f));
    factoryDemoViewer->addVisualization("testLayer", pointCloud3);

    VirtualRobot::VisualizationPtr pointCloud4 = VirtualRobot::VisualizationFactory::getInstance()->createPointCloud(vec, 6.0f);
    pointCloud4->setGlobalPose(getPosition(index++));
    pointCloud4->setColor(VirtualRobot::Visualization::Color(0.0f, 0.0f, 1.0f));
    factoryDemoViewer->addVisualization("testLayer", pointCloud4);

    // CONE
    VirtualRobot::VisualizationPtr cone1 = VirtualRobot::VisualizationFactory::getInstance()->createCone(400.0f, 500.0f);
    cone1->setGlobalPose(getPosition(index++));
    factoryDemoViewer->addVisualization("testLayer", cone1);

    VirtualRobot::VisualizationPtr cone2 = VirtualRobot::VisualizationFactory::getInstance()->createCone(400.0f, 500.0f);
    cone2->setGlobalPose(getPosition(index++));
    cone2->setColor(VirtualRobot::Visualization::Color(1.0f, 0.0f, 0.0f));
    factoryDemoViewer->addVisualization("testLayer", cone2);

    VirtualRobot::VisualizationPtr cone3 = VirtualRobot::VisualizationFactory::getInstance()->createCone(400.0f, 500.0f);
    cone3->setGlobalPose(getPosition(index++));
    cone3->setColor(VirtualRobot::Visualization::Color(0.0f, 1.0f, 0.0f));
    factoryDemoViewer->addVisualization("testLayer", cone3);

    VirtualRobot::VisualizationPtr cone4 = VirtualRobot::VisualizationFactory::getInstance()->createCone(400.0f, 500.0f);
    cone4->setGlobalPose(getPosition(index++));
    cone4->setColor(VirtualRobot::Visualization::Color(0.0f, 0.0f, 1.0f));
    factoryDemoViewer->addVisualization("testLayer", cone4);

    // ELLIPSE
    VirtualRobot::VisualizationPtr ellipse1 = VirtualRobot::VisualizationFactory::getInstance()->createEllipse(300.0f, 200.0f, 100.0f);
    ellipse1->setGlobalPose(getPosition(index++));
    factoryDemoViewer->addVisualization("testLayer", ellipse1);

    VirtualRobot::VisualizationPtr ellipse2 = VirtualRobot::VisualizationFactory::getInstance()->createEllipse(300.0f, 200.0f, 100.0f);
    ellipse2->setGlobalPose(getPosition(index++));
    ellipse2->setColor(VirtualRobot::Visualization::Color(1.0f, 0.0f, 0.0f));
    factoryDemoViewer->addVisualization("testLayer", ellipse2);

    VirtualRobot::VisualizationPtr ellipse3 = VirtualRobot::VisualizationFactory::getInstance()->createEllipse(300.0f, 200.0f, 100.0f);
    ellipse3->setGlobalPose(getPosition(index++));
    ellipse3->setColor(VirtualRobot::Visualization::Color(0.0f, 1.0f, 0.0f));
    factoryDemoViewer->addVisualization("testLayer", ellipse3);

    VirtualRobot::VisualizationPtr ellipse4 = VirtualRobot::VisualizationFactory::getInstance()->createEllipse(300.0f, 200.0f, 100.0f);
    ellipse4->setGlobalPose(getPosition(index++));
    ellipse4->setColor(VirtualRobot::Visualization::Color(0.0f, 0.0f, 1.0f));
    factoryDemoViewer->addVisualization("testLayer", ellipse4);

    // ARROW
    VirtualRobot::VisualizationPtr arrow1 = VirtualRobot::VisualizationFactory::getInstance()->createArrow(Eigen::Vector3f(0.0f, 1.0f, 0.0f), 500.0f, 50.0f);
    arrow1->setGlobalPose(getPosition(index++));
    factoryDemoViewer->addVisualization("testLayer", arrow1);

    VirtualRobot::VisualizationPtr arrow2 = VirtualRobot::VisualizationFactory::getInstance()->createArrow(Eigen::Vector3f(1.0f, 0.0f, 0.0f), 500.0f, 50.0f);
    arrow2->setGlobalPose(getPosition(index++));
    arrow2->setColor(VirtualRobot::Visualization::Color(1.0f, 0.0f, 0.0f));
    factoryDemoViewer->addVisualization("testLayer", arrow2);

    VirtualRobot::VisualizationPtr arrow3 = VirtualRobot::VisualizationFactory::getInstance()->createArrow(Eigen::Vector3f(0.0f, 0.0f, 1.0f), 500.0f, 50.0f);
    arrow3->setGlobalPose(getPosition(index++));
    arrow3->setColor(VirtualRobot::Visualization::Color(0.0f, 1.0f, 0.0f));
    factoryDemoViewer->addVisualization("testLayer", arrow3);

    VirtualRobot::VisualizationPtr arrow4 = VirtualRobot::VisualizationFactory::getInstance()->createArrow(Eigen::Vector3f(1.0f, 1.0f, 0.0f), 500.0f, 50.0f);
    arrow4->setGlobalPose(getPosition(index++));
    arrow4->setColor(VirtualRobot::Visualization::Color(0.0f, 0.0f, 1.0f));
    factoryDemoViewer->addVisualization("testLayer", arrow4);

    // TEXT
    std::string text1string = "H";

    VirtualRobot::VisualizationPtr text1 = VirtualRobot::VisualizationFactory::getInstance()->createText("H", false, 0.0f, 0.0f, 0.0f);
    text1->scale(20.0f);
    text1->setGlobalPose(getPosition(index++));
    factoryDemoViewer->addVisualization("testLayer", text1);

    VirtualRobot::VisualizationPtr text2 = VirtualRobot::VisualizationFactory::getInstance()->createText("E", false, 0.0f, 0.0f, 0.0f);
    text2->scale(20.0f);
    text2->setGlobalPose(getPosition(index++));
    text2->setColor(VirtualRobot::Visualization::Color(1.0f, 0.0f, 0.0f));
    factoryDemoViewer->addVisualization("testLayer", text2);

    VirtualRobot::VisualizationPtr text3 = VirtualRobot::VisualizationFactory::getInstance()->createText("L", false, 0.0f, 0.0f, 0.0f);
    text3->scale(20.0f);
    text3->setGlobalPose(getPosition(index++));
    text3->setColor(VirtualRobot::Visualization::Color(0.0f, 1.0f, 0.0f));
    factoryDemoViewer->addVisualization("testLayer", text3);

    VirtualRobot::VisualizationPtr text4 = VirtualRobot::VisualizationFactory::getInstance()->createText("P", false, 0.0f, 0.0f, 0.0f);
    text4->scale(20.0f);
    text4->setGlobalPose(getPosition(index++));
    text4->setColor(VirtualRobot::Visualization::Color(0.0f, 0.0f, 1.0f));
    factoryDemoViewer->addVisualization("testLayer", text4);

    factoryDemoViewer->viewAll();
}

int FullDemoWindow::main()
{

}


