#include "FullDemoWindow.h"
#include "../../VirtualRobotException.h"


#include "../../Visualization/VisualizationFactory.h"

#include <random>
#include <algorithm>
#include <iterator>
#include <functional>

#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <VirtualRobot/Visualization/OffscreenRenderer.h>

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

Eigen::Matrix4f getPosition2(int index)
{
    Eigen::Matrix4f mat;

    mat <<
         1.0f, 0.0f, 0.0f, 200.0f + (index / 2) * 600.0f + (index % 2) * 150.0f,
         0.0f, 1.0f, 0.0f, 80.0f,
         0.0f, 0.0f, 1.0f, 800.0f - (index % 2) * 400.0f,
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
    screenshotViewer = viewerFactory->createViewer(UI.frameViewer2);
    triMeshViewer = viewerFactory->createViewer(UI.frameViewer3);

    {
        int index = 0;

        // CUBES
        VirtualRobot::VisualizationPtr cube1 = VirtualRobot::VisualizationFactory::getInstance()->createBox(500.0f, 500.0f, 500.0f);
        cube1->setGlobalPose(getPosition(index++));
        factoryDemoViewer->addVisualization(cube1, "testLayer");

        VirtualRobot::VisualizationPtr cube2 = VirtualRobot::VisualizationFactory::getInstance()->createBox(500.0f, 500.0f, 500.0f);
        cube2->setGlobalPose(getPosition(index++));
        cube2->setColor(VirtualRobot::Visualization::Color(1.0f, 0.0f, 0.0f));
        factoryDemoViewer->addVisualization(cube2, "testLayer");

        VirtualRobot::VisualizationPtr cube3 = VirtualRobot::VisualizationFactory::getInstance()->createBox(500.0f, 500.0f, 500.0f);
        cube3->setGlobalPose(getPosition(index++));
        cube3->setColor(VirtualRobot::Visualization::Color(0.0f, 1.0f, 0.0f));
        factoryDemoViewer->addVisualization(cube3, "testLayer");

        VirtualRobot::VisualizationPtr cube4 = VirtualRobot::VisualizationFactory::getInstance()->createBox(500.0f, 500.0f, 500.0f);
        cube4->setGlobalPose(getPosition(index++));
        cube4->setColor(VirtualRobot::Visualization::Color(0.0f, 0.0f, 1.0f));
        factoryDemoViewer->addVisualization(cube4, "testLayer");

        // SPHERES
        VirtualRobot::VisualizationPtr sphere1 = VirtualRobot::VisualizationFactory::getInstance()->createSphere(250.0f);
        sphere1->setGlobalPose(getPosition(index++));
        factoryDemoViewer->addVisualization(sphere1, "testLayer");

        VirtualRobot::VisualizationPtr sphere2 = VirtualRobot::VisualizationFactory::getInstance()->createSphere(250.0f);
        sphere2->setGlobalPose(getPosition(index++));
        sphere2->setColor(VirtualRobot::Visualization::Color(1.0f, 0.0f, 0.0f));
        factoryDemoViewer->addVisualization(sphere2, "testLayer");

        VirtualRobot::VisualizationPtr sphere3 = VirtualRobot::VisualizationFactory::getInstance()->createSphere(250.0f);
        sphere3->setGlobalPose(getPosition(index++));
        sphere3->setColor(VirtualRobot::Visualization::Color(0.0f, 1.0f, 0.0f));
        factoryDemoViewer->addVisualization(sphere3, "testLayer");

        VirtualRobot::VisualizationPtr sphere4 = VirtualRobot::VisualizationFactory::getInstance()->createSphere(250.0f);
        sphere4->setGlobalPose(getPosition(index++));
        sphere4->setColor(VirtualRobot::Visualization::Color(0.0f, 0.0f, 1.0f));
        factoryDemoViewer->addVisualization(sphere4, "testLayer");

        // CIRCLES
        VirtualRobot::VisualizationPtr circle1 = VirtualRobot::VisualizationFactory::getInstance()->createCircle(250.0f, 1.0f, 3.0f);
        circle1->setGlobalPose(getPosition(index++));
        factoryDemoViewer->addVisualization(circle1, "testLayer");

        VirtualRobot::VisualizationPtr circle2 = VirtualRobot::VisualizationFactory::getInstance()->createCircle(250.0f, 0.8f, 3.0f);
        circle2->setGlobalPose(getPosition(index++));
        circle2->setColor(VirtualRobot::Visualization::Color(1.0f, 0.0f, 0.0f));
        factoryDemoViewer->addVisualization(circle2, "testLayer");

        VirtualRobot::VisualizationPtr circle3 = VirtualRobot::VisualizationFactory::getInstance()->createCircle(250.0f, 0.6f, 3.0f);
        circle3->setGlobalPose(getPosition(index++));
        circle3->setColor(VirtualRobot::Visualization::Color(0.0f, 1.0f, 0.0f));
        factoryDemoViewer->addVisualization(circle3, "testLayer");

        VirtualRobot::VisualizationPtr circle4 = VirtualRobot::VisualizationFactory::getInstance()->createCircle(250.0f, 0.4f, 3.0f);
        circle4->setGlobalPose(getPosition(index++));
        circle4->setColor(VirtualRobot::Visualization::Color(0.0f, 0.0f, 1.0f));
        factoryDemoViewer->addVisualization(circle4, "testLayer");

        // TORUS
        VirtualRobot::VisualizationPtr torus1 = VirtualRobot::VisualizationFactory::getInstance()->createTorus(250.0f, 50.0f, 1.0f);
        torus1->setGlobalPose(getPosition(index++));
        factoryDemoViewer->addVisualization(torus1, "testLayer");

        VirtualRobot::VisualizationPtr torus2 = VirtualRobot::VisualizationFactory::getInstance()->createTorus(250.0f, 50.0f, 0.8f);
        torus2->setGlobalPose(getPosition(index++));
        torus2->setColor(VirtualRobot::Visualization::Color(1.0f, 0.0f, 0.0f));
        factoryDemoViewer->addVisualization(torus2, "testLayer");

        VirtualRobot::VisualizationPtr torus3 = VirtualRobot::VisualizationFactory::getInstance()->createTorus(250.0f, 50.0f, 0.6f);
        torus3->setGlobalPose(getPosition(index++));
        torus3->setColor(VirtualRobot::Visualization::Color(0.0f, 1.0f, 0.0f));
        factoryDemoViewer->addVisualization(torus3, "testLayer");

        VirtualRobot::VisualizationPtr torus4 = VirtualRobot::VisualizationFactory::getInstance()->createTorus(250.0f, 50.0f, 0.4f);
        torus4->setGlobalPose(getPosition(index++));
        torus4->setColor(VirtualRobot::Visualization::Color(0.0f, 0.0f, 1.0f));
        factoryDemoViewer->addVisualization(torus4, "testLayer");

        // CIRCLE ARROW
        VirtualRobot::VisualizationPtr circleArrow1 = VirtualRobot::VisualizationFactory::getInstance()->createCircleArrow(250.0f, 30.0f, 1.0f);
        circleArrow1->setGlobalPose(getPosition(index++));
        factoryDemoViewer->addVisualization(circleArrow1, "testLayer");

        VirtualRobot::VisualizationPtr circleArrow2 = VirtualRobot::VisualizationFactory::getInstance()->createCircleArrow(250.0f, 30.0f, 0.8f);
        circleArrow2->setGlobalPose(getPosition(index++));
        circleArrow2->setColor(VirtualRobot::Visualization::Color(1.0f, 0.0f, 0.0f));
        factoryDemoViewer->addVisualization(circleArrow2, "testLayer");

        VirtualRobot::VisualizationPtr circleArrow3 = VirtualRobot::VisualizationFactory::getInstance()->createCircleArrow(250.0f, 30.0f, 0.6f);
        circleArrow3->setGlobalPose(getPosition(index++));
        circleArrow3->setColor(VirtualRobot::Visualization::Color(0.0f, 1.0f, 0.0f));
        factoryDemoViewer->addVisualization(circleArrow3, "testLayer");

        VirtualRobot::VisualizationPtr circleArrow4 = VirtualRobot::VisualizationFactory::getInstance()->createCircleArrow(250.0f, 30.0f, 0.4f);
        circleArrow4->setGlobalPose(getPosition(index++));
        circleArrow4->setColor(VirtualRobot::Visualization::Color(0.0f, 0.0f, 1.0f));
        factoryDemoViewer->addVisualization(circleArrow4, "testLayer");

        // CYLINDER
        VirtualRobot::VisualizationPtr cylinder1 = VirtualRobot::VisualizationFactory::getInstance()->createCylinder(250.0f, 500.0f);
        cylinder1->setGlobalPose(getPosition(index++));
        factoryDemoViewer->addVisualization(cylinder1, "testLayer");

        VirtualRobot::VisualizationPtr cylinder2 = VirtualRobot::VisualizationFactory::getInstance()->createCylinder(250.0f, 500.0f);
        cylinder2->setGlobalPose(getPosition(index++));
        cylinder2->setColor(VirtualRobot::Visualization::Color(1.0f, 0.0f, 0.0f));
        factoryDemoViewer->addVisualization(cylinder2, "testLayer");

        VirtualRobot::VisualizationPtr cylinder3 = VirtualRobot::VisualizationFactory::getInstance()->createCylinder(250.0f, 500.0f);
        cylinder3->setGlobalPose(getPosition(index++));
        cylinder3->setColor(VirtualRobot::Visualization::Color(0.0f, 1.0f, 0.0f));
        factoryDemoViewer->addVisualization(cylinder3, "testLayer");

        VirtualRobot::VisualizationPtr cylinder4 = VirtualRobot::VisualizationFactory::getInstance()->createCylinder(250.0f, 500.0f);
        cylinder4->setGlobalPose(getPosition(index++));
        cylinder4->setColor(VirtualRobot::Visualization::Color(0.0f, 0.0f, 1.0f));
        factoryDemoViewer->addVisualization(cylinder4, "testLayer");

        // COORD SYSTEM
        std::string text = "SYSTEM";

        VirtualRobot::VisualizationPtr coordSystem1 = VirtualRobot::VisualizationFactory::getInstance()->createCoordSystem(&text, 500.0f, 9.0f);
        coordSystem1->setGlobalPose(getPosition(index++));
        factoryDemoViewer->addVisualization(coordSystem1, "testLayer");

        VirtualRobot::VisualizationPtr coordSystem2 = VirtualRobot::VisualizationFactory::getInstance()->createCoordSystem(&text, 500.0f, 9.0f);
        coordSystem2->setGlobalPose(getPosition(index++));
        //coordSystem2->setColor(VirtualRobot::Visualization::Color(1.0f, 0.0f, 0.0f));
        factoryDemoViewer->addVisualization(coordSystem2, "testLayer");

        VirtualRobot::VisualizationPtr coordSystem3 = VirtualRobot::VisualizationFactory::getInstance()->createCoordSystem(&text, 500.0f, 9.0f);
        coordSystem3->setGlobalPose(getPosition(index++));
        //coordSystem3->setColor(VirtualRobot::Visualization::Color(0.0f, 1.0f, 0.0f));
        factoryDemoViewer->addVisualization(coordSystem3, "testLayer");

        VirtualRobot::VisualizationPtr coordSystem4 = VirtualRobot::VisualizationFactory::getInstance()->createCoordSystem(&text, 500.0f, 9.0f);
        coordSystem4->setGlobalPose(getPosition(index++));
        //coordSystem4->setColor(VirtualRobot::Visualization::Color(0.0f, 0.0f, 1.0f));
        factoryDemoViewer->addVisualization(coordSystem4, "testLayer");

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
        factoryDemoViewer->addVisualization(pointCloud1, "testLayer");

        VirtualRobot::VisualizationPtr pointCloud2 = VirtualRobot::VisualizationFactory::getInstance()->createPointCloud(vec, 6.0f);
        pointCloud2->setGlobalPose(getPosition(index++));
        pointCloud2->setColor(VirtualRobot::Visualization::Color(1.0f, 0.0f, 0.0f));
        factoryDemoViewer->addVisualization(pointCloud2, "testLayer");

        VirtualRobot::VisualizationPtr pointCloud3 = VirtualRobot::VisualizationFactory::getInstance()->createPointCloud(vec, 6.0f);
        pointCloud3->setGlobalPose(getPosition(index++));
        pointCloud3->setColor(VirtualRobot::Visualization::Color(0.0f, 1.0f, 0.0f));
        factoryDemoViewer->addVisualization(pointCloud3, "testLayer");

        VirtualRobot::VisualizationPtr pointCloud4 = VirtualRobot::VisualizationFactory::getInstance()->createPointCloud(vec, 6.0f);
        pointCloud4->setGlobalPose(getPosition(index++));
        pointCloud4->setColor(VirtualRobot::Visualization::Color(0.0f, 0.0f, 1.0f));
        factoryDemoViewer->addVisualization(pointCloud4, "testLayer");

        // CONE
        VirtualRobot::VisualizationPtr cone1 = VirtualRobot::VisualizationFactory::getInstance()->createCone(400.0f, 500.0f);
        cone1->setGlobalPose(getPosition(index++));
        factoryDemoViewer->addVisualization(cone1, "testLayer");

        VirtualRobot::VisualizationPtr cone2 = VirtualRobot::VisualizationFactory::getInstance()->createCone(400.0f, 500.0f);
        cone2->setGlobalPose(getPosition(index++));
        cone2->setColor(VirtualRobot::Visualization::Color(1.0f, 0.0f, 0.0f));
        factoryDemoViewer->addVisualization(cone2, "testLayer");

        VirtualRobot::VisualizationPtr cone3 = VirtualRobot::VisualizationFactory::getInstance()->createCone(400.0f, 500.0f);
        cone3->setGlobalPose(getPosition(index++));
        cone3->setColor(VirtualRobot::Visualization::Color(0.0f, 1.0f, 0.0f));
        factoryDemoViewer->addVisualization(cone3, "testLayer");

        VirtualRobot::VisualizationPtr cone4 = VirtualRobot::VisualizationFactory::getInstance()->createCone(400.0f, 500.0f);
        cone4->setGlobalPose(getPosition(index++));
        cone4->setColor(VirtualRobot::Visualization::Color(0.0f, 0.0f, 1.0f));
        factoryDemoViewer->addVisualization(cone4, "testLayer");

        // ELLIPSE
        VirtualRobot::VisualizationPtr ellipse1 = VirtualRobot::VisualizationFactory::getInstance()->createEllipse(300.0f, 200.0f, 100.0f);
        ellipse1->setGlobalPose(getPosition(index++));
        factoryDemoViewer->addVisualization(ellipse1, "testLayer");

        VirtualRobot::VisualizationPtr ellipse2 = VirtualRobot::VisualizationFactory::getInstance()->createEllipse(300.0f, 200.0f, 100.0f);
        ellipse2->setGlobalPose(getPosition(index++));
        ellipse2->setColor(VirtualRobot::Visualization::Color(1.0f, 0.0f, 0.0f));
        factoryDemoViewer->addVisualization(ellipse2, "testLayer");

        VirtualRobot::VisualizationPtr ellipse3 = VirtualRobot::VisualizationFactory::getInstance()->createEllipse(300.0f, 200.0f, 100.0f);
        ellipse3->setGlobalPose(getPosition(index++));
        ellipse3->setColor(VirtualRobot::Visualization::Color(0.0f, 1.0f, 0.0f));
        factoryDemoViewer->addVisualization(ellipse3, "testLayer");

        VirtualRobot::VisualizationPtr ellipse4 = VirtualRobot::VisualizationFactory::getInstance()->createEllipse(300.0f, 200.0f, 100.0f);
        ellipse4->setGlobalPose(getPosition(index++));
        ellipse4->setColor(VirtualRobot::Visualization::Color(0.0f, 0.0f, 1.0f));
        factoryDemoViewer->addVisualization(ellipse4, "testLayer");

        // ARROW
        VirtualRobot::VisualizationPtr arrow1 = VirtualRobot::VisualizationFactory::getInstance()->createArrow(Eigen::Vector3f(0.0f, 1.0f, 0.0f), 500.0f, 50.0f);
        arrow1->setGlobalPose(getPosition(index++));
        factoryDemoViewer->addVisualization(arrow1, "testLayer");

        VirtualRobot::VisualizationPtr arrow2 = VirtualRobot::VisualizationFactory::getInstance()->createArrow(Eigen::Vector3f(1.0f, 0.0f, 0.0f), 500.0f, 50.0f);
        arrow2->setGlobalPose(getPosition(index++));
        arrow2->setColor(VirtualRobot::Visualization::Color(1.0f, 0.0f, 0.0f));
        factoryDemoViewer->addVisualization(arrow2, "testLayer");

        VirtualRobot::VisualizationPtr arrow3 = VirtualRobot::VisualizationFactory::getInstance()->createArrow(Eigen::Vector3f(0.0f, 0.0f, 1.0f), 500.0f, 50.0f);
        arrow3->setGlobalPose(getPosition(index++));
        arrow3->setColor(VirtualRobot::Visualization::Color(0.0f, 1.0f, 0.0f));
        factoryDemoViewer->addVisualization(arrow3, "testLayer");

        VirtualRobot::VisualizationPtr arrow4 = VirtualRobot::VisualizationFactory::getInstance()->createArrow(Eigen::Vector3f(1.0f, 1.0f, 0.0f), 500.0f, 50.0f);
        arrow4->setGlobalPose(getPosition(index++));
        arrow4->setColor(VirtualRobot::Visualization::Color(0.0f, 0.0f, 1.0f));
        factoryDemoViewer->addVisualization(arrow4, "testLayer");

        // TEXT
        std::string text1string = "H";

        VirtualRobot::VisualizationPtr text1 = VirtualRobot::VisualizationFactory::getInstance()->createText("H", false, 0.0f, 0.0f, 0.0f);
        text1->scale(20.0f);
        text1->setGlobalPose(getPosition(index++));
        factoryDemoViewer->addVisualization(text1, "testLayer");

        VirtualRobot::VisualizationPtr text2 = VirtualRobot::VisualizationFactory::getInstance()->createText("E", false, 0.0f, 0.0f, 0.0f);
        text2->scale(20.0f);
        text2->setGlobalPose(getPosition(index++));
        text2->setColor(VirtualRobot::Visualization::Color(1.0f, 0.0f, 0.0f));
        factoryDemoViewer->addVisualization(text2, "testLayer");

        VirtualRobot::VisualizationPtr text3 = VirtualRobot::VisualizationFactory::getInstance()->createText("L", false, 0.0f, 0.0f, 0.0f);
        text3->scale(20.0f);
        text3->setGlobalPose(getPosition(index++));
        text3->setColor(VirtualRobot::Visualization::Color(0.0f, 1.0f, 0.0f));
        factoryDemoViewer->addVisualization(text3, "testLayer");

        VirtualRobot::VisualizationPtr text4 = VirtualRobot::VisualizationFactory::getInstance()->createText("P", false, 0.0f, 0.0f, 0.0f);
        text4->scale(20.0f);
        text4->setGlobalPose(getPosition(index++));
        text4->setColor(VirtualRobot::Visualization::Color(0.0f, 0.0f, 1.0f));
        factoryDemoViewer->addVisualization(text4, "testLayer");

        factoryDemoViewer->viewAll();
    }

    {
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
        screenshotViewer->addVisualization(pointCloud1, "testLayer");
        screenshotViewer->viewAll();
    }

    {
        int index = 0;
        auto text1 = VirtualRobot::VisualizationFactory::getInstance()->createText("Simple TriMesh", false, 0.0f, 0.0f, 0.0f);
        text1->setGlobalPose(getPosition2(index++));
        text1->scale(3.0f);
        triMeshViewer->addVisualization(text1, "test");

        VirtualRobot::TriMeshModelPtr triMesh = VirtualRobot::TriMeshModelPtr(new VirtualRobot::TriMeshModel());
        triMesh->addVertex(Eigen::Vector3f(0.0f, 0.0f, 0.0f));
        triMesh->addVertex(Eigen::Vector3f(1.0f, 0.0f, 0.0f));
        triMesh->addVertex(Eigen::Vector3f(0.0f, 1.0f, 0.0f));
        triMesh->addVertex(Eigen::Vector3f(1.0f, 1.0f, 0.0f));
        triMesh->addFace(0, 1, 3);
        triMesh->addFace(3, 2, 0);
        VirtualRobot::VisualizationPtr triMeshVisu = triMesh->getVisualization(false, true);
        triMeshVisu->scale(100.0f);
        triMeshVisu->setGlobalPose(getPosition2(index++));
        triMeshViewer->addVisualization(triMeshVisu, "test");

        VirtualRobot::VisualizationPtr text2 = VirtualRobot::VisualizationFactory::getInstance()->createText("Complex Model", false, 0.0f, 0.0f, 0.0f);
        text2->setGlobalPose(getPosition2(index++));
        text2->scale(3.0f);
        triMeshViewer->addVisualization(text2, "test");

        model = VirtualRobot::VisualizationFactory::getInstance()->createVisualizationFromFile("/home/philipp/simox/data/robots/Armar4/urdf/stl/Head_2_joint_visu.dae");
        //model = VirtualRobot::VisualizationFactory::getInstance()->createBox(1.0f, 1.0f, 1.0f);
        model->scale(2500.0f);
        model->setGlobalPose(getPosition2(index++));
        model->applyDisplacement(VirtualRobot::MathTools::rpy2eigen4f(0.0f, 3 * M_PI / 4.0f, 0.0f));
        triMeshViewer->addVisualization(model, "test");

        VirtualRobot::VisualizationPtr text3 = VirtualRobot::VisualizationFactory::getInstance()->createText("Complex TriMesh", false, 0.0f, 0.0f, 0.0f);
        text3->setGlobalPose(getPosition2(index++));
        text3->scale(3.0f);
        triMeshViewer->addVisualization(text3, "test");

        triMeshViewer->viewAll();
    }
}

int FullDemoWindow::main()
{

}

void FullDemoWindow::on_pushButton_clicked()
{
    QImage image = screenshotViewer->getScreenshot();
    UI.label->setPixmap(QPixmap::fromImage(image));
}
void FullDemoWindow::on_pushButton_2_clicked()
{
    VirtualRobot::TriMeshModelPtr triMesh = model->getTriMeshModel();
    auto visu = triMesh->getVisualization(false, true);
    visu->setGlobalPose(getPosition2(5));
    visu->applyDisplacement(VirtualRobot::MathTools::rpy2eigen4f(0.0f, (3 * M_PI) / 4.0f, 0.0f));
    triMeshViewer->addVisualization(visu, "test");
}

void FullDemoWindow::on_pushButton_3_clicked()
{
    std::vector<VirtualRobot::VisualizationPtr> scene;
    scene.push_back(VirtualRobot::VisualizationFactory::getInstance()->createBox(500.0f, 500.0f, 500.0f));

    std::vector<unsigned char> buffer;
    Eigen::Matrix4f mat;
    mat <<
         1.0f, 0.0f, 0.0f, 0.0f,
         0.0f, 1.0f, 0.0f, 0.0f,
         0.0f, 0.0f, 1.0f, 0.0f,
         0.0f, 0.0f, 0.0f, 1.0f;

    VirtualRobot::OffscreenRenderer::getInstance()->renderOffscreenRgbImage(mat, scene, UI.label01->size().width(), UI.label01->size().height(), buffer);
    QImage img01(buffer.data(), UI.label01->size().width(), UI.label01->size().height(), QImage::Format_RGB888);
    UI.label01->setPixmap(QPixmap::fromImage(img01));

    VirtualRobot::OffscreenRenderer::getInstance()->renderOffscreenRgbImage(mat, scene, UI.label02->size().width(), UI.label02->size().height(), buffer);
    QImage img02(buffer.data(), UI.label02->size().width(), UI.label02->size().height(), QImage::Format_RGB888);
    UI.label02->setPixmap(QPixmap::fromImage(img02));

    VirtualRobot::OffscreenRenderer::getInstance()->renderOffscreenRgbImage(mat, scene, UI.label03->size().width(), UI.label03->size().height(), buffer);
    QImage img03(buffer.data(), UI.label03->size().width(), UI.label03->size().height(), QImage::Format_RGB888);
    UI.label03->setPixmap(QPixmap::fromImage(img03));

    VirtualRobot::OffscreenRenderer::getInstance()->renderOffscreenRgbImage(mat, scene, UI.label04->size().width(), UI.label04->size().height(), buffer);
    QImage img04(buffer.data(), UI.label04->size().width(), UI.label04->size().height(), QImage::Format_RGB888);
    UI.label04->setPixmap(QPixmap::fromImage(img04));

    VirtualRobot::OffscreenRenderer::getInstance()->renderOffscreenRgbImage(mat, scene, UI.label05->size().width(), UI.label05->size().height(), buffer);
    QImage img05(buffer.data(), UI.label05->size().width(), UI.label05->size().height(), QImage::Format_RGB888);
    UI.label05->setPixmap(QPixmap::fromImage(img05));

    VirtualRobot::OffscreenRenderer::getInstance()->renderOffscreenRgbImage(mat, scene, UI.label06->size().width(), UI.label06->size().height(), buffer);
    QImage img06(buffer.data(), UI.label06->size().width(), UI.label06->size().height(), QImage::Format_RGB888);
    UI.label06->setPixmap(QPixmap::fromImage(img06));

    VirtualRobot::OffscreenRenderer::getInstance()->renderOffscreenRgbImage(mat, scene, UI.label07->size().width(), UI.label07->size().height(), buffer);
    QImage img07(buffer.data(), UI.label07->size().width(), UI.label07->size().height(), QImage::Format_RGB888);
    UI.label07->setPixmap(QPixmap::fromImage(img07));

    VirtualRobot::OffscreenRenderer::getInstance()->renderOffscreenRgbImage(mat, scene, UI.label08->size().width(), UI.label08->size().height(), buffer);
    QImage img08(buffer.data(), UI.label08->size().width(), UI.label08->size().height(), QImage::Format_RGB888);
    UI.label08->setPixmap(QPixmap::fromImage(img08));

    VirtualRobot::OffscreenRenderer::getInstance()->renderOffscreenRgbImage(mat, scene, UI.label09->size().width(), UI.label09->size().height(), buffer);
    QImage img09(buffer.data(), UI.label09->size().width(), UI.label09->size().height(), QImage::Format_RGB888);
    UI.label09->setPixmap(QPixmap::fromImage(img09));
}
