
#include "showDemoWindow.h"

#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Visualization/SelectionManager.h>

using namespace VirtualRobot;

showDemoWindow::showDemoWindow()
    : QMainWindow(nullptr)
{
    setupUI();

    viewer->viewAll();
}


showDemoWindow::~showDemoWindow()
{
}

void showDemoWindow::setupUI()
{
    UI.setupUi(this);

    SimoxGui::ViewerFactoryPtr viewerFactory = SimoxGui::ViewerFactory::getInstance();
    THROW_VR_EXCEPTION_IF(!viewerFactory,"No viewer factory?!");
    viewer = viewerFactory->createViewer(UI.frameViewer);

    Eigen::Matrix4f disp = Eigen::Matrix4f::Zero();
    disp(0, 3) = 1100;
    auto factory = VisualizationFactory::getInstance();
    box_red = factory->createBox(1000, 1000, 1000);
    box_red->setGlobalPose(Eigen::Matrix4f::Identity() + 0*disp);
    box_red->addSelectionChangedCallback([this](bool selected)
    {
        UI.checkBox_box_red->setChecked(selected);
    });
    viewer->addVisualization("boxes", box_red);
    box_green = factory->createBox(1000, 1000, 1000);
    box_green->setColor(Visualization::Color::Green());
    box_green->setGlobalPose(Eigen::Matrix4f::Identity() + 1*disp);
    box_green->addSelectionChangedCallback([this](bool selected)
    {
        UI.checkBox_box_green->setChecked(selected);
    });
    viewer->addVisualization("boxes", box_green);
    box_blue = factory->createBox(1000, 1000, 1000);
    box_blue->setColor(Visualization::Color::Blue());
    box_blue->setGlobalPose(Eigen::Matrix4f::Identity() + 2*disp);
    box_blue->addSelectionChangedCallback([this](bool selected)
    {
        UI.checkBox_box_blue->setChecked(selected);
    });
    viewer->addVisualization("boxes", box_blue);

    cylinder_red = factory->createCylinder(500, 1000);
    cylinder_red->setColor(Visualization::Color::Red());
    cylinder_red->setGlobalPose(Eigen::Matrix4f::Identity() + 3*disp);
    cylinder_red->addSelectionChangedCallback([this](bool selected)
    {
        UI.checkBox_cylinder_red->setChecked(selected);
    });
    viewer->addVisualization("cylinders", cylinder_red);
    cylinder_green = factory->createCylinder(500, 1000);
    cylinder_green->setColor(Visualization::Color::Green());
    cylinder_green->setGlobalPose(Eigen::Matrix4f::Identity() + 4*disp);
    cylinder_green->addSelectionChangedCallback([this](bool selected)
    {
        UI.checkBox_cylinder_green->setChecked(selected);
    });
    viewer->addVisualization("cylinders", cylinder_green);

    cone_blue = factory->createCone(500, 1000);
    cone_blue->setColor(Visualization::Color::Blue());
    cone_blue->setGlobalPose(Eigen::Matrix4f::Identity() + 5*disp);
    cone_blue->addSelectionChangedCallback([this](bool selected)
    {
        UI.checkBox_cone_blue->setChecked(selected);
    });
    viewer->addVisualization("cones", cone_blue);

    boxes = factory->createVisualisationSet({box_red, box_green, box_blue});
    cylinders = factory->createVisualisationSet({cylinder_red, cylinder_green});
    cones = factory->createVisualisationSet({cone_blue});

    red = factory->createVisualisationSet({box_red, cylinder_red});
    red->setColor(Visualization::Color::Red());
    green = factory->createVisualisationSet({box_green, cylinder_green});
    blue = factory->createVisualisationSet({box_blue, cone_blue});

    connect(UI.buttonGroup_selectionMode, SIGNAL(buttonClicked(QAbstractButton *)), this, SLOT(selectionModeChanged(QAbstractButton *)));

    connect(UI.checkBox_box_red, SIGNAL(clicked(bool)), this, SLOT(selectionButtonChecked_box_red(bool)));
    connect(UI.checkBox_box_green, SIGNAL(clicked(bool)), this, SLOT(selectionButtonChecked_box_green(bool)));
    connect(UI.checkBox_box_blue, SIGNAL(clicked(bool)), this, SLOT(selectionButtonChecked_box_blue(bool)));
    connect(UI.checkBox_cylinder_red, SIGNAL(clicked(bool)), this, SLOT(selectionButtonChecked_cylinder_red(bool)));
    connect(UI.checkBox_cylinder_green, SIGNAL(clicked(bool)), this, SLOT(selectionButtonChecked_cylinder_green(bool)));
    connect(UI.checkBox_cone_blue, SIGNAL(clicked(bool)), this, SLOT(selectionButtonChecked_cone_blue(bool)));

    connect(UI.radioButton_groupShapes, SIGNAL(clicked()), this, SLOT(groupShapes()));
    connect(UI.radioButton_groupColors, SIGNAL(clicked()), this, SLOT(groupColors()));
    connect(UI.radioButton_ungroup, SIGNAL(clicked()), this, SLOT(ungroup()));
}

void showDemoWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}

void showDemoWindow::selectionModeChanged(QAbstractButton *button)
{
    VirtualRobot::SelectionManager::SelectionMode mode;
    if (button == UI.radioButton_eNone)
    {
        std::cout << "selection mode: eNone" << std::endl;
        mode = VirtualRobot::SelectionManager::SelectionMode::eNone;
    }
    else if (button == UI.radioButton_eSingle)
    {
        std::cout << "selection mode: eSingle" << std::endl;
        mode = VirtualRobot::SelectionManager::SelectionMode::eSingle;
    }
    else if (button == UI.radioButton_eMultiple)
    {
        std::cout << "selection mode: eMultiple" << std::endl;
        mode = VirtualRobot::SelectionManager::SelectionMode::eMultiple;
    }
    VirtualRobot::SelectionManager::getInstance()->setSelectionMode(mode);
}

void showDemoWindow::selectionButtonChecked_box_red(bool checked)
{
    std::cout << (checked ? "select: " : "deselect: ") << " red box" << std::endl;
    box_red->setSelected(checked);
    UI.checkBox_box_red->setChecked(box_red->isSelected());
}

void showDemoWindow::selectionButtonChecked_box_green(bool checked)
{
    std::cout << (checked ? "select: " : "deselect: ") << " green box" << std::endl;
    box_green->setSelected(checked);
    UI.checkBox_box_green->setChecked(box_green->isSelected());
}

void showDemoWindow::selectionButtonChecked_box_blue(bool checked)
{
    std::cout << (checked ? "select: " : "deselect: ") << " blue box" << std::endl;
    box_blue->setSelected(checked);
    UI.checkBox_box_blue->setChecked(box_blue->isSelected());
}

void showDemoWindow::selectionButtonChecked_cylinder_red(bool checked)
{
    std::cout << (checked ? "select: " : "deselect: ") << " red cylinder" << std::endl;
    cylinder_red->setSelected(checked);
    UI.checkBox_cylinder_red->setChecked(cylinder_red->isSelected());
}

void showDemoWindow::selectionButtonChecked_cylinder_green(bool checked)
{
    std::cout << (checked ? "select: " : "deselect: ") << " green cylinder" << std::endl;
    cylinder_green->setSelected(checked);
    UI.checkBox_cylinder_green->setChecked(cylinder_green->isSelected());
}

void showDemoWindow::selectionButtonChecked_cone_blue(bool checked)
{
    std::cout << (checked ? "select: " : "deselect: ") << " blue cone" << std::endl;
    cone_blue->setSelected(checked);
    UI.checkBox_cone_blue->setChecked(cone_blue->isSelected());
}

void showDemoWindow::groupShapes()
{
    std::cout << "group by shapes" << std::endl;
    auto factory = VirtualRobot::VisualizationFactory::getInstance();
    boxes->setSelectionGroup(factory->createSelectionGroup());
    cylinders->setSelectionGroup(factory->createSelectionGroup());
    cones->setSelectionGroup(factory->createSelectionGroup());
}

void showDemoWindow::groupColors()
{
    std::cout << "group by colors" << std::endl;
    auto factory = VirtualRobot::VisualizationFactory::getInstance();
    red->setSelectionGroup(factory->createSelectionGroup());
    green->setSelectionGroup(factory->createSelectionGroup());
    blue->setSelectionGroup(factory->createSelectionGroup());
}

void showDemoWindow::ungroup()
{
    std::cout << "ungroup" << std::endl;
    auto factory = VirtualRobot::VisualizationFactory::getInstance();
    box_red->setSelectionGroup(factory->createSelectionGroup());
    box_green->setSelectionGroup(factory->createSelectionGroup());
    box_blue->setSelectionGroup(factory->createSelectionGroup());
    cylinder_red->setSelectionGroup(factory->createSelectionGroup());
    cylinder_green->setSelectionGroup(factory->createSelectionGroup());
    cone_blue->setSelectionGroup(factory->createSelectionGroup());
}


void showDemoWindow::quit()
{
    this->close();
}
