
#include "MTPlanningWindow.h"

#include <vector>
#include <iostream>
#include <qlayout.h>
#include <qlabel.h>
#include <qpixmap.h>
#include <qprogressbar.h>
#include <qcheckbox.h>
#include <qlcdnumber.h>
#include <qslider.h>
#include <qimage.h>
#include <qgl.h>

#include <sstream>

using namespace std;

//Global variables
float TIMER_MS = 30.0f;
bool runtimeDisplayed = false;
bool optiTimeDisplayed = false;

MTPlanningWindow::MTPlanningWindow(const std::string &robotFile)
    : QMainWindow(nullptr)
{
    scene = nullptr;

    setupLayoutMTPlanning(robotFile);

    timer1 = new QTimer(this);
    connect(timer1, SIGNAL(timeout()), this, SLOT(timerCBPlanning()));
    timer1->start(TIMER_MS);
    timer2 = new QTimer(this);
    connect(timer2, SIGNAL(timeout()), this, SLOT(timerCBOptimize()));
    timer2->start(TIMER_MS);
}


MTPlanningWindow::~MTPlanningWindow()
{
    if (scene != nullptr)
    {
        delete scene;
    }
}


void MTPlanningWindow::timerCBPlanning()
{
    scene->checkPlanningThreads();
    int nThreadsWorking = 0;
    int nThreadsIdle = 0;
    scene->getThreadCount(nThreadsWorking, nThreadsIdle);
    QString sText;
    sText = "Threads_Working: " + QString::number(nThreadsWorking);
    UI.labelThreads->setText(sText);
    sText = "Threads_Idle: " + QString::number(nThreadsIdle);
    UI.labelThreadsIdle->setText(sText);

    if (!runtimeDisplayed && scene->getPlannersStarted() && nThreadsWorking == 0)
    {
        endTime = clock();
        double runtime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
        sText = "Runtime: " + QString::number(runtime) + "s";
        UI.labelRuntime->setText(sText);
        cout << "Runtime = " << runtime << endl;
        runtimeDisplayed = true;
    }
}


void MTPlanningWindow::timerCBOptimize()
{
    scene->checkOptimizeThreads();
    int nWorking = 0;
    int nIdle = 0;
    scene->getOptimizeThreadCount(nWorking, nIdle);
    QString sText;
    sText = "Opti_Threads_Working: " + QString::number(nWorking);
    UI.labelOptiWork->setText(sText);
    sText = "Opti_Threads_Idle: " + QString::number(nIdle);
    UI.labelOptiIdle->setText(sText);

    if (!optiTimeDisplayed && scene->getOptimizeStarted() && (nWorking == 0))
    {
        optiEndTime = clock();
        double runtime = (double)(optiEndTime - optiStartTime) / CLOCKS_PER_SEC;
        cout << "Runtime:" << runtime << endl;
        sText = "Optimizing Time: " + QString::number(runtime) + "s";
        UI.labelOptiTime->setText(sText);
        cout << "Optimizing Time = " << runtime << endl;
        optiTimeDisplayed = true;
    }
}


void MTPlanningWindow::setupLayoutMTPlanning(const std::string &robotFile)
{
    UI.setupUi(this);

    // setup
    SimoxGui::ViewerFactoryPtr viewerFactory = SimoxGui::ViewerFactory::getInstance();
    THROW_VR_EXCEPTION_IF(!viewerFactory,"No viewer factory?!");
    viewer = viewerFactory->createViewer(UI.frameViewer);

    scene = new MTPlanningScenery(robotFile, viewer);

    connect(UI.pushButtonBuild, SIGNAL(clicked()), this, SLOT(buildScene()));
    connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(reset()));

    UI.comboBoxColChecking->addItem(QString("Singleton Col Checker"));
    UI.comboBoxColChecking->addItem(QString("Multiple Col Checker Instances"));
    UI.comboBoxColChecking->setCurrentIndex(1);
    connect(UI.comboBoxColChecking, SIGNAL(activated(int)), this, SLOT(selectColCheckerComboBoxChanged(int)));
    connect(UI.pushButtonAdd, SIGNAL(clicked()), this, SLOT(addThread()));
    connect(UI.pushButtonPlanning, SIGNAL(clicked()), this, SLOT(startThreads()));
    connect(UI.pushButtonPlanningStop, SIGNAL(clicked()), this, SLOT(stopThreads()));
    connect(UI.pushButtonPost, SIGNAL(clicked()), this, SLOT(startOptimize()));
    connect(UI.pushButtonPostStop, SIGNAL(clicked()), this, SLOT(stopOptimize()));
}

void MTPlanningWindow::selectColCheckerComboBoxChanged(int /*value*/)
{
    reset();
}

void MTPlanningWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}


void MTPlanningWindow::quit()
{
    std::cout << "MTPlanningWindow: Closing" << std::endl;
    this->close();
    timer1->stop();
    timer2->stop();
}


void MTPlanningWindow::reset()
{
    std::cout << "MTPlanningWindow: Reset" << std::endl;
    runtimeDisplayed = false;
    optiTimeDisplayed = false;
    scene->reset();
    viewer->viewAll();
}

void MTPlanningWindow::buildScene()
{
    std::cout << "MTPlanningWindow: buildScene " << std::endl;
    scene->reset();
    scene->buildScene();
    viewer->viewAll();
}

void MTPlanningWindow::addThread()
{
    int n = UI.spinBoxThreads->value();
    std::cout << "MTPlanningWindow: addThread " << n << std::endl;
    bool bMultipleThreads = false;

    if (UI.comboBoxColChecking->currentIndex() == 1)
    {
        bMultipleThreads = true;
    }

    int thr = scene->getThreads();

    for (int i = 0; i < n; i++)
    {
        scene->buildPlanningThread(bMultipleThreads, thr + i);
    }
}


void MTPlanningWindow::startThreads()
{
    std::cout << "MTPlanningWindow: startThreads " << std::endl;
    this->startTime = clock();
    scene->startPlanning();
    runtimeDisplayed = false;
}


void MTPlanningWindow::stopThreads()
{
    std::cout << "MTPlanningWindow: stopThreads " << std::endl;
    scene->stopPlanning();
}

void MTPlanningWindow::startOptimize()
{
    std::cout << "MTPlanningWindow: startOptimize " << std::endl;
    this->optiStartTime = clock();
    scene->startOptimizing();
    optiTimeDisplayed = false;
}

void MTPlanningWindow::stopOptimize()
{
    std::cout << "MTPlanningWindow: stopOptimize " << std::endl;
    scene->stopOptimizing();
}
