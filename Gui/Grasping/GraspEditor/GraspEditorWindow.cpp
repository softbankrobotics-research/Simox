
#include "GraspEditorWindow.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/Workspace/Reachability.h"
#include "VirtualRobot/Grasping/Grasp.h"
#include "VirtualRobot/Grasping/GraspSet.h"
#include "VirtualRobot/XML/ObjectIO.h"
#include "VirtualRobot/XML/ModelIO.h"
#include "VirtualRobot/Import/SimoxXMLFactory.h"
#include "VirtualRobot/Tools/SphereApproximator.h"
#include "VirtualRobot/Visualization/TriMeshModel.h"

#include <QFileDialog>
#include <Eigen/Geometry>

#include <time.h>
#include <vector>
#include <iostream>
#include <cmath>

#include <sstream>

#include "Gui/ui_GraspEditor.h"

#ifdef Simox_USE_COIN_VISUALIZATION
    #include "../../../Gui/Coin/CoinViewerFactory.h"
    // need this to ensure that static Factory methods are called across library boundaries (otherwise coin Gui lib is not loaded since it is not referenced by us)
    SimoxGui::CoinViewerFactory f;
#endif

using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 30.0f;

namespace VirtualRobot
{

    GraspEditorWindow::GraspEditorWindow(std::string& objFile, std::string& robotFile,
                                         bool embeddedGraspEditor)
        : QMainWindow(nullptr), UI(new Ui::MainWindowGraspEditor)
    {
        VR_INFO << " start " << endl;

        // Indicates whether this program is started inside another extern program
        this->embeddedGraspEditor = embeddedGraspEditor;

        objectFile = objFile;
        this->robotFile = robotFile;

        setupUI();

        loadObject();
        loadRobot();

        viewer->viewAll();

        timer = new QTimer(this);
        connect(timer, SIGNAL(timeout()), this, SLOT(timerCB()));
        timer->start(TIMER_MS);
    }


    GraspEditorWindow::~GraspEditorWindow()
    {
        timer->stop();
        delete timer;
        viewer.reset();
        delete UI;
    }


    void GraspEditorWindow::timerCB()
    {
        float x[6];
        x[0] = (float)UI->horizontalSliderX->value();
        x[1] = (float)UI->horizontalSliderY->value();
        x[2] = (float)UI->horizontalSliderZ->value();
        x[3] = (float)UI->horizontalSliderRo->value();
        x[4] = (float)UI->horizontalSliderPi->value();
        x[5] = (float)UI->horizontalSliderYa->value();
        x[0] /= 10.0f;
        x[1] /= 10.0f;
        x[2] /= 10.0f;
        x[3] /= 300.0f;
        x[4] /= 300.0f;
        x[5] /= 300.0f;

        if (x[0] != 0 || x[1] != 0 || x[2] != 0 || x[3] != 0 || x[4] != 0 || x[5] != 0)
        {
            updateEEF(x);
        }
    }


    void GraspEditorWindow::setupUI()
    {
        UI->setupUi(this);

        SimoxGui::ViewerFactoryPtr viewerFactory = SimoxGui::ViewerFactory::first(nullptr);
        THROW_VR_EXCEPTION_IF(!viewerFactory,"No viewer factory?!");
        viewer = viewerFactory->createViewer(UI->frameViewer);

        connect(UI->pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
        connect(UI->pushButtonLoadObject, SIGNAL(clicked()), this, SLOT(selectObject()));
        connect(UI->pushButtonSave, SIGNAL(clicked()), this, SLOT(saveObject()));
        connect(UI->pushButtonClose, SIGNAL(clicked()), this, SLOT(closeEEF()));
        connect(UI->pushButtonOpen, SIGNAL(clicked()), this, SLOT(openEEF()));
        connect(UI->pushButtonLoadRobot, SIGNAL(clicked()), this, SLOT(selectRobot()));
        connect(UI->comboBoxEEF, SIGNAL(activated(int)), this, SLOT(selectEEF(int)));
        connect(UI->comboBoxGrasp, SIGNAL(activated(int)), this, SLOT(selectGrasp(int)));
        connect(UI->pushButtonAddGrasp, SIGNAL(clicked()), this, SLOT(addGrasp()));
        connect(UI->pushButtonRenameGrasp, SIGNAL(clicked()), this, SLOT(renameGrasp()));
        connect(UI->checkBoxTCP, SIGNAL(clicked()), this, SLOT(buildVisu()));

        connect(UI->horizontalSliderX, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectX()));
        connect(UI->horizontalSliderY, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectY()));
        connect(UI->horizontalSliderZ, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectZ()));
        connect(UI->horizontalSliderRo, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectA()));
        connect(UI->horizontalSliderPi, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectB()));
        connect(UI->horizontalSliderYa, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectG()));
        connect(UI->checkBoxColModel, SIGNAL(clicked()), this, SLOT(buildVisu()));
        connect(UI->checkBoxGraspSet, SIGNAL(clicked()), this, SLOT(buildVisu()));


        // In case of embedded use of this program it should not be possible to load an object after the editor is started
        if (embeddedGraspEditor)
        {
            UI->pushButtonLoadObject->setVisible(false);
        }
    }

    void GraspEditorWindow::resetSceneryAll()
    {

    }

    void GraspEditorWindow::closeEvent(QCloseEvent* event)
    {
        quit();
        QMainWindow::closeEvent(event);
    }



    void GraspEditorWindow::buildVisu()
    {
        viewer->clearLayer("eefLayer");

        showCoordSystem();
        ModelLink::VisualizationType colModel = (UI->checkBoxColModel->isChecked()) ? ModelLink::VisualizationType::Collision : ModelLink::VisualizationType::Full;
        VisualizationFactoryPtr f = VisualizationFactory::getGlobalVisualizationFactory();
        if (!f)
            return;

        if (!UI->checkBoxTCP->isChecked())
        {
            if (robotEEF)
            {
                VisualizationSetPtr visu = robotEEF->getVisualization(colModel);
                viewer->addVisualization("eefLayer", visu);
            }
        }
        else
        {
            if (robotEEF && robotEEF_EEF)
            {
                FramePtr tcp = robotEEF_EEF->getTcp();
                if (tcp)
                {
                    Eigen::Matrix4f tcpGP = tcp->getGlobalPose();
                    VisualizationPtr visu = f->createCoordSystem(nullptr);
                    visu->setGlobalPose(tcpGP);
                    viewer->addVisualization("eefLayer", visu);
                }
            }
        }
        viewer->clearLayer("objectLayer");

        if (object)
        {
            VisualizationSetPtr visu = object->getVisualization(colModel);
            viewer->addVisualization("objectLayer", visu);
        }

        buildGraspSetVisu();
    }

    void GraspEditorWindow::quit()
    {
        std::cout << "GraspEditorWindow: Closing" << std::endl;
        this->close();
        timer->stop();
    }

    void GraspEditorWindow::selectRobot()
    {
        QString fi = QFileDialog::getOpenFileName(this, tr("Open Robot File"), QString(), tr("XML Files (*.xml)"));
        if(fi.isEmpty())
            return;
        robotFile = std::string(fi.toLatin1());
        loadRobot();
    }

    void GraspEditorWindow::selectObject(std::string file)
    {
        std::string s;

        // The object must be selected manually, cannot be done in the constructor
        if (embeddedGraspEditor)
        {
            s = file;
        }
        else
        {
            QString fi;
            QFileDialog dialog(this);
            dialog.setFileMode(QFileDialog::ExistingFile);
            dialog.setAcceptMode(QFileDialog::AcceptOpen);
            QStringList nameFilters;
            nameFilters << "Manipulation Object XML Files (*.xml *.moxml)"
//                        << "XML Files (*.xml)"
                        << "All Files (*.*)";
            dialog.setNameFilters(nameFilters);

            if (dialog.exec())
            {
                if (dialog.selectedFiles().size() == 0)
                    return;

                fi = dialog.selectedFiles()[0];
            }
            else
            {
                VR_INFO << "load dialog canceled" << std::endl;
                return;
            }
            s = std::string(fi.toLatin1());
        }

        if (s != "")
        {
            objectFile = s;
            loadObject();
        }
    }

    void GraspEditorWindow::saveObject()
    {
        if (!object)
        {
            return;
        }

        // No need to select a file where the object is saved, it is the same as the input file
        if (!embeddedGraspEditor)
        {
            QString fi;
            QFileDialog dialog(this);
            dialog.setFileMode(QFileDialog::AnyFile);
            dialog.setAcceptMode(QFileDialog::AcceptSave);
            dialog.setDefaultSuffix("moxml");
            QStringList nameFilters;
            nameFilters << "Manipulation Object XML Files (*.moxml)"
                        << "XML Files (*.xml)"
                        << "All Files (*.*)";
            dialog.setNameFilters(nameFilters);

            if (dialog.exec())
            {
                if (dialog.selectedFiles().size() == 0)
                    return;

                fi = dialog.selectedFiles()[0];
            }
            else
            {
                VR_INFO << "load dialog canceled" << std::endl;
                return;
            }
            objectFile = std::string(fi.toLatin1());
        }

        bool ok = false;

        try
        {
            ok = ObjectIO::saveManipulationObject(object, objectFile);
        }
        catch (VirtualRobotException& e)
        {
            cout << " ERROR while saving object" << endl;
            cout << e.what();
            return;
        }

        if (!ok)
        {
            cout << " ERROR while saving object" << endl;
            return;
        }
        else
        {
            if (embeddedGraspEditor)
            {
                cout << "Changes successful saved to " << objectFile << endl;
                QMessageBox msgBox;
                msgBox.setText(QString::fromStdString("Changes successful saved to " + objectFile));
                msgBox.setIcon(QMessageBox::Information);
                msgBox.setStandardButtons(QMessageBox::Ok);
                msgBox.setDefaultButton(QMessageBox::Ok);
                msgBox.exec();
            }
        }

    }
    void GraspEditorWindow::loadRobot()
    {
        cout << "Loading Robot from " << robotFile << endl;

        try
        {
            robot = ModelIO::loadModel(robotFile);
        }
        catch (VirtualRobotException& e)
        {
            cout << " ERROR while creating robot" << endl;
            cout << e.what();
            return;
        }

        if (!robot)
        {
            cout << " ERROR while creating robot" << endl;
            return;
        }

        eefs = robot->getEndEffectors();
        updateEEFBox();

        if (eefs.size() == 0)
        {
            selectEEF(-1);
        }
        else
        {
            selectEEF(0);
        }

        buildVisu();
    }

    void GraspEditorWindow::selectEEF(int n)
    {
        currentEEF.reset();
        currentGraspSet.reset();
        currentGrasp.reset();

        robotEEF.reset();

        if (n < 0 || n >= (int)eefs.size() || !robot)
        {
            return;
        }

        currentEEF = eefs[n];

        currentEEF->print();

        robotEEF = currentEEF->createEefRobot(currentEEF->getName(), currentEEF->getName());
        //robotEEF->print();
        robotEEF_EEF = robotEEF->getEndEffector(currentEEF->getName());
        robotEEF_EEF->print();

        // select grasp set
        if (object)
        {
            currentGraspSet = object->getGraspSet(currentEEF);

            if (!currentGraspSet)
            {
                currentGraspSet.reset(new GraspSet(currentEEF->getName(), robot->getType(), currentEEF->getName()));
                currentGrasp.reset(new Grasp("Grasp 0", robot->getType(), currentEEF->getName(), Eigen::Matrix4f::Identity(), "GraspEditor"));
                currentGraspSet->addGrasp(currentGrasp);
                object->addGraspSet(currentGraspSet);
            }

            updateGraspBox();
            selectGrasp(0);
        }
        else
        {
            updateGraspBox();
        }

        buildVisu();

    }

    void GraspEditorWindow::selectGrasp(int n)
    {
        currentGrasp.reset();

        if (!currentGraspSet || n < 0 || n >= (int)currentGraspSet->getSize() || !robot)
        {
            return;
        }

        currentGrasp = currentGraspSet->getGrasp(n);

        if (currentGrasp && robotEEF && robotEEF_EEF && object)
        {
            Eigen::Matrix4f gp;
            gp = currentGrasp->getTransformation().inverse();
            gp = object->toGlobalCoordinateSystem(gp);
            std::string preshape = currentGrasp->getPreshapeName();

            if (!preshape.empty() && robotEEF_EEF->hasPreshape(preshape))
            {
                robotEEF_EEF->setPreshape(preshape);
            }

            setCurrentGrasp(gp);
        }

        buildVisu();
    }

    void GraspEditorWindow::loadObject()
    {
        cout << "Loading Object from " << objectFile << endl;

        try
        {
            object = ObjectIO::loadManipulationObject(objectFile);
        }
        catch (VirtualRobotException& e)
        {
            cout << " ERROR while creating object" << endl;
            cout << e.what();

            if (embeddedGraspEditor)
            {
                QMessageBox msgBox;
                msgBox.setText(QString::fromStdString(" ERROR while creating object."));
                msgBox.setInformativeText("Please select a valid manipulation file.");
                msgBox.setIcon(QMessageBox::Information);
                msgBox.setStandardButtons(QMessageBox::Ok);
                msgBox.setDefaultButton(QMessageBox::Ok);
                msgBox.exec();
            }

            return;
        }

        if (!object)
        {
            cout << " ERROR while creating object" << endl;
            return;
        }

        //object->print();

        selectEEF(0);

        buildVisu();
    }

    void GraspEditorWindow::updateEEFBox()
    {
        UI->comboBoxEEF->clear();

        for (size_t i = 0; i < eefs.size(); i++)
        {
            UI->comboBoxEEF->addItem(QString(eefs[i]->getName().c_str()));
        }
    }

    void GraspEditorWindow::updateGraspBox()
    {
        UI->comboBoxGrasp->clear();

        if (!currentGraspSet || currentGraspSet->getSize() == 0)
        {
            return;
        }

        for (unsigned int i = 0; i < currentGraspSet->getSize(); i++)
        {
            UI->comboBoxGrasp->addItem(QString(currentGraspSet->getGrasp(i)->getName().c_str()));
        }
    }

    void GraspEditorWindow::closeEEF()
    {
        if (robotEEF_EEF)
        {
            robotEEF_EEF->closeActors(object);
        }
    }

    void GraspEditorWindow::openEEF()
    {
        if (robotEEF_EEF)
        {
            robotEEF_EEF->openActors();
        }
    }

    void GraspEditorWindow::renameGrasp()
    {
        if (!currentGrasp)
            return;
        bool ok;
        QString text = QInputDialog::getText(this, tr("Rename Grasp"),
                                             tr("New name:"), QLineEdit::Normal,
                                             tr(currentGrasp->getName().c_str()), &ok);


        if (ok && !text.isEmpty())
        {
            std::string sText = text.toStdString();
            currentGrasp->setName(sText);

            updateGraspBox();
        }
    }

    void GraspEditorWindow::addGrasp()
    {
        if (!object || !robot || !currentGraspSet)
        {
            return;
        }

        std::stringstream ss;
        ss << "Grasp " << (currentGraspSet->getSize());
        std::string name = ss.str();
        Eigen::Matrix4f pose;

        if (currentGrasp)
        {
            pose = currentGrasp->getTransformation();
        }
        else
        {
            pose = Eigen::Matrix4f::Identity();
        }

        GraspPtr g(new Grasp(name, robot->getType(), currentEEF->getName(), pose, std::string("GraspEditor")));
        currentGraspSet->addGrasp(g);
        updateGraspBox();
        UI->comboBoxGrasp->setCurrentIndex(UI->comboBoxGrasp->count() - 1);
        selectGrasp(UI->comboBoxGrasp->count() - 1);
        buildVisu();
    }

    void GraspEditorWindow::updateEEF(float x[6])
    {
        if (robotEEF)
        {
            //cout << "getGlobalPose robot:" << endl << robotEEF->getGlobalPose() << endl;
            //cout << "getGlobalPose TCP:" << endl <<  robotEEF_EEF->getTcp()->getGlobalPose() << endl;
            Eigen::Matrix4f m;
            MathTools::posrpy2eigen4f(x, m);
            FramePtr tcp = robotEEF_EEF->getTcp();
            m = tcp->getGlobalPose() * m;
            //cout << "pose:" << endl << m << endl;
            setCurrentGrasp(m);
        }
    }

    void GraspEditorWindow::sliderReleased_ObjectX()
    {
        UI->horizontalSliderX->setValue(0);
    }

    void GraspEditorWindow::sliderReleased_ObjectY()
    {
        UI->horizontalSliderY->setValue(0);
    }

    void GraspEditorWindow::sliderReleased_ObjectZ()
    {
        UI->horizontalSliderZ->setValue(0);
    }

    void GraspEditorWindow::sliderReleased_ObjectA()
    {
        UI->horizontalSliderRo->setValue(0);
    }

    void GraspEditorWindow::sliderReleased_ObjectB()
    {
        UI->horizontalSliderPi->setValue(0);
    }

    void GraspEditorWindow::sliderReleased_ObjectG()
    {
        UI->horizontalSliderYa->setValue(0);
    }


    void GraspEditorWindow::setCurrentGrasp(Eigen::Matrix4f& p)
    {
        if (robotEEF && robotEEF_EEF && currentGrasp && object)
        {
            FramePtr tcp = robotEEF_EEF->getTcp();
            robotEEF->setGlobalPoseForModelNode(tcp, p);
            Eigen::Matrix4f objP = object->getGlobalPose();
            Eigen::Matrix4f pLocal = tcp->toLocalCoordinateSystem(objP);
            currentGrasp->setTransformation(pLocal);
        }
    }

    void GraspEditorWindow::showCoordSystem()
    {
        if (robotEEF && robotEEF_EEF)
        {
            FramePtr tcp = robotEEF_EEF->getTcp();

            if (!tcp)
            {
                return;
            }

            //tcp->showCoordinateSystem(UI->checkBoxTCP->isChecked());
        }

        if (object)
        {
            //object->showCoordinateSystem(UI->checkBoxTCP->isChecked());
        }
    }


    void GraspEditorWindow::buildGraspSetVisu()
    {        
        viewer->clearLayer("graspsetLayer");

        if (UI->checkBoxGraspSet->isChecked() && robotEEF && robotEEF_EEF && currentGraspSet && object)
        {
            VisualizationFactoryPtr f = VisualizationFactory::getGlobalVisualizationFactory();
            if (!f)
                return;

            GraspSetPtr gs = currentGraspSet->clone();
            gs->removeGrasp(currentGrasp);

            VisualizationSetPtr visu = gs->getVisualization(ModelLink::VisualizationType::Full, robotEEF_EEF, object->getGlobalPose());
            viewer->addVisualization("graspsetLayer", visu);
        }
    }

}
