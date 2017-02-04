#include "setupdialog.h"
#include "ui_setupdialog.h"
#include <QFileInfo>
#include <QFileDialog>

SetupDialog::SetupDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SetupDialog)
{
    ui->setupUi(this);
    ui->lblInvHint->setStyleSheet("QLabel { color : grey; }");

    connect(ui->btnBrowseRobot, SIGNAL(clicked()), this, SLOT(browseRobotFile()));
    connect(ui->btnBrowseWS, SIGNAL(clicked()), this, SLOT(browseWsFile()));
    connect(ui->btnBrowseInv, SIGNAL(clicked()), this, SLOT(browseInvWsFile()));
}

SetupDialog::~SetupDialog()
{
    delete ui;
}

void SetupDialog::setRobotFile(std::string absPath)
{
    ui->edtRobot->setText(QString::fromStdString(absPath));
}

void SetupDialog::setWsFile(std::string absPath)
{
    ui->edtWS->setText(QString::fromStdString(absPath));
}

void SetupDialog::setInvWsFile(std::string absPath)
{
    ui->edtInv->setText(QString::fromStdString(absPath));
}

std::string SetupDialog::getRobotFile()
{
    return ui->edtRobot->text().toStdString();
}

std::string SetupDialog::getWsFile()
{
    return ui->edtWS->text().toStdString();
}

std::string SetupDialog::getInvWsFile()
{
    return ui->edtInv->text().toStdString();
}

void SetupDialog::browseRobotFile()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Select Robot File"), QString::fromStdString(getRobotFile()), tr("Robot Files (*.xml);;All Files (*)"));
    if (!fileName.isEmpty())
    {
        ui->edtRobot->setText(fileName);
    }
}

void SetupDialog::browseWsFile()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Select Workspace File"), QString::fromStdString(getWsFile()), tr("WS Files (*.bin);;All Files (*)"));
    if (!fileName.isEmpty())
    {
        ui->edtWS->setText(fileName);
    }
}

void SetupDialog::browseInvWsFile()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Select Inverse Workspace File"), QString::fromStdString(getWsFile()), tr("WS Files (*.bin);;All Files (*)"));
    if (!fileName.isEmpty())
    {
        ui->edtInv->setText(fileName);
    }
}
