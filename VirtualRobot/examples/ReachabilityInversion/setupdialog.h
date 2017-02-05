#ifndef SETUPDIALOG_H
#define SETUPDIALOG_H

#include <QDialog>

namespace Ui {
class SetupDialog;
}

class SetupDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SetupDialog(QWidget *parent = 0);
    ~SetupDialog();

    void setRobotFile(std::string absPath);
    void setWsFile(std::string absPath);
    void setInvWsFile(std::string absPath);
    std::string getRobotFile();
    std::string getWsFile();
    std::string getInvWsFile();
    std::string getEEF();

private:
    Ui::SetupDialog *ui;

private slots:
    void browseRobotFile();
    void browseWsFile();
    void browseInvWsFile();
};

#endif // SETUPDIALOG_H
