#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <ControlCANFD.h>
#include "canthread.h"
#include <QThread>
#include <QCloseEvent>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void canRecvedCANData(ZCAN_Receive_Data *recvCANData,UINT frameCount,UINT channel);
    void canRecvedCANFDData(ZCAN_ReceiveFD_Data *recvCANFDData,UINT frameCount,UINT channel);
    void on_cleanListBtn_clicked();

    void on_openDeviceBtn_clicked();

    void on_closeDeviceBtn_clicked();

    void on_initCANBtn_clicked();

    void on_StartCANBtn_clicked();

    bool afterReSet();

    void on_reSetCANBtn_clicked();

    void on_sendBtn_clicked();

    void AddDataToList(QStringList strList);

    void closeEvent(QCloseEvent *event);

private:
    Ui::MainWindow *ui;
    CANThread *canthread;
};

#endif // MAINWINDOW_H
