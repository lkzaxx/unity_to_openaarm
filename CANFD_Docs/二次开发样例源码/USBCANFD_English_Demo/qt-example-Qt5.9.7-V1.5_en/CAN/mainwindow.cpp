#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QMessageBox>
#include <QTime>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->filterModeCombo->setCurrentIndex(2);

    QStringList listHeader;
    listHeader << "Time"  << "Chan" << "Rx/Tx" << "ID" << "Frame" << "Type" << "DLC" << "CAN-FD" << "Data";

    ui->tableWidget->setColumnCount(listHeader.count());
    ui->tableWidget->setHorizontalHeaderLabels(listHeader);
    ui->tableWidget->setColumnWidth(0,80);
    ui->tableWidget->setColumnWidth(1,40);
    ui->tableWidget->setColumnWidth(2,60);
    ui->tableWidget->setColumnWidth(3,80);
    ui->tableWidget->setColumnWidth(4,90);
    ui->tableWidget->setColumnWidth(5,90);
    ui->tableWidget->setColumnWidth(6,80);
    ui->tableWidget->setColumnWidth(7,90);
    ui->tableWidget->setColumnWidth(8,200);

    ui->tableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->tableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);

    canthread = new CANThread();
    connect(canthread,&CANThread::recvedCANData,this,&MainWindow::canRecvedCANData);
    connect(canthread,&CANThread::recvedCANFDData,this,&MainWindow::canRecvedCANFDData);
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    canthread->stop();
    canthread->closeDevice();
    canthread->terminate();
    canthread->wait();
    //delete canthread;
    //event->accept();
    QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::canRecvedCANData(ZCAN_Receive_Data *recvCANData,UINT frameCount,UINT channel)
{
    QStringList messageList;
    QString str;
    for(int i = 0;i < frameCount;i ++)
    {
        messageList.clear();
        messageList << QTime::currentTime().toString("hh:mm:ss zzz");//time
        messageList << QString::number(channel);
        messageList << "Rx";//rx/tx
        messageList << "0x" + QString::number(GET_ID(recvCANData[i].frame.can_id),16);//ID
        messageList << (IS_EFF(recvCANData[i].frame.can_id) ? "Extended Frame" : "Standard Frame");//Frame
        messageList << (IS_RTR(recvCANData[i].frame.can_id) ? "Remote Frame" : "Data Frame");//Type
        messageList << QString::number(recvCANData[i].frame.can_dlc);//DLC
        messageList << "CAN";//CAN-FD
        str = "";
        if(!IS_RTR(recvCANData[i].frame.can_id))//display
        {
            for(int j = 0;j < recvCANData[i].frame.can_dlc;j ++)
                str += QString("%1 ").arg(recvCANData[i].frame.data[j],2,16,QChar('0'));//QString::number(recvCANData[i].frame.data[j],16) + " ";
        }
        messageList << str;//data
        AddDataToList(messageList);
    }
}

void MainWindow::canRecvedCANFDData(ZCAN_ReceiveFD_Data *recvCANFDData,UINT frameCount,UINT channel)
{
    QStringList messageList;
    QString str;
    for(int i = 0;i < frameCount;i ++)
    {
        messageList.clear();
        messageList << QTime::currentTime().toString("hh:mm:ss zzz");//time
        messageList << QString::number(channel);
        messageList << "Rx";//tx/tx
        messageList << "0x" + QString::number(GET_ID(recvCANFDData[i].frame.can_id),16);//ID
        messageList << (IS_EFF(recvCANFDData[i].frame.can_id) ? "Extended Frame" : "Standard Frame");//Frame
        messageList << (IS_RTR(recvCANFDData[i].frame.can_id) ? "Remote Frame" : "Data Frame");//Type
        messageList << QString::number(recvCANFDData[i].frame.len);//DLC
        messageList << "CANFD";//CAN-FD
        str = "";
        if(!IS_RTR(recvCANFDData[i].frame.can_id))//display
        {
            for(int j = 0;j < recvCANFDData[i].frame.len;j ++)
                str += QString("%1 ").arg(recvCANFDData[i].frame.data[j],2,16,QChar('0'));//QString::number(recvCANFDData[i].frame.data[j],16) + " ";
        }
        if(IS_RTR(recvCANFDData[i].frame.can_id))
            messageList << "";
        else
            messageList << str;//data
        AddDataToList(messageList);
    }
}

void MainWindow::on_cleanListBtn_clicked()
{
    ui->tableWidget->setRowCount(0);
}

void MainWindow::on_openDeviceBtn_clicked()
{
    //1.open
    if(canthread->openDevice(ui->deviceTypeCombo->currentIndex() + 41,ui->deviceIndexCombo->currentIndex(),0))
    {
        ui->openDeviceBtn->setEnabled(false);
        ui->initCANBtn->setEnabled(true);
        ui->closeDeviceBtn->setEnabled(true);
    }
    else
        QMessageBox::warning(this,"Warnning","Open Device Failed!");
}

void MainWindow::on_initCANBtn_clicked()
{
    //2.set CANFD standard
    if(!canthread->setCANFDStandard(ui->canFDStandardCombo->currentIndex()))
    {
        QMessageBox::warning(this,"Warnning","Set CANFD Standard Failed");
        return;
    }
    UINT rate1 = 0,rate2 = 0;
    /*
    if(ui->CustomBaudrateCheckBox->checkState())//自定义
    {
        if(!canthread->setCustomBaudrateFD(ui->CustomBaudrateEdit->text()))
        {
            QMessageBox::warning(this,"警告","设置自定义波特率失败！");
            return;
        }
    }
    else
    */
    {
        switch(ui->ABIT1Combo->currentIndex())
        {
            case 0:
                rate1 = 1000000;
            break;
            case 1:
                rate1 = 800000;
            break;
            case 2:
                rate1 = 500000;
            break;
            case 3:
                rate1 = 250000;
            break;
            case 4:
                rate1 = 125000;
            break;
            case 5:
                rate1 = 100000;
            break;
            case 6:
                rate1 = 50000;
            break;
            default:
                rate1 = 50000;
            break;
        }
        switch(ui->ABIT2Combo->currentIndex())
        {
            case 0:
                rate2 = 5000000;
            break;
            case 1:
                rate2 = 4000000;
            break;
            case 2:
                rate2 = 2000000;
            break;
            case 3:
                rate2 = 1000000;
            break;
            default:
                rate2 = 1000000;
            break;
        }
        if(!canthread->setBaudrateFD(rate1,rate2))
        {
            QMessageBox::warning(this,"Warnning","set baudrate failed!");
            return;
        }
    }
    afterReSet();
}

void MainWindow::on_closeDeviceBtn_clicked()
{
    //9.close device
    canthread->stop();
    canthread->closeDevice();
    ui->initCANBtn->setEnabled(false);
    ui->StartCANBtn->setEnabled(false);
    ui->reSetCANBtn->setEnabled(false);
    ui->closeDeviceBtn->setEnabled(false);
    ui->sendBtn->setEnabled(false);
    ui->openDeviceBtn->setEnabled(true);
}


void MainWindow::on_StartCANBtn_clicked()
{
    //7.start CAN
    if(!canthread->startCAN())
    {
        QMessageBox::warning(this,"Warnning","start CAN Failed");
        return;
    }
    ui->StartCANBtn->setEnabled(false);
    ui->reSetCANBtn->setEnabled(true);
    ui->sendBtn->setEnabled(true);
    canthread->start();//start recv
}

bool MainWindow::afterReSet()
{
    //4.init CAN
    if(!canthread->initCAN())
    {
        QMessageBox::warning(this,"Warnning","init CAN Failed!");
        return false;
    }
    //5.enable resistance
    /*
    if(!canthread->setResistanceEnable(ui->resistanceCheckBox->checkState() ? 1: 0))
    {
        QMessageBox::warning(this,"Warnning","enable resistance failed");
        return false;
    }
    */
    //6.set filter, optional
    if(ui->filterModeCombo->currentIndex() != 2)
    {
        if(!canthread->setFilter(ui->filterModeCombo->currentIndex(),"0x" + ui->StartIDEdit->text(),"0x" + ui->endIDEdit->text()))
        {
            QMessageBox::warning(this,"Warnning","config filter failed!");
            return false;
        }
    }
    ui->initCANBtn->setEnabled(false);
    ui->StartCANBtn->setEnabled(true);
    return true;
}

void MainWindow::on_reSetCANBtn_clicked()
{
    //0.reset device, after reset , go to step 4.
    canthread->stop();
    if(!canthread->reSetCAN())
    {
        QMessageBox::warning(this,"Warnning","Reset CAN Failed!");
        return;
    }
    ui->initCANBtn->setEnabled(true);
    ui->StartCANBtn->setEnabled(false);
    ui->reSetCANBtn->setEnabled(false);
    ui->sendBtn->setEnabled(false);
}

void MainWindow::on_sendBtn_clicked()
{
    if(ui->frameTypeCombo->currentIndex() == 0)//standard id:0-0x7FF
    {
        if(ui->sendIDEdit->text().toInt(Q_NULLPTR,16) > 0x7FF)
        {
            QMessageBox::warning(this,"Warnning","Send failed,standard id:0~0x7FF");
            return;
        }
    }
    else
    {
        if(ui->sendIDEdit->text().toInt(Q_NULLPTR,16) > 0x1FFFFFFF)
        {
            QMessageBox::warning(this,"Warnning","Send failed,standard id:0~0x1FFFFFFF!");
            return;
        }
    }
    QStringList strList = ui->sendDataEdit->text().split(" ");
    char data[64];
    memset(data,0,64);
    for(int i = 0;i < strList.count();i ++)
        data[i] = strList.at(i).toInt(Q_NULLPTR,16);
    UINT dlc = 0;
    if(ui->protocolCombo->currentIndex() == 0)//CAN
        dlc = strList.count() > 8 ? 8 : strList.count();
    else if(ui->protocolCombo->currentIndex() == 1)//CANFD
    {
        if(strList.count() <= 8)
            dlc = strList.count();
        else if(strList.count() <= 12)
            dlc = 12;
        else if(strList.count() <= 16)
            dlc = 16;
        else if(strList.count() <= 20)
            dlc = 20;
        else if(strList.count() <= 24)
            dlc = 24;
        else if(strList.count() <= 32)
            dlc = 32;
        else if(strList.count() <= 48)
            dlc = 48;
        else
            dlc = 64;
    }
    //8.send recv data
    if(canthread->sendData(ui->sendIDEdit->text().toInt(Q_NULLPTR,16),
                           ui->frameTypeCombo->currentIndex(),
                           ui->protocolCombo->currentIndex(),
                           (ui->CANFDaccCheck->checkState() ? 1 : 0),
                           ui->sendPathCombo->currentIndex(),
                           data,dlc))
    {//Send ok, display
        QStringList messageList;
        messageList << QTime::currentTime().toString("hh:mm:ss zzz");//time
        messageList << QString::number(ui->sendPathCombo->currentIndex());//channel
        messageList << "Tx";//rx/tx
        messageList << "0x" + QString::number(ui->sendIDEdit->text().toUInt());//ID
        messageList << ((ui->frameTypeCombo->currentIndex() == 0) ? "Standard Frame" : "Extended Frame");//Frame
        messageList << "Data Frame";//Type
        messageList << QString::number(dlc);//DLC
        messageList << ((ui->protocolCombo->currentIndex() == 0) ? "CAN" : "CANFD");//CAN-FD
        QString str = "";
        for(int j = 0;j < dlc;j ++)
            str += QString("%1 ").arg((unsigned char)data[j],2,16,QChar('0'));//QString::number((unsigned char)data[j],16) + " ";
        messageList << str;//data
        AddDataToList(messageList);
    }
}

void MainWindow::AddDataToList(QStringList strList)
{
    int row = ui->tableWidget->rowCount();
    ui->tableWidget->insertRow(row);
    for(int i = 0; i < strList.count();i ++)
    {
        QTableWidgetItem *item = new QTableWidgetItem(strList.at(i),0);
        ui->tableWidget->setItem(row, i, item);
        if(i != strList.count() - 1)
            ui->tableWidget->item(row,i)->setTextAlignment(Qt::AlignCenter | Qt::AlignHCenter);

    }
}
