#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "qmath.h"
#include <QtXml>
#include "../include/drrobot_jaguar_v6/mainwindow.hpp"
#include "../include/drrobot_jaguar_v6/ui_mainwindow.hpp"
#include "../include/drrobot_jaguar_v6/drrobotprotocol.hpp"

namespace drrobot_jaguar_v6 {

using namespace Qt;
double resTable[25] = {114660,84510,62927,47077,35563,27119,20860,16204,12683,10000,7942,6327,5074,4103,3336,2724,2237,1846,1530,1275,1068,899.3,760.7,645.2,549.4};
double tempTable[25] = { -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100 };
double FULLAD = 4095;
MainWindow::MainWindow(int argc, char** argv, QMainWindow *parent)
    : QMainWindow(parent)
    , qnode(argc,argv),
      ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->robotIPLineEdit->setInputMask("000.000.000.000");
    IP4Validator *ip4Validator = new IP4Validator();
    ui->robotIPLineEdit->setValidator(ip4Validator);

    connect( ui->connectButton, SIGNAL(clicked()),this, SLOT(connectToRobot()));
    connect( ui->forwardButton, SIGNAL(clicked()),this, SLOT(sendForwardCmd()));
    connect( ui->backwardButton, SIGNAL(clicked()),this, SLOT(sendBackwardCmd()));
    connect( ui->stopButton, SIGNAL(clicked()),this, SLOT(sendStopCmd()));
    connect( ui->leftButton, SIGNAL(clicked()),this, SLOT(sendLeftTurnCmd()));
    connect( ui->rightButton, SIGNAL(clicked()),this, SLOT(sendRightTurnCmd()));
    connect(ui->lightsOffButton, SIGNAL( clicked()),this,SLOT(setLightsOff()));
    connect(ui->lightsOnButton, SIGNAL( clicked()),this,SLOT(setLightsOn()));
    connect(ui->eStopButton, SIGNAL(clicked()),this,SLOT(sendEStopCmd()));
    connect(ui->releaseEStopButton,SIGNAL(clicked()),this,SLOT(sendReleaseEStopCmd()));
    connect(ui->pushButtonFFEStop,SIGNAL(clicked()),this,SLOT(sendFrontFLipEStop()));
    connect(ui->pushButtonFFRelease,SIGNAL(clicked()),this,SLOT(sendFrontFLipReleaseEStop()));
    connect(ui->pushButtonRFEStop,SIGNAL(clicked()),this,SLOT(sendRearFLipEStop()));
    connect(ui->pushButtonRFRelease,SIGNAL(clicked()),this,SLOT(sendRearFLipReleaseEStop()));
    connect(ui->pushButtonGyroCal,SIGNAL(clicked()),this,SLOT(gyroCalibrate()));
    wheelRadius = WHEEL_R;
    disOf2Wheel = WHEEL_DIS;
    encoderCntOneCircle = MOTOR_CNT;
    motDir = MOTOR_DIR;


    leftWheelPos = 0;
    rightWheelPos = 0;
    watchDogCnt = 2;
    pingTimer.setInterval(200);
    pingTimer.stop();
    ui->robotHSlider->setValue(200);
    for (int i = 0; i < 4; i++){
        motorBoardData[i].driverState = 0;
    }

    //flip arm control
    for (int i = 0; i < 4; i++){
        flipArmMotor[i].angle = 0;
        flipArmMotor[i].preEncoder = 0;
        flipArmMotor[i].encoderPos = 0;
        flipArmMotor[i].iniFlag = false;
    }
    leftFrontFlipCmd = 0;
    leftFrontFlipFlag = false;
    rightFrontFlipCmd = 0;
    rightFrontFlipFlag = false;
    leftRearFlipCmd = 0;
    leftRearFlipFlag = false;
    rightRearFlipCmd = 0;
    rightRearFlipFlag = false;
    flipCtrlTimer.setInterval(100);
    connect(&flipCtrlTimer, SIGNAL(timeout()), this, SLOT(flipCtrlFun()));
    flipCtrlTimer.stop();

    if ( !qnode.init()){
        showNoMasterMessage();
    }

	QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
	QObject::connect(&qnode, SIGNAL(wheelCmdUpdated(int,int)), this, SLOT(wheelCmdSend(int,int)));
	QObject::connect(&qnode, SIGNAL(flipCmdUpdated(int,int,int,int)), this, SLOT(flipCmdSend(int,int,int,int)));
        QObject::connect(&qnode, SIGNAL(EstopCmdUpdated()), this, SLOT(sendEStopCmd()));
        QObject::connect(&qnode, SIGNAL(ReleaseEstopCmdUpdated()), this, SLOT(sendReleaseEStopCmd()));
        QObject::connect(&qnode, SIGNAL(EstopCmdUpdated()), this, SLOT(sendFrontFLipEStop()));
        QObject::connect(&qnode, SIGNAL(ReleaseEstopCmdUpdated()), this, SLOT(sendFrontFLipReleaseEStop()));
        QObject::connect(&qnode, SIGNAL(ResetInitCmdUpdated()), this, SLOT(ResetInitCmd()));
        QObject::connect(&qnode, SIGNAL(SetInitCmdUpdated()), this, SLOT(SetInitCmd()));
        QObject::connect(&qnode, SIGNAL(Flipper30UpCmdUpdated()), this, SLOT(Flipper30UpCmd()));
        QObject::connect(&qnode, SIGNAL(Flipper60DownCmdUpdated()), this, SLOT(Flipper60DownCmd()));

   setWindowIcon(QIcon(":/images/icon.png"));
	tcpRobot  = NULL;


}

MainWindow::~MainWindow()
{
  if (tcpRobot != NULL) delete tcpRobot;
    delete ui;
}

void MainWindow::connectToRobot()
{
    bool ok = false;
    QString temp = ui->connectButton->text();

    if (temp == tr("Connect"))
    {

        QString addr = ui->robotIPLineEdit->text().trimmed();
        int port = ui->robotPortLineEdit->text().toInt(&ok,10);
        ui->connectButton ->setText(tr("Disconnect"));
        tcpRobot = new QTcpSocket(this);

        connect(tcpRobot,SIGNAL(readyRead()), this, SLOT(processRobotData()));
        tcpRobot->connectToHost(QHostAddress(addr),port);
        tcpRobot->write("PING");
        connect(&pingTimer, SIGNAL(timeout()), this, SLOT(sendPing()));
        pingTimer.start();
        watchDogCnt = 2;
        flipCtrlTimer.start();

    }
    else
    {
        pingTimer.stop();
        disconnect(&pingTimer, SIGNAL(timeout()), this, SLOT(sendPing()));
        ui->connectButton ->setText(tr("Connect"));
        tcpRobot->close();
        watchDogCnt = 2;
        ui->motorControlGroupBox->setEnabled(false);
        ui->setIOgroupBox->setEnabled(false);
    }
}

void MainWindow::processRobotData()
{

    qint64 count = 0;
    char revData[512];
        watchDogCnt = 0;
    do{
        count = tcpRobot->readLine(revData,512);
        dealWithPackage(QString::fromUtf8(revData),count);
    }while(tcpRobot->canReadLine());



}

void MainWindow:: dealWithPackage(QString revData, int len)
{
    int index;
    if (revData.startsWith("#")){
        //IMU sensor data package
        revData = revData.remove(0,1);
        QStringList data = revData.split(",");
        if (data.length() > 15){
            imuData.seqNo = data[0].toInt();
            imuData.estYaw = data[2].toDouble();
            imuData.gyroRaw[0] = data[4].toInt();
            imuData.gyroRaw[1] = data[5].toInt();
            imuData.gyroRaw[2] = data[6].toInt();
            imuData.accelRaw[0] = data[8].toInt();
            imuData.accelRaw[1] = data[9].toInt();
            imuData.accelRaw[2] = data[10].toInt();
            imuData.compassRaw[0] = data[12].toInt();
            imuData.compassRaw[1] = data[13].toInt();
            imuData.compassRaw[2] = data[14].toInt();
            ui->imuSeqLineEdit->setText(QString::number(imuData.seqNo));
            ui->yawLineEdit->setText(QString::number(imuData.estYaw * 180 / M_PI,'f',2));
            ui->gyroXLineEdit->setText(QString::number(imuData.gyroRaw[0]));
            ui->gyroYLineEdit->setText(QString::number(imuData.gyroRaw[1]));
            ui->gyroZLineEdit->setText(QString::number(imuData.gyroRaw[2]));
            ui->accelXLineEdit->setText(QString::number(imuData.accelRaw[0]));
            ui->accelYLineEdit->setText(QString::number(imuData.accelRaw[1]));
            ui->accelZLineEdit->setText(QString::number(imuData.accelRaw[2]));
            ui->compXLineEdit->setText(QString::number(imuData.compassRaw[0]));
            ui->compYLineEdit->setText(QString::number(imuData.compassRaw[1]));
            ui->compZLineEdit->setText(QString::number(imuData.compassRaw[2]));
	
	   qnode.publisherIMUData(imuData);
        }
    }
    else if(revData.startsWith("$GPRMC")){
        //GPS sensor data package

        QStringList data = revData.split(",");
        if (data.length() > 9){
            gpsData.gpsTimeStamp = data[1].toDouble();
            ui->gpsTimeStampLineEdit->setText(data[1]);
            if (data[2] == "A"){
                gpsData.gpsState = 1;
            }
            else if(data[2] == "V"){
                gpsData.gpsState = 0;
            }
            if (gpsData.gpsState >0){
                ui->gpsStateLineEdit->setText("Valid");
            }
            else{
                ui->gpsStateLineEdit->setText("InValid");
            }
            gpsData.gpsLat = data[3].toDouble();
            if (data[4] == "S"){
                gpsData.gpsLat = -gpsData.gpsLat;
            }
            ui->gpsLatLineEdit->setText(data[4] + ":" + data[3]);
            gpsData.gpsLong = data[5].toDouble();
            if (data[6] == "W"){
                gpsData.gpsLong = -gpsData.gpsLong;
            }
            ui->gpsLongLineEdit->setText(data[6] + ":" + data[5]);
            if (!data[7].isEmpty()){
                gpsData.gpsVog = data[7].toDouble() * KNNOT2MS;
                ui->gpsVogLineEdit->setText(QString::number(gpsData.gpsVog,'f',2));
            }
            if (!data[8].isEmpty()){
                gpsData.gpsCog = data[8].toDouble();
                ui->gpsCogLineEdit->setText(data[8]);
            }
        
            qnode.publisherGPSInfo(gpsData);

        }
    }
    else if(revData.startsWith("MM")){
        // motor and driver board data package
        if (revData.startsWith("MM0")){
            index = 0;
        }
        else if (revData.startsWith("MM1")){
            index = 1;
        }
        else if (revData.startsWith("MM2")){
            index = 2;
        }
        else if (revData.startsWith("MM3")){
            index = 3;
        }
                        //driver 1 and front motors
        revData = revData.remove(0,4);

        if (revData.startsWith("A=")){
            //current data
            revData = revData.remove(0,2);
            QStringList data = revData.split(":");
            motorData[index * 2+ 0].motorAmp = data[0].toDouble()/10;
            motorData[index * 2+ 1].motorAmp = data[1].toDouble()/10;
        }
        else if(revData.startsWith("AI=")){
            // A/D data, here 3,4 will be motor temperature sensor
            revData = revData.remove(0,3);
            QStringList data = revData.split(":");
            motorData[index * 2+ 0].motorTemp = ad2Temperature(data[2].toInt());
            motorData[index * 2+ 1].motorTemp = ad2Temperature(data[3].toInt());
        }
        else if(revData.startsWith("C=")){
            // encoder position data
            revData = revData.remove(0,2);
            QStringList data = revData.split(":");
            motorData[index * 2+ 0].encoderPos = data[0].toInt();
            motorData[index * 2+ 1].encoderPos = data[1].toInt();
            if (index == 2){
                flipArmMotor[0].encoderPos = motorData[4].encoderPos;
                flipArmMotor[1].encoderPos = motorData[5].encoderPos;
                getFrontFlipAngle();
            }
            else if(index == 3){
                flipArmMotor[2].encoderPos = motorData[6].encoderPos;
                flipArmMotor[3].encoderPos = motorData[7].encoderPos;
                getRearFlipAngle();
            }
        }
        else if(revData.startsWith("P=")){
            // output PWM value, 0 ~ 1000
            revData = revData.remove(0,2);
            QStringList data = revData.split(":");
            motorData[index * 2+ 0].motorPower = data[0].toInt();
            motorData[index * 2+ 1].motorPower = data[1].toInt();

        }
        else if(revData.startsWith("S=")){
            // encoder velocity data RPM
            revData = revData.remove(0,2);
             QStringList data = revData.split(":");
            motorData[index * 2+ 0].encoderSpeed = data[0].toInt();
            motorData[index * 2+ 1].encoderSpeed = data[1].toInt();

        }
        else if(revData.startsWith("T=")){
            // motor driver board temperature
            revData = revData.remove(0,2);
             QStringList data = revData.split(":");
            motorBoardData[index].ch1Temp = data[0].toDouble();
            motorBoardData[index].ch2Temp = data[1].toDouble();
        }
        else if(revData.startsWith("V=")){
            // voltage data
            revData = revData.remove(0,2);
            QStringList data = revData.split(":");
            motorBoardData[index].drvVoltage = data[0].toDouble()/10;
            motorBoardData[index].motVoltage = data[1].toDouble()/10;
            motorBoardData[index].reg5Voltage = data[2].toDouble() /1000;

        }
        else if(revData.startsWith("CR=")){
            // here is the encoder relative difference reading,
            // very useful to estimate the encoder/motor traveling distance
        }
        else if(revData.startsWith("FF=")){
            // driver board state
            revData = revData.remove(0,3);
            motorBoardData[index].driverState = revData.toInt();
        }

        if (index == 0)
        {
            ui->lfmPosLineEdit->setText(QString::number(motorData[0].encoderPos));
            ui->lfmCurrentLineEdit->setText(QString::number(motorData[0].motorAmp,'f',2));
            ui->lfmPWMLineEdit->setText(QString::number(motorData[0].motorPower));
            ui->lfmTempLineEdit->setText(QString::number(motorData[0].motorTemp,'f',2));
            ui->lfmVelLineEdit->setText(QString::number(motorData[0].encoderSpeed));

            ui->motorVolLineEdit->setText(QString::number(motorBoardData[0].motVoltage,'f',2));
            ui->rfmPosLineEdit->setText(QString::number(motorData[1].encoderPos));
            ui->rfmCurrentLineEdit->setText(QString::number(motorData[1].motorAmp,'f',2));
            ui->rfmPWMLineEdit->setText(QString::number(motorData[1].motorPower));
            ui->rfmTempLineEdit->setText(QString::number(motorData[1].motorTemp,'f',2));
            ui->rfmVelLineEdit->setText(QString::number(motorData[1].encoderSpeed));

        }
        else if(index == 1)
        {
            ui->lrmPosLineEdit->setText(QString::number(motorData[2].encoderPos));
            ui->lrmCurrentLineEdit->setText(QString::number(motorData[2].motorAmp,'f',2));
            ui->lrmPWMLineEdit->setText(QString::number(motorData[2].motorPower));
            ui->lrmTempLineEdit->setText(QString::number(motorData[2].motorTemp,'f',2));
            ui->lrmVelLineEdit->setText(QString::number(motorData[2].encoderSpeed));

            ui->rrmPosLineEdit->setText(QString::number(motorData[3].encoderPos));
            ui->rrmCurrentLineEdit->setText(QString::number(motorData[3].motorAmp,'f',2));
            ui->rrmPWMLineEdit->setText(QString::number(motorData[3].motorPower));
            ui->rrmTempLineEdit->setText(QString::number(motorData[3].motorTemp,'f',2));
            ui->rrmVelLineEdit->setText(QString::number(motorData[3].encoderSpeed));
        }
        else if(index == 2)
        {
            ui->lffmPosLineEdit->setText(QString::number(motorData[4].encoderPos));
            ui->lffmCurrentLineEdit->setText(QString::number(motorData[4].motorAmp,'f',2));
            ui->lffmPWMLineEdit->setText(QString::number(motorData[4].motorPower));
            ui->lffmTempLineEdit->setText(QString::number(motorData[4].motorTemp,'f',2));
            ui->lffmVelLineEdit->setText(QString::number(motorData[4].encoderSpeed));

            ui->rffmPosLineEdit->setText(QString::number(motorData[5].encoderPos));
            ui->rffmCurrentLineEdit->setText(QString::number(motorData[5].motorAmp,'f',2));
            ui->rffmPWMLineEdit->setText(QString::number(motorData[5].motorPower));
            ui->rffmTempLineEdit->setText(QString::number(motorData[5].motorTemp,'f',2));
            ui->rffmVelLineEdit->setText(QString::number(motorData[5].encoderSpeed));
        }
        else if(index == 3)
        {
            ui->lrfmPosLineEdit->setText(QString::number(motorData[6].encoderPos));
            ui->lrfmCurrentLineEdit->setText(QString::number(motorData[6].motorAmp,'f',2));
            ui->lrfmPWMLineEdit->setText(QString::number(motorData[6].motorPower));
            ui->lrfmTempLineEdit->setText(QString::number(motorData[6].motorTemp,'f',2));
            ui->lrfmVelLineEdit->setText(QString::number(motorData[6].encoderSpeed));

            ui->rrfmPosLineEdit->setText(QString::number(motorData[7].encoderPos));
            ui->rrfmCurrentLineEdit->setText(QString::number(motorData[7].motorAmp,'f',2));
            ui->rrfmPWMLineEdit->setText(QString::number(motorData[7].motorPower));
            ui->rrfmTempLineEdit->setText(QString::number(motorData[7].motorTemp,'f',2));
            ui->rrfmVelLineEdit->setText(QString::number(motorData[7].encoderSpeed));
        }



        QString strError ="";
        for (int i = 0; i < 4; i++){
            strError.clear();
            if ((motorBoardData[i].driverState & 0x1) != 0){
                strError = "OH";
            }

            if ((motorBoardData[i].driverState & 0x2) != 0){
                strError += "OV";
            }
            if ((motorBoardData[i].driverState & 0x4) != 0){
                strError += "UV";
            }
            if ((motorBoardData[i].driverState & 0x8) != 0){
                strError += "SHT";
            }
            if ((motorBoardData[i].driverState & 0x10) != 0){
                strError += "ESTOP";
            }
            if ((motorBoardData[i].driverState & 0x20) != 0){
                strError += "SEPF";
            }
            if ((motorBoardData[i].driverState & 0x40) != 0){
                strError += "PromF";
            }
            if ((motorBoardData[i].driverState & 0x80) != 0){
                strError += "ConfF";
            }
            if (strError.length()< 1){
                strError = "OK";
            }
            if (i == 0){
                ui->driver1StateLineEdit->setText(strError);
            }
            else if (i == 1){
                ui->driver2StateLineEdit->setText(strError);
            }
            else if (i == 2){
                ui->driver3StateLineEdit->setText(strError);
            }
            else if (i == 3){
                ui->driver4StateLineEdit->setText(strError);
            }


        }
        //publish sensor here
	qnode.publisherMotorData(motorData,8);
        qnode.publisherMotorBoardInfoArray(motorBoardData,4);
        qnode.publisherFlipperData(flipperData);
    }

}

double MainWindow::ad2Temperature(int adValue)
{
    //for new temperature sensor
               double tempM = 0;
               double k = (adValue / FULLAD);
               double resValue = 0;
               if (k != 1)
               {
                   resValue = 10000 * k / (1 - k);      //AD value to resistor
               }
               else
               {
                   resValue = resTable[0];
               }


               int index = -1;
               if (resValue >= resTable[0])       //too lower
               {
                   tempM = -20;
               }
               else if (resValue <= resTable[24])
               {
                   tempM = 100;
               }
               else
               {
                   for (int i = 0; i < 24; i++)
                   {
                       if ((resValue <= resTable[i]) && (resValue >= resTable[i + 1]))
                       {
                           index = i;
                           break;
                       }
                   }
                   if (index >= 0)
                   {
                       tempM = tempTable[index] + (resValue - resTable[index]) / (resTable[index + 1] - resTable[index]) * (tempTable[index + 1] - tempTable[index]);
                   }
                   else
                   {
                       tempM = 0;
                   }

               }

               return tempM;
}


void MainWindow::sendForwardCmd()
{

    int cmd = ui->robotHSlider->value();
    QString strCmd = "MMW !M " + QString::number(-cmd) + " " + QString::number(cmd) + "\r\n";
    tcpRobot->write(strCmd.toUtf8().constData());
    ROS_INFO("Forward");

}

void MainWindow::sendStopCmd()
{
    QString strCmd = "MMW !M 0 0\r\n";
    tcpRobot->write(strCmd.toUtf8().constData());
}

void MainWindow::sendBackwardCmd()
{
    int cmd = ui->robotHSlider->value();
    QString strCmd = "MMW !M " + QString::number(cmd) + " " + QString::number(-cmd) + "\r\n";
    tcpRobot->write(strCmd.toUtf8().constData());

}

void MainWindow::sendLeftTurnCmd()
{
    int cmd = ui->robotHSlider->value();
    QString strCmd = "MMW !M " + QString::number(cmd) + " " + QString::number(cmd) + "\r\n";
    tcpRobot->write(strCmd.toUtf8().constData());

}

void MainWindow::sendRightTurnCmd()
{
    int cmd = ui->robotHSlider->value();
    QString strCmd = "MMW !M " + QString::number(-cmd) + " " + QString::number(-cmd) + "\r\n";
    tcpRobot->write(strCmd.toUtf8().constData());
}

void MainWindow::setLightsOff()
{
    QString strCmd = "SYS MMC " + QString::number(0x7f) +"\r\n";
    tcpRobot->write(strCmd.toUtf8().constData());
}

void MainWindow::setLightsOn()
{
    QString strCmd = "SYS MMC " + QString::number(0xff) + "\r\n";
    tcpRobot->write(strCmd.toUtf8().constData());
}

void MainWindow::sendEStopCmd()
{
    QString strCmd = "MMW !EX\r\n";
    tcpRobot->write(strCmd.toUtf8().constData());
}

void MainWindow::sendReleaseEStopCmd()
{
    QString strCmd = "MMW !MG\r\n";
    tcpRobot->write(strCmd.toUtf8().constData());
}

void MainWindow::SetInitCmd()
{
    for (int i = 0; i < 2; i++){
        flipArmMotor[i].angle = 0;
        flipArmMotor[i].iniFlag = true;
        flipArmMotor[i].preEncoder = flipArmMotor[i].encoderPos;
    }
}

void MainWindow::ResetInitCmd()
{
    if ((flipArmMotor[0].iniFlag) && (flipArmMotor[1].iniFlag)){
        driveFrontFlipDegree(0);
    }
    else {
        std::cout << "Set Init First !" << std::endl;
    }
}

void MainWindow::Flipper30UpCmd()
{
    if ((flipArmMotor[0].iniFlag) && (flipArmMotor[1].iniFlag)){
        driveFrontFlipDegree(30);
    }
    else {
        std::cout << "Set Init First !" << std::endl;
    }
}

void MainWindow::Flipper60DownCmd()
{
    if ((flipArmMotor[0].iniFlag) && (flipArmMotor[1].iniFlag)){
        driveFrontFlipDegree(-60);
    }
    else {
        std::cout << "Set Init First !" << std::endl;
    }
}

//////// send out command
void MainWindow::sendPing()
{
    QString strCmd = "PING\r\n";
        ++watchDogCnt;
        if (watchDogCnt == 1)
        {
                ui->motorControlGroupBox->setEnabled(true);
                ui->flipmotorControlGroupBox->setEnabled(true);
                ui->setIOgroupBox->setEnabled(true);
                tcpRobot->write(strCmd.toUtf8().constData());
	
        }
        else
        {
                if (watchDogCnt> 10)
                {
                        ui->motorControlGroupBox->setEnabled(false);
                        ui->flipmotorControlGroupBox->setEnabled(false);
                        ui->setIOgroupBox->setEnabled(false);
                        pingTimer.stop();
                        disconnect(&pingTimer, SIGNAL(timeout()), this, SLOT(sendPing()));
                        ui->connectButton ->setText(tr("Connect"));
                        tcpRobot->close();
                        watchDogCnt = 2;
                }
        }
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    QString strCmd;
    if (event->isAutoRepeat())
    {
        qDebug()<< "KeyPressEvent ignore";
        event->ignore();
    }else{
        if (!leftFrontFlipFlag)
        {
            if (event->key() == Qt::Key_A){
                leftFrontFlipFlag = true;
                leftFrontFlipCmd = -LEFTFRONTFLIP_CMD;
                ROS_INFO("FR: %f",flipArmMotor[0].angle);
                strCmd = "MM2 !PR 1 " + QString::number(leftFrontFlipCmd) + "\r\n";

                if (tcpRobot != NULL){
                    if (tcpRobot->isWritable())
                    {
                        tcpRobot->write(strCmd.toUtf8().constData());
                    }
                }
            }
            else if(event->key() == Qt::Key_Q){
                leftFrontFlipFlag = true;
                leftFrontFlipCmd = LEFTFRONTFLIP_CMD;
                strCmd = "MM2 !PR 1 " + QString::number(leftFrontFlipCmd) + "\r\n";
                if (tcpRobot != NULL){
                    if (tcpRobot->isWritable())
                    {
                        tcpRobot->write(strCmd.toUtf8().constData());
                    }
                }
            }
        }

        if (!rightFrontFlipFlag)
        {
            if (event->key() == Qt::Key_W){
                rightFrontFlipFlag = true;
                rightFrontFlipCmd = -RIGHTFRONTFLIP_CMD;
                strCmd = "MM2 !PR 2 " + QString::number(rightFrontFlipCmd) + "\r\n";
                if (tcpRobot != NULL){
                    if (tcpRobot->isWritable())
                    {
                        tcpRobot->write(strCmd.toUtf8().constData());
                    }
                }
            }
            else if(event->key() == Qt::Key_S){
                rightFrontFlipFlag = true;
                rightFrontFlipCmd = RIGHTFRONTFLIP_CMD;
                strCmd = "MM2 !PR 2 " + QString::number(rightFrontFlipCmd) + "\r\n";
                if (tcpRobot != NULL){
                    if (tcpRobot->isWritable())
                    {
                        tcpRobot->write(strCmd.toUtf8().constData());
                    }
                }
            }
        }
        if (!leftRearFlipFlag)
        {
            if (event->key() == Qt::Key_O){
                leftRearFlipFlag = true;
                leftRearFlipCmd = -LEFTREARFLIP_CMD;
                strCmd = "MM3 !PR 1 " + QString::number(leftRearFlipCmd) + "\r\n";
                if (tcpRobot != NULL){
                    if (tcpRobot->isWritable())
                    {
                        tcpRobot->write(strCmd.toUtf8().constData());
                    }
                }
            }
            else if(event->key() == Qt::Key_K){
                leftRearFlipFlag = true;
                leftRearFlipCmd = LEFTREARFLIP_CMD;
                strCmd = "MM3 !PR 1 " + QString::number(leftRearFlipCmd) + "\r\n";
                if (tcpRobot != NULL){
                    if (tcpRobot->isWritable())
                    {
                        tcpRobot->write(strCmd.toUtf8().constData());
                    }
                }
            }
        }
        if (!rightRearFlipFlag)
        {
            if (event->key() == Qt::Key_L){
                rightRearFlipFlag = true;
                rightRearFlipCmd = -RIGHTREARFLIP_CMD;
                strCmd = "MM3 !PR 2 " + QString::number(rightRearFlipCmd) + "\r\n";
                if (tcpRobot != NULL){
                    if (tcpRobot->isWritable())
                    {
                        tcpRobot->write(strCmd.toUtf8().constData());
                    }
                }
            }
            else if(event->key() == Qt::Key_P){
                rightRearFlipFlag = true;
                rightRearFlipCmd = RIGHTREARFLIP_CMD;
                strCmd = "MM3 !PR 2 " + QString::number(rightRearFlipCmd) + "\r\n";
                if (tcpRobot != NULL){
                    if (tcpRobot->isWritable())
                    {
                        tcpRobot->write(strCmd.toUtf8().constData());
                    }
                }
            }
        }
        if (event->key() == Qt::Key_Z){
//                if (tcpRobot != NULL){
//                    if (tcpRobot->isWritable()){
//                        double aux = 0.0;
//                        int pos = 0;
//                        aux = motorData[4].motorAmp;
//                        pos = motorData[4].encoderPos;
//                        int flag_exit = 0;

//                        for (int i=0;i<10;i++)
//                        {
//                            flipCmdSend(-100,0,0,0);
//                            ROS_INFO("IN LOOP FOR nº%d I=%f",i,aux);
//                            if(aux < 0)
//                            {
//                                sendFrontFLipEStop();
//                                ROS_INFO("Stop");
//                                break;
//                            }
//                            processRobotData();
//                            aux = motorData[4].motorAmp;
//                            sleep(1000);
//                            ROS_INFO("SLEEP");
//                        }

//                            flag_exit++;
//                            if(flag_exit > 1000)
//                                break;
//                        sendFrontFLipEStop();
//                        else if (motorData[4].motorAmp<0.0){
//                           sendFrontFLipEStop();
//                        }
//                    }
//                }
            for (int i = 0; i < 4; i++){
                flipArmMotor[i].angle = 0;
                flipArmMotor[i].iniFlag = true;
                flipArmMotor[i].preEncoder = flipArmMotor[i].encoderPos;
            }
        }
        if (event->key() == Qt::Key_X){
            if ((flipArmMotor[0].iniFlag) && (flipArmMotor[1].iniFlag)){
                double deltaAngle = (flipArmMotor[0].angle - flipArmMotor[1].angle)/2;
                int targetPos = (int)(-deltaAngle /(M_PI * 2) * FLIPARM_CIRCLE_CNT);
                strCmd = "MM2 !PR 1 " + QString::number(targetPos) + "\r\n";
                if (tcpRobot != NULL){
                    if (tcpRobot->isWritable()){
                        tcpRobot->write(strCmd.toUtf8().constData());
                    }
                }
                strCmd = "MM2 !PR 2 " + QString::number(targetPos) + "\r\n";
                if (tcpRobot != NULL){
                    if (tcpRobot->isWritable()){
                        tcpRobot->write(strCmd.toUtf8().constData());
                    }
                }
            }
        }

        if (event->key() == Qt::Key_M){
            if ((flipArmMotor[2].iniFlag) && (flipArmMotor[3].iniFlag)){
                double deltaAngle = (flipArmMotor[2].angle - flipArmMotor[3].angle)/2;
                int targetPos = (int)(deltaAngle /(M_PI * 2) * FLIPARM_CIRCLE_CNT);
                strCmd = "MM3 !PR 1 " + QString::number(targetPos) + "\r\n";
                if (tcpRobot != NULL){
                    if (tcpRobot->isWritable()){
                        tcpRobot->write(strCmd.toUtf8().constData());
                    }
                }
                strCmd = "MM3 !PR 2 " + QString::number(targetPos) + "\r\n";
                if (tcpRobot != NULL){
                    if (tcpRobot->isWritable()){
                        tcpRobot->write(strCmd.toUtf8().constData());
                    }
                }
            }
        }
        if (event->key() == Qt::Key_E){
            if ((flipArmMotor[0].iniFlag) && (flipArmMotor[1].iniFlag)){
                driveFrontFlipDegree(30);
            }
        }
        if (event->key() == Qt::Key_D){
            if ((flipArmMotor[0].iniFlag) && (flipArmMotor[1].iniFlag)){
                driveFrontFlipDegree(-60);
            }
        }
        if (event->key() == Qt::Key_I){
            if ((flipArmMotor[2].iniFlag) && (flipArmMotor[3].iniFlag)){
                driveRearFlipDegree(30);
            }
        }
        if (event->key() == Qt::Key_J){
            if ((flipArmMotor[2].iniFlag) && (flipArmMotor[3].iniFlag)){
                driveRearFlipDegree(-60);
            }
        }
        if (event->key() == Qt::Key_C){
            if ((flipArmMotor[0].iniFlag) && (flipArmMotor[1].iniFlag)){
                driveFrontFlipDegree(0);
            }
            if ((flipArmMotor[2].iniFlag) && (flipArmMotor[3].iniFlag)){
                driveRearFlipDegree(0);
            }
        }
        event->accept();
    }
}

void MainWindow::driveFrontFlipDegree(double targetAngle)
{
    double deltaAngle;
    QString strCmd;
    int targetPos = 0;
    targetAngle = targetAngle * M_PI/180;
    deltaAngle = targetAngle - flipArmMotor[0].angle;
    targetPos = (int)(deltaAngle/(M_PI * 2) * FLIPARM_CIRCLE_CNT);
    strCmd = "MM2 !PR 1 " + QString::number(targetPos) + "\r\n";
    if (tcpRobot != NULL){
        if (tcpRobot->isWritable()){
            tcpRobot->write(strCmd.toUtf8().constData());
        }
    }
    deltaAngle = targetAngle - flipArmMotor[1].angle;
    targetPos = (int)(deltaAngle/(M_PI * 2) * FLIPARM_CIRCLE_CNT);
    strCmd = "MM2 !PR 2 " + QString::number(targetPos) + "\r\n";
    if (tcpRobot != NULL){
        if (tcpRobot->isWritable()){
            tcpRobot->write(strCmd.toUtf8().constData());
        }
    }
    
}

void MainWindow::driveRearFlipDegree(double targetAngle)
{
    double deltaAngle;
    QString strCmd;
    int targetPos = 0;
    targetAngle = targetAngle * M_PI/180;
    deltaAngle = targetAngle - flipArmMotor[2].angle;
    targetPos = (int)(-deltaAngle/(M_PI * 2) * FLIPARM_CIRCLE_CNT);
    strCmd = "MM3 !PR 1 " + QString::number(targetPos) + "\r\n";
    if (tcpRobot != NULL){
        if (tcpRobot->isWritable()){
            tcpRobot->write(strCmd.toUtf8().constData());
        }
    }
    deltaAngle = targetAngle - flipArmMotor[3].angle;
    targetPos = (int)(deltaAngle/(M_PI * 2) * FLIPARM_CIRCLE_CNT);
    strCmd = "MM3 !PR 2 " + QString::number(targetPos) + "\r\n";
    if (tcpRobot != NULL){
        if (tcpRobot->isWritable()){
            tcpRobot->write(strCmd.toUtf8().constData());
        }
    }
    
}

void MainWindow::keyReleaseEvent(QKeyEvent *event)
{
if (event->isAutoRepeat())
    {
        qDebug()<< "KeyRelease event ignore";
        event->ignore();
    }else{
        //qDebug()<<event->key();
        if( (event->key() == Qt::Key_A) || (event->key() == Qt::Key_Q)){
            leftFrontFlipCmd = 0;
            leftFrontFlipFlag = false;
        }
        if( (event->key() == Qt::Key_W) || (event->key() == Qt::Key_S)){
            rightFrontFlipCmd = 0;
            rightFrontFlipFlag = false;
        }
        if( (event->key() == Qt::Key_O) || (event->key() == Qt::Key_K)){
            leftRearFlipCmd = 0;
            leftRearFlipFlag = false;
        }
        if( (event->key() == Qt::Key_P) || (event->key() == Qt::Key_L)){
            rightRearFlipCmd = 0;
            rightRearFlipFlag = false;
        }
        event->accept();
    }
}

void MainWindow::flipCtrlFun()
{
    QString strCmd;
    if (tcpRobot != NULL){
        if (leftFrontFlipFlag){
            strCmd = "MM2 !PR 1 " + QString::number(leftFrontFlipCmd) + "\r\n";
            if (tcpRobot->isWritable())
            {
                tcpRobot->write(strCmd.toUtf8().constData());
            }
        }
        if (rightFrontFlipFlag){
            strCmd = "MM2 !PR 2 " + QString::number(rightFrontFlipCmd) + "\r\n";
            if (tcpRobot->isWritable())
            {
                tcpRobot->write(strCmd.toUtf8().constData());
            }
        }
        if (leftRearFlipFlag){
            strCmd = "MM3 !PR 1 " + QString::number(leftRearFlipCmd) + "\r\n";
            if (tcpRobot->isWritable())
            {
                tcpRobot->write(strCmd.toUtf8().constData());
            }
        }
        if (rightRearFlipFlag){
            strCmd = "MM3 !PR 2 " + QString::number(rightRearFlipCmd) + "\r\n";
            if (tcpRobot->isWritable())
            {
                tcpRobot->write(strCmd.toUtf8().constData());
            }
        }
    }
}

void MainWindow::getFrontFlipAngle()
{
    int deltaEncoder = 0;
    if (flipArmMotor[0].iniFlag){       //Front
        deltaEncoder = flipArmMotor[0].encoderPos - flipArmMotor[0].preEncoder;
        flipArmMotor[0].angle = (double)(deltaEncoder % FLIPARM_CIRCLE_CNT)/FLIPARM_CIRCLE_CNT * M_PI * 2 + flipArmMotor[0].angle;
        if (flipArmMotor[0].angle > M_PI) flipArmMotor[0].angle = -(2 * M_PI - flipArmMotor[0].angle);
        if (flipArmMotor[0].angle < -M_PI) flipArmMotor[0].angle = (2 * M_PI + flipArmMotor[0].angle);
        flipperData.Frontangle = flipArmMotor[0].angle;
        flipArmMotor[0].preEncoder = flipArmMotor[0].encoderPos;
    }

    if (flipArmMotor[1].iniFlag){   //Rear
        deltaEncoder = flipArmMotor[1].encoderPos - flipArmMotor[1].preEncoder;
        flipArmMotor[1].angle = (double)(deltaEncoder % FLIPARM_CIRCLE_CNT)/FLIPARM_CIRCLE_CNT * M_PI * 2 + flipArmMotor[1].angle;
        if (flipArmMotor[1].angle > M_PI) flipArmMotor[1].angle = -(2 * M_PI - flipArmMotor[1].angle);
        if (flipArmMotor[1].angle < -M_PI) flipArmMotor[1].angle = (2 * M_PI + flipArmMotor[1].angle);
        flipperData.Rearangle = flipArmMotor[1].angle;
        flipArmMotor[1].preEncoder = flipArmMotor[1].encoderPos;
    }
}

void MainWindow::getRearFlipAngle()
{
     int deltaEncoder = 0;
    if (flipArmMotor[2].iniFlag){       //leftRear
        deltaEncoder = flipArmMotor[2].encoderPos - flipArmMotor[2].preEncoder;
        flipArmMotor[2].angle = (double)(-deltaEncoder % FLIPARM_CIRCLE_CNT)/FLIPARM_CIRCLE_CNT * M_PI * 2 + flipArmMotor[2].angle;
        if (flipArmMotor[2].angle > M_PI) flipArmMotor[2].angle = -(2 * M_PI - flipArmMotor[2].angle);
        if (flipArmMotor[2].angle < -M_PI) flipArmMotor[2].angle = (2 * M_PI + flipArmMotor[2].angle);
        flipArmMotor[2].preEncoder = flipArmMotor[2].encoderPos;
    }

    if (flipArmMotor[3].iniFlag){   //rightRear
        deltaEncoder = flipArmMotor[3].encoderPos - flipArmMotor[3].preEncoder;
        flipArmMotor[3].angle = (double)(deltaEncoder % FLIPARM_CIRCLE_CNT)/FLIPARM_CIRCLE_CNT * M_PI * 2 + flipArmMotor[3].angle;
        if (flipArmMotor[3].angle > M_PI) flipArmMotor[1].angle = -(2 * M_PI - flipArmMotor[3].angle);
        if (flipArmMotor[3].angle < -M_PI) flipArmMotor[1].angle = (2 * M_PI + flipArmMotor[3].angle);
        flipArmMotor[3].preEncoder = flipArmMotor[3].encoderPos;
    }

}
void MainWindow::sendFrontFLipEStop()
{
    QString strCmd = "MM2 !EX\r\n";
    if (tcpRobot != NULL){
        if (tcpRobot->isWritable()){
            tcpRobot->write(strCmd.toUtf8().constData());
        }
    }
}
void MainWindow::sendFrontFLipReleaseEStop()
{
    QString strCmd = "MM2 !MG\r\n";
    if (tcpRobot != NULL){
        if (tcpRobot->isWritable()){
            tcpRobot->write(strCmd.toUtf8().constData());
        }
    }
}

void MainWindow::sendRearFLipEStop()
{
    QString strCmd = "MM3 !EX\r\n";
    if (tcpRobot != NULL){
        if (tcpRobot->isWritable()){
            tcpRobot->write(strCmd.toUtf8().constData());
        }
    }
}
void MainWindow::sendRearFLipReleaseEStop()
{
    QString strCmd = "MM3 !MG\r\n";
    if (tcpRobot != NULL){
        if (tcpRobot->isWritable()){
            tcpRobot->write(strCmd.toUtf8().constData());
        }
    }
}

void MainWindow::gyroCalibrate()
{
    QString strCmd = "SYS CAL\r\n";
    if (tcpRobot != NULL){
        if (tcpRobot->isWritable()){
            tcpRobot->write(strCmd.toUtf8().constData());
        }
    }
}


void MainWindow::closeEvent(QCloseEvent *event)
{
	qDebug()<<"CloseEvent";
	if (tcpRobot != NULL)
	{
		if (tcpRobot->isOpen()) tcpRobot->close();
	}
	pingTimer.stop();
	flipCtrlTimer.stop();
	QWidget::closeEvent(event);
}


//for qnode
void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

void MainWindow::wheelCmdSend(int cmdValue1,int cmdValue2)
{
	QString strCmd;


	if ((cmdValue1 < -1000) || (cmdValue2 < -1000))
	{
		strCmd = "MMW !EX\r\n";
	}
	else if((cmdValue1 > 1000) || (cmdValue2 > 1000))
	{
		strCmd = "MMW !MG\r\n";
	}
	else
	{
		strCmd = "MMW !M " + QString::number(cmdValue1) + " " + QString::number(cmdValue2) + "\r\n";
	}

	if (tcpRobot != NULL){
		if (tcpRobot->isWritable())
		{
		    tcpRobot->write(strCmd.toUtf8().constData());
		}
	}
}
void MainWindow::flipCmdSend (int leftFrontCmd,int rightFrontCmd,int leftRearCmd,int rightRearCmd)
{
	QString strCmd;
        strCmd = "MM2 !PR 1 " + QString::number(leftFrontCmd) + "\r\n";
	if (tcpRobot != NULL){
		if (tcpRobot->isWritable())
		{
		    tcpRobot->write(strCmd.toUtf8().constData());
		}
	}

	strCmd = "MM2 !PR 2 " + QString::number(rightFrontCmd) + "\r\n";
	if (tcpRobot != NULL){
		if (tcpRobot->isWritable())
		{
		    tcpRobot->write(strCmd.toUtf8().constData());
		}
	}

	strCmd = "MM3 !PR 1 " + QString::number(leftRearCmd) + "\r\n";
	if (tcpRobot != NULL){
		if (tcpRobot->isWritable())
		{
		    tcpRobot->write(strCmd.toUtf8().constData());
		}
	}

	strCmd = "MM3 !PR 2 " + QString::number(rightRearCmd) + "\r\n";
	if (tcpRobot != NULL){
		if (tcpRobot->isWritable())
		{
		    tcpRobot->write(strCmd.toUtf8().constData());
		}
	}
}


}  //end namespace
