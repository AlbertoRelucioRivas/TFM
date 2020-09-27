#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtNetwork>
#include <QTimer>
#include <QtGui>
//#include "ui_mainwindow.hpp"
#include "qnode.hpp"
#include "drrobotprotocol.hpp"
namespace drrobot_jaguar_v6 {


class MainWindow : public QThread
{
    Q_OBJECT

public:
    
    MainWindow(int argc, char** argv);
    ~MainWindow();

   void showNoMasterMessage();
   void connectToRobot();

signals:
    void CloseProg();

private:
    QNode qnode;
    QTcpSocket *tcpRobot;

    QTimer pingTimer;

    void dealWithPackage( QString revData, int len);

    //for control
    int motDir;
    int leftWheelPos;
    int rightWheelPos;

    double wheelRadius;
    double disOf2Wheel;
    int encoderCntOneCircle;

    int ctrlMode;
    double ad2Temperature(int value);
    int watchDogCnt;
    //IMU sensor data
    IMUData imuData;
    //GPS sensor data
    GPSData gpsData;
    //motor and driver board data
    MotorData motorData[8];
    MotorBoardData motorBoardData[4];
    //flipper angle
    FlipperData flipperData;
    //for flip arm control
    bool leftFrontFlipFlag;
    int leftFrontFlipCmd;
    bool rightFrontFlipFlag;
    int rightFrontFlipCmd;
    bool leftRearFlipFlag;
    int leftRearFlipCmd;
    bool rightRearFlipFlag;
    int rightRearFlipCmd;
    QTimer flipperTimer;
    MotorData flipArmMotor[4];

    void getFrontFlipAngle();
    void driveFrontFlipDegree(double targetAngle);

private slots:

    void sendPing();
    void processRobotData();
    void setLightsOff();
    void setLightsOn();

    void sendEStopCmd();        //Estop command
    void sendReleaseEStopCmd(); // release EStop command
    void sendFLipEStop();
    void sendFLipReleaseEStop();
    void ResetInitCmd();
    void SetInitCmd();
    void InitCmdReceived();
    void Flipper30UpCmd();
    void Flipper60DownCmd();
    void Close();

// slot for qnode
    void wheelCmdSend(int cmdValue1,int cmdValue2);
    void flipCmdSend (int FrontFlipCmd,int RearFlipCmd);

};

class IP4Validator : public QValidator
{
    public:
        IP4Validator(QObject *parent=0) : QValidator(parent){}
        void fixup(QString &input) const {}
        State validate(QString &input, int &pos) const
        {
            if(input.isEmpty())
                return Acceptable;
            QStringList slist = input.split(".");
            int s = slist.size();
            if(s>4) return Invalid;
            bool emptyGroup = false;
            for(int i=0;i<s;i++)
            {
                bool ok;
                if(slist[i].isEmpty())
                {
                    emptyGroup = true;
                    continue;
                }
                int val = slist[i].toInt(&ok);
                if(!ok || val<0 || val>255)
                    return Invalid;
            }
            if(s<4 || emptyGroup)
                return Intermediate;

            return Acceptable;
            }
};

} //end namespace
#endif // MAINWINDOW_H
