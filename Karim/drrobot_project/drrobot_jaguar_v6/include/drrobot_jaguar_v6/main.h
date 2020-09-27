#ifndef MAIN_H
#define MAIN_H

#include <QCoreApplication>
#include <QDebug>
#include <QtNetwork>
#include <QElapsedTimer>
#include <iostream>
#include </home/robcib/catkin_ws/src/drrobot_jaguar_v6/include/drrobot_jaguar_v6/drrobotprotocol.hpp>

void dealWithPackage(QString revData, int len);
void setLightsOff();
void setLightsOn();
void sendEStopCmd();
void sendReleaseEStopCmd();


#endif // MAIN_H
