#include <QtGui>
#include <QApplication>
#include "../include/drrobot_jaguar_v6/mainwindow.hpp"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    drrobot_jaguar_v6::MainWindow w(argc,argv);

    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();
    return app.exec();
}
