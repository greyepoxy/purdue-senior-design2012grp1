/****************************************************************************
**
** Written by Justin Huffaker
**  adapted form Qt Opengl_ES2 example
**
****************************************************************************/

#include <QApplication>
#include <QMainWindow>
#include "mainwindow.h"

int main( int argc, char ** argv )
{
    Q_INIT_RESOURCE(texture);
    QApplication a( argc, argv );
    MainWindow mw;
    mw.showMaximized();
    return a.exec();
}
