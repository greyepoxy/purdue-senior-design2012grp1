/****************************************************************************
**
** Written by Justin Huffaker
**  adapted form Qt Opengl_ES2 example
**
****************************************************************************/

#include "mainwindow.h"

#include <QApplication>
#include <QMenuBar>
#include <QGroupBox>
#include <QGridLayout>
#include <QLabel>
#include <QTimer>

#include "glwidget.h"

MainWindow::MainWindow()
{
    GLWidget *glwidget = new GLWidget();
    QLabel *label = new QLabel(this);
    QTimer *timer = new QTimer(this);
    
    timer->setInterval(33);
    label->setText("Kalman Filter Example");
    label->setAlignment(Qt::AlignHCenter);

    QGroupBox * groupBox = new QGroupBox(this);
    setCentralWidget(groupBox);
    groupBox->setTitle("VIPER");

    QGridLayout *layout = new QGridLayout(groupBox);

    layout->addWidget(glwidget,1,0,8,1);
    layout->addWidget(label,9,0,1,1);

    groupBox->setLayout(layout);

    QMenu *fileMenu = new QMenu("File");
    QMenu *showMenu = new QMenu("Show");
    menuBar()->addMenu(fileMenu);
    menuBar()->addMenu(showMenu);
    QAction *exit = new QAction("Exit", fileMenu);
    QAction *showCube= new QAction("Show Cube", showMenu);
    QAction *showTeapot = new QAction("Show Monkey", showMenu);
    QAction *showBubbles = new QAction("Show bubbles", showMenu);
    showBubbles->setCheckable(true);
    showBubbles->setChecked(false);
    fileMenu->addAction(exit);
    showMenu->addAction(showCube);
    showMenu->addAction(showTeapot);
    showMenu->addAction(showBubbles);
    this->setFocusPolicy(Qt::StrongFocus);
    glwidget->setFocus();

    QObject::connect(timer, SIGNAL(timeout()), glwidget, SLOT(updateGL()));
    QObject::connect(exit, SIGNAL(triggered(bool)), this, SLOT(close()));

    QObject::connect(showCube, SIGNAL(triggered(bool)), glwidget, SLOT(setToCube()));
    QObject::connect(showTeapot, SIGNAL(triggered(bool)), glwidget, SLOT(setToMonkey()));
    QObject::connect(showBubbles, SIGNAL(triggered(bool)), glwidget, SLOT(showBubbles(bool)));
    timer->start();
}

void MainWindow::keyReleaseEvent(QKeyEvent *e)
{
    switch (e->key())
    {
    case (Qt::Key_Escape):
        this->close();
        break;
    }
}
