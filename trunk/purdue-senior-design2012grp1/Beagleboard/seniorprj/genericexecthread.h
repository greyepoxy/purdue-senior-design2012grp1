#ifndef GENERICEXECTHREAD_H
#define GENERICEXECTHREAD_H

#include <QThread>
#include <QTimer>
#include <stdio.h>

class GenericExecThread : public QThread
{
    Q_OBJECT
public:
    explicit GenericExecThread(QThread *parent = 0);
    ~GenericExecThread();
    void run();

    void forceSleep(unsigned long secs);
    
signals:
    
public slots:
    
};

#endif // GENERICEXECTHREAD_H
