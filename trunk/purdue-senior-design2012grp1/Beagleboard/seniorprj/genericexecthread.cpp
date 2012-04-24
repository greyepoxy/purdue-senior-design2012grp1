#include "genericexecthread.h"

GenericExecThread::GenericExecThread(QThread *parent)
    : QThread(parent)
{
}

GenericExecThread::~GenericExecThread()
{
    this->quit();
    this->wait();
}

void GenericExecThread::run()
{
    printf("Starting New thread ID: %d\n",QThread::currentThreadId());
    exec();
}

void GenericExecThread::forceSleep(unsigned long secs)
{
    this->sleep(secs);
}


