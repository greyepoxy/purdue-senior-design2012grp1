#ifndef USBTHREAD_H
#define USBTHREAD_H

#include <libusb-1.0/libusb.h>
#include <QTimer>
#include <QMutex>
#include <stdio.h>

#include <QFile>
#include <QDataStream>

#include "kalmanfilter.h"
#include "genericexecthread.h"

class USBThread : public QObject
{
    Q_OBJECT

    bool connected;
    int* numSignals;
    QMutex* numSignalsLock;
    QTimer* USBtimer;
    libusb_context* sensor_cxt;
    libusb_device_handle* handle;
    void FailedToConnect();
    unsigned char OutputPacketBuffer[64]; //Allocate a memory buffer which will contain data to send to the USB device
    unsigned char InputPacketBuffer[64];  //Allocate a memory buffer for the data which will be read from the USB device
    void ConnectToUSBDevice();

    QFile *magDataFile;
    QDataStream *magOut;

public:
    USBThread(int* numSignals, QMutex* intLock);
    ~USBThread();

    QString curPacket;
    QMutex* packetLock;

    void StartUSBTimer();

    static QString PrintDataPacket(unsigned char *packet);
    
signals:
    void newKalmanFilterData(kalmanFilter::SensorData d);
    
public slots:
    void GetUSBData();
    
};

#endif // USBTHREAD_H
