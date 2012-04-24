#include <stdio.h>

#include "usbthread.h"

#define GYRO_DATA_CONV 0.06103701895199438459425641651662 //degreesPerBit

#define MY_VID 0x04D8
#define MY_PID 0x0204

#define EP_IN 0x81
#define EP_OUT 0x01

#define MAG_XMAX 2577.0
#define MAG_XMIN 1695.0
#define MAG_YMAX -1176.0
#define MAG_YMIN -2135.0
#define MAG_ZMAX -2499.0
#define MAG_ZMIN -3380.0

USBThread::USBThread(int* numSignals, QMutex* intLock) :
    QObject()
{
    //initilize the data queue
    this->numSignalsLock = intLock;
    this->numSignals= numSignals;
    packetLock = new QMutex();

    USBtimer = new QTimer(this);
    USBtimer->setInterval(1);
    QObject::connect(USBtimer, SIGNAL(timeout()), this, SLOT(GetUSBData()),Qt::UniqueConnection);

    magDataFile = new QFile("./magData.txt");
    magDataFile->open(QIODevice::WriteOnly);
    magOut = new QDataStream(magDataFile);   //  serialize the data into the file

    connected = false;
    USBtimer->start();
}

USBThread::~USBThread()
{
    libusb_release_interface(handle, 0);
    libusb_exit(sensor_cxt);
    delete USBtimer;
    delete packetLock;
    delete magOut;
    magDataFile->close();
    delete magDataFile;
}

void USBThread::ConnectToUSBDevice()
{
    printf("Connect to the USB device -> THREAD: %d\n",QThread::currentThreadId());
    //initilize usb
    libusb_init(&sensor_cxt);
    libusb_set_debug(sensor_cxt, 1);

    bool usbSetUp = false;
    while (!usbSetUp) {
        usbSetUp = true;
        printf("going to try and get handle for USB\n");
        handle = libusb_open_device_with_vid_pid(sensor_cxt,MY_VID,MY_PID);
        if (handle == 0) {
            printf("failed to get handle\n");
            usbSetUp = false;
            ((GenericExecThread*)(this->thread()))->forceSleep(1);
            continue;
        }
        printf("got handle!\n");
        //printf("Reseting device\n");
        //libusb_reset_device(handle);
        //printf("Device Reset\n");

        int usbConfig;
        libusb_get_configuration(handle, &usbConfig);
        if (usbConfig == 0)
        {
            printf("Setting Configuration\n");
            if (!libusb_set_configuration(handle, 1)) {
                printf("Failed to set configuration\n");
                libusb_close(handle);
                FailedToConnect();
                usbSetUp = false;
                ((GenericExecThread*)(this->thread()))->forceSleep(10);
                continue;
            }
            printf("Set Configuration\n");
        }
        printf("Claiming Interface\n");
        int claimResults = libusb_claim_interface(handle, 0);
        if (claimResults != 0) {
            printf("Failed to claim Interface\n");
            libusb_close(handle);
            usbSetUp = false;
            FailedToConnect();
            if (LIBUSB_ERROR_NOT_FOUND == claimResults)
                printf("Requested Interface does not exist.\n");
            else if (LIBUSB_ERROR_BUSY == claimResults)
                printf("Another program or driver has claimed the interface.\n");
            else if (LIBUSB_ERROR_NO_DEVICE == claimResults)
                printf("The device has been disconnected .\n");
            ((GenericExecThread*)(this->thread()))->forceSleep(10);
            continue;
        }
        printf("Claimed Interface\n");
    }
    // done connecting and configuring, should be able to
    //    send bulk packets now
}

void USBThread::GetUSBData()
{
    if (!connected) {
        ConnectToUSBDevice();
        connected = true;
    }
    int actualLength;
    OutputPacketBuffer[0] = 0x80; // Send command 0x80, get data
    //Send data
    int done = libusb_bulk_transfer(handle,EP_OUT,&OutputPacketBuffer[0], 64, &actualLength, 1000);
    if (done != 0 || actualLength != 64)
    {
        fprintf(stderr, "Error during transfer request command.\n");
        if (done == LIBUSB_ERROR_TIMEOUT)
            fprintf(stderr, "USB write TIMEOUT.\n");
        else if (actualLength != 64)
            fprintf(stderr, "Did not send expected number of bytes.\n");
        else if (done == LIBUSB_ERROR_PIPE)
            fprintf(stderr, "The control request was not supported by the device.\n");
        else if (done == LIBUSB_ERROR_NO_DEVICE)
            fprintf(stderr, "The device has been disconnected.\n");
        else
            fprintf(stderr, "Some other problem, err code: %d.\n", done);
        return;
    }

    //Get the response to our transfer
    done = libusb_bulk_transfer(handle,EP_IN,&InputPacketBuffer[0], 64, &actualLength, 1000);
    if (done != 0 || actualLength != 64)
    {
        fprintf(stderr, "Error during recieve request command.\n");
        if (done == LIBUSB_ERROR_TIMEOUT)
            fprintf(stderr, "USB read TIMEOUT.\n");
        else
            fprintf(stderr, "Some other problem.\n");
        if (actualLength != 64)
            fprintf(stderr, "Did not recieve expected number of bytes.\n");
        return;
    }

    //PrintDataPacket(&InputPacketBuffer[0]);
    packetLock->lock();
    curPacket = PrintDataPacket(&InputPacketBuffer[0]);
    packetLock->unlock();

    //QString tMagOutput;

    //Read the data!!!
    int newData, curByte = 0;
    for (int k = 0; k < 5; k++)
    {
        newData = InputPacketBuffer[curByte];
        QVector3D data;
        if (newData > 16)
        {
            newData -= 16; // remove new data byte
            switch (newData) // switch on data type
            {
            int tdata,i;
            qint8 tdata8;
            qint16 tdata16;
            case kalmanFilter::accel:
                for (i=curByte+1; i<curByte+7; i+=2)
                {
                    // Turn the MSB and LSB into a 16-bit value
                    tdata16 = (((InputPacketBuffer[i] << 8) & 0xFF00) | (InputPacketBuffer[i+1] & 0x00FF));
                    if (i == curByte+1)
                        data.setZ(tdata16/1024.0); //incoming x is actually z in g
                    else if (i == curByte+3)
                        data.setX(-tdata16/1024.0); //incoming y is actuall -x in g
                    else
                        data.setY(tdata16/1024.0); //incoming z is actually y in g
                }
                break;
            case kalmanFilter::mag:
                /* loop to calculate 16-bit ADC values for each axis */
                for (i=curByte+1; i<curByte+7; i+=2)
                {
                    // Turn the MSB and LSB into a 16-bit value
                    tdata16 = (((InputPacketBuffer[i] << 8) & 0xFF00) | (InputPacketBuffer[i+1] & 0x00FF));
                    if (i == curByte+1)
                        data.setZ(((2*tdata16)-(MAG_XMIN+MAG_XMAX))/(MAG_XMAX-MAG_XMIN)); //incoming x is z
                    else if (i == curByte+3)
                        data.setX(((2*tdata16)-(MAG_YMIN+MAG_YMAX))/(MAG_YMAX-MAG_YMIN)); //incoming y is x
                    else
                        data.setY(-((2*tdata16)-(MAG_ZMIN+MAG_ZMAX))/(MAG_ZMAX-MAG_ZMIN)); //incoming z is -y
                }
                //tMagOutput.sprintf("%.2f,%.2f,%.2f\n", data.x(),data.y(),data.z());
                //(*magOut) << tMagOutput.toAscii();
                data.normalize();
                break;
            case kalmanFilter::gyro:
                /* loop to calculate 16-bit ADC values for each axis */
                for (i=curByte+1; i<curByte+7; i+=2)
                {
                    // Turn the MSB and LSB into a 16-bit value
                    tdata16 = (((InputPacketBuffer[i] << 8) & 0xFF00) | (InputPacketBuffer[i+1] & 0x00FF));
                    if (i == curByte+1)
                        data.setX(tdata16*GYRO_DATA_CONV); //incoming x is x in deg/sec
                    else if (i == curByte+3)
                        data.setZ(tdata16*GYRO_DATA_CONV); //incoming y is z in deg/sec
                    else
                        data.setY(-tdata16*GYRO_DATA_CONV); //incoming z is -y in deg/sec
                }
                break;
            case kalmanFilter::ultra:
                // Turn the MSB and LSB into a 16-bit value
                tdata = (InputPacketBuffer[curByte+1] << 8) | (InputPacketBuffer[curByte+2]);
                data.setX(tdata/1000.0); //convert to meters
                break;
            case kalmanFilter::cam:
                tdata8 = InputPacketBuffer[curByte+1] & 0x00FF; //pixel x
                data.setX(tdata8);
                tdata8 = InputPacketBuffer[curByte+2] & 0x00FF; //pixel y
                data.setY(tdata8);
                break;
            default:
                break;
            }
            // data is new add to queue
            //printf("Data for: %d -> THREAD=%d\n",newData,QThread::currentThreadId());
            numSignalsLock->lock();
            *(numSignals) = *(numSignals) + 1;
            numSignalsLock->unlock();
            //printf("New KalmanFilter for %d: (%.2f,%.2f,%.2f)\n", newData, data.x(),data.y(),data.z());
            emit newKalmanFilterData(kalmanFilter::SensorData((kalmanFilter::SensorInputFrom)newData,QVector3D(data)));
        }
        // If data is not new, increment curByte and do nothing else
        switch (newData)
        {
        case kalmanFilter::accel:
        case kalmanFilter::gyro:
        case kalmanFilter::mag:
            curByte += 7;
            break;
        case kalmanFilter::ultra:
        case kalmanFilter::cam:
            curByte += 3;
            break;
        }
    }
}

void USBThread::FailedToConnect() {
    fprintf(stderr, "Failed to connect to USB device, will try again in 10 seconds\n");
}

QString USBThread::PrintDataPacket(unsigned char packet[])
{
    QString outP,tempStr;
    outP = "Printing Data Packet: ";
    int index,i,j;
    for (i = 0; i < 2; i++) {
        for (j = 0; j < 16; j++) {
            index = i * 16 + j;
            tempStr.sprintf("%2.2x ",packet[index]);
            outP += tempStr;
        }
        //outP += "\n";
    }
    return outP;
}
