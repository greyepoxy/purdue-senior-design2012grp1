#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <QtGui/qvector3d.h>
#include <QtGui/qmatrix4x4.h>
#include <vector>
#include <QVector>
#include <QGenericMatrix>
#include <QTime>
#include <QMutex>
#include <stdio.h>
#include <QThread>

class kalmanFilter : public QObject
{
    Q_OBJECT

public:
    enum SensorInputFrom
    {
        accel = 1,
        gyro = 3,
        mag = 2,
        ultra = 4,
        cam = 5
    };

    typedef struct SensorData
    {
        SensorInputFrom from;
        QVector3D data;
        SensorData()
        {
            this->data = QVector3D(0,0,0);
            this->from = accel;
        }
        ~SensorData()
        {
        }
        SensorData(SensorData *tocopy)
        {
            this->data = QVector3D(tocopy->data);
            this->from = tocopy->from;
        }
        SensorData(SensorInputFrom fromS, QVector3D data)
        {
            this->data = data;
            this->from = fromS;
        }
    } SensorData;

    kalmanFilter();
    ~kalmanFilter();
    static QMatrix4x4 GetRotationMatrix(qreal x,qreal y,qreal z);
    static void print15x15Matrix(QGenericMatrix<15,15,qreal> *m);
    static void print15x1Matrix(QGenericMatrix<1,15,qreal> *m);
    static void print3x15Matrix(QGenericMatrix<15,3,qreal> *m);
    static void print15x3Matrix(QGenericMatrix<3,15,qreal> *m);
    static qreal wrapClamp(qreal v,int min,int max);
    QMatrix4x4 GetRotMatAroundAxis(qreal angle, QVector3D axisOfRot);

    //Kalman Filter public variables
    QGenericMatrix<1,15,qreal> stateVector;
    //QQuaternion globalCoordFrame;
    QVector3D globalCoordFrame;

    int kfUScount;

    static void PrintSensorData(SensorData *d);

    //most recent sensor readings
    QVector3D curAccelReading;
    QVector3D curGyroReading;
    QVector3D curMagReading;
    qreal curDist;
    QVector3D curCamCoord;
    QMutex *kalMutex;
    int NumPendingUpdates;
    //eye orientation estimate
    QVector3D camDirEst;
    QVector3D camUpDirEst;
    QVector3D camPos;

private:
    QGenericMatrix<15,15,qreal> GetPFromZeta(qreal *v);
    void updateKalmanFilter(SensorData d, qreal deltaTsec);
    QGenericMatrix<15,15,qreal> modelStateDiscreateTransition(qreal deltaT);
    void AddQdt(QGenericMatrix<15,15,qreal> *Pm, float deltaTsec);
    QGenericMatrix<3,3,qreal> GetAccelGyroMagR(SensorInputFrom f);
    QGenericMatrix<2,2,qreal> GetCamR();
    QGenericMatrix<1,1,qreal> GetDistR();
    QVector3D PredictMeasurment(SensorInputFrom device);

    QGenericMatrix<15,3,qreal> GetAccelJacobian();
    QGenericMatrix<15,3,qreal> GetGyroJacobian();
    QGenericMatrix<15,3,qreal> GetMagJacobian();
    QGenericMatrix<15,2,qreal> GetCamJacobian();
    QGenericMatrix<15,1,qreal> GetUltraJacobian();

    QGenericMatrix<15,15,qreal> P;
    QVector3D muLocation;
    QVector3D muOrientation;
    QTime previousCalcTime;

    //sensor variance values
    QVector3D varAccel;
    QVector3D varMag;
    QVector3D varGyro;
    QVector3D varCam;
    qreal varDist;

    QVector3D gAccel;
    QVector3D gGyro;
    QVector3D gMag;
    QVector3D gCam;
    qreal gDist;

signals:

public slots:
    void update(kalmanFilter::SensorData d);
};

Q_DECLARE_METATYPE(kalmanFilter::SensorData)

#endif // KALMANFILTER_H
