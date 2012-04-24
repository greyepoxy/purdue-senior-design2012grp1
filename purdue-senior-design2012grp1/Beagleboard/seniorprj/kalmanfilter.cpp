#include <math.h>
#include "kalmanfilter.h"

//#define DEBUGOUTPUT
#define G   9.80665    // acceleration due to gravity m/s
#define MG  1         // default expected value of magnetic field in microTesla
#define NF  3.095       // Near field of camera in mm
#define PIXELWIDTH  128 // num pixels in width of camera
#define PIXELHEIGHT 96  // num pixels in height of camera
#define IPWIDTH  3.394  // width of camera's image plane in mm
#define IPHEIGHT 2.53   // height of camera's image plane in mm
#define DEGREEWIDTH 57.4 // angle width of camera
#define DEGREEHEIGHT 44.5 // angle height of camera
#define ANGLEStoRAD 3.14159265/180


kalmanFilter::kalmanFilter()
    : QObject()
{
    // Initial position estimate
    qreal stateVecValues[15] = {0,-4,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0};
    globalCoordFrame = QVector3D(0,0,0);//QQuaternion(1,0,0,0);
    stateVector = QGenericMatrix<1,15,qreal>(stateVecValues);
    qreal zeta[15] = {2,2,1, 1,1,1, .5,.5,.5, 90,90,90, 0,0,0};
    P = GetPFromZeta(zeta);

    // Persistent error estimate
    muLocation = QVector3D(4.48e-2,4.48e-2,3.47e-2);
    muOrientation = QVector3D(15.35526,8.479775368,2.539348948);
    // Sensor error estimate
    varAccel = QVector3D(0.025*0.025,0.025*0.025,0.025*0.025); //var of accel in m/s^2
    varMag = QVector3D(.25*.25,.25*.25,.25*.25); //var of mag in microtesla rms
    varGyro = QVector3D(0.38*0.38,0.38*0.38,0.38*0.38); //var of gyro in degrees/sec
    varCam = QVector3D(2*2,2*2,0); //var of camera in pixels
    varDist = 0.05*0.05; //var of dist in meters

    //Register data type
    int id = qRegisterMetaType<SensorData>();
    //printf("ID of SensorData: %d\n",id);

    //Most recent Sensor reading
    curAccelReading = QVector3D(0,0,0);
    curGyroReading = QVector3D(0,0,0);
    curMagReading = QVector3D(0,0,0);
    curCamCoord = QVector3D(0,0,0);
    curDist = 0;
    //Initilize mutex and number of pending updates
    kalMutex = new QMutex();
    NumPendingUpdates = 0;
    //Global Complementry filtered values
    gAccel = QVector3D(0,0,0);
    gGyro = QVector3D(0,0,0);
    gMag = QVector3D(0,0,0);
    camDirEst = QVector3D(0,1,0);
    camUpDirEst = QVector3D(0,0,1);
    camPos = QVector3D(0,-4,0);
    gCam = QVector3D(0,0,0);
    gDist = 1;


    kfUScount = 0;

    previousCalcTime = QTime::currentTime();
}

kalmanFilter::~kalmanFilter()
{
    delete kalMutex;
}

void kalmanFilter::update(kalmanFilter::SensorData d)
{
    QTime curCalcTime = QTime::currentTime();
    qreal deltaT = previousCalcTime.msecsTo(curCalcTime);
    qreal deltaTsec = deltaT / 1000.0f;
    //printf("New KalmanFilter Data %d: (%.2f,%.2f,%.2f) -> THREAD=%d\n", d.from, d.data.x(),d.data.y(),d.data.z(),QThread::currentThreadId());
    //filter->updateKalmanFilter();
    //if (kfUScount < 10)
    //{
//        switch (d.from)
//        {
//        case kalmanFilter::accel:
//            //d.data *= G;
//            updateKalmanFilter(d);
//            //kfUScount++;
//            break;
//        case kalmanFilter::mag:
//            //updateKalmanFilter(d);
//        default:
//            break;
//        }
    //}
    QVector3D tcamDirEst = camDirEst,tcamUpDirEst = camUpDirEst;
    QVector3D rotationAxis,orginVec;
    QMatrix4x4 rotMat;
    rotMat.setToIdentity();

    //Low pass filter noisy accel and mag values
    switch (d.from)
    {
    case accel:
        gAccel = 0.96*gAccel+0.04*d.data;
        break;
    case mag:
        gMag = 0.96*gMag+0.04*d.data;
        break;
    case gyro:
        gGyro = d.data;//0.98*gGyro+0.02*d.data;
        //Integreate Gyro with respect to dt
        // find new cam direction using rotations
        //rotationAxis = QVector3D::crossProduct(camDirEst, camUpDirEst);
        //(gGyro.x()*deltaTsec*camUpDirEst)/1000;
        //gyroRotMat = GetRotMatAroundAxis(gGyro.x()*deltaTsec,rotationAxis);
        //gyroRotMat.rotate(-gGyro.y()*deltaTsec,camDirEst);
        //gyroRotMat.rotate(gGyro.z()*deltaTsec,camUpDirEst);
        //tcamDirEst = gyroRotMat*camDirEst;
        //tcamDirEst.normalize();
        //tcamUpDirEst = gyroRotMat*camUpDirEst;
        //tcamUpDirEst.normalize();
        //printf("deltaT: %f\n",deltaTsec);
        //printf("gGyro: (%.3f,%.3f,%.3f)\n",gGyro.x(),gGyro.y(),gGyro.z());
        //printf("camDirEst: (%.3f,%.3f,%.3f)\n",camDirEst.x(),camDirEst.y(),camDirEst.z());
        //printf("tcamDirEst: (%.3f,%.3f,%.3f)\n",tcamDirEst.x(),tcamDirEst.y(),tcamDirEst.z());
        //printf("camUpDirEst: (%.3f,%.3f,%.3f)\n",camUpDirEst.x(),camUpDirEst.y(),camUpDirEst.z());
        //printf("tcamUpDirEst: (%.3f,%.3f,%.3f)\n",tcamUpDirEst.x(),tcamUpDirEst.y(),tcamUpDirEst.z());
        previousCalcTime = curCalcTime;
        break;
    case ultra:
        gDist = 0.6*gDist + 0.4*d.data.x();
        break;
    case cam:
        gCam = QVector3D(d.data.x()*(DEGREEWIDTH/PIXELWIDTH),d.data.y()*(DEGREEHEIGHT/PIXELHEIGHT),0);
        //calculate the cam position
        rotationAxis = QVector3D::crossProduct(camDirEst, camUpDirEst).normalized();
        rotMat.rotate(gCam.y(),rotationAxis);
        rotMat.rotate(-gCam.x(),camUpDirEst.normalized());
        orginVec = rotMat*camDirEst;
        orginVec = orginVec.normalized()*gDist*15;
        camPos = -orginVec;
        break;
    default:
        break;
    }
    //Based on accel and mag data update orientation vector
    QVector3D gVec,nVec,bVec;
    gVec = -gAccel;
    gVec.normalize();
    bVec = QVector3D::crossProduct(gMag.normalized(), -gVec);
    bVec.normalize();
    nVec = QVector3D::crossProduct(-gVec, bVec);
    nVec.normalize();

    QVector3D t2camDirEst,t2camDirUpEst;
    QMatrix4x4 coordTransform;
    coordTransform = QMatrix4x4(nVec.x(),nVec.y(),nVec.z(),0.0f,bVec.x(),bVec.y(),bVec.z(),0.0f,gVec.x(),gVec.y(),gVec.z(),0.0f,0.0f,0.0f,0.0f,1.0f);
    t2camDirUpEst = coordTransform*QVector3D(0.0f,0.0f,1.0f);
    t2camDirEst = coordTransform*QVector3D(0.0f,1.0f,0.0f);
    t2camDirEst.normalize();
    t2camDirUpEst.normalize();

    camDirEst = tcamDirEst*0.98 + t2camDirEst*0.2;
    camUpDirEst = tcamUpDirEst*0.98 + t2camDirUpEst*0.2;
    camDirEst.normalize();
    camUpDirEst.normalize();

    kalMutex->lock();
    //kalmanFilter::PrintSensorData(&data);
    switch (d.from)
    {
    case kalmanFilter::accel:
        curAccelReading = gAccel;
        break;
    case kalmanFilter::mag:
        curMagReading = gMag;
        break;
    case kalmanFilter::gyro:
        curGyroReading = gGyro;
        break;
    case kalmanFilter::cam:
        curCamCoord = gCam;
        break;
    case kalmanFilter::ultra:
        curDist = gDist;
        break;
    }
    NumPendingUpdates--;
    kalMutex->unlock();
}

QMatrix4x4 kalmanFilter::GetRotMatAroundAxis(qreal angle, QVector3D axisOfRot)
{
    QMatrix4x4 gyroRotMat;
    gyroRotMat.setToIdentity();
    qreal sTheta,cTheta,u,v,w;
    axisOfRot.normalize();
    u = axisOfRot.x();
    v = axisOfRot.y();
    w = axisOfRot.z();
    sTheta = sin(angle*ANGLEStoRAD);
    cTheta = cos(angle*ANGLEStoRAD);
    gyroRotMat(0,0) = u*u + (1-u*u)*cTheta;
    gyroRotMat(1,0) = u*v*(1-cTheta)+w*sTheta;
    gyroRotMat(2,0) = u*w*(1-cTheta)-v*sTheta;
    gyroRotMat(0,1) = u*v*(1-cTheta)-w*sTheta;
    gyroRotMat(1,1) = v*v + (1-v*v)*cTheta;
    gyroRotMat(2,1) = v*w*(1-cTheta)+u*sTheta;
    gyroRotMat(0,2) = u*w*(1-cTheta)+v*sTheta;
    gyroRotMat(1,2) = v*w*(1-cTheta)-u*sTheta;
    gyroRotMat(2,2) = w*w + (1-w*w)*cTheta;

    return gyroRotMat;
}

void kalmanFilter::print15x15Matrix(QGenericMatrix<15,15,qreal> *m)
{
    for (int i = 0; i < 15; i++)
    {
        for (int j = 0; j < 15; j++)
            printf(" %4.3f",(*m)(i,j));
        printf("\n");
    }
}

void kalmanFilter::print15x1Matrix(QGenericMatrix<1,15,qreal> *m)
{
    for (int i = 0; i < 15; i++)
    {
        for (int j = 0; j < 1; j++)
            printf(" %4.3f",(*m)(i,j));
        printf("\n");
    }
}

void kalmanFilter::print3x15Matrix(QGenericMatrix<15,3,qreal> *m)
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 15; j++)
            printf(" %4.3f",(*m)(i,j));
        printf("\n");
    }
}

void kalmanFilter::print15x3Matrix(QGenericMatrix<3,15,qreal> *m)
{
    for (int i = 0; i < 15; i++)
    {
        for (int j = 0; j < 3; j++)
            printf(" %4.3f",(*m)(i,j));
        printf("\n");
    }
}

void kalmanFilter::updateKalmanFilter(kalmanFilter::SensorData d, qreal deltaTsec)
{
    QGenericMatrix<15,15,qreal> Pm, Adt;
    //a. update state vector        xhat- = A(dt)xhat(t-dt)
    //   update error covariance    P- = A(dt)P(t-dt)A^T(dt) + Q(dt)
    Adt = modelStateDiscreateTransition(deltaTsec);
    Pm = Adt * P * Adt.transposed();
    AddQdt(&Pm, deltaTsec);
#ifdef DEBUGOUTPUT
    printf("Kalman Update\n");
    printf("deltaT: %f\n",deltaTsec);
    printf("Sv\n");
    print15x1Matrix(&stateVector);
    printf("\nAngles");
    printf("\n(%.3f,%.3f,%.3f)\n",globalCoordFrame.x(),globalCoordFrame.y(),globalCoordFrame.z());
    printf("\nP\n");
    print15x15Matrix(&P);
    printf("\nAdt\n");
    print15x15Matrix(&Adt);
    printf("\nPm\n");
    print15x15Matrix(&Pm);
    printf("\n");
#endif
    //b. find estimated measurement zhat = h_omega
    //   find Jacobian matrix       H = h_omega
    QVector3D zhat = PredictMeasurment(d.from);
#ifdef DEBUGOUTPUT
    printf("\nzhat\n");
    printf("\n(%.3f,%.3f,%.3f)\n",zhat.x(),zhat.y(),zhat.z());
#endif
    //c. Find the Kalman gain       K = P-*H^T*(H*P-*H^T + R_omega(t))^(-1)
    //d. Compute the residual       dz = z_omega(t) - zhat
    //e. Correct the xhat and P     xhat(t) = xhat- + K*dz
    //                              P(t) = (I - K*H)*P-
    QGenericMatrix<15,15,qreal> I;
    I.setToIdentity();
    QGenericMatrix<15,3,qreal> H3;
    QGenericMatrix<15,2,qreal> H2;
    QGenericMatrix<15,1,qreal> H1;
    QGenericMatrix<3,15,qreal> K3;
    QGenericMatrix<2,15,qreal> K2;
    QGenericMatrix<1,15,qreal> K1;
    QMatrix4x4 tM3;
    QVector3D dzv = d.data - zhat;
    QGenericMatrix<1,3,qreal> dz3;
    QGenericMatrix<1,2,qreal> dz2;
    QGenericMatrix<1,1,qreal> dz1;
    switch (d.from)
    {
    case accel:
    case mag:
    case gyro:
        switch(d.from)
        {
        case accel:
            H3 = GetAccelJacobian();
            break;
        case mag:
            H3 = GetMagJacobian();
            break;
        case gyro:
            H3 = GetGyroJacobian();
            break;
        default:
            break;
        }
        dz3(0,0) = dzv.x();
        dz3(1,0) = dzv.y();
        dz3(2,0) = dzv.z();
#ifdef DEBUGOUTPUT
        printf("\ndzAD\n");
        printf("\n(%.3f,%.3f,%.3f)\n",d.data.x(),d.data.y(),d.data.z());
        printf("\ndz\n");
        printf("\n(%.3f,%.3f,%.3f)\n",dzv.x(),dzv.y(),dzv.z());
        printf("H\n");
        print3x15Matrix(&H3);
#endif
        tM3 = QMatrix4x4((H3*Pm*H3.transposed()+GetAccelGyroMagR(d.from)));
        K3 = Pm*H3.transposed()*tM3.normalMatrix().transposed();
        stateVector = stateVector + K3*dz3;
        P = (I-(K3*H3))*Pm;
#ifdef DEBUGOUTPUT
        printf("\nK\n");
        print15x3Matrix(&K3);
        printf("\nNsV");
        print15x1Matrix(&stateVector);
        printf("\nAngles");
        printf("\n(%.3f,%.3f,%.3f)\n",globalCoordFrame.x(),globalCoordFrame.y(),globalCoordFrame.z());
#endif
        break;
    case cam:
        H2 = GetCamJacobian();
        K2 = Pm*H2.transposed()*(H2*Pm*H2.transposed()+GetCamR());
        stateVector = stateVector + K2*dz2;
        P = (I-(K3*H3))*Pm;
        break;
    case ultra:
        H1 = GetUltraJacobian();
        K1 = Pm*H1.transposed()*(H1*Pm*H1.transposed()+GetDistR());
        stateVector = stateVector + K1*dz1;
        P = (I-(K3*H3))*Pm;
        break;
    default:
        break;
    }

    //f. update external angles     dalpha_hat = Quaternion(xhat[dphi],xhat[dtheta],xhat[dpsi])
    //globalCoordFrame *= QQuaternion(1.0, stateVector(9,0),stateVector(10,0),stateVector(11,0));
    globalCoordFrame += QVector3D(stateVector(9,0),stateVector(10,0),stateVector(11,0));
    globalCoordFrame.setX(wrapClamp(globalCoordFrame.x(),0,360));
    globalCoordFrame.setY(wrapClamp(globalCoordFrame.y(),0,360));
    globalCoordFrame.setZ(wrapClamp(globalCoordFrame.x(),0,360));
    globalCoordFrame = d.data * 180;
    // zero orientation elements of the state vector
    stateVector(9,0) = 0;
    stateVector(10,0) = 0;
    stateVector(11,0) = 0;
}

qreal kalmanFilter::wrapClamp(qreal v, int min, int max)
{
    int iv = (int)v;
    if(v < min)
    {
        v = (iv % (max-min));// + (1 + (v - iv));
    }
    else if (v > max)
    {
        v = (iv % (max-min));// - (v - iv);
    }
    return v;
}

QGenericMatrix<15,3,qreal> kalmanFilter::GetAccelJacobian()
{
    qreal s1 = sinf(globalCoordFrame.x()*ANGLEStoRAD),s2 = sinf(globalCoordFrame.y()*ANGLEStoRAD),s3 = sinf(globalCoordFrame.z())*ANGLEStoRAD;
    qreal c1 = cosf(globalCoordFrame.x()*ANGLEStoRAD),c2 = cosf(globalCoordFrame.y()*ANGLEStoRAD),c3 = cosf(globalCoordFrame.z()*ANGLEStoRAD);
    qreal ax = stateVector(6,0),ay = stateVector(7,0),az = stateVector(8,0) - G;
    QGenericMatrix<15,3,qreal> H = QGenericMatrix<15,3,qreal>();
    H.fill(0);
    //H(0,6) = c2*c3;
    //H(0,7) = -c2*s3;
    //H(0,8) = s2;
    //H(1,6) = c1*s3 + c3*s1*s2;
    //H(1,7) = c1*c3 - s1*s2*s3;
    //H(1,8) = -c2*s1;
    //H(2,6) = s1*s3 - c1*c3*s2;
    //H(2,7) = c3*s1 + c1*s2*s3;
    //H(2,8) = c1*c2;
    H(0,9) = (-s1*s3+c1*s2*c3)*ay+(c1*s3+s1*s2*c3)*az;
    H(0,10) = (-s2*c3)*ax+(s1*c2*c3)*ay+(-c1*c2*c3)*az;
    H(0,11) = (-c2*s3)*ax+(c1*c3-s1*s2*s3)*ay+(s1*c3+c1*s2*s3)*az;
    H(1,9) = (-s1*c3-c1*s2*s3)*ay+(c1*c3-(s1*s2*s3))*az;
    H(1,10) = (s2*s3)*ax-(s1*c2*s3)*ay+(c1*c2*s3)*az;
    H(1,11) = (-c2*c3)*ax+(-c1*s3-s1*s2*c3)*ay+(-s1*s3+c1*s2*c3)*az;
    H(2,9) = (-c1*c2)*ay+(-s1*c2)*az;
    H(2,10) = c2*ax+(s1*s2)*ay+(-c1*s2)*az;
    return H;
}

QGenericMatrix<15,3,qreal> kalmanFilter::GetGyroJacobian()
{
    qreal s1 = sinf(globalCoordFrame.x()*ANGLEStoRAD),s2 = sinf(globalCoordFrame.y()*ANGLEStoRAD),s3 = sinf(globalCoordFrame.z()*ANGLEStoRAD);
    qreal c1 = cosf(globalCoordFrame.x()*ANGLEStoRAD),c2 = cosf(globalCoordFrame.y()*ANGLEStoRAD),c3 = cosf(globalCoordFrame.z()*ANGLEStoRAD);
    QGenericMatrix<15,3,qreal> H;
    H.fill(0);
    H(0,0) = c2*c3;
    H(0,1) = -c2*s3;
    H(0,2) = s2;
    H(1,0) = c1*s3 + c3*s1*s2;
    H(1,1) = c1*c3 - s1*s2*s3;
    H(1,2) = -c2*s1;
    H(2,0) = s1*s3 - c1*c3*s2;
    H(2,1) = c3*s1 + c1*s2*s3;
    H(2,2) = c1*c2;
    return H;
}

QGenericMatrix<15,3,qreal> kalmanFilter::GetMagJacobian()
{
    qreal s2 = sinf(globalCoordFrame.y()*ANGLEStoRAD),s3 = sinf(globalCoordFrame.z()*ANGLEStoRAD);
    qreal c2 = cosf(globalCoordFrame.y()*ANGLEStoRAD),c3 = cosf(globalCoordFrame.z()*ANGLEStoRAD);
    QGenericMatrix<15,3,qreal> H = QGenericMatrix<15,3,qreal>();
    H.fill(0);
    H(0,10) = -s2*c3*MG;
    H(1,10) = s2*s3*MG;
    H(2,10) = c2*MG;
    H(0,11) = -c2*s3*MG;
    H(1,11) = -c2*c3*MG;
    return H;
}

QGenericMatrix<15,1,qreal> kalmanFilter::GetUltraJacobian()
{
    QGenericMatrix<15,1,qreal> H;
    H.fill(0);
    H(0,0) = 2*stateVector(0,0);
    H(0,1) = 2*stateVector(1,0);
    H(0,2) = 2*stateVector(2,0);
    return H;
}

QGenericMatrix<15,2,qreal> kalmanFilter::GetCamJacobian()
{
    qreal s1 = sinf(globalCoordFrame.x()*ANGLEStoRAD),s2 = sinf(globalCoordFrame.y()*ANGLEStoRAD),s3 = sinf(globalCoordFrame.z()*ANGLEStoRAD);
    qreal c1 = cosf(globalCoordFrame.x()*ANGLEStoRAD),c2 = cosf(globalCoordFrame.y()*ANGLEStoRAD),c3 = cosf(globalCoordFrame.z()*ANGLEStoRAD);
    qreal xp = (c2*c3)*stateVector(0,0) + (c1*s3+s1*s2*c3)*stateVector(1,0) + (s1*s3-c1*s2*c3)*stateVector(2,0);
    qreal yp = (-c2*s3)*stateVector(0,0) + (c1*c3-s1*s2*s3)*stateVector(1,0) + (s1*c3+c1*s2*s3)*stateVector(2,0);
    qreal zp = (s2)*stateVector(0,0) + (-s1*c2)*stateVector(1,0) + (c1*c2)*stateVector(2,0);
    QGenericMatrix<15,2,qreal> H;
    H.fill(0);
    H(0,0) = (((c2*c3)/zp)+((xp)/-(s2*s2)))*NF*(PIXELWIDTH/IPWIDTH);
    H(1,0) = (((-c2*s3)/zp)+((yp)/-(s2*s2)))*NF*(PIXELHEIGHT/IPHEIGHT);
    H(0,1) = (((c1*s3+s1*s2*c3)/zp)+((xp)/-((-s1*c2)*(-s1*c2))))*NF*(PIXELWIDTH/IPWIDTH);
    H(1,1) = (((c1*c3-s1*s2*s3)/zp)+((yp)/-((-s1*c2)*(-s1*c2))))*NF*(PIXELHEIGHT/IPHEIGHT);
    H(0,2) = (((s1*s3-c1*s2*c3)/zp)+((xp)/-((c1*c2)*(c1*c2))))*NF*(PIXELWIDTH/IPWIDTH);
    H(1,1) = (((s1*c3+c1*s2*s3)/zp)+((yp)/-((c1*c2)*(c1*c2))))*NF*(PIXELHEIGHT/IPHEIGHT);
    return H;
}

QVector3D kalmanFilter::PredictMeasurment(SensorInputFrom device)
{
    QMatrix4x4 rotMat = GetRotationMatrix(globalCoordFrame.x(),globalCoordFrame.y(),globalCoordFrame.z());
    QVector3D stateAngularVelocity = QVector3D(stateVector(12,0),stateVector(13,0),stateVector(14,0));
    QVector3D stateAccel = QVector3D(stateVector(6,0),stateVector(7,0),stateVector(8,0));
    QVector3D camPerspectivePos;
    QVector3D globalPos = QVector3D(stateVector(0,0),stateVector(1,0),stateVector(2,0));
    switch (device)
    {
    case accel:
        return rotMat * (stateAccel - QVector3D(0,0,G));
    case gyro:
        return rotMat * stateAngularVelocity;
    case mag:
        return rotMat * QVector3D(MG,0,0);
    case cam:
        camPerspectivePos = rotMat * globalPos;
        return QVector3D((camPerspectivePos.x()*NF*(PIXELWIDTH/IPWIDTH))/camPerspectivePos.z(),
                         (camPerspectivePos.y()*NF*(PIXELHEIGHT/IPHEIGHT))/camPerspectivePos.z(),
                         0);
    case ultra:
        return QVector3D(globalPos.lengthSquared(),0,0);
    }
    return QVector3D(0,0,0);
}

// Updates the current state to the next state given the delta t in
//   seconds
QGenericMatrix<15,15,qreal> kalmanFilter::modelStateDiscreateTransition(qreal deltaTsec)
{
    qreal deltaTsecSqr = deltaTsec * deltaTsec;
    QVector3D statePos = QVector3D(stateVector(0,0),stateVector(1,0),stateVector(2,0));
    QVector3D stateVel = QVector3D(stateVector(3,0),stateVector(4,0),stateVector(5,0));
    QVector3D stateAccel = QVector3D(stateVector(6,0),stateVector(7,0),stateVector(8,0));
    QVector3D stateAngle = QVector3D(stateVector(9,0),stateVector(10,0),stateVector(11,0));
    QVector3D stateAngularAccel = QVector3D(stateVector(12,0),stateVector(13,0),stateVector(14,0));

    QGenericMatrix<15,15,qreal> A = QGenericMatrix<15,15,qreal>();
    A.setToIdentity();
    qreal tx, ty, tz;
    tx = statePos.x() + deltaTsec * stateVel.x() + deltaTsecSqr * stateAccel.x() / 2;
    ty = statePos.y() + deltaTsec * stateVel.y() + deltaTsecSqr * stateAccel.y() / 2;
    tz = statePos.z() + deltaTsec * stateVel.z() + deltaTsecSqr * stateAccel.z() / 2;
    //Update Position
    stateVector(0,0) = tx;
    stateVector(1,0) = ty;
    stateVector(2,0) = tz;
    A(0,3) = deltaTsec;
    A(0,6) = deltaTsecSqr / 2;
    A(1,4) = deltaTsec;
    A(1,7) = deltaTsecSqr / 2;
    A(2,5) = deltaTsec;
    A(2,8) = deltaTsecSqr / 2;
    tx = stateVel.x() + deltaTsec * stateAccel.x();
    ty = stateVel.y() + deltaTsec * stateAccel.y();
    tz = stateVel.z() + deltaTsec * stateAccel.z();
    //Update Velocity
    stateVector(3,0) = tx;
    stateVector(4,0) = ty;
    stateVector(5,0) = tz;
    A(3,6) = deltaTsec;
    A(4,7) = deltaTsec;
    A(5,8) = deltaTsec;
    // stateAccel does not change
    tx = stateAngle.x() + deltaTsec * stateAngularAccel.x();
    ty = stateAngle.y() + deltaTsec * stateAngularAccel.y();
    tz = stateAngle.z() + deltaTsec * stateAngularAccel.z();
    //Update Orientation
    stateVector(9,0) = tx;
    stateVector(10,0) = ty;
    stateVector(11,0) = tz;
    A(9,12) = deltaTsec;
    A(10,13) = deltaTsec;
    A(11,14) = deltaTsec;
    // stateAngularAccel does not change
    return A;
}

QGenericMatrix<15,15,qreal> kalmanFilter::GetPFromZeta(qreal *v)
{
    QGenericMatrix<15,15,qreal> nP;
    nP.setToIdentity();
    nP(0,0) = v[0]*v[0];//x
    nP(0,3) = v[0]*v[3];
    nP(0,6) = v[0]*v[6];
    nP(1,1) = v[1]*v[1];//y
    nP(1,4) = v[1]*v[4];
    nP(1,7) = v[1]*v[7];
    nP(2,2) = v[2]*v[2];//z
    nP(2,5) = v[2]*v[5];
    nP(2,8) = v[2]*v[8];
    nP(3,0) = v[3]*v[0];//vx
    nP(3,3) = v[3]*v[3];
    nP(3,6) = v[3]*v[6];
    nP(4,1) = v[4]*v[1];//vy
    nP(4,4) = v[4]*v[4];
    nP(4,7) = v[4]*v[7];
    nP(5,2) = v[5]*v[2];//vz
    nP(5,5) = v[5]*v[5];
    nP(5,8) = v[5]*v[8];
    nP(6,0) = v[6]*v[0];//ax
    nP(6,3) = v[6]*v[3];
    nP(6,6) = v[6]*v[6];
    nP(7,1) = v[7]*v[1];//ay
    nP(7,4) = v[7]*v[4];
    nP(7,7) = v[7]*v[7];
    nP(8,2) = v[8]*v[2];//az
    nP(8,5) = v[8]*v[5];
    nP(8,8) = v[8]*v[8];
    nP(9,9) = v[9]*v[9];//phi
    nP(9,12) = v[9]*v[12];
    nP(10,10) = v[10]*v[10];//theta
    nP(10,13) = v[10]*v[13];
    nP(11,11) = v[11]*v[11];//psi
    nP(11,14) = v[11]*v[14];
    nP(12,9) = v[12]*v[9];//vphi
    nP(12,12) = v[12]*v[12];
    nP(13,10) = v[13]*v[10];//vtheta
    nP(13,13) = v[13]*v[13];
    nP(14,11) = v[14]*v[11];//vpsi
    nP(14,14) = v[14]*v[14];
    return nP;
}

QGenericMatrix<2,2,qreal> kalmanFilter::GetCamR()
{
    QGenericMatrix<2,2,qreal> r;
    r.setToIdentity();
    r(0,0) = varCam.x();
    r(0,0) = varCam.y();
    return r;
}

QGenericMatrix<1,1,qreal> kalmanFilter::GetDistR()
{
    QGenericMatrix<1,1,qreal> r;
    r.setToIdentity();
    r(0,0) = varDist;
    return r;
}

QGenericMatrix<3,3,qreal> kalmanFilter::GetAccelGyroMagR(SensorInputFrom f)
{
    QGenericMatrix<3,3,qreal> r;
    r.setToIdentity();
    switch (f)
    {
    case accel:
        r(0,0) = varAccel.x();
        r(1,1) = varAccel.y();
        r(2,2) = varAccel.z();
        break;
    case gyro:
        r(0,0) = varGyro.x();
        r(1,1) = varGyro.y();
        r(2,2) = varGyro.z();
        break;
    case mag:
        r(0,0) = varMag.x();
        r(1,1) = varMag.y();
        r(2,2) = varMag.z();
        break;
    default:
        break;
    }
    return r;
}

void kalmanFilter::AddQdt(QGenericMatrix<15,15,qreal> *Pm, float deltaTsec)
{
    qreal deltaTsec2,deltaTsec3, deltaTsec4;
    deltaTsec2 = deltaTsec * deltaTsec;
    deltaTsec3 = deltaTsec2 * deltaTsec;
    deltaTsec4 = deltaTsec3 * deltaTsec;
    // x variance
    (*Pm)(0,0) += deltaTsec4 * muLocation.x() / 4;
    (*Pm)(0,3) += deltaTsec3 * muLocation.x() / 2;
    (*Pm)(3,0) += deltaTsec3 * muLocation.x() / 2;
    (*Pm)(0,6) += deltaTsec2 * muLocation.x() / 2;
    (*Pm)(6,0) += deltaTsec2 * muLocation.x() / 2;
    (*Pm)(3,3) += deltaTsec2 * muLocation.x();
    (*Pm)(3,6) += deltaTsec * muLocation.x();
    (*Pm)(6,3) += deltaTsec * muLocation.x();
    (*Pm)(6,6) += muLocation.x();
    //y variance
    (*Pm)(1,1) += deltaTsec4 * muLocation.y() / 4;
    (*Pm)(1,4) += deltaTsec3 * muLocation.y() / 2;
    (*Pm)(4,1) += deltaTsec3 * muLocation.y() / 2;
    (*Pm)(1,7) += deltaTsec2 * muLocation.y() / 2;
    (*Pm)(7,1) += deltaTsec2 * muLocation.y() / 2;
    (*Pm)(4,4) += deltaTsec2 * muLocation.y();
    (*Pm)(4,7) += deltaTsec * muLocation.y();
    (*Pm)(7,4) += deltaTsec * muLocation.y();
    (*Pm)(7,7) += muLocation.y();
    //z variance
    (*Pm)(2,2) += deltaTsec4 * muLocation.z() / 4;
    (*Pm)(2,5) += deltaTsec3 * muLocation.z() / 2;
    (*Pm)(5,2) += deltaTsec3 * muLocation.z() / 2;
    (*Pm)(2,8) += deltaTsec2 * muLocation.z() / 2;
    (*Pm)(8,2) += deltaTsec2 * muLocation.z() / 2;
    (*Pm)(5,5) += deltaTsec2 * muLocation.z();
    (*Pm)(5,8) += deltaTsec * muLocation.z();
    (*Pm)(8,5) += deltaTsec * muLocation.z();
    (*Pm)(8,8) += muLocation.z();
    //phi variance
    (*Pm)(9,9) += deltaTsec4 * muOrientation.x() / 4;
    (*Pm)(9,12) += deltaTsec3 * muOrientation.x() / 2;
    (*Pm)(12,9) += deltaTsec3 * muOrientation.x() / 2;
    (*Pm)(12,12) += deltaTsec * muOrientation.x();
    //theta variance
    (*Pm)(10,10) += deltaTsec4 * muOrientation.y() / 4;
    (*Pm)(10,13) += deltaTsec3 * muOrientation.y() / 2;
    (*Pm)(13,10) += deltaTsec3 * muOrientation.y() / 2;
    (*Pm)(13,13) += deltaTsec * muOrientation.y();
    //psi variance
    (*Pm)(11,11) += deltaTsec4 * muOrientation.z() / 4;
    (*Pm)(11,14) += deltaTsec3 * muOrientation.z() / 2;
    (*Pm)(14,11) += deltaTsec3 * muOrientation.z() / 2;
    (*Pm)(14,14) += deltaTsec * muOrientation.z();
}

QMatrix4x4 kalmanFilter::GetRotationMatrix(qreal x,qreal y,qreal z)
{
    QMatrix4x4 rot;
    rot.setToIdentity();
    qreal c1 = cosf(x*ANGLEStoRAD),c2 = cosf(y*ANGLEStoRAD),c3 = cosf(z*ANGLEStoRAD);
    qreal s1 = sinf(x*ANGLEStoRAD),s2 = sinf(y*ANGLEStoRAD),s3 = sinf(z*ANGLEStoRAD);
    rot(0,0) = c2*c3;
    rot(0,1) = -c2*s3;
    rot(0,2) = s2;
    rot(1,0) = c1*s3 + c3*s1*s2;
    rot(1,1) = c1*c3 - s1*s2*s3;
    rot(1,2) = -c2*s1;
    rot(2,0) = s1*s3 - c1*c3*s2;
    rot(2,1) = c3*s1 + c1*s2*s3;
    rot(2,2) = c1*c2;
    return rot;
}

void kalmanFilter::PrintSensorData(SensorData *d)
{
    switch (d->from)
    {
    case accel:
        printf("accel: (%.3f,%.3f,%.3f)\n",d->data.x(),d->data.y(),d->data.z());
        break;
    case mag:
        printf("mag: (%.3f,%.3f,%.3f)\n",d->data.x(),d->data.y(),d->data.z());
    case gyro:
        printf("gyro: (%.3f,%.3f,%.3f)\n",d->data.x(),d->data.y(),d->data.z());
        break;
    case ultra:
        printf("dist: %.3f)\n",d->data.x());
        break;
    case cam:
        printf("cam: (%.3f,%.3f)\n",d->data.x(),d->data.y());
        break;
    }
}
