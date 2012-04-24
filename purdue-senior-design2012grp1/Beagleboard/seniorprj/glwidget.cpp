/****************************************************************************
**
** Written by Justin Huffaker
**  adapted form Qt Opengl_ES2 example
**
****************************************************************************/

#include "glwidget.h"
#include <QPainter>
#include <QPaintEngine>
#include <math.h>
#include <unistd.h>
#include <string>
#include <sstream>

#include "bubble.h"
#include "kalmanfilter.h"


const int bubbleNum = 8;

GLWidget::GLWidget(QWidget *parent)
    : QGLWidget(parent)
{
    frames = 0;
    setAttribute(Qt::WA_PaintOnScreen);
    setAttribute(Qt::WA_NoSystemBackground);
    setAutoBufferSwap(false);
    m_showBubbles = false;
    dispTexture = 1;
    failedToConfigureShader = false;
    eyePos = QVector3D(0, -4, 0);
    eyeDir = QVector3D(0, 1, 0);
    eyeUpDir = QVector3D(0, 0, 1);
    eyeVel = QVector3D(0,0,0);
    eyeAccel = QVector3D(0,0,0);
    eyeAngleVel = QVector3D(0,0,0);
    camSpeed = 0.5;
    //camAngleSpeed = 5.0;
    eyeAngle = QVector3D(0,0,0);

    //Initilize threads, Kalman filter object, and Usb thread object
    kalfilter = new kalmanFilter();
    usbConnection = new USBThread(&(kalfilter->NumPendingUpdates),kalfilter->kalMutex);
    kalThread = new GenericExecThread(QThread::currentThread());
    usbThread = new GenericExecThread(QThread::currentThread());
    connect(usbConnection,SIGNAL(newKalmanFilterData(kalmanFilter::SensorData)),kalfilter,SLOT(update(kalmanFilter::SensorData)),Qt::QueuedConnection);
    kalfilter->moveToThread(kalThread);
    usbConnection->moveToThread(usbThread);
    usbThread->start();
    kalThread->start();
    //usbConnection->StartUSBTimer();
    //usbConnection->StartUSBTimer();
#ifndef Q_WS_QWS
    setMinimumSize(300, 250);
#endif
    printf("MAIN thread ID: %d\n",QThread::currentThreadId());
}

GLWidget::~GLWidget()
{
    delete usbConnection;
    delete kalfilter;
    delete kalThread;
    delete usbThread;
}

void GLWidget::setScaling(int scale) {


    if (scale > 50)
        m_fScale = 1 + qreal(scale -50) / 50 * 0.5;
    else if (scale < 50)
        m_fScale =  1- (qreal(50 - scale) / 50 * 1/2);
    else 
      m_fScale = 1;
}

void GLWidget::setToCube() {

    dispTexture = 2;
}

void GLWidget::setToMonkey() {
    dispTexture = 1;
}

void GLWidget::showBubbles(bool bubbles)
{
   m_showBubbles = bubbles;
}

void GLWidget::initializeGL ()
{
    glClearColor(0.0f, 0.75f, 1.0f, 1.0f);

    bool successCompiling;
    QGLShader *vshader2 = new QGLShader(QGLShader::Vertex);
    const char *vsrc2 =
        "attribute highp vec4 vertex;\n"
        "attribute highp vec4 texCoord;\n"
        "attribute highp vec3 normal;\n"
        "uniform highp vec3 lightDir;\n"
        "uniform mediump mat4 matrix;\n"
        "uniform mediump mat3 normalMatrix;\n"
        "uniform mediump mat4 proj;\n"
        "varying highp vec4 texc;\n"
        "varying highp vec4 frag_pos;\n"
        "varying mediump float angle;\n"
        "void main(void)\n"
        "{\n"
        "    vec3 toLight = normalize(lightDir);\n"
        "    vec3 wsNormal = normalize(normalMatrix * normal);\n"
        "    angle = max(dot(wsNormal, toLight), 0.0);\n"
        "    frag_pos = matrix * vertex;\n"
        "    gl_Position = proj * frag_pos;\n"
        "    texc = texCoord;\n"
        "}\n";
    successCompiling = vshader2->compileSourceCode(vsrc2);
    if (!successCompiling) {
            printf("Vertex Shader Failed to Compile.\n");
            failedToConfigureShader = true;
    }

    QGLShader *fshader2 = new QGLShader(QGLShader::Fragment);
    const char *fsrc2 =
        "uniform sampler2D tex;\n"
        "uniform mediump vec4 ambientLight;\n"
        "varying highp vec4 texc;\n"
        "varying highp vec4 frag_pos;\n"
        "varying mediump float angle;\n"
        "void main(void)\n"
        "{\n"
        "    highp vec3 color = texture2D(tex, texc.st).rgb;\n"
        "    color = ambientLight.rgb * 0.6 + color * 0.8 * angle;\n"
        "    gl_FragColor = vec4(clamp(color, 0.0, 1.0), 1.0);\n"
        "}\n";
    successCompiling = fshader2->compileSourceCode(fsrc2);
    if (!successCompiling) {
            printf("Fragment Shader Failed to Compile.\n");
            failedToConfigureShader = true;
    }

    if (!failedToConfigureShader) {
        if (!program2.addShader(vshader2)) {
            printf("Failed to add vertex shader.\n");
            failedToConfigureShader = true;
        }
        if (!program2.addShader(fshader2)) {
            printf("Failed to add fragment shader.\n");
            failedToConfigureShader = true;
        }
        if (!program2.link()) {
            printf("Failed to link shaders.\n");
            failedToConfigureShader = true;
        }
    }

    if (!failedToConfigureShader) {
        loadGemoetry();
    }

    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

    m_fAngle = 0;
    m_fScale = 1;
    createBubbles(bubbleNum - bubbles.count());

//    QMatrix4x4 rot;
//    rot.rotate(camAngle.x(), 1.0f, 0.0f, 0.0f);
//    rot.rotate(camAngle.y(), 0.0f, 1.0f, 0.0f);
//    rot.rotate(camAngle.z(), 0.0f, 0.0f, 1.0f);
//    printf("\nMatrix using rotate method\n");
//    printf("[%.2f,%.2f,%.2f,%.2f]\n",rot(0,0),rot(0,1),rot(0,2),rot(0,3));
//    printf("[%.2f,%.2f,%.2f,%.2f]\n",rot(1,0),rot(1,1),rot(1,2),rot(1,3));
//    printf("[%.2f,%.2f,%.2f,%.2f]\n",rot(2,0),rot(2,1),rot(2,2),rot(2,3));
//    printf("[%.2f,%.2f,%.2f,%.2f]\n",rot(3,0),rot(3,1),rot(3,2),rot(3,3));
//    QMatrix4x4 rot2 = kalmanFilter::GetRotationMatrix(camAngle.x(),camAngle.y(),camAngle.z());
//    printf("\nMatrix using my method\n");
//    printf("[%.2f,%.2f,%.2f,%.2f]\n",rot2(0,0),rot2(0,1),rot2(0,2),rot(0,3));
//    printf("[%.2f,%.2f,%.2f,%.2f]\n",rot2(1,0),rot2(1,1),rot2(1,2),rot(1,3));
//    printf("[%.2f,%.2f,%.2f,%.2f]\n",rot2(2,0),rot2(2,1),rot2(2,2),rot(2,3));
//    printf("[%.2f,%.2f,%.2f,%.2f]\n",rot2(3,0),rot2(3,1),rot2(3,2),rot(3,3));
}

void GLWidget::paintGL()
{
    int numKalUpdatesPending = 0;
    kalfilter->kalMutex->lock();
    numKalUpdatesPending = kalfilter->NumPendingUpdates;
    kalfilter->kalMutex->unlock();

    createBubbles(bubbleNum - bubbles.count());

    QPainter painter;
    painter.begin(this);

    QVector3D camUpDir, camDir;
    QVector3D lAccel,lGyro,lMag,lCam;
    qreal lDist;
    // Get data from KalmanFilter
    kalfilter->kalMutex->lock();
    //eye estimates
    camDir = kalfilter->camDirEst;
    camUpDir = kalfilter->camUpDirEst;
    //Raw sensor data
    lAccel = kalfilter->curAccelReading;
    lGyro = kalfilter->curGyroReading;
    lMag = kalfilter->curMagReading;
    lCam = kalfilter->curCamCoord;
    lDist = kalfilter->curDist;
    //Estimated Position
    const qreal *stateData = kalfilter->stateVector.constData();
    eyePos = kalfilter->camPos;//QVector3D(stateData[0],stateData[1],stateData[2]);
    eyeVel = QVector3D(stateData[3],stateData[4],stateData[5]);
    eyeAccel = QVector3D(stateData[6],stateData[7],stateData[8]);
    eyeAngle = kalfilter->globalCoordFrame;//.vector();
    eyeAngleVel = QVector3D(stateData[12],stateData[13],stateData[14]);
    kalfilter->kalMutex->unlock();

    if (numKalUpdatesPending < 2)
    {
        painter.beginNativePainting();

        glClearColor(0.0f, 0.75f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

        glFrontFace(GL_CCW);
        glCullFace(GL_BACK);
        glDepthFunc(GL_LEQUAL);
        glEnable(GL_CULL_FACE);
        glEnable(GL_DEPTH_TEST);

        QMatrix4x4 projection;
        //projection.ortho(-0.5, +0.5, -0.5, +0.5, 0.1, 20.0);
        projection.perspective(45.0f,width()/height(),0.1f,20.0f); //perspective projection matix

//        QVector3D gVec,nVec,bVec;
//        gVec = -lAccel;
//        gVec.normalize();
//        bVec = QVector3D::crossProduct(lMag.normalized(), -gVec);
//        bVec.normalize();
//        nVec = QVector3D::crossProduct(-gVec, bVec);
//        nVec.normalize();

//        QMatrix4x4 coordTransform;
//        coordTransform = QMatrix4x4(nVec.x(),nVec.y(),nVec.z(),0.0f,bVec.x(),bVec.y(),bVec.z(),0.0f,gVec.x(),gVec.y(),gVec.z(),0.0f,0.0f,0.0f,0.0f,1.0f);

//        //lAccel //vector torwards ground
//        //lMag //vector torwards north
//        camUpDir = coordTransform*QVector3D(0.0f,0.0f,1.0f);
//        camDir = coordTransform*QVector3D(0.0f,1.0f,0.0f);
//        camDir.normalize();
//        //printf("camUpDir: (%.3f,%.3f,%.3f)\n",camUpDir.x(),camUpDir.y(),camUpDir.z());
//        //printf("camDir: (%.3f,%.3f,%.3f)\n",camDir.x(),camDir.y(),camDir.z());
//        camUpDir.normalize();


        //QMatrix4x4 eyeRotMatrix;
        //eyeRotMatrix.setToIdentity();
        //eyeRotMatrix = kalmanFilter::GetRotationMatrix(0,0,0);//eyeAngle.x(),eyeAngle.y(),eyeAngle.z());
        //eyeDir = eyeRotMatrix * QVector3D(0.0f,1.0f,0.0f);
        //eyeUpDir = eyeRotMatrix * QVector3D(0.0f,0.0f,1.0f);
        QMatrix4x4 modelview;
        modelview.lookAt(eyePos,eyePos+camDir,camUpDir);

        if (!failedToConfigureShader) {
            if (dispTexture == 1) {
                m1.draw(modelview,projection);
            }
            else {
                m2.draw(modelview,projection);
            }
        }

        glDisable(GL_DEPTH_TEST);
        glDisable(GL_CULL_FACE);

        painter.endNativePainting();

        if (m_showBubbles)
            foreach (Bubble *bubble, bubbles) {
                bubble->drawBubble(&painter);
        }

        if (!(frames % 100)) {
            time.start();
            frames = 0;
        }
        frames ++;
    }

    QString framesPerSecond;
    framesPerSecond.setNum(frames /(time.elapsed() / 1000.0), 'f', 2);

    painter.setPen(Qt::black);
    //painter.setRenderHint(QPainter::TextAntialiasing);

    painter.drawText(20, 40, framesPerSecond + " fps");

    QString tstr;//, tPacket;
    //usbConnection->packetLock->lock();
    //tPacket = usbConnection->curPacket;
    //usbConnection->packetLock->unlock();
    //painter.drawText(20, 280, tPacket);

    tstr.sprintf("m_Accel: (%0.2f,%0.2f,%0.2f) m/s^2",lAccel.x(),lAccel.y(),lAccel.z());
    painter.drawText(20, 80, tstr);
    tstr.sprintf("m_Gyro: (%0.2f,%0.2f,%0.2f) deg/s",lGyro.x(),lGyro.y(),lGyro.z());
    painter.drawText(20, 120, tstr);
    tstr.sprintf("m_Mag: (%0.2f,%0.2f,%0.2f)",lMag.x(),lMag.y(),lMag.z());
    painter.drawText(20, 160, tstr);
    tstr.sprintf("m_Cam: (%0.2f,%0.2f) deg",lCam.x(),lCam.y());
    painter.drawText(20, 200, tstr);
    tstr.sprintf("m_Dist: (%0.6f) m",lDist);
    painter.drawText(20, 240, tstr);

    //tstr.sprintf("Pos: (%0.2f,%0.2f,%0.2f) m",eyePos.x(),eyePos.y(),eyePos.z());
    //painter.drawText(700, 80, tstr);
    //tstr.sprintf("Vel: (%0.2f,%0.2f,%0.2f) m/s",eyeVel.x(),eyeVel.y(),eyeVel.z());
    //painter.drawText(700, 120, tstr);
    //tstr.sprintf("Accel: (%0.2f,%0.2f,%0.2f) m/s^2",eyeAccel.x(),eyeAccel.y(),eyeAccel.z());
    //painter.drawText(700, 160, tstr);
    //std::ostringstream buffer;
    //buffer << "Angle: (" << eyeAngle.x() << "," << eyeAngle.y() << "," << eyeAngle.z() << ")";
    //const std::string t = buffer.str();
    //tstr = QString(t.c_str());
    //tstr.sprintf("Angle: (%2.2f,%2.2f,%2.2f) deg",eyeAngle.x(),eyeAngle.y(),eyeAngle.z());
    //painter.drawText(700, 200, tstr);
    //tstr.sprintf("AngleVel: (%0.2f,%0.2f,%0.2f) deg/s",eyeAngleVel.x(),eyeAngleVel.y(),eyeAngleVel.z());
    //painter.drawText(700, 240, tstr);

    painter.end();
    
    swapBuffers();

    QMutableListIterator<Bubble*> iter(bubbles);

    while (iter.hasNext()) {
        Bubble *bubble = iter.next();
        bubble->move(rect());
    }
}

void GLWidget::createBubbles(int number)
{
    for (int i = 0; i < number; ++i) {
        QPointF position(width()*(0.1 + (0.8*qrand()/(RAND_MAX+1.0))),
                        height()*(0.1 + (0.8*qrand()/(RAND_MAX+1.0))));
        qreal radius = qMin(width(), height())*(0.0175 + 0.0875*qrand()/(RAND_MAX+1.0));
        QPointF velocity(width()*0.0175*(-0.5 + qrand()/(RAND_MAX+1.0)),
                        height()*0.0175*(-0.5 + qrand()/(RAND_MAX+1.0)));

        bubbles.append(new Bubble(position, radius, velocity));
    }
}

void GLWidget::loadGemoetry()
{
    char buf[1024];
    ssize_t len;
    len = readlink("/proc/self/exe",buf,sizeof(buf)-1);
    if (len == -1)
        fprintf(stderr,"Failed to read executable location");
    QString exeLoc = QString(buf);
    exeLoc.chop(10);

    //m1.load(QString("./models/cube/cube.obj"));
    //m1.setTexture(bindTexture(QImage("./models/cube/default.png")));
    QString path = QString(exeLoc);
    path.append("/models/monkey/monkey1.obj");
    m2.load(path);
    path = QString(exeLoc);
    path.append("/models/monkey/fur.resized.jpg");
    m2.setTexture(bindTexture(QImage(path)));
    m2.setProgram(&program2);
    m2.initShaderProgram();
    //m1.printDebugInfo();
    path = QString(exeLoc);
    path.append("/models/cube/cube.obj");
    m1.load(QString(path));
    path = QString(exeLoc);
    path.append("/models/cube/default.png");
    m1.setTexture(bindTexture(QImage(path)));
    m1.setProgram(&program2);
    m1.initShaderProgram();
}

void GLWidget::keyReleaseEvent(QKeyEvent *e)
{
    QMatrix4x4 rotMat;
    e->accept();
    switch (e->key())
    {
    case Qt::Key_W: //move forward along camDir
        eyePos += eyeDir * camSpeed;
        //eyePos.setY(eyePos.y() + camSpeed);
        break;
    case Qt::Key_S: //move backward along camDir
        eyePos -= eyeDir * camSpeed;
        //eyePos.setY(eyePos.y() - camSpeed);
        break;
    case Qt::Key_A: //move -(camDir x camUpDir)
        eyePos -= QVector3D::crossProduct(eyeDir, eyeUpDir) * camSpeed;
        //eyePos.setX(eyePos.x() - camSpeed);
        break;
    case Qt::Key_D: //move (camDir x camUpDir)
        eyePos += QVector3D::crossProduct(eyeDir, eyeUpDir) * camSpeed;
        //eyePos.setX(eyePos.x() + camSpeed);
        break;
    case Qt::Key_I: //rotate camDir and camUpDir up along camDir and camUpDir plane
        //rotMat.rotate(camAngleSpeed, -QVector3D::crossProduct(eyeDir, eyeUpDir));
        //camAngle.setX(getClampAngle(camAngle.x() + camAngleSpeed));
        break;
    case Qt::Key_K: //rotate camDir and camUpDir down along camDir and camUpDir plane
        //rotMat.rotate(camAngleSpeed, QVector3D::crossProduct(eyeDir, eyeUpDir));
        //camAngle.setX(getClampAngle(camAngle.x() - camAngleSpeed));
        break;
    case Qt::Key_J: //rotate camDir to the left along camDir and (camDir x camUpDir) plane
        //rotMat.rotate(camAngleSpeed, eyeUpDir);
        //camAngle.setY(getClampAngle(camAngle.y() + camAngleSpeed));
        break;
    case Qt::Key_L: //rotate camDir to the left along camDir and (camDir x camUpDir) plane
        //rotMat.rotate(camAngleSpeed, -eyeUpDir);
        //camAngle.setY(getClampAngle(camAngle.y() - camAngleSpeed));
        break;
    case Qt::Key_U: //rotate camUpDir left along -(camDir x camUpDir) and camUpDir plane
        //rotMat.rotate(camAngleSpeed, eyeDir);
        //camAngle.setZ(getClampAngle(camAngle.z() + camAngleSpeed));
        break;
    case Qt::Key_O: //rotate camUpDir right along -(camDir x camUpDir) and camUpDir plane
        //rotMat.rotate(camAngleSpeed, -eyeDir);
        //camAngle.setZ(getClampAngle(camAngle.z() - camAngleSpeed));
        break;
    default:
        e->ignore();
        break;
    }
//    QMatrix4x4 rotMat;
//    rotMat.rotate(camAngle.x(),QVector3D(1,0,0));
//    rotMat.rotate(camAngle.y(),QVector3D(0,1,0));
//    rotMat.rotate(camAngle.z(),QVector3D(0,0,1));
//    eyeDir = rotMat * eyeDir;
//    eyeUpDir = rotMat * eyeUpDir;
}

qreal GLWidget::getClampAngle(qreal angle)
{
    if (angle > 360)
        angle = angle - 360;
    else if (angle < 0)
        angle = angle + 360;
    return angle;
}
