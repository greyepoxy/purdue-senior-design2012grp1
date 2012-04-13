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

#include "bubble.h"


const int bubbleNum = 8;

GLWidget::GLWidget(QWidget *parent)
    : QGLWidget(parent)
{
    frames = 0;
    setAttribute(Qt::WA_PaintOnScreen);
    setAttribute(Qt::WA_NoSystemBackground);
    setAutoBufferSwap(false);
    m_showBubbles = true;
    dispTexture = 1;
    failedToConfigureShader = false;
#ifndef Q_WS_QWS
    setMinimumSize(300, 250);
#endif
}

GLWidget::~GLWidget()
{
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

    dispTexture = 1;
}

void GLWidget::setToTeapot() {
    dispTexture = 2;
}

void GLWidget::showBubbles(bool bubbles)
{
   m_showBubbles = bubbles;
}

void GLWidget::initializeGL ()
{
    glClearColor(0.1f, 0.1f, 0.2f, 1.0f);

    bool successCompiling;
    QGLShader *vshader2 = new QGLShader(QGLShader::Vertex);
    const char *vsrc2 =
        "attribute highp vec4 vertex;\n"
        "attribute highp vec4 texCoord;\n"
        "attribute highp vec3 normal;\n"
        "uniform highp vec3 lightDir;\n"
        "uniform mediump mat4 matrix;\n"
        "uniform mediump mat3 normalMatrix;\n"
        "varying highp vec4 texc;\n"
        "varying mediump float angle;\n"
        "void main(void)\n"
        "{\n"
        "    vec3 toLight = normalize(lightDir);\n"
        "    vec3 wsNormal = normalize(normalMatrix * normal);\n"
        "    angle = max(dot(wsNormal, toLight), 0.0);\n"
        "    gl_Position = matrix * vertex;\n"
        "    texc = texCoord;\n"
        "}\n";
    successCompiling = vshader2->compileSourceCode(vsrc2);
    if (!successCompiling) {
            printf("Vertex Shader Failed to Compile.\n");
            failedToConfigureShader = true;
    }

    QGLShader *fshader2 = new QGLShader(QGLShader::Fragment);
    const char *fsrc2 =
        "varying highp vec4 texc;\n"
        "uniform sampler2D tex;\n"
        "uniform mediump vec4 ambientLight;\n"
        "varying mediump float angle;\n"
        "void main(void)\n"
        "{\n"
        "    highp vec3 color = texture2D(tex, texc.st).rgb;\n"
        "    color = ambientLight.rgb * 0.2 + color * 0.8 * angle;\n"
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
}

void GLWidget::paintGL()
{
    createBubbles(bubbleNum - bubbles.count());

    QPainter painter;
    painter.begin(this);

    painter.beginNativePainting();

    glClearColor(0.1f, 0.1f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

    glFrontFace(GL_CW);
    glCullFace(GL_FRONT);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);

    QMatrix4x4 modelview;
    modelview.rotate(m_fAngle, 0.0f, 1.0f, 0.0f);
    modelview.rotate(m_fAngle, 1.0f, 0.0f, 0.0f);
    modelview.rotate(m_fAngle, 0.0f, 0.0f, 1.0f);
    modelview.scale(m_fScale);
    modelview.translate(0.0f, -0.2f, 0.0f);


    if (!failedToConfigureShader) {
        if (dispTexture == 1) {
            m1.draw(modelview);
        }
    }

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);

    painter.endNativePainting();

    if (m_showBubbles)
        foreach (Bubble *bubble, bubbles) {
            bubble->drawBubble(&painter);
    }

    QString framesPerSecond;
    framesPerSecond.setNum(frames /(time.elapsed() / 1000.0), 'f', 2);

    painter.setPen(Qt::white);

    painter.drawText(20, 40, framesPerSecond + " fps");

    painter.end();
    
    swapBuffers();

    QMutableListIterator<Bubble*> iter(bubbles);

    while (iter.hasNext()) {
        Bubble *bubble = iter.next();
        bubble->move(rect());
    }
    if (!(frames % 100)) {
        time.start();
        frames = 0;
    }
    m_fAngle += 1.0f;
    frames ++;
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
    m1.load(QString("./models/cube/cube.obj"));
    m1.setTexture(bindTexture(QImage("./models/cube/default.png")));
    m1.setProgram(&program2);
    m1.initShaderProgram();
    //m1.printDebugInfo();
}
