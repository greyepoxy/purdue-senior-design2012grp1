#ifndef MODEL_H
#define MODEL_H

#include <QObject>
#include <QtOpenGL>
#include <QPainter>
#include <QPaintEngine>
#include <stdio.h>

#include "glm.h"
#include "triangle.h"

class ModelGroup
{
public:
    QVector<Triangle> triangles;
signals:

public slots:

};

class Model
{
public:
    Model();
    Model(QString filename);

    // functions
    void load(QString filename);
    void draw(QMatrix4x4 modelview);
    void setTexture(GLuint texture);
    bool setVertexShaderFile(QString filename);
    bool setFragmentShaderFile(QString filename);
    void setProgram(QGLShaderProgram *program);
    bool linkShaderProgram();
    void initShaderProgram();
    void printDebugInfo();
private:
    // variables
    GLMmodel *model;
    QVector<QVector3D> vertices;
    QVector<QVector3D> normals;
    QVector<ModelGroup> groups;
    GLuint texture;
    QGLShaderProgram* program;
    int vertexAttr;
    int normalAttr;
    int matrixUniform;
    int texCoordAttr;
    int textureUniform;
    int matrixNormalUniform;
};

#endif // MODEL_H
