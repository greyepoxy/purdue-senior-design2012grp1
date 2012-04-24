#include "model.h"

Model::Model() {
}

Model::Model(QString objectFile) {
    load(objectFile);
}
void Model::load(QString filename) {
    model = glmReadOBJ(filename.toLatin1().data());
    if(model->numtexcoords < 1) {
        qWarning() << "Missing UV map.";
    }
    GLMgroup* group;
    group = model->groups;
    while (group) {
        ModelGroup grp;
        for(uint i = 0; i < group->numtriangles; i++) {
            Triangle triangle;
            QVector<QVector3D> verts;
            for(int j = 0; j < 3; j++) {
                QVector3D vector(model->vertices[3 * model->triangles[group->triangles[i]].vindices[j] + 0],
                                 model->vertices[3 * model->triangles[group->triangles[i]].vindices[j] + 1],
                                 model->vertices[3 * model->triangles[group->triangles[i]].vindices[j] + 2]);
                verts.append(vector);
            }
            QVector<QVector3D> norms;
            for(int j = 0; j < 3; j++) {
                QVector3D vector(model->normals[3 * model->triangles[group->triangles[i]].nindices[j] + 0],
                                 model->normals[3 * model->triangles[group->triangles[i]].nindices[j] + 1],
                                 model->normals[3 * model->triangles[group->triangles[i]].nindices[j] + 2]);
                norms.append(vector);
            }
            if(model->numtexcoords > 0) {
                QVector<QVector3D> texs;
                for(int j = 0; j < 3; j++) {
                    QVector3D vector(model->texcoords[2 * model->triangles[group->triangles[i]].tindices[j] + 0],
                                     model->texcoords[2 * model->triangles[group->triangles[i]].tindices[j] + 1],
                                     model->texcoords[2 * model->triangles[group->triangles[i]].tindices[j] + 2]);
                    texs.append(vector);
                }
                triangle.texcoords = texs;
            }
            triangle.vertices = verts;
            triangle.normals = norms;
            grp.triangles.append(triangle);
        }
        groups.append(grp);
        group = group->next;
    }
    qDebug() << "loading file";
}
bool Model::setFragmentShaderFile(QString filename) {
    if(!program->addShaderFromSourceFile(QGLShader::Fragment, filename)) {
        qDebug() << "Could not load shader file " + filename + ": " << program->log();
        return false;
    } else {
        qDebug() << "Loaded " + filename + " successfully";
        return true;
    }
}
bool Model::setVertexShaderFile(QString filename) {
    if(!program->addShaderFromSourceFile(QGLShader::Vertex, filename)) {
        qDebug() << "Could not load shader file " + filename + ": " << program->log();
        return false;
    } else {
        qDebug() << "Loaded " + filename + " successfully";
        return true;
    }
}
bool Model::linkShaderProgram() {
    if(program->link()) {
        qDebug() << "Program linked";
        return true;
    } else {
        qDebug() << "Failed to link program:" << program->log();
        return false;
    }
}

void Model::initShaderProgram() {
    vertexAttr = program->attributeLocation("vertex");
    normalAttr = program->attributeLocation("normal");
    texCoordAttr = program->attributeLocation("texCoord");
    matrixUniform = program->uniformLocation("matrix");
    matrixNormalUniform = program->uniformLocation("normalMatrix");
    textureUniform = program->uniformLocation("tex");
    projUniform = program->uniformLocation("proj");
}

void Model::draw(QMatrix4x4 modelview, QMatrix4x4 proj) {
    program->bind();
    glBindTexture(GL_TEXTURE_2D, texture);
    foreach(ModelGroup grp, groups) {
        foreach(Triangle triangle, grp.triangles) {
            program->setUniformValue(textureUniform, 0);    // use texture unit 0
            // my uniforms
            program->setUniformValue(program->uniformLocation("ambientLight"), QColor(150,150,150,255));
            program->setUniformValue(program->uniformLocation("lightDir"), QVector3D(0.0, 0.3, 1.0));
            //

            program->setAttributeArray(vertexAttr, triangle.vertices.constData());
            program->setAttributeArray(normalAttr, triangle.normals.constData());
            program->setAttributeArray(texCoordAttr, triangle.texcoords.constData());

            program->enableAttributeArray(normalAttr);
            program->enableAttributeArray(vertexAttr);
            program->enableAttributeArray(texCoordAttr);
            glDrawArrays(GL_TRIANGLES, 0, triangle.vertices.size());
            program->disableAttributeArray(normalAttr);
            program->disableAttributeArray(vertexAttr);
            program->disableAttributeArray(texCoordAttr);
        }
    }
    program->setUniformValue(projUniform, proj);
    program->setUniformValue(matrixUniform, modelview);
    program->setUniformValue(matrixNormalUniform, modelview.normalMatrix());
    program->release();
    program->bind();
}
void Model::setTexture(GLuint texture) {
    this->texture = texture;
}
void Model::setProgram(QGLShaderProgram *program) {
    this->program = program;
}

void Model::printDebugInfo() {
    int count = 0;
    printf("Model: %s************\n",model->pathname);
    foreach(ModelGroup grp, groups) {
        printf("Group Num: %d\n",count);
        foreach(Triangle triangle, grp.triangles) {
            for (int i = 0; i < triangle.vertices.size(); i++) {
                printf("  ver(%.2f,%.2f,%.2f) norm(%.2f,%.2f,%.2f) texCoord(%.2f,%.2f,%.2f)\n",
                        triangle.vertices[i].x(),
                        triangle.vertices[i].y(),
                        triangle.vertices[i].z(),
                        triangle.normals[i].x(),
                        triangle.normals[i].y(),
                        triangle.normals[i].z(),
                        triangle.texcoords[i].x(),
                        triangle.texcoords[i].y(),
                        triangle.texcoords[i].z());
            }
        }
        count++;
    }
    printf("END Model: %s********\n",model->pathname);
}
