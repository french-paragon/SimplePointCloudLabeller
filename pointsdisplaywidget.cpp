/*Simple point cloud labeller is a simple labeller for points clouds.

Copyright (C) 2023 Paragon<french.paragon@gmail.com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "pointsdisplaywidget.h"

#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>

#include <QPainter>

#include <cmath>

PointsDisplayWidget::PointsDisplayWidget(QWidget *parent)
    : QOpenGLWidget(parent),
      _n_points(0),
      _displayColors(true)
{

    _viewViewports.resize(4);
    _paintersViewports.resize(4);
    _viewProjections.resize(4);
    _viewNames.resize(4);

}

void PointsDisplayWidget::setPoints(QVector<QVector3D> const& points, QVector<QColor> const& colors) {

    _data.resize(points.size()*3);
    _color.resize(points.size()*3);

    float minX = points[0].x();
    float maxX = points[0].x();

    float minY = points[0].y();
    float maxY = points[0].y();

    float minZ = points[0].z();
    float maxZ = points[0].z();

    for (int i = 0; i < points.size(); i++) {

        _data[i*3] = points[i].x();
        _data[i*3+1] = points[i].y();
        _data[i*3+2] = points[i].z();

        if (colors.size() > i) {

            _color[i*3] = colors[i].redF();
            _color[i*3+1] = colors[i].greenF();
            _color[i*3+2] = colors[i].blueF();

        } else {

            _color[i*3] = 0;
            _color[i*3+1] = 0;
            _color[i*3+2] = 0;

        }

        if (minX > points[i].x()) {
            minX = points[i].x();
        }

        if (maxX < points[i].x()) {
            maxX = points[i].x();
        }

        if (minY > points[i].y()) {
            minY = points[i].y();
        }

        if (maxY < points[i].y()) {
            maxY = points[i].y();
        }

        if (minZ > points[i].z()) {
            minZ = points[i].z();
        }

        if (maxZ < points[i].z()) {
            maxZ = points[i].z();
        }
    }

    _min_z = minZ;
    _max_z = maxZ;

    _n_points = points.size();

    _needToLoad = true;

    QMatrix4x4 transformBase;

    QMatrix4x4 translation;
    translation.translate(QVector3D((maxX - minX)/2 - maxX,
                                    (maxY - minY)/2 - maxY,
                                    (maxZ - minZ)/2 - maxZ));

    float scaleX = 1./std::max(maxX - minX, 0.01f);
    float scaleY = 1./std::max(maxY - minY, 0.01f);
    float scaleZ = 1./std::max(maxZ - minZ, 0.01f);

    float scale = std::min(scaleX, std::min(scaleY, scaleZ));
    QMatrix4x4 scalemat;
    scalemat.scale(scale);

    _inv_scale = 1./scale;

    transformBase = scalemat*translation;

    QMatrix4x4 proj;
    proj.ortho(-1.0, 1.0, -1.0, 1.0, -10, 100);

    QMatrix4x4 transform0;
    transform0.lookAt(QVector3D(1,0,0), QVector3D(0,0,0), QVector3D(0,0,1));
    _viewProjections[0] = proj*transform0*transformBase;
    _viewNames[0] = "Front";

    QMatrix4x4 transform1;
    transform1.lookAt(QVector3D(0,1,0), QVector3D(0,0,0), QVector3D(0,0,1));
    _viewProjections[1] = proj*transform1*transformBase;
    _viewNames[1] = "Side";

    QMatrix4x4 transform2;
    transform2.lookAt(QVector3D(0,0,1), QVector3D(0,0,0), QVector3D(0,1,0));
    _viewProjections[2] = proj*transform2*transformBase;
    _viewNames[2] = "Top";

    QMatrix4x4 transform3;
    transform3.lookAt(QVector3D(1,1,1), QVector3D(0,0,0), QVector3D(0,0,1));
    _viewProjections[3] = proj*transform3*transformBase;
    _viewNames[3] = "Angle";

    update();
}

PointsDisplayWidget::~PointsDisplayWidget() {

    makeCurrent();

    if (_landMarkPointProgram != nullptr) {
        delete _landMarkPointProgram;
    }

    if (_lm_pos_buffer.isCreated()) {
        _lm_pos_buffer.destroy();
    }

    if (_lm_col_buffer.isCreated()) {
        _lm_col_buffer.destroy();
    }
    _scene_vao.destroy();

    doneCurrent();
}


void PointsDisplayWidget::initializeGL() {

    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    f->glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    f->glEnable(GL_MULTISAMPLE);
    f->glEnable(GL_POINT_SMOOTH);
    f->glEnable(GL_PROGRAM_POINT_SIZE);
    f->glEnable(GL_DEPTH_TEST);

    _scene_vao.create();

    _lm_pos_buffer.create();
    _lm_pos_buffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);

    _lm_col_buffer.create();
    _lm_col_buffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);

    _landMarkPointProgram = new QOpenGLShaderProgram();
    _landMarkPointProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/viewPersp.vert");
    _landMarkPointProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/viewPoints.frag");

    _landMarkPointProgram->link();
}

void PointsDisplayWidget::paintGL() {

    QPainter painter(this);

    painter.beginNativePainting();

    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();

    if (_n_points <= 0) {
        return;
    }

    _landMarkPointProgram->bind();
    _scene_vao.bind();
    _lm_pos_buffer.bind();

    if (_needToLoad) {
        _lm_pos_buffer.allocate(_data.data(), _data.size()*sizeof (GLfloat));
    }

    int vertexLocation = _landMarkPointProgram->attributeLocation("in_location");
    _landMarkPointProgram->enableAttributeArray(vertexLocation);
    _landMarkPointProgram->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3);
    _lm_col_buffer.bind();

    if (_needToLoad) {
        _lm_col_buffer.allocate(_color.data(), _color.size()*sizeof (GLfloat));
        _needToLoad = false;
    }

    int colorLocation = _landMarkPointProgram->attributeLocation("in_color");
    _landMarkPointProgram->enableAttributeArray(colorLocation);
    _landMarkPointProgram->setAttributeBuffer(colorLocation, GL_FLOAT, 0, 3);

    _landMarkPointProgram->setUniformValue("minZ", _min_z);
    _landMarkPointProgram->setUniformValue("maxZ", _max_z);
    _landMarkPointProgram->setUniformValue("displayPointsColor", _displayColors);

    for (int i = 0; i < 4; i++) {

        QRect v = _viewViewports[i];
        f->glViewport(v.left(), v.top(), v.width(), v.height());

        _landMarkPointProgram->setUniformValue("matrixViewProjection", _viewProjections[i]);

        f->glDrawArrays(GL_POINTS, 0, _n_points);
    }

    _scene_vao.release();
    _lm_pos_buffer.release();
    _lm_col_buffer.release();

    _landMarkPointProgram->disableAttributeArray(vertexLocation);
    _landMarkPointProgram->release();

    //reset the viewport
    f->glViewport(_fullViewport.left(),
                  _fullViewport.top(),
                  _fullViewport.width(),
                  _fullViewport.height());

    painter.endNativePainting();

    constexpr int marginDelta = 22;
    constexpr int smlMarginDelta = 5;

    QFont font = painter.font();

    QFontMetrics metric(font);

    for (int i = 0; i < 4; i++) {

        font.setPointSize(8);
        painter.setFont(font);

        QRect v = _paintersViewports[i];
        QRect shrinked = v.marginsRemoved(QMargins(marginDelta, marginDelta, marginDelta, marginDelta));

        QRect box = metric.boundingRect(_viewNames[i]);

        double delta = shrinked.width() - box.width();

        QPoint textStart = shrinked.topLeft();
        textStart.rx() += delta;
        textStart.ry() += box.height();

        painter.drawText(textStart, _viewNames[i]);

        if (_viewNames[i] != "Angle") {

            QPoint origin = shrinked.bottomLeft();
            QPoint yAxis = shrinked.topLeft();
            QPoint xAxis = shrinked.bottomRight();

            QPoint xLimTop = xAxis;
            xLimTop.ry() -= smlMarginDelta;

            QPoint xLimBottom = xAxis;
            xLimBottom.ry() += smlMarginDelta;

            QPoint yLimTop = yAxis;
            yLimTop.rx() -= smlMarginDelta;

            QPoint yLimBottom = yAxis;
            yLimBottom.rx() += smlMarginDelta;

            painter.drawLine(origin, xAxis);
            painter.drawLine(xLimTop, xLimBottom);

            painter.drawLine(origin, yAxis);
            painter.drawLine(yLimTop, yLimBottom);

            if (_viewNames[i] == "Top") {
                QString length = QString("%1 pc length unit").arg(_inv_scale, 0, 'f', 2);

                font.setPointSize(8);
                painter.setFont(font);

                QFontMetrics metric(font);

                box = metric.boundingRect(length);

                QPoint lengthOrigin = origin;
                lengthOrigin.ry() += 1 + box.height();
                lengthOrigin.rx() += shrinked.width()/2 - box.width()/2;

                painter.drawText(lengthOrigin, length);
            }

        }

    }


}

void PointsDisplayWidget::resizeGL(int w, int h) {

    int whalf = w/2;
    int hhalf = h/2;

    int min = std::min(whalf, hhalf);

    int delta_w = (whalf - min)/2;
    int delta_h = (hhalf - min)/2;

    _viewViewports[0] = QRect(delta_w, delta_h, min, min);
    _viewViewports[1] = QRect(whalf+delta_w, hhalf+delta_h, min, min);
    _viewViewports[2] = QRect(delta_w, hhalf+delta_h, min, min);
    _viewViewports[3] = QRect(whalf+delta_w, delta_h, min, min);
    _fullViewport = QRect(0, 0, w, h);

    _paintersViewports[0] = _viewViewports[2];
    _paintersViewports[1] = _viewViewports[3];
    _paintersViewports[2] = _viewViewports[0];
    _paintersViewports[3] = _viewViewports[1];

}

bool PointsDisplayWidget::displayColors() const
{
    return _displayColors;
}

void PointsDisplayWidget::displayColors(bool displayColors)
{
    _displayColors = displayColors;
    update();
}
