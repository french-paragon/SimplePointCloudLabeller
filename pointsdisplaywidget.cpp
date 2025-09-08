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
#include <QMouseEvent>

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

    _azimuth_angle = 45;
    _zenith_angle = 45;

    _ptSize = 2;

}

void PointsDisplayWidget::setPoints(QVector<QVector3D> const& points, QVector<QColor> const& colors, QVector<bool> const& mask) {

    _data.resize(points.size()*3);
    _color.resize(points.size()*3);
    _mask.resize(points.size());

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

        if (mask.size() > i) {

            _mask[i] = (mask[i]) ? 1 : 0;

        } else {

            _mask[i] = 0;

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

    _min_x = minX;
    _center_x = (maxX + minX)/2;
    _max_x = maxX;

    _min_y = minY;
    _center_y = (maxY + minY)/2;
    _max_y = maxY;

    _min_z = minZ;
    _center_z = minZ;
    _max_z = maxZ;

    _n_points = points.size();

    _needToLoad = true;

    recomputeViews();

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
    f->glClearDepthf(1.0f);
    f->glEnable(GL_MULTISAMPLE);
    f->glEnable(GL_POINT_SMOOTH);
    f->glEnable(GL_PROGRAM_POINT_SIZE);
    f->glEnable(GL_DEPTH_TEST);

    _scene_vao.create();

    _lm_pos_buffer.create();
    _lm_pos_buffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);

    _lm_col_buffer.create();
    _lm_col_buffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);

    _lm_mask_buffer.create();
    _lm_mask_buffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);

    _landMarkPointProgram = new QOpenGLShaderProgram();
    _landMarkPointProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/viewPersp.vert");
    _landMarkPointProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/viewPoints.frag");

    _landMarkPointProgram->link();
}

void PointsDisplayWidget::paintGL() {

    QPainter painter(this);

    painter.beginNativePainting();

    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    f->glEnable(GL_DEPTH_TEST);
    f->glDepthFunc(GL_LESS);
    f->glClear(GL_COLOR_BUFFER_BIT);
    f->glClear(GL_DEPTH_BUFFER_BIT);

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
    }

    int colorLocation = _landMarkPointProgram->attributeLocation("in_color");
    _landMarkPointProgram->enableAttributeArray(colorLocation);
    _landMarkPointProgram->setAttributeBuffer(colorLocation, GL_FLOAT, 0, 3);

    _lm_mask_buffer.bind();

    if (_needToLoad) {
        _lm_mask_buffer.allocate(_mask.data(), _mask.size()*sizeof (GLfloat));
    }

    int maskLocation = _landMarkPointProgram->attributeLocation("in_cluster_mask");
    _landMarkPointProgram->enableAttributeArray(maskLocation);
    _landMarkPointProgram->setAttributeBuffer(maskLocation, GL_FLOAT, 0, 1);

    _needToLoad = false;

    _landMarkPointProgram->setUniformValue("passId", 1);
    _landMarkPointProgram->setUniformValue("minZ", _min_z);
    _landMarkPointProgram->setUniformValue("maxZ", _max_z);
    _landMarkPointProgram->setUniformValue("ptSize", _ptSize);
    _landMarkPointProgram->setUniformValue("displayPointsColor", _displayColors);

    //draw the first pass
    for (int i = 0; i < 4; i++) {

        QRect v = _viewViewports[i];
        f->glViewport(v.left(), v.top(), v.width(), v.height());

        _landMarkPointProgram->setUniformValue("matrixViewProjection", _viewProjections[i]);
        _landMarkPointProgram->setUniformValue("origin", QVector3D(_center_x, _center_y, _center_z));

        f->glDrawArrays(GL_POINTS, 0, _n_points);
    }

    _landMarkPointProgram->setUniformValue("passId", 2);

    //draw the second pass, only the main cluster

    f->glClear(GL_DEPTH_BUFFER_BIT);

    for (int i = 0; i < 4; i++) {

        QRect v = _viewViewports[i];
        f->glViewport(v.left(), v.top(), v.width(), v.height());

        _landMarkPointProgram->setUniformValue("matrixViewProjection", _viewProjections[i]);
        _landMarkPointProgram->setUniformValue("origin", QVector3D(_center_x, _center_y, _center_z));

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

    //reset depth buffer
    f->glClear(GL_DEPTH_BUFFER_BIT);
    f->glDepthFunc(GL_ALWAYS);

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

void PointsDisplayWidget::mousePressEvent(QMouseEvent *e) {

    _previously_pressed = e->buttons();

    _motion_origin_pos = e->pos();

    if (_previously_pressed == Qt::MiddleButton or
            _previously_pressed == Qt::LeftButton or
            _previously_pressed == Qt::RightButton) {

        e->accept();
    } else {
        e->ignore();
    }

}
void PointsDisplayWidget::mouseReleaseEvent(QMouseEvent *e) {

    if (_previously_pressed == Qt::MiddleButton) {

        e->ignore();

    } else if (_previously_pressed == Qt::LeftButton) {

        return;
    }

}
void PointsDisplayWidget::mouseMoveEvent(QMouseEvent *e) {

    int b = e->buttons();

    if (b == Qt::MiddleButton) {
        QPoint nP = e->pos();

        QPoint t = nP - _motion_origin_pos;
        _motion_origin_pos = nP;

        rotateZenith(-t.y()/3.);
        rotateAzimuth(t.x()/3.);

        e->accept();

    } else if (b == Qt::RightButton) {

        return;

    } else if (b == Qt::NoButton) {

        return;
    }

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

void PointsDisplayWidget::rotateZenith(float degrees) {

    float newAngle = _zenith_angle - degrees;
    if (newAngle < -89) {
        newAngle = -89;
    } else if (newAngle > 89) {
        newAngle = 89;
    }

    if (newAngle != _zenith_angle) {
        _zenith_angle = newAngle;
        recomputeViews();
        update();
    }
}
void PointsDisplayWidget::rotateAzimuth(float degrees) {

    float newAngle = _azimuth_angle - degrees;

    if (newAngle < 0) {
        newAngle += (floor(fabs(newAngle)/360.) + 1)*360.;
    } else if (newAngle > 360) {
        newAngle -= floor(fabs(newAngle)/360.)*360.;
    }

    if (newAngle != _azimuth_angle) {
        _azimuth_angle = newAngle;
        recomputeViews();
        update();
    }
}

void PointsDisplayWidget::increaseSize(float scale) {
    _ptSize *= scale;
    update();
}
void PointsDisplayWidget::decreaseSize(float scale) {
    _ptSize /= scale;
    update();
}

void PointsDisplayWidget::recomputeViews() {


    QMatrix4x4 transformBase;

    QMatrix4x4 translation;
    translation.translate(QVector3D((_max_x - _min_x)/2 - _max_x,
                                    (_max_y - _min_y)/2 - _max_y,
                                    (_max_z - _min_z)/2 - _max_z));

    float scaleX = 1./std::max(_max_x - _min_x, 0.01f);
    float scaleY = 1./std::max(_max_y - _min_y, 0.01f);
    float scaleZ = 1./std::max(_max_z - _min_z, 0.01f);

    float scale = std::min(scaleX, std::min(scaleY, scaleZ));
    QMatrix4x4 scalemat;
    scalemat.scale(scale);

    _inv_scale = 1./scale;

    transformBase = scalemat*translation;

    QMatrix4x4 proj;
    proj.ortho(-1.0, 1.0, -1.0, 1.0, -10, 10);

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

    float cosZ = std::cos(_zenith_angle/180*M_PI);
    float sinZ = std::sin(_zenith_angle/180*M_PI);
    float cosA = std::cos(_azimuth_angle/180*M_PI);
    float sinA = std::sin(_azimuth_angle/180*M_PI);

    QVector3D eye(cosA*cosZ,sinA*cosZ,sinZ);
    QVector3D up(-cosA*sinZ,-sinA*sinZ,cosZ);
    eye *= std::sqrt(6); //set the view a bit further away than 1

    QMatrix4x4 transform3;
    transform3.lookAt(eye, QVector3D(0,0,0), up);

    _viewProjections[3] = proj*transform3*transformBase;
    _viewNames[3] = "Angle";

    return;
}
