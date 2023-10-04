#ifndef POINTSDISPLAYWIDGET_H
#define POINTSDISPLAYWIDGET_H
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

#include <QOpenGLWidget>

#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>

class QOpenGLShaderProgram;

class PointsDisplayWidget : public QOpenGLWidget
{
    Q_OBJECT
public:
    explicit PointsDisplayWidget(QWidget *parent = nullptr);
    ~PointsDisplayWidget();

    void setPoints(QVector<QVector3D> const& points);

Q_SIGNALS:

protected:

    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int w, int h) override;

    int _n_points;
    float _min_z;
    float _max_z;
    std::vector<GLfloat> _data;
    bool _needToLoad;

    QOpenGLVertexArrayObject _scene_vao;

    QOpenGLBuffer _lm_pos_buffer;

    QOpenGLShaderProgram* _landMarkPointProgram;

    QVector<QRect> _viewViewports;
    QRect _fullViewport;
    QVector<QMatrix4x4> _viewProjections;

};

#endif // POINTSDISPLAYWIDGET_H
