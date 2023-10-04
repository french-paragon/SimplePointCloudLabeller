#include "mainwindow.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QComboBox>

#include "pointsdisplaywidget.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <QVector>
#include <QVector3D>

MainWindow::MainWindow(QWidget *parent, Qt::WindowFlags flags) :
    QMainWindow(parent, flags)
{

    QWidget* centralWidget = new QWidget(this);

    QVBoxLayout* vLayout = new QVBoxLayout();

    _displayWidget = new PointsDisplayWidget(this);
    _displayWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    vLayout->addWidget(_displayWidget);

    QComboBox* displayModeSelect = new QComboBox(_displayWidget);
    displayModeSelect->move(5, 5);

    displayModeSelect->addItem(tr("Color"), QVariant("color"));
    displayModeSelect->addItem(tr("Height"), QVariant("height"));

    connect(displayModeSelect, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, [this, displayModeSelect] (int idx) {
        QString userData = displayModeSelect->currentData(Qt::UserRole).toString();

        if (userData == QString("color")) {
            _displayWidget->displayColors(true);
        } else {
            _displayWidget->displayColors(false);
        }
    });

    QWidget* buttonRow = new QWidget(this);
    buttonRow->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);

    _labelButtonLayout = new QHBoxLayout();
    _labelButtonLayout->setSpacing(5);
    _labelButtonLayout->setMargin(5);
    buttonRow->setLayout(_labelButtonLayout);

    vLayout->addWidget(buttonRow);

    centralWidget->setLayout(vLayout);
    setCentralWidget(centralWidget);

    resize(900, 600);
}

void MainWindow::setPossibleLabels(QStringList const& labels) {

    QLayoutItem* item;
    while ( ( item = _labelButtonLayout->takeAt( 0 ) ) != nullptr ) {
        delete item->widget();
        delete item;
    }

    for (int i = 0; i < labels.size(); i++) {

        QString const& label = labels[i];

        QPushButton* button = new QPushButton(this);

        button->setText(label);

        connect(button, &QPushButton::clicked, this, [i, this] () {
            Q_EMIT labelChoosen(i);
        });

        _labelButtonLayout->addWidget(button);
    }

    QPushButton* button = new QPushButton(this);

    button->setText("Skip");

    connect(button, &QPushButton::clicked, this, [this] () {
        Q_EMIT moveToNext();
    });

    _labelButtonLayout->addWidget(button);

}

bool MainWindow::openPointCloud(QString const& path) {

    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (path.toStdString(), cloud) == -1) {
        return false;
    }

    QVector<QVector3D> points(cloud.size());
    QVector<QColor> colors(cloud.size());

    for (int i = 0; i < cloud.size(); i++) {
        points[i] = QVector3D(cloud[i].x, cloud[i].y, cloud[i].z);
        colors[i] = QColor(cloud[i].r, cloud[i].g, cloud[i].b);
    }


    _displayWidget->setPoints(points, colors);

    return true;

}
void MainWindow::openDefaultPointCloud() {
    _displayWidget->setPoints(QVector<QVector3D>{QVector3D(0.5, 0.5, 0.5), QVector3D(-0.5, -0.5, -0.5), QVector3D(0.5, -0.5, 0), QVector3D(-0.5, 0.5, 0)},
                              QVector<QColor>{QColor(0, 255, 255), QColor(0, 0, 0), QColor(255, 255, 0), QColor(255, 0, 255)});
}
