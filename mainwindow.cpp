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
void MainWindow::configureViewerMode(bool withCorrections) {

    QLayoutItem* item;
    while ( ( item = _labelButtonLayout->takeAt( 0 ) ) != nullptr ) {
        delete item->widget();
        delete item;
    }

    QPushButton* previous = new QPushButton(this);

    previous->setText("Previous");

    connect(previous, &QPushButton::clicked, this, [this] () {
        Q_EMIT navigate(-1);
    });

    _labelButtonLayout->addWidget(previous);

    QPushButton* next = new QPushButton(this);

    next->setText("Next");

    connect(next, &QPushButton::clicked, this, [this] () {
        Q_EMIT navigate(1);
    });

    _labelButtonLayout->addWidget(next);

    if (withCorrections) {

        QPushButton* correct = new QPushButton(this);

        correct->setText("Correct");

        connect(correct, &QPushButton::clicked, this, [this] () {
            Q_EMIT requestLabelCorrection();
        });

        _labelButtonLayout->addWidget(correct);

    }

}

template<uint8_t type>
float getFieldValue(void* ptr) {

    switch (type) {
    case pcl::PCLPointField::INT8 :
    {
        int8_t rawVal = *reinterpret_cast<int8_t*>(ptr);
        return static_cast<float>(rawVal);
    }
    case pcl::PCLPointField::UINT8 :
    {
        uint8_t rawVal = *reinterpret_cast<uint8_t*>(ptr);
        return static_cast<float>(rawVal);
    }
    case pcl::PCLPointField::INT16 :
    {
        int16_t rawVal = *reinterpret_cast<int16_t*>(ptr);
        return static_cast<float>(rawVal);
    }
    case pcl::PCLPointField::UINT16 :
    {
        uint16_t rawVal = *reinterpret_cast<uint16_t*>(ptr);
        return static_cast<float>(rawVal);
    }
    case pcl::PCLPointField::INT32 :
    {
        int32_t rawVal = *reinterpret_cast<int32_t*>(ptr);
        return static_cast<float>(rawVal);
    }
    case pcl::PCLPointField::UINT32 :
    {
        uint32_t rawVal = *reinterpret_cast<uint32_t*>(ptr);
        return static_cast<float>(rawVal);
    }
    case pcl::PCLPointField::FLOAT32 :
    {
        float val = *reinterpret_cast<float*>(ptr);
        return val;
    }
    case pcl::PCLPointField::FLOAT64 :
    {
        double rawVal = *reinterpret_cast<double*>(ptr);
        return static_cast<float>(rawVal);
    }
    default:
        break;
    }

    return 0.0;
}

float getFieldValue(void* ptr, uint8_t type) {

    switch (type) {
    case pcl::PCLPointField::INT8 :
    {
        int8_t rawVal = *reinterpret_cast<int8_t*>(ptr);
        return static_cast<float>(rawVal);
    }
    case pcl::PCLPointField::UINT8 :
    {
        uint8_t rawVal = *reinterpret_cast<uint8_t*>(ptr);
        return static_cast<float>(rawVal);
    }
    case pcl::PCLPointField::INT16 :
    {
        int16_t rawVal = *reinterpret_cast<int16_t*>(ptr);
        return static_cast<float>(rawVal);
    }
    case pcl::PCLPointField::UINT16 :
    {
        uint16_t rawVal = *reinterpret_cast<uint16_t*>(ptr);
        return static_cast<float>(rawVal);
    }
    case pcl::PCLPointField::INT32 :
    {
        int32_t rawVal = *reinterpret_cast<int32_t*>(ptr);
        return static_cast<float>(rawVal);
    }
    case pcl::PCLPointField::UINT32 :
    {
        uint32_t rawVal = *reinterpret_cast<uint32_t*>(ptr);
        return static_cast<float>(rawVal);
    }
    case pcl::PCLPointField::FLOAT32 :
    {
        float val = *reinterpret_cast<float*>(ptr);
        return val;
    }
    case pcl::PCLPointField::FLOAT64 :
    {
        double rawVal = *reinterpret_cast<double*>(ptr);
        return static_cast<float>(rawVal);
    }
    default:
        break;
    }

    return 0.0;
}

float colorScaleFromType(uint8_t type) {
    switch (type) {
    case pcl::PCLPointField::INT8 :
        return 2;
    case pcl::PCLPointField::UINT8 :
        return 1;
    case pcl::PCLPointField::INT16 :
        return 1./(256);
    case pcl::PCLPointField::UINT16 :
        return 1./(256);
    case pcl::PCLPointField::INT32 :
        return 1./(256*256*256);
    case pcl::PCLPointField::UINT32 :
        return 1./(256*256*256);
    case pcl::PCLPointField::FLOAT32 :
        return 255;
    case pcl::PCLPointField::FLOAT64 :
        return 255;
    default:
        break;
    }

    return 1.0;
}

bool MainWindow::openPointCloud(QString const& path) {

    pcl::PCLPointCloud2 cloud;

    if (pcl::io::loadPCDFile (path.toStdString(), cloud) == -1) {
        return false;
    }

    int pointStructSize = cloud.point_step; //size of a point struct

    //fields offsets
    int x_field_offset = -1;
    int y_field_offset = -1;
    int z_field_offset = -1;

    int r_field_offset = -1;
    int g_field_offset = -1;
    int b_field_offset = -1;

    int rgb_field_offset = -1;

    //fields types
    uint8_t x_field_type = 0;
    uint8_t y_field_type = 0;
    uint8_t z_field_type = 0;

    uint8_t r_field_type = 0;
    uint8_t g_field_type = 0;
    uint8_t b_field_type = 0;

    for (pcl::PCLPointField const& field : cloud.fields) {

        if (QString::fromStdString(field.name).toLower() == "x") {
            x_field_offset = field.offset;
            x_field_type = field.datatype;
        }

        if (QString::fromStdString(field.name).toLower() == "y") {
            y_field_offset = field.offset;
            y_field_type = field.datatype;
        }

        if (QString::fromStdString(field.name).toLower() == "z") {
            z_field_offset = field.offset;
            z_field_type = field.datatype;
        }

        if (QString::fromStdString(field.name).toLower() == "r") {
            r_field_offset = field.offset;
            r_field_type = field.datatype;
        }

        if (QString::fromStdString(field.name).toLower() == "g") {
            g_field_offset = field.offset;
            g_field_type = field.datatype;
        }

        if (QString::fromStdString(field.name).toLower() == "b") {
            b_field_offset = field.offset;
            b_field_type = field.datatype;
        }

        if (QString::fromStdString(field.name).toLower() == "red") {
            r_field_offset = field.offset;
            r_field_type = field.datatype;
        }

        if (QString::fromStdString(field.name).toLower() == "green") {
            g_field_offset = field.offset;
            g_field_type = field.datatype;
        }

        if (QString::fromStdString(field.name).toLower() == "blue") {
            b_field_offset = field.offset;
            b_field_type = field.datatype;
        }

        if (QString::fromStdString(field.name).toLower() == "rgb" and
                field.datatype == pcl::PCLPointField::UINT32) {
            rgb_field_offset = field.offset;
        }

    }

    if (x_field_offset < 0 or y_field_offset < 0 or z_field_offset < 0) {
        return false;
    }

    //get the r, g and b fields from rgb if no individual fields have been found
    if (rgb_field_offset >= 0 and
            (r_field_offset < 0 or g_field_offset < 0 or b_field_offset < 0)) {

        b_field_offset = rgb_field_offset;
        g_field_offset = rgb_field_offset+1;
        r_field_offset = rgb_field_offset+2;

        r_field_type = pcl::PCLPointField::UINT8;
        g_field_type = pcl::PCLPointField::UINT8;
        b_field_type = pcl::PCLPointField::UINT8;
    }

    float r_scale = colorScaleFromType(r_field_type);
    float g_scale = colorScaleFromType(g_field_type);
    float b_scale = colorScaleFromType(b_field_type);

    auto cloudSize = cloud.height*cloud.width;

    QVector<QVector3D> points(cloudSize);
    QVector<QColor> colors(cloudSize);

    for (int i = 0; i < cloudSize; i++) {

        uint8_t* dataRow = cloud.data.data() + i*pointStructSize;

        float x = getFieldValue(dataRow + x_field_offset, x_field_type);
        float y = getFieldValue(dataRow + y_field_offset, y_field_type);
        float z = getFieldValue(dataRow + z_field_offset, z_field_type);

        float r = 128;
        float g = 128;
        float b = 128;

        if (r_field_offset > 0) {
            r = getFieldValue(dataRow + r_field_offset, r_field_type);
            r *= r_scale;
        }

        if (g_field_offset > 0) {
            g = getFieldValue(dataRow + g_field_offset, g_field_type);
            g *= g_scale;
        }

        if (b_field_offset > 0) {
            b = getFieldValue(dataRow + b_field_offset, b_field_type);
            b *= b_scale;
        }

        points[i] = QVector3D(x, y, z);
        colors[i] = QColor(static_cast<int>(r), static_cast<int>(g), static_cast<int>(b));
    }


    _displayWidget->setPoints(points, colors);

    return true;

}
void MainWindow::openDefaultPointCloud() {
    _displayWidget->setPoints(QVector<QVector3D>{QVector3D(0.5, 0.5, 0.5), QVector3D(-0.5, -0.5, -0.5), QVector3D(0.5, -0.5, 0), QVector3D(-0.5, 0.5, 0)},
                              QVector<QColor>{QColor(0, 255, 255), QColor(0, 0, 0), QColor(255, 255, 0), QColor(255, 0, 255)});
}
