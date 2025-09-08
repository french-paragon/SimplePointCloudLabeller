#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

class PointsDisplayWidget;
class QHBoxLayout;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = nullptr, Qt::WindowFlags flags = Qt::WindowFlags());

    void setPossibleLabels(QStringList const& labels);
    bool openPointCloud(QString const& path, const QString &mainClusterArg = "");
    void openDefaultPointCloud();
    void configureViewerMode(bool withCorrections);

Q_SIGNALS:

    void labelChoosen(int labelId);
    void moveToNext();
    void navigate(int delta);
    void requestLabelCorrection();

protected:

    void keyReleaseEvent(QKeyEvent *event) override;

    QStringList _possibleLabels;
    PointsDisplayWidget* _displayWidget;
    QHBoxLayout* _labelButtonLayout;
};

#endif // MAINWINDOW_H
