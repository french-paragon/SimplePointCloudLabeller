#ifndef FILELISTMANAGER_H
#define FILELISTMANAGER_H

#include <QList>
#include <QString>
#include <QFile>
#include <QDir>
#include <QFileInfo>
#include <QMap>

class FileListManager
{
public:
    static constexpr char defaultConfigurationFile[] = "points_classifications.labels";
    static constexpr char noClassLabel[] = ".no_class";
    static constexpr char skippedClassLabel[] = ".skipped_clouds";

    FileListManager();
    FileListManager(QString const& dirPath, QString const& currentLabel = "", QString const& configurationFile = defaultConfigurationFile);
    FileListManager(QDir const& dir, QString const& currentLabel = "", QString const& configurationFile = defaultConfigurationFile);

    int nFiles() const;
    int nInClass(QString const classLabel) const;
    void setCurrentIndex(int idx);
    void moveCurrentIndex(int delta);

    bool isEmpty();

    QString currentFilePath() const;
    bool classifyCurrentFile(QString const& className);
    bool refreshSkipped();

    bool saveClassification(QString const& altConfigurationFile = "");
    bool loadDataFromConfigurationFile(QString const& altConfigurationFile = "");

protected:

    QDir _currentDir;
    QString _configurationFile;
    QMap<QString,QStringList> _classifications;

    QString _currentLabel;
    int _currentIndex;
};

#endif // FILELISTMANAGER_H
