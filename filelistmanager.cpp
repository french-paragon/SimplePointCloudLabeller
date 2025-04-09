#include "filelistmanager.h"

#include <QTextStream>

FileListManager::FileListManager() :
    FileListManager(QDir(),"","")
{

}

FileListManager::FileListManager(QString const& dirPath, QString const& currentLabel, QString const& configurationFile) :
    FileListManager(QDir(dirPath), currentLabel, configurationFile)
{

}
FileListManager::FileListManager(QDir const& dir, QString const& currentLabel, QString const& configurationFile) :
    _currentDir(dir),
    _configurationFile(configurationFile),
    _currentLabel(currentLabel),
    _currentIndex(0)
{
    if (_currentLabel.isEmpty()) {
        _currentLabel = noClassLabel;
    }
    loadDataFromConfigurationFile();
}

int FileListManager::nFiles() const {
    if (!_classifications.contains(_currentLabel)) {
        return 0;
    }
    return _classifications[_currentLabel].size();
}
int FileListManager::nInClass(QString const classLabel) const {
    if (!_classifications.contains(classLabel)) {
        return 0;
    }
    return _classifications[classLabel].size();
}
void FileListManager::setCurrentIndex(int idx) {
    _currentIndex = idx;
    _currentIndex %= std::max(1,nFiles());
}
void FileListManager::moveCurrentIndex(int delta) {
    _currentIndex += delta;
    _currentIndex %= std::max(1,nFiles());
}

bool FileListManager::isEmpty() {
    return nFiles() == 0;
}

QString FileListManager::currentFilePath() const {
    if (!_classifications.contains(_currentLabel)) {
        return "";
    }

    if (nFiles() == 0) {
        return "";
    }

    QString filename = _classifications[_currentLabel].at(_currentIndex);
    return _currentDir.absoluteFilePath(filename);
}

bool FileListManager::classifyCurrentFile(QString const& className) {

    if (className == _currentLabel) {
        return true;
    }

    if (!_classifications.contains(_currentLabel)) {
        return false;
    }

    if (nFiles() == 0) {
        return false;
    }

    _currentIndex %= std::max(1,nFiles());

    _classifications[className].push_back(_classifications[_currentLabel].at(_currentIndex));

    _classifications[_currentLabel].removeAt(_currentIndex);

    _currentIndex %= std::max(1,nFiles());

    saveClassification();

    return true;
}
bool FileListManager::refreshSkipped() {

    if (!_classifications.contains(skippedClassLabel)) {
        return false;
    }

    if (skippedClassLabel == _currentLabel) {
        return false;
    }

    if (!_classifications.contains(_currentLabel)) {
        _classifications[_currentLabel] = _classifications[skippedClassLabel];
    } else {
        _classifications[_currentLabel].append(_classifications[skippedClassLabel]);
    }

    _classifications[skippedClassLabel].clear();
    return true;
}

bool FileListManager::saveClassification(QString const& altConfigurationFile) {

    QString fileName = _configurationFile;
    if (!altConfigurationFile.isEmpty()) {
        fileName = altConfigurationFile;
    }

    if (fileName.isEmpty()) {
        fileName = defaultConfigurationFile;
    }

    QStringList classes = _classifications.keys();

    QFile out(_currentDir.absoluteFilePath(fileName));

    if (!out.open(QFile::WriteOnly|QFile::Text)) {
        return false;
    }

    QTextStream outStream(&out);

    for (QString const& key : qAsConst(classes)) {
        outStream << key << "\n";

        QStringList& list = _classifications[key];

        for (QString const& file : qAsConst(list)) {
            outStream << "\t" << file << "\n";
        }
    }

    return true;

}
bool FileListManager::loadDataFromConfigurationFile(QString const& altConfigurationFile) {

    QString fileName = _configurationFile;
    if (!altConfigurationFile.isEmpty()) {
        fileName = altConfigurationFile;
    }

    if (fileName.isEmpty()) {
        fileName = defaultConfigurationFile;
    }

    _configurationFile = fileName;

    QFile in(_currentDir.absoluteFilePath(fileName));

    if (!in.exists()) {

        _configurationFile = fileName;

        QStringList files = _currentDir.entryList();

        QString currentClass = noClassLabel;

        for (QString const& file : files) {
            if (file.endsWith(".pcd", Qt::CaseInsensitive)) {

                _classifications[currentClass].append(file);
            }
        }

        return true;
    }

    if (!in.open(QFile::ReadOnly|QFile::Text)) {
        return false;
    }

    QString currentClass = noClassLabel;

    _classifications.clear();

    while(!in.atEnd()) {
        QString line = QString::fromLocal8Bit(in.readLine());

        if (!line.startsWith("\t")) {
            currentClass = line.trimmed();
            continue;

        }

        QString file = line.trimmed();

        _classifications[currentClass].append(file);

    }

    return true;

}
