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

#include <QApplication>
#include <QTextStream>
#include <QCommandLineParser>

#include <QDir>
#include <QFileInfo>

#include <QVector3D>
#include <QSurfaceFormat>

#include "mainwindow.h"

int main(int argc, char** argv) {

    QTextStream out(stdout);
    QTextStream err(stderr);

    QApplication app(argc, argv);

    //surface setup

    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setSamples(4);
    QSurfaceFormat::setDefaultFormat(format);

    QCommandLineParser parser;

    QString targetFolderArgName = "target_folder";
    QString classesArgName = "classes";

    QCommandLineOption classOption(QStringList{"c", classesArgName});
    classOption.setDescription("Possible classes, once per invocation of the option");
    classOption.setValueName("class");

    parser.addPositionalArgument(targetFolderArgName, "Folder where the targets points cloud are located");
    parser.addOption(classOption);

    QStringList arguments = app.arguments();
    parser.process(arguments);

    if (parser.positionalArguments().size() != 1) {
        err << "SimplePointsCloudLabeler takes one and only one positional argument: the input folder" << Qt::endl;
        return 1;
    }

    QString folderPath = parser.positionalArguments()[0];
    QDir folder(folderPath);

    if (!folder.exists()) {
        err << "Invalid input folder: \"" << folderPath << "\""<< Qt::endl;
        return 1;
    }

    QStringList files = folder.entryList();
    QStringList filesSelected;

    for (QString const& file : files) {
        if (file.endsWith(".pcd", Qt::CaseInsensitive)) {
            filesSelected << file;
        }
    }

    if (filesSelected.isEmpty()) {
        err << "Could not find point cloud in input folder: \"" << folderPath << "\""<< Qt::endl;
        return 1;
    }

    QStringList classes = parser.values(classOption);

    if (classes.isEmpty()) {
        err << "No classes provided!" << Qt::endl;
        return 1;
    }

    out << "\nList of provided classes: " << Qt::endl;

    for (int i = 0; i < classes.size(); i++) {
        out << "\t" << i << ". " << classes[i] << Qt::endl;

        if (!folder.exists(classes[i])) {
            bool ok = folder.mkdir(classes[i]);

            if (!ok) {
                err << "Could not create class output directory: \"" << folder.filePath(classes[i]) << "\"" << Qt::endl;
                return 1;
            }
        }
    }

    out << "\nList of files to treat: " << Qt::endl;

    for (int i = 0; i < filesSelected.size(); i++) {
        out << "\t" << i << ". " << filesSelected[i] << Qt::endl;
    }

    MainWindow mw;

    std::reverse(filesSelected.begin(), filesSelected.end());

    QString currentFile = filesSelected.last();
    filesSelected.pop_back();

    QObject::connect(&mw, &MainWindow::labelChoosen, [&currentFile, &filesSelected, &classes, &folder, &mw, &out, &err] (int id) -> void {
       QString label = classes[id];
       out << "Label: " << label << " requested for file: " << currentFile << Qt::endl;

       QDir labelDir = folder.filePath(label);

       QFile pointCloudFile = folder.filePath(currentFile);

       if (!pointCloudFile.exists()) {
           err << "File: " << currentFile << " do not exists!" << Qt::endl;
           return;
       }

       bool ok = pointCloudFile.rename(labelDir.filePath(currentFile));

       if (!ok) {
           err << "File: " << currentFile << " could not be moved to: " << labelDir.filePath(currentFile) << "!" << Qt::endl;
           return;
       } else {
           out << "File: " << currentFile << " moved to: " << labelDir.filePath(currentFile) << "!" << Qt::endl;
       }

       currentFile = filesSelected.last();
       filesSelected.pop_back();

       QFile newPointCloudFile = folder.filePath(currentFile);

       if (!newPointCloudFile.exists()) {
           err << "File: " << currentFile << " do not exists!" << Qt::endl;
           return;
       }

       mw.openPointCloud(folder.filePath(currentFile));

    });

    QObject::connect(&mw, &MainWindow::moveToNext, [&currentFile, &filesSelected, &classes, &folder, &mw, &out, &err] () -> void {
       out << "Skip requested for file: " << currentFile << Qt::endl;

       QFile pointCloudFile = folder.filePath(currentFile);

       if (!pointCloudFile.exists()) {
           err << "File: " << currentFile << " do not exists!" << Qt::endl;
           return;
       }

       filesSelected.push_front(currentFile);

       currentFile = filesSelected.last();
       filesSelected.pop_back();

       QFile newPointCloudFile = folder.filePath(currentFile);

       if (!newPointCloudFile.exists()) {
           err << "File: " << currentFile << " do not exists!" << Qt::endl;
           return;
       }

       mw.openPointCloud(folder.filePath(currentFile));

    });

    mw.setPossibleLabels(classes);
    mw.openPointCloud(folder.filePath(currentFile));
    //mw.openDefaultPointCloud();

    mw.show();

    return app.exec();

}
