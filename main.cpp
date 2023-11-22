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

#include <chrono>

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
    QString noTimingArgName = "noTiming";

    QCommandLineOption classOption(QStringList{"c", classesArgName});
    classOption.setDescription("Possible classes, once per invocation of the option");
    classOption.setValueName("class");

    QCommandLineOption noTimingOption(QStringList{"t", noTimingArgName});
    noTimingOption.setDescription("Disable timing of the session.");

    parser.addPositionalArgument(targetFolderArgName, "Folder where the targets points cloud are located");
    parser.addOption(classOption);
    parser.addOption(noTimingOption);

    QStringList arguments = app.arguments();
    parser.process(arguments);

    if (parser.positionalArguments().size() != 1) {
        err << "SimplePointsCloudLabeler takes one and only one positional argument: the input folder" << endl;
        return 1;
    }

    QString folderPath = parser.positionalArguments()[0];
    QDir folder(folderPath);

    if (!folder.exists()) {
        err << "Invalid input folder: \"" << folderPath << "\""<< endl;
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
        err << "Could not find point cloud in input folder: \"" << folderPath << "\""<< endl;
        return 1;
    }

    QStringList reservedClasses{".skipped_clouds"};
    QStringList classes = parser.values(classOption);

    if (classes.isEmpty()) {
        err << "No classes provided!" << endl;
        return 1;
    }

    for (QString const& c : classes) {
        if (reservedClasses.contains(c)) {
            err << "Class " << c << " is a reserved class name!" << endl;
            return 1;
        }
    }

    out << "\nList of provided classes: " << endl;

    for (int i = 0; i < classes.size(); i++) {
        out << "\t" << i << ". " << classes[i] << endl;

        if (!folder.exists(classes[i])) {
            bool ok = folder.mkdir(classes[i]);

            if (!ok) {
                err << "Could not create class output directory: \"" << folder.filePath(classes[i]) << "\"" << endl;
                return 1;
            }
        }
    }

    for (int i = 0; i < reservedClasses.size(); i++) {

        if (!folder.exists(reservedClasses[i])) {
            bool ok = folder.mkdir(reservedClasses[i]);

            if (!ok) {
                err << "Could not create class output directory: \"" << folder.filePath(reservedClasses[i]) << "\"" << endl;
                return 1;
            }
        }
    }

    bool timing = !parser.isSet(noTimingOption);
    int nLabeledElements = 0;
    int nSkippedElements = 0;

    out << "\nList of files to treat: " << endl;

    for (int i = 0; i < filesSelected.size(); i++) {
        out << "\t" << i << ". " << filesSelected[i] << endl;
    }

    MainWindow mw;

    std::reverse(filesSelected.begin(), filesSelected.end());

    QString currentFile = filesSelected.last();
    filesSelected.pop_back();

    QObject::connect(&mw, &MainWindow::labelChoosen, [&currentFile, &filesSelected, &classes, &folder, &mw, &out, &err, &nLabeledElements] (int id) -> void {
       QString label = classes[id];
       out << "Label: " << label << " requested for file: " << currentFile << endl;

       QDir labelDir = folder.filePath(label);

       QFile pointCloudFile = folder.filePath(currentFile);

       if (!pointCloudFile.exists()) {
           err << "File: " << currentFile << " do not exists!" << endl;
           return;
       }

       bool ok = pointCloudFile.rename(labelDir.filePath(currentFile));

       if (!ok) {
           err << "File: " << currentFile << " could not be moved to: " << labelDir.filePath(currentFile) << "!" << endl;
           return;
       } else {
           out << "File: " << currentFile << " moved to: " << labelDir.filePath(currentFile) << "!" << endl;
       }

       nLabeledElements++;

       currentFile = filesSelected.last();
       filesSelected.pop_back();

       QFile newPointCloudFile = folder.filePath(currentFile);

       if (!newPointCloudFile.exists()) {
           err << "File: " << currentFile << " do not exists!" << endl;
           return;
       }

       mw.openPointCloud(folder.filePath(currentFile));

    });

    QObject::connect(&mw, &MainWindow::moveToNext, [&currentFile, &filesSelected, &classes, &folder, &mw, &out, &err, &nSkippedElements] () -> void {
       out << "Skip requested for file: " << currentFile << endl;

       QFile pointCloudFile = folder.filePath(currentFile);

       if (!pointCloudFile.exists()) {
           err << "File: " << currentFile << " do not exists!" << endl;
           return;
       }

       QDir outDir = folder.filePath(".skipped_clouds");

       if (!pointCloudFile.exists()) {
           err << "File: " << currentFile << " do not exists!" << endl;
           return;
       }

       bool ok = pointCloudFile.rename(outDir.filePath(currentFile));

       if (!ok) {
           err << "File: " << currentFile << " could not be moved to: " << outDir.filePath(currentFile) << "!" << endl;
           return;
       } else {
           out << "File: " << currentFile << " moved to: " << outDir.filePath(currentFile) << "!" << endl;
       }

       nSkippedElements++;

       currentFile = filesSelected.last();
       filesSelected.pop_back();

       QFile newPointCloudFile = folder.filePath(currentFile);

       if (!newPointCloudFile.exists()) {
           err << "File: " << currentFile << " do not exists!" << endl;
           return;
       }

       mw.openPointCloud(folder.filePath(currentFile));

    });

    mw.setPossibleLabels(classes);
    mw.openPointCloud(folder.filePath(currentFile));
    //mw.openDefaultPointCloud();

    mw.show();

    auto startTime = std::chrono::high_resolution_clock::now();

    int code = app.exec();

    auto endTime = std::chrono::high_resolution_clock::now();

    if (timing) {

        out << "\n" << "Timing infos" << "\n\n";

        auto duration_s = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime);
        out << "Labelling session time: " << duration_s.count() << "s" << "\n";
        out << "Labelled instances: " << nLabeledElements << " instances" << "\n";
        out << "Skipped instances: " << nSkippedElements << " instances" << endl;
    }

    return code;

}
