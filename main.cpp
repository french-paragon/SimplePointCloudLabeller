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
#include <QMessageBox>

#include <chrono>

#include "mainwindow.h"
#include "filelistmanager.h"

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
    QString viewerModeArgName = "viewerMode";
    QString correctionModeArgName = "correctionMode";
    QString maskName = "mask";

    QCommandLineOption classOption(QStringList{"c", classesArgName});
    classOption.setDescription("Possible classes, once per invocation of the option");
    classOption.setValueName("class");

    QCommandLineOption noTimingOption(QStringList{"t", noTimingArgName});
    noTimingOption.setDescription("Disable timing of the session.");

    QCommandLineOption viewerModeOption(QStringList{"v", viewerModeArgName});
    viewerModeOption.setDescription("Open the prog in viewer mode (just explore a folder rather than labelling points).");

    QCommandLineOption correctionModeOption(QStringList{"k", correctionModeArgName});
    correctionModeOption.setDescription("Add a button to correct point clouds in viewer mode.");

    QCommandLineOption maskArgumentOption(QStringList{"m", maskName}, "attribute used for masking", "attribute_name", "");

    parser.addPositionalArgument(targetFolderArgName, "Folder where the targets points cloud are located");
    parser.addOption(classOption);
    parser.addOption(noTimingOption);
    parser.addOption(viewerModeOption);
    parser.addOption(correctionModeOption);
    parser.addOption(maskArgumentOption);

    parser.addHelpOption();

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

    QStringList reservedClasses{FileListManager::noClassLabel, FileListManager::skippedClassLabel};
    QStringList classes = parser.values(classOption);
    QString maskAttribute = parser.value(maskArgumentOption);

    bool viewerMode = parser.isSet(viewerModeOption);
    bool correctionMode = parser.isSet(correctionModeOption);

    FileListManager fileListManager(folder);

    if (classes.isEmpty() and !viewerMode) {
        err << "No classes provided!" << Qt::endl;
        return 1;
    }

    if (viewerMode) {
        if (classes.size() > 1) {
            err << "Cannot view more than one class in viewer mode!" << Qt::endl;
            return 1;
        }

        if (classes.size() == 1) {
            fileListManager.setCurrentClass(classes[0]);
        }

        classes.clear();
        reservedClasses.clear();
    }

    if (fileListManager.isEmpty()) {
        err << "Could not find point cloud for input folder: \"" << folderPath << "\""<< Qt::endl;
        return 1;
    }

    for (QString const& c : classes) {
        if (reservedClasses.contains(c)) {
            err << "Class " << c << " is a reserved class name!" << Qt::endl;
            return 1;
        }
    }

    if (!viewerMode) {
        out << "\nList of provided classes: " << Qt::endl;
    }

    for (int i = 0; i < classes.size(); i++) {
        out << "\t" << i << ". " << classes[i] << Qt::endl;
    }

    bool timing = !parser.isSet(noTimingOption);
    int nLabeledElements = 0;
    int nSkippedElements = 0;

    if (!viewerMode) {
        out << "\nList of files to treat: " << Qt::endl;
    }

    if (viewerMode) {
        out << "\nList of files to view: " << Qt::endl;
    }

    for (int i = 0; i < fileListManager.nFiles(); i++) {
        fileListManager.setCurrentIndex(i);
        out << "\t" << i << ". " << fileListManager.currentFilePath() << Qt::endl;
    }

    MainWindow mw;

    QObject::connect(&mw, &MainWindow::labelChoosen, [&fileListManager, &classes, &folder, &mw, &out, &err, &nLabeledElements, &maskAttribute] (int id) -> void {
       QString label;

       if (id >= 0 and id < classes.size()) {
           label = classes[id];
       } else {
           label = FileListManager::skippedClassLabel;
       }
       out << "Label: " << label << " requested for file: " << fileListManager.currentFilePath() << Qt::endl;

       fileListManager.classifyCurrentFile(label);

       if (fileListManager.nFiles() == 0 and fileListManager.nInClass(FileListManager::skippedClassLabel) > 0) {
           fileListManager.refreshSkipped();
       }

       if (fileListManager.nFiles() == 0) {
           QMessageBox::information(&mw,
                                    QObject::tr("No more point cloud"),
                                    QObject::tr("All file in the class have been corrected!"));
       }

       nLabeledElements++;

       mw.openPointCloud(fileListManager.currentFilePath(), maskAttribute);

    });

    QObject::connect(&mw, &MainWindow::moveToNext, [&fileListManager, &classes, &folder, &mw, &out, &err, &nSkippedElements, &maskAttribute] () -> void {
       out << "Skip requested for file: " << fileListManager.currentFilePath() << Qt::endl;

       fileListManager.classifyCurrentFile(FileListManager::skippedClassLabel);

       if (fileListManager.nFiles() == 0 and fileListManager.nInClass(FileListManager::skippedClassLabel) > 0) {
           QMessageBox::information(&mw,
                                    QObject::tr("No more point cloud"),
                                    QObject::tr("All unclassified files in the class have been skipped!"));
           fileListManager.refreshSkipped();
       }

       nSkippedElements++;

       mw.openPointCloud(fileListManager.currentFilePath(), maskAttribute);

    });

    QObject::connect(&mw, &MainWindow::navigate, [&fileListManager, &classes, &folder, &mw, &out, &err, &maskAttribute] (int delta) -> void {

        fileListManager.moveCurrentIndex(delta);

        mw.openPointCloud(fileListManager.currentFilePath(), maskAttribute);

    });

    QObject::connect(&mw, &MainWindow::requestLabelCorrection, [&fileListManager, &classes, &folder, &mw, &out, &err, &maskAttribute] () -> void {

        fileListManager.classifyCurrentFile(FileListManager::noClassLabel);

        if (fileListManager.nFiles() == 0) {
            QMessageBox::information(&mw,
                                     QObject::tr("No more point cloud"),
                                     QObject::tr("All file in the class have been corrected!"));
        }

        mw.openPointCloud(fileListManager.currentFilePath(), maskAttribute);

    });

    if (!viewerMode) {
        mw.setPossibleLabels(classes);
    } else {
        mw.configureViewerMode(correctionMode);
    }
    mw.openPointCloud(fileListManager.currentFilePath(), maskAttribute);
    //mw.openDefaultPointCloud();

    mw.show();

    auto startTime = std::chrono::high_resolution_clock::now();

    int code = app.exec();

    auto endTime = std::chrono::high_resolution_clock::now();

    if (timing and !viewerMode) {

        out << "\n" << "Timing infos" << "\n\n";

        auto duration_s = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime);
        out << "Labelling session time: " << duration_s.count() << "s" << "\n";
        out << "Labelled instances: " << nLabeledElements << " instances" << "\n";
        out << "Skipped instances: " << nSkippedElements << " instances" << Qt::endl;
    }

    return code;

}
