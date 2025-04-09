cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0095 NEW)

include(CheckCXXCompilerFlag)
project(SimplePointCloudLabeller LANGUAGES C CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fopenmp -march=native -ftree-vectorize")

find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets Core Test REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets Core Test REQUIRED)

find_package(PCL 1.3 REQUIRED)

set(PROJ_SRC  main.cpp
    ressources.qrc
    pointsdisplaywidget.h pointsdisplaywidget.cpp
    mainwindow.h mainwindow.cpp
    filelistmanager.h filelistmanager.cpp
    )

add_executable(simplePointCloudLabeller
    ${PROJ_SRC}
)

target_link_libraries(simplePointCloudLabeller PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Widgets ${PCL_LIBRARIES})

install(TARGETS simplePointCloudLabeller RUNTIME DESTINATION usr/bin)
install(FILES simplePointCloudLabeller.desktop DESTINATION usr/share/applications)
install(FILES icon/simplePointCloudLabeller.png DESTINATION .)
