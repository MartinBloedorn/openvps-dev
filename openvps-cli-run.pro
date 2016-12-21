QT += core
QT -= gui

CONFIG += c++11

TARGET = openvps-cli-run
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += vpsrun.cpp

INCLUDEPATH += C:/Users/Martin/lib/opencv-3.1.0/sources/build/install/include

#LIBS += C:/Users/Martin/lib/opencv-3.1.0/build/x64/vc14/lib/opencv_world310.lib

LIBS += -L"C:/Users/Martin/lib/opencv-3.1.0/sources/build/install/x86/mingw/bin" -lws2_32

LIBS += -lopencv_calib3d310 -lopencv_core310 -lopencv_features2d310 -lopencv_flann310 \
-lopencv_flann310 -lopencv_highgui310 -lopencv_imgcodecs310 -lopencv_imgproc310 \
-lopencv_ml310 -lopencv_video310 -lopencv_videoio310 -lopencv_aruco310


HEADERS += \
    vpsstorage.h vpssocket.h vpsmath.h
