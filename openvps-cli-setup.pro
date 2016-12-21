QT += core
QT -= gui

CONFIG += c++11

TARGET = openvps-cli-config
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += \
    setup.cpp

INCLUDEPATH += C:/Users/Martin/lib/opencv-3.1.0/sources/build/install/include

#LIBS += C:/Users/Martin/lib/opencv-3.1.0/build/x64/vc14/lib/opencv_world310.lib

LIBS += -L"C:/Users/Martin/lib/opencv-3.1.0/sources/build/install/x86/mingw/bin" -lws2_32

LIBS += -lopencv_calib3d310 -lopencv_core310 -lopencv_features2d310 -lopencv_flann310 \
-lopencv_flann310 -lopencv_highgui310 -lopencv_imgcodecs310 -lopencv_imgproc310 \
-lopencv_ml310 -lopencv_video310 -lopencv_videoio310 -lopencv_aruco310

#LIBS += -lopencv_calib3d -lopencv_core -lopencv_features2d -lopencv_flann \
#-lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_ml -lopencv_objdetect \
#-lopencv_photo -lopencv_shape -lopencv_stitching -lopencv_superres -lopencv_video \
#-lopencv_videoio -lopencv_videostab -lopencv_viz -lopencv_aruco -lopencv_bgsegm \
#-lopencv_bioinspired -lopencv_ccalib -lopencv_cvv -lopencv_datasets -lopencv_dnn \
#-lopencv_dpm -lopencv_face -lopencv_fuzzy -lopencv_line_descriptor -lopencv_optflow \
#-lopencv_plot -lopencv_reg -lopencv_rgbd -lopencv_saliency -lopencv_stereo \
#-lopencv_structured_light -lopencv_surface_matching -lopencv_text -lopencv_tracking \
#-lopencv_xfeatures2d -lopencv_ximgproc -lopencv_xobjdetect -lopencv_xphoto

HEADERS += \
    vpsstorage.h vpsmath.h
