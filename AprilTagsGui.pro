QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = AprilTagsGui
TEMPLATE = app

ICON = APRILTAGS.icns

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += src/mainwindow.cpp\
            src/main.cpp \
            src/Gaussian.cc \
            src/Edge.cc \
            src/FloatImage.cc \
            src/GLine2D.cc \
            src/GLineSegment2D.cc \
            src/GrayModel.cc \
            src/Homography33.cc \
            src/MathUtil.cc \
            src/Quad.cc \
            src/Segment.cc \
            src/TagDetection.cc \
            src/TagDetector.cc \
            src/TagFamily.cc \
            src/UnionFindSimple.cc \
    qcustomplot.cpp

HEADERS  += mainwindow.h \
            AprilTags/Edge.h \
            AprilTags/FloatImage.h \
            AprilTags/Gaussian.h \
            AprilTags/GLine2D.h \
            AprilTags/GLineSegment2D.h \
            AprilTags/GrayModel.h \
            AprilTags/Gridder.h \
            AprilTags/Homography33.h \
            AprilTags/MathUtil.h \
            AprilTags/pch.h \
            AprilTags/Quad.h \
            AprilTags/Segment.h \
            AprilTags/Tag16h5.h \
            AprilTags/Tag16h5_other.h \
            AprilTags/Tag25h7.h \
            AprilTags/Tag25h9.h \
            AprilTags/Tag36h9.h \
            AprilTags/Tag36h11.h \
            AprilTags/Tag36h11_other.h \
            AprilTags/TagDetection.h \
            AprilTags/TagDetector.h \
            AprilTags/TagFamily.h \
            AprilTags/UnionFindSimple.h \
            AprilTags/XYWeight.h \
            qcustomplot.h



FORMS    += mainwindow.ui

#SUBDIRS += Data

LIBS += -L/usr/local/lib
QT_CONFIG -= no-pkg-config
CONFIG  += link_pkgconfig
PKGCONFIG += opencv eigen3

#!contains(QT_CONFIG, no-pkg-config) {
#    CONFIG += link_pkgconfig
#    PKGCONFIG += opencv eigen3
#} else {
#    LIBS += -lopencv_core -lopencv_imgproc -lopencv_objdetect
#}

DISTFILES +=
