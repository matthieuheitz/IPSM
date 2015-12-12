#-------------------------------------------------
#
# Project created by QtCreator 2015-12-03T14:36:34
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = IPSM
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    TensorField.cpp \
    StreetGraph.cpp

HEADERS  += mainwindow.h \
    TensorField.h \
    StreetGraph.h

FORMS    += mainwindow.ui
