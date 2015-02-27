#-------------------------------------------------
#
# Project created by QtCreator 2014-10-19T11:02:44
#
#-------------------------------------------------

QT       += core gui widgets serialport declarative

RC_FILE = myapp.rc

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Serial
TEMPLATE = app


SOURCES += main.cpp\
        dialog.cpp

    masterthread.cpp
HEADERS  += dialog.h

FORMS    += dialog.ui

OTHER_FILES += \
    HST.png
