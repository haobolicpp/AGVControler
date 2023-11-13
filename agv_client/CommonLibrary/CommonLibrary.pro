#-------------------------------------------------
#
# Project created by QtCreator 2018-03-20T19:07:08
#
#-------------------------------------------------

QT       += core gui xml
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = CommonLibrary
TEMPLATE = lib
DEFINES += COMMONLIBRARY_LIBRARY
DEFINES += QT_DEPRECATED_WARNINGS
CONFIG += C++11
include(../Header.pri)

SOURCES += \
    AutoIncBuffer.cpp \
    ConfigManager.cpp \
    GeometryAlgorithm.cpp \
    GlogWrapper.cpp \
    GoerTimeSpan.cpp \
    GoerToolUtile.cpp \
    JsonUtil.cpp \
    LockFreeRingBuffer.cpp

HEADERS += \
    ../include/CommonLibrary/AutoIncBuffer.h \
    ../include/CommonLibrary/ConfigManager.h \
    ../include/CommonLibrary/GoerLockQueue.h \
    ../include/CommonLibrary/GoerTimeSpan.h \
    ../include/CommonLibrary/GoerToolUtile.h \
    ../include/CommonLibrary/JsonUtil.h \
    ../include/CommonLibrary/LockFreeRingBuffer.h \
    ../include/SingletonMacro_Def.h \
    ../include/CommonLibrary/GeometryAlgorithm.h \
    ../include/CommonLibrary/GlogWrapper.h

INCLUDEPATH += ..\include \
               ..\include\CommonLibrary

#包含的lib
CONFIG(debug, debug|release){
    LIBS += -L../Bin/ -lglogd
}else{
    LIBS += -L../Bin/ -lglog
}
