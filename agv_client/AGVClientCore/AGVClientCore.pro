QT -= gui

TEMPLATE = lib
DEFINES += AGVCLIENTCORE_LIBRARY

CONFIG += c++11

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


# Default rules for deployment.
unix {
    target.path = /usr/lib
}
!isEmpty(target.path): INSTALLS += target
include(../Header.pri)

HEADERS += \
    ../include/AGVClientCore/AGVClientCore_global.h \
    ../include/AGVClientCore/AGVManager.h \
    ../include/AGVClientCore/MapManager.h \
    ../include/AGVClientCore/CoreControl.h


SOURCES += \
    AGVManager.cpp \
    CoreControlontrol.cpp \
    MapManager.cpp

#包含的lib
CONFIG(debug, debug|release){
    LIBS += -L../Bin/ -lCommonLibraryd
    LIBS += -L../Bin/ -lglogd
    LIBS += -L../Bin/ -lb64c
}else{
    LIBS += -L../Bin/ -lCommonLibrary
    LIBS += -L../Bin/ -lglog
    LIBS += -L../Bin/ -lb64c
}
