QT -= gui

TEMPLATE = lib
DEFINES += AGVCLIENTNET_LIBRARY

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

include(../Header.pri)

INCLUDEPATH += ../3rdparty/libuv/include


HEADERS += \
    ../include/AGVClientNet/AGVClientNet_global.h \
    ../AGVClientNet/AGVDiscovery.h \
    ../include/AGVClientNet/NetManager.h \
    ../AGVClientNet/TcpClientServer.h \
    ../include/AGVClientNet/agv_msg_def.h

# Default rules for deployment.
unix {
    target.path = /usr/lib
}
!isEmpty(target.path): INSTALLS += target

SOURCES += \
    AGVDiscovery.cpp \
    NetManager.cpp \
    TcpClientServer.cpp

CONFIG(debug, debug|release){
    LIBS += -L../Bin/ -lCommonLibraryd
    LIBS += -L../Bin/ -luvd
    LIBS += -L../Bin/ -lAGVClientCored
    LIBS += -L../Bin/ -lglogd
}else{
    LIBS += -L../Bin/ -lCommonLibrary
    LIBS += -L../Bin/ -luv
    LIBS += -L../Bin/ -lAGVClientCore
    LIBS += -L../Bin/ -lglog
}

LIBS += -lws2_32
