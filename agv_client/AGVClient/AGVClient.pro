QT       += core gui xml serialport
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = AGVClient
TEMPLATE = app
DEFINES += QT_DEPRECATED_WARNINGS
include(../Header.pri)

SOURCES += \
        AGVListWindow/AGVListWindow.cpp \
        AGVWindow/AGVWidget.cpp \
        AGVWindow/ManualControl.cpp \
        AGVWindow/PropertyWidget.cpp \
        AGVWindow/UpMapWidget.cpp \
        CNameHelper.cpp \
        Commands/CmdAddLine.cpp \
        Commands/CmdAddStation.cpp \
        Commands/CmdBase.cpp \
        Commands/CmdDelete.cpp \
        Commands/CmdEraseMap.cpp \
        Commands/CmdModifyStation.cpp \
        Commands/CommandManager.cpp \
        Commands/CmdModifyLine.cpp \
        Commands/CmdMoveStations.cpp \
        MapView/AgvItem.cpp \
        MapView/CursorFactory.cpp \
        MapView/GlobalPathItem.cpp \
        MapView/LineItem.cpp \
        MapView/MapGraphicsScene.cpp \
        MapView/MapGraphicsView.cpp \
        MapView/MapItem.cpp \
        MapView/StationItem.cpp \
        MapView/StationTextItem.cpp \
        WidgetBase.cpp \
        main.cpp \
        mainwindow.cpp

HEADERS += \
        AGVListWindow/AGVListWindow.h \
        AGVWindow/AGVWidget.h \
        AGVWindow/ManualControl.h \
        AGVWindow/PropertyWidget.h \
        AGVWindow/UpMapWidget.h \
        CNameHelper.h \
        Commands/CmdAddLine.h \
        Commands/CmdAddStation.h \
        Commands/CmdBase.h \
        Commands/CmdDelete.h \
        Commands/CmdEraseMap.h \
        Commands/CmdModifyLine.h \
        Commands/CmdModifyStation.h \
        Commands/CommandManager.h \
        Commands/CmdMoveStations.h \
        MapView/AgvItem.h \
        MapView/CursorFactory.h \
        MapView/GlobalPathItem.h \
        MapView/LineItem.h \
        MapView/MapGraphicsScene.h \
        MapView/MapGraphicsView.h \
        MapView/MapItem.h \
        MapView/MapView_def.h \
        MapView/StationItem.h \
        MapView/StationTextItem.h \
        WidgetBase.h \
        mainwindow.h \
        ../include/AGVClient_def.h

FORMS += \
        AGVListWindow/AGVListWindow.ui \
        AGVWindow/AGVWidget.ui \
        AGVWindow/ManualControl.ui \
        AGVWindow/PropertyWidget.ui \
        AGVWindow/UpMapWidget.ui \
        mainwindow.ui 

RESOURCES += \
    AGVClient.qrc \

#包含的lib
CONFIG(debug, debug|release){
    LIBS += -L../Lib/ -lCommonLibraryd
    LIBS += -L../Lib/ -lAGVClientCored
    LIBS += -L../Lib/ -lAGVClientNetd
    LIBS += -L../Lib/ -lAGVClientUIExd
    LIBS += -L../Bin/ -lglogd
}else{
    LIBS += -L../Lib/ -lCommonLibrary
    LIBS += -L../Lib/ -lAGVClientCore
    LIBS += -L../Lib/ -lAGVClientNet
    LIBS += -L../Lib/ -lAGVClientUIEx
    LIBS += -L../Bin/ -lglog
}


RC_ICONS = agv_client.ico
