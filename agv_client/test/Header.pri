
INCLUDEPATH +=  ..\..\include \
                ..\..\include\CommonLibrary \
                ..\..\include\AGVClientCore \
                ..\..\include\AGVClientNet \
                ..\..\include\AGVClientUIEx

CONFIG(debug, debug|release){
    LIBS += -L../../Lib/ -lCommonLibraryd
    LIBS += -L../../Lib/ -lAGVClientCored
    LIBS += -L../../Lib/ -lAGVClientNetd
    LIBS += -L../../Lib/ -lAGVClientUIExd
}else{
    LIBS += -L../../Lib/ -lCommonLibrary
    LIBS += -L../../Lib/ -lAGVClientCore
    LIBS += -L../../Lib/ -lAGVClientNet
    LIBS += -L../../Lib/ -lAGVClientUIEx
}
