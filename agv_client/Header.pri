
DESTDIR = ../Bin/         #应用程序放在bin目录

contains(TEMPLATE, "lib") {
    DESTDIR = ../lib        #将库放在lib文件夹下
    DLLDESTDIR = ../Bin/     #将动态库自动拷贝至bin目录下
}

CONFIG(debug, debug|release){
    TARGET = $$join(TARGET,,,d) #修改目标名称，debug下后缀添加d
}

INCLUDEPATH +=  ..\include \
                ..\include\CommonLibrary \
                ..\include\AGVClientCore \
                ..\include\AGVClientNet \
                ..\include\AGVClientUIEx

