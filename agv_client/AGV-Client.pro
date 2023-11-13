TEMPLATE=subdirs#表示这个是Qt跨目录,由多个子项目(如1个应用程序+多个共享库)组成的大项目

#大项目包含的各个子项目,其中BigWay是应用程序,SmallBoy是共享库
SUBDIRS =   CommonLibrary\
            AGVClientCore\
            AGVClientUIEx\
            AGVClientNet\
            AGVClient
CONFIG+=ordered#CONFIG选项要求各个子项目按顺序编译，子目录的编译顺序在SUBDIRS中指明
