/******************************************************************************
*
*文件名称：SingletonMacro_Def.h
*摘    要：声明单例类公用的宏声明-
*
>>添加单例对象全局收集器，程序退出时，自动删除所有的单例对象。
>>定义了一个全局静态收集器，包含该文件的cpp文件中会独有一份，也就是有多少单例就有多少收集对象。
>>修改单例定义宏中，GetInstance()实现，调用过后加入全局收集器。
>>添加单例类的共同基类：CSingleObject,只有从该基类派生，才能实现自动回收
>>删除Destroy接口
>>注意：单例基类派生的单例类，不能在头文件中包含使用。


>>由于全局对象释放时机不可控，当两个单例有依赖时，非常容易出问题，删除全局收集器

>>添加Destory()接口，统一释放资源的接口
*
******************************************************************************/
#ifndef _SINGLETON_MACRO_H_
#define _SINGLETON_MACRO_H_
#include <vector>
using namespace std;

//类中添加声明
#define SINGLETON_DECLARE(Class) \
public:\
    static Class *GetInstance();\
    void Destory();\
private:\
    static Class *m_pInstance;

//cpp中添加实现
#define SINGLETON_IMPLEMENT(Class) \
    Class * Class::m_pInstance = new Class;\
    Class * Class::GetInstance()\
{\
    return m_pInstance;\
}\


#endif
