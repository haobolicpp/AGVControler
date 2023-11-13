/******************************************************************************
*
*文件名称：JsonUtil.h
*摘    要：json转换的一些通用接口封装
*
******************************************************************************/
#pragma once
#include <qjsondocument.h>
#include <QJsonParseError> 
#include <qjsonobject.h>
#include <qjsonarray.h>
#include <string>
#include "gr_type.h"
#include "commonlibrary_global.h"

class COMMONLIBRARYSHARED_EXPORT CJsonUtil
{
public:
    CJsonUtil();
    ~CJsonUtil();

    //获取字符串节点
    static std::string GetStringNode(QJsonObject jsonObject, std::string strNodeName);
    //获取对象节点
    static QJsonObject GetObjectNode(QJsonObject jsonObject, std::string strNodeName);
    //获取对象数组节点
    static QJsonArray GetArrayNode(QJsonObject jsonObject, std::string strNodeName);
    //获取对象QString节点
    static QString GetQStringNode(QJsonObject jsonObject, std::string strNodeName);

    //新增接口

    /******************************************************************************
    * 函数名称: GetStringNodeValue
    * 功能描述: 获取string字符串节点值
    * 输入参数: jsonObject：传入的JSON数据
    *          strNodeName:节点名称
    * 输出参数: strNodeValue:节点值
    * 返 回 值: GR_TRUE：成功 GR_FALSE:失败
    ******************************************************************************/
    static GR_BOOL GetStringNodeValue(QJsonObject jsonObject, std::string strNodeName, std::string &strNodeValue);

    /******************************************************************************
    * 函数名称: GetObjectNodeValue
    * 功能描述: 获取对象节点值
    * 输入参数: jsonObject：传入的JSON数据
    *          strNodeName:节点名称
    * 输出参数: jsonObjectNodeValue:节点值
    * 返 回 值: GR_TRUE：成功 GR_FALSE:失败
    ******************************************************************************/
    static GR_BOOL GetObjectNodeValue(QJsonObject jsonObject, std::string strNodeName, QJsonObject &jsonObjectNodeValue);

    /******************************************************************************
    * 函数名称: GetArrayNodeValue
    * 功能描述: 获取对象数组节点值
    * 输入参数: jsonObject：传入的JSON数据
    *          strNodeName:节点名称
    * 输出参数: jsonArrayNodeValue:节点值
    * 返 回 值: GR_TRUE：成功 GR_FALSE:失败
    ******************************************************************************/
    static GR_BOOL GetArrayNodeValue(QJsonObject jsonObject, std::string strNodeName, QJsonArray &jsonArrayNodeValue);

    /******************************************************************************
    * 函数名称: GetQStringNodeValue
    * 功能描述: 获取对象QString节点值
    * 输入参数: jsonObject：传入的JSON数据
    *          strNodeName:节点名称
    * 输出参数: qstrNodeValue:节点值
    * 返 回 值: GR_TRUE：成功 GR_FALSE:失败
    ******************************************************************************/
    static GR_BOOL GetQStringNodeValue(QJsonObject jsonObject, std::string strNodeName, QString &qstrNodeValue);

    /******************************************************************************
    * 函数名称: GetBoolNodeValue
    * 功能描述: 获取bool节点值
    * 输入参数: jsonObject：传入的JSON数据
    *          strNodeName:节点名称
    * 输出参数: boolNodeValue:节点值
    * 返 回 值: GR_TRUE：成功 GR_FALSE:失败
    ******************************************************************************/
    static GR_BOOL GetBoolNodeValue(QJsonObject jsonObject, std::string strNodeName, GR_BOOL &boolNodeValue);

    /******************************************************************************
    * 函数名称: GetIntNodeValue
    * 功能描述: 获取int节点值
    * 输入参数: jsonObject：传入的JSON数据
    *          strNodeName:节点名称
    * 输出参数: intNodeValue:节点值
    * 返 回 值: GR_TRUE：成功 GR_FALSE:失败
    ******************************************************************************/
    static GR_BOOL GetIntNodeValue(QJsonObject jsonObject, std::string strNodeName, GR_INT &intNodeValue);

    /******************************************************************************
    * 函数名称: GetS16NodeValue
    * 功能描述: 获取short节点值
    * 输入参数: jsonObject：传入的JSON数据
    *          strNodeName:节点名称
    * 输出参数: s16NodeValue:节点值
    * 返 回 值: GR_TRUE：成功 GR_FALSE:失败
    ******************************************************************************/
    static GR_BOOL GetS16NodeValue(QJsonObject jsonObject, std::string strNodeName, GR_S16 &s16NodeValue);

    /******************************************************************************
    * 函数名称: GetU16NodeValue
    * 功能描述: 获取unsigned short节点值
    * 输入参数: jsonObject：传入的JSON数据
    *          strNodeName:节点名称
    * 输出参数: u16NodeValue:节点值
    * 返 回 值: GR_TRUE：成功 GR_FALSE:失败
    ******************************************************************************/
    static GR_BOOL GetU16NodeValue(QJsonObject jsonObject, std::string strNodeName, GR_U16 &u16NodeValue);

    /******************************************************************************
    * 函数名称: GetUnsignedIntNodeValue
    * 功能描述: 获取unsigned int节点值
    * 输入参数: jsonObject：传入的JSON数据
    *          strNodeName:节点名称
    * 输出参数: uiNodeValue:节点值
    * 返 回 值: GR_TRUE：成功 GR_FALSE:失败
    ******************************************************************************/
    static GR_BOOL GetUnsignedIntNodeValue(QJsonObject jsonObject, std::string strNodeName, GR_UINT &uiNodeValue);

    /******************************************************************************
    * 函数名称: GetDoubleNodeValue
    * 功能描述: 获取double节点值
    * 输入参数: jsonObject：传入的JSON数据
    *          strNodeName:节点名称
    * 输出参数: doubleNodeValue:节点值
    * 返 回 值: GR_TRUE：成功 GR_FALSE:失败
    ******************************************************************************/
    static GR_BOOL GetDoubleNodeValue(QJsonObject jsonObject, std::string strNodeName, GR_DOUBLE &doubleNodeValue);
};


