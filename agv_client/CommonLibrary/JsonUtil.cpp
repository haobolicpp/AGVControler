/******************************************************************************
*
*文件名称：JsonUtil.cpp
*摘    要：json转换的一些通用接口封装
*
*
******************************************************************************/
#include "JsonUtil.h"

CJsonUtil::CJsonUtil()
{
}

CJsonUtil::~CJsonUtil()
{
}

std::string CJsonUtil::GetStringNode(QJsonObject jsonObject, std::string strNodeName)
{
    if (jsonObject.contains(strNodeName.c_str()))
    {
        QJsonValue value = jsonObject.value(strNodeName.c_str());
        if (value.isString())
        {
            return value.toString().toLocal8Bit().data();
        }
    }
    return "";
}

QJsonObject CJsonUtil::GetObjectNode(QJsonObject jsonObject, std::string strNodeName)
{
    QJsonObject outJsonObject;
    if (jsonObject.contains(strNodeName.c_str()))
    {
        QJsonValue value = jsonObject.value(strNodeName.c_str());
        if (value.isObject())
        {
            outJsonObject = value.toObject();
        }
    }
    return outJsonObject;
}

QJsonArray CJsonUtil::GetArrayNode(QJsonObject jsonObject, std::string strNodeName)
{
    QJsonArray jsonArray;
    if (jsonObject.contains(strNodeName.c_str()))
    {
        QJsonValue value = jsonObject.value(strNodeName.c_str());
        if (value.isArray())
        {
            jsonArray = value.toArray();
        }
    }
    return jsonArray;
}

QString CJsonUtil::GetQStringNode(QJsonObject jsonObject, std::string strNodeName)
{
    if (jsonObject.contains(strNodeName.c_str()))
    {
        QJsonValue value = jsonObject.value(strNodeName.c_str());
        if (value.isString())
        {
            return value.toString();
        }
    }
    return "";
}

//获取string字符串节点值
GR_BOOL CJsonUtil::GetStringNodeValue(QJsonObject jsonObject, std::string strNodeName, std::string &strNodeValue)
{
    GR_BOOL bRet = GR_FALSE;
    if (jsonObject.contains(strNodeName.c_str()))
    {
        QJsonValue value = jsonObject.value(strNodeName.c_str());
        if (value.isString())
        {
            strNodeValue = value.toString().toLocal8Bit().data();
            bRet = GR_TRUE;
        }
    }
    return bRet;
}

//获取对象节点值
GR_BOOL CJsonUtil::GetObjectNodeValue(QJsonObject jsonObject, std::string strNodeName, QJsonObject &jsonObjectNodeValue)
{
    GR_BOOL bRet = GR_FALSE;
    if (jsonObject.contains(strNodeName.c_str()))
    {
        QJsonValue value = jsonObject.value(strNodeName.c_str());
        if (value.isObject())
        {
            jsonObjectNodeValue = value.toObject();
            bRet = GR_TRUE;
        }
    }
    return bRet;
}

//获取对象数组节点值
GR_BOOL CJsonUtil::GetArrayNodeValue(QJsonObject jsonObject, std::string strNodeName, QJsonArray &jsonArrayNodeValue)
{
    GR_BOOL bRet = GR_FALSE;
    if (jsonObject.contains(strNodeName.c_str()))
    {
        QJsonValue value = jsonObject.value(strNodeName.c_str());
        if (value.isArray())
        {
            jsonArrayNodeValue = value.toArray();
            bRet = GR_TRUE;
        }
    }
    return bRet;
}

//获取对象QString节点值
GR_BOOL CJsonUtil::GetQStringNodeValue(QJsonObject jsonObject, std::string strNodeName, QString &qstrNodeValue)
{
    GR_BOOL bRet = GR_FALSE;
    if (jsonObject.contains(strNodeName.c_str()))
    {
        QJsonValue value = jsonObject.value(strNodeName.c_str());
        if (value.isString())
        {
            qstrNodeValue = value.toString();
            bRet = GR_TRUE;
        }
    }
    return bRet;
}

//获取unsigned int节点值
GR_BOOL CJsonUtil::GetUnsignedIntNodeValue(QJsonObject jsonObject, std::string strNodeName, GR_UINT &uiNodeValue)
{
    GR_BOOL bRet = GR_FALSE;
    if (jsonObject.contains(strNodeName.c_str()))
    {
        QJsonValue value = jsonObject.value(strNodeName.c_str());
        if(value.isString())
        {
            uiNodeValue = (GR_UINT)value.toString().toDouble();
            bRet = GR_TRUE;
        }
        if(value.isDouble())
        {
            uiNodeValue = (GR_UINT)value.toInt();
            bRet = GR_TRUE;
        }
    }
    return bRet;                                                                                                           
}

//获取bool节点值
GR_BOOL CJsonUtil::GetBoolNodeValue(QJsonObject jsonObject, std::string strNodeName, GR_BOOL &boolNodeValue)
{
    GR_BOOL bRet = GR_FALSE;
    if (jsonObject.contains(strNodeName.c_str()))
    {
        QJsonValue value = jsonObject.value(strNodeName.c_str());
        if (value.isBool())
        {
            boolNodeValue = value.toBool();
            bRet = GR_TRUE;
        }
        if(value.isString())
        {
            GR_INT iRet = (GR_INT)value.toString().toDouble();
            if(0 == iRet)
            {
                boolNodeValue = GR_FALSE;
            }
            else
            {
                boolNodeValue = GR_TRUE;
            }
            bRet = GR_TRUE;
        }
    }
    return bRet;
}

//获取int节点值
GR_BOOL CJsonUtil::GetIntNodeValue(QJsonObject jsonObject, std::string strNodeName, GR_INT &intNodeValue)
{
    GR_BOOL bRet = GR_FALSE;
    if (jsonObject.contains(strNodeName.c_str()))
    {
        QJsonValue value = jsonObject.value(strNodeName.c_str());
        if(value.isString())
        {
            intNodeValue = (GR_INT)value.toString().toDouble();
            bRet = GR_TRUE;
        }
        if(value.isDouble())
        {
            intNodeValue = value.toInt();
            bRet = GR_TRUE;
        }
    }
    return bRet;
}

//获取short节点值
GR_BOOL CJsonUtil::GetS16NodeValue(QJsonObject jsonObject, std::string strNodeName, GR_S16 &s16NodeValue)
{
    GR_BOOL bRet = GR_FALSE;
    if (jsonObject.contains(strNodeName.c_str()))
    {
        QJsonValue value = jsonObject.value(strNodeName.c_str());
        if(value.isString())
        {
            s16NodeValue = (GR_S16)value.toString().toDouble();
            bRet = GR_TRUE;
        }
        if(value.isDouble())
        {
            s16NodeValue = (GR_S16)value.toInt();
            bRet = GR_TRUE;
        }
    }
    return bRet;
}

//获取unsigned short节点值
GR_BOOL CJsonUtil::GetU16NodeValue(QJsonObject jsonObject, std::string strNodeName, GR_U16 &u16NodeValue)
{
    GR_BOOL bRet = GR_FALSE;
    if (jsonObject.contains(strNodeName.c_str()))
    {
        QJsonValue value = jsonObject.value(strNodeName.c_str());
        if(value.isString())
        {
            u16NodeValue = (GR_U16)value.toString().toDouble();
            bRet = GR_TRUE;
        }
        if(value.isDouble())
        {
            u16NodeValue = (GR_U16)value.toInt();
            bRet = GR_TRUE;
        }
    }
    return bRet;
}

//获取double节点值
GR_BOOL CJsonUtil::GetDoubleNodeValue(QJsonObject jsonObject, std::string strNodeName, GR_DOUBLE &doubleNodeValue)
{
    GR_BOOL bRet = GR_FALSE;
    if (jsonObject.contains(strNodeName.c_str()))
    {
        QJsonValue value = jsonObject.value(strNodeName.c_str());
        if (value.isDouble())
        {
            doubleNodeValue = value.toDouble();
            bRet = GR_TRUE;
        }
        if(value.isString())
        {
            doubleNodeValue = value.toString().toDouble();
            bRet = GR_TRUE;
        }
    }
    return bRet;
}
