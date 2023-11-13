/******************************************************************************
*
*文件名称：GoerToolUtile.h
*摘    要：工具类，提供一些通用的接口
*
*
******************************************************************************/
#ifndef _GOERTOOLUTILE_H
#define _GOERTOOLUTILE_H
#include <QString>
#include <QStringList>
#include "gr_type.h"
#include "commonlibrary_global.h"

class COMMONLIBRARYSHARED_EXPORT CGoerToolUtile
{
public:
    CGoerToolUtile(void);
    ~CGoerToolUtile(void);

    /******************************************************************************
    * 函数名称: GetExeDirectory
    * 功能描述: 获取exe所在目录
    * 输出参数:
    * 返 回 值: QString &strDir：输出目录
    *
    ******************************************************************************/
    static QString GetExeDirectory();

    /******************************************************************************
    * 函数名称: GetExeUpDirectory
    * 功能描述: 获取exe的上n层目录
    * 输出参数: QString &strDir：输出目录
    * 返 回 值: GR_BOOL，GR_TRUE成功。 
    *
    ******************************************************************************/
    static GR_BOOL GetExeUpDirectory(QString &strDir, GR_INT iForntNum);

    /******************************************************************************
        * 函数名称: GetFileNameFromDirectory
        * 功能描述: 从指定的目录下，返回该目录下的文件（不会递归到子目录）
        * 输出参数: 无
        * 输入参数: QString strDirectory：指定的目录
        *          QStringList strNameFilters：指定过滤的文件类型，如
        *                   QStringList filters;
        *                   filters << "*.cpp" << "*.cxx" << "*.cc";
        *           为空则过滤所有。
        * 返 回 值: QStringList，返回文件名称列表，包括后缀名
        *
        ******************************************************************************/
        static QStringList GetFileNameFromDirectory(QString strDirectory, QStringList strNameFilters);

    /******************************************************************************
    * 函数名称: GetCurrentTimeMSec/GetCurrentTimeSec
    * 功能描述: 返回当前系统时间(从1970经过的毫秒/秒)
    * 输出参数: 无
    * 返 回 值: GR_INT 最大上限为03:14:07 January 19, 2038
    ******************************************************************************/
    static GR_INT GetCurrentTimeSec();//秒
    static GR_S64 GetCurrentTimeMSec();//毫秒

    /**
     * @brief HexToString : 十六进制串转字符串
     * @param pbuffer
     * @param ilen
     * @return
     */
    static QString HexToString(char *pbuffer, int ilen);

};

#endif
