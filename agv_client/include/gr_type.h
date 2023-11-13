/******************************************************************************
*
* 文件名称：gr_type.h
* 摘    要：定义规范的通用数据类型
*
******************************************************************************/
#pragma once

/*----------------------------------------------*
 * 数据类型定义                                 *
 *----------------------------------------------*/
typedef signed char         GR_S8;
typedef unsigned char       GR_U8;

typedef signed short int    GR_S16;
typedef unsigned short int  GR_U16;

typedef signed long int     GR_S32;
typedef unsigned long int   GR_U32;

typedef signed long long    GR_S64;
typedef unsigned long long  GR_U64;

typedef signed int          GR_INT;
typedef unsigned int        GR_UINT;

typedef long                GR_LONG;
typedef unsigned long       GR_UL;

typedef unsigned char       GR_UCHAR;
typedef unsigned char       UCHAR;
typedef char                GR_CHAR;
typedef char*               GR_PCHAR;

typedef float               GR_FLOAT;
typedef double              GR_DOUBLE;

typedef int                GR_BOOL;

typedef unsigned long int   GR_HANDLE;

#define GR_VOID     void


#define GR_NULL     (0U)
//#define GR_NULL     nullptr

#define GR_FALSE    (0 == 1)
#define GR_TRUE     (!(GR_FALSE))

#define GR_SUCCESS  (0)
#define GR_FAILURE  (-1)

#ifndef MIN
#define MIN(a,b)    (((a) < (b)) ? (a) : (b)) /* Choose minimum value */
#endif

#ifndef MAX
#define MAX(a,b)    (((a) > (b)) ? (a) : (b)) /* Choose maximum value */
#endif


#ifndef PI      //圆周率,float保留7有效数，算上小数点前面的；
#define PI 3.141592653589793
#endif

#ifndef arcunit //弧度转换为度数乘以该值
#define arcunit 57.295779513082
#endif

#ifndef EPSILON
#define EPSILON 0.0001
#endif

#ifndef DOUBLE_VALUE_MAX 
#define DOUBLE_VALUE_MAX 1e7
#endif

#ifndef DOUBLE_VALUE_MIN 
#define DOUBLE_VALUE_MIN -1e7
#endif

#ifndef LINEDEG_VALUE_MIN 
#define LINEDEG_VALUE_MIN  0
#endif

#ifndef LINEDEG_VALUE_MAX 
#define LINEDEG_VALUE_MAX 180
#endif

//删除内存操作
#define DELETE_PTR(ptr) \
    if (ptr != GR_NULL) \
    { \
        delete ptr; \
        ptr = GR_NULL; \
    } \

#define DELETE_PTRA(ptr) \
    if (ptr != GR_NULL) \
    { \
        delete []ptr; \
        ptr = GR_NULL; \
    } \


