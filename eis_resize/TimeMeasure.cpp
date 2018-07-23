/*********版权所有（C）2013，武汉高德红外股份有限公司***************
* 文件名称：timeMeasure.cpp
* 文件标识：稳像库库入口
* 当前版本：V1.0
* 创建作者：谢伯勇
* 创建日期：2018年2月5日
*******************************************************************/

#include "TimeMeasure.hpp"

/**********************************************************
* 函数名称：calcCostTime()
* 功能描述：计算flag标记的代码耗时
* 输入参数：std::string flag    			--  标记
* 输出参数：无
* 返 回 值：无
**********************************************************/
double TimeMeasure::calcCostTime(std::string flag)
{
    if (record.find(flag) == record.end()) return -1.0;
    struct timeval now;
    gettimeofday(&now, NULL);
    struct timeval ago = record[flag];
    double time = (now.tv_sec-ago.tv_sec) * 1000.0 + (now.tv_usec-ago.tv_usec) / 1000.0;
    costTime[flag] = time;
    return time;
}

/**********************************************************
* 函数名称：recordTime()
* 功能描述：开始记录flag标记的时间
* 输入参数：std::string flag    			--  标记
* 输出参数：无
* 返 回 值：无
**********************************************************/
void TimeMeasure::recordTime(std::string flag)
{
    struct timeval time;
    gettimeofday(&time, NULL);
    if (costTime.find(flag) != costTime.end()) 
	{
        costTime.erase(flag);
    }
    record[flag] = time;
}

/**********************************************************
* 函数名称：diffMsWithRecord()
* 功能描述：返回flag标记的程序执行时间
* 输入参数：std::string flag    			--  标记
* 输出参数：无
* 返 回 值：无
**********************************************************/
double TimeMeasure::diffMsWithRecord(std::string flag)
{
    return getTimeWithRecord(flag);
}

/**********************************************************
* 函数名称：diffMsWithRecord()
* 功能描述：返回并暂停flag标记的程序执行时间
* 输入参数：std::string flag    			--  标记
* 输出参数：无
* 返 回 值：无
**********************************************************/
double TimeMeasure::pauseRecord(std::string flag)
{
    if (costTime.find(flag) == costTime.end())
        return calcCostTime(flag);
    else
        return costTime[flag];
}

/**********************************************************
* 函数名称：getTimeWithRecord()
* 功能描述：获取flag标记的程序执行时间   	
* 输入参数：std::string flag    			--  标记
* 输出参数：无
* 返 回 值：无
**********************************************************/
double TimeMeasure::getTimeWithRecord(std::string flag)
{
    if (costTime.find(flag) == costTime.end())
        return pauseRecord(flag);
    else
        return costTime[flag];
}

/**********************************************************
* 函数名称：diffMsStringWithRecord()
* 功能描述：字符串类型返回flag标记的程序执行时间
* 输入参数：std::string flag    			--  标记
* 输出参数：无
* 返 回 值：无
**********************************************************/
std::string TimeMeasure::diffMsStringWithRecord(std::string flag)
{
    double ms = diffMsWithRecord(flag);
    return flag + " : " + std::to_string(ms) + "ms";
}

/**********************************************************
* 函数名称：removeRecord()
* 功能描述：移除flag标记的程序执行时间
* 输入参数：std::string flag    			--  标记
* 输出参数：无
* 返 回 值：无
**********************************************************/
void TimeMeasure::removeRecord(std::string flag)
{
    record.erase(flag);
    costTime.erase(flag);
}

/**********************************************************
* 函数名称：clear()
* 功能描述：清除所有记录
* 输入参数：std::string flag    			--  标记
* 输出参数：无
* 返 回 值：无
**********************************************************/
void TimeMeasure::clear()
{
    record.clear();
    costTime.clear();
}

