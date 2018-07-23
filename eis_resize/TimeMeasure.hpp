/*********版权所有（C）2013，武汉高德红外股份有限公司***************
* 文件名称：timeMeasure.cpp
* 文件标识：稳像库库入口
* 当前版本：V1.0
* 创建作者：谢伯勇
* 创建日期：2018年2月5日
*
*******************************************************************/

/************************使用方法*****************************

1.记录一组时间
	TimeMeasure tm;
	tm.recordTime("flag");

	do something;

	//获取浮点类型时间
	double cost_t = tm.diffMsWithRecord("flag");
	//直接输出string类型时间  ---- 输出结果  flag : xxxms
	std::cout << tm.diffMsStringWithRecord("flag");


2.记录多组时间
	TimeMeasure tm;
	
	tm.recordTime("flag");
	DO1;
	//直接输出string类型时间  ---- 输出结果  flag : xxxms
	std::cout << tm.diffMsStringWithRecord("flag");

	tm.recordTime("flag2");
	DO2;
	//直接输出string类型时间  ---- 输出结果  flag2 : xxxms
	std::cout << tm.diffMsStringWithRecord("flag");


	tm.recordTime("flag3");
	tm.recordTime("flag34");
	DO3;
	//暂停时间
	tm.pauseRecord("flag3");
	tm.recordTime("flag45");
	DO4;
	tm.pauseRecord("flag34");
	DO5;
	
	std::cout << tm.diffMsStringWithRecord("flag3") << tm.diffMsStringWithRecord("flag34") << tm.diffMsStringWithRecord("flag45");
	
******************************************************************/

#ifndef _TIMEMEASURE_HPP_
#define _TIMEMEASURE_HPP_

#include <iostream>
#include <string>
#include <map>
#include <sys/time.h>



class TimeMeasure {
public:

    void    recordTime(std::string flag);
    double  pauseRecord(std::string flag);
    double  getTimeWithRecord(std::string flag);
    double  diffMsWithRecord(std::string flag);
    //
    //  return like this  : if flag is "cost time", time is 1ms, will return "cost time : 1ms";
    //
    std::string diffMsStringWithRecord(std::string flag);
    void    removeRecord(std::string);
    void    clear();
private:
    double  calcCostTime(std::string flag);

    std::map<std::string, struct timeval> record;
    std::map<std::string, double> costTime;
};

#endif
