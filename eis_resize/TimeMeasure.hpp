/*********��Ȩ���У�C��2013���人�ߵº���ɷ����޹�˾***************
* �ļ����ƣ�timeMeasure.cpp
* �ļ���ʶ�����������
* ��ǰ�汾��V1.0
* �������ߣ�л����
* �������ڣ�2018��2��5��
*
*******************************************************************/

/************************ʹ�÷���*****************************

1.��¼һ��ʱ��
	TimeMeasure tm;
	tm.recordTime("flag");

	do something;

	//��ȡ��������ʱ��
	double cost_t = tm.diffMsWithRecord("flag");
	//ֱ�����string����ʱ��  ---- ������  flag : xxxms
	std::cout << tm.diffMsStringWithRecord("flag");


2.��¼����ʱ��
	TimeMeasure tm;
	
	tm.recordTime("flag");
	DO1;
	//ֱ�����string����ʱ��  ---- ������  flag : xxxms
	std::cout << tm.diffMsStringWithRecord("flag");

	tm.recordTime("flag2");
	DO2;
	//ֱ�����string����ʱ��  ---- ������  flag2 : xxxms
	std::cout << tm.diffMsStringWithRecord("flag");


	tm.recordTime("flag3");
	tm.recordTime("flag34");
	DO3;
	//��ͣʱ��
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
