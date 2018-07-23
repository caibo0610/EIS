/*********��Ȩ���У�C��2013���人�ߵº���ɷ����޹�˾***************
* �ļ����ƣ�timeMeasure.cpp
* �ļ���ʶ�����������
* ��ǰ�汾��V1.0
* �������ߣ�л����
* �������ڣ�2018��2��5��
*******************************************************************/

#include "TimeMeasure.hpp"

/**********************************************************
* �������ƣ�calcCostTime()
* ��������������flag��ǵĴ����ʱ
* ���������std::string flag    			--  ���
* �����������
* �� �� ֵ����
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
* �������ƣ�recordTime()
* ������������ʼ��¼flag��ǵ�ʱ��
* ���������std::string flag    			--  ���
* �����������
* �� �� ֵ����
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
* �������ƣ�diffMsWithRecord()
* ��������������flag��ǵĳ���ִ��ʱ��
* ���������std::string flag    			--  ���
* �����������
* �� �� ֵ����
**********************************************************/
double TimeMeasure::diffMsWithRecord(std::string flag)
{
    return getTimeWithRecord(flag);
}

/**********************************************************
* �������ƣ�diffMsWithRecord()
* �������������ز���ͣflag��ǵĳ���ִ��ʱ��
* ���������std::string flag    			--  ���
* �����������
* �� �� ֵ����
**********************************************************/
double TimeMeasure::pauseRecord(std::string flag)
{
    if (costTime.find(flag) == costTime.end())
        return calcCostTime(flag);
    else
        return costTime[flag];
}

/**********************************************************
* �������ƣ�getTimeWithRecord()
* ������������ȡflag��ǵĳ���ִ��ʱ��   	
* ���������std::string flag    			--  ���
* �����������
* �� �� ֵ����
**********************************************************/
double TimeMeasure::getTimeWithRecord(std::string flag)
{
    if (costTime.find(flag) == costTime.end())
        return pauseRecord(flag);
    else
        return costTime[flag];
}

/**********************************************************
* �������ƣ�diffMsStringWithRecord()
* �����������ַ������ͷ���flag��ǵĳ���ִ��ʱ��
* ���������std::string flag    			--  ���
* �����������
* �� �� ֵ����
**********************************************************/
std::string TimeMeasure::diffMsStringWithRecord(std::string flag)
{
    double ms = diffMsWithRecord(flag);
    return flag + " : " + std::to_string(ms) + "ms";
}

/**********************************************************
* �������ƣ�removeRecord()
* �����������Ƴ�flag��ǵĳ���ִ��ʱ��
* ���������std::string flag    			--  ���
* �����������
* �� �� ֵ����
**********************************************************/
void TimeMeasure::removeRecord(std::string flag)
{
    record.erase(flag);
    costTime.erase(flag);
}

/**********************************************************
* �������ƣ�clear()
* ����������������м�¼
* ���������std::string flag    			--  ���
* �����������
* �� �� ֵ����
**********************************************************/
void TimeMeasure::clear()
{
    record.clear();
    costTime.clear();
}

