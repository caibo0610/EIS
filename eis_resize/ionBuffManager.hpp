#ifndef __IONBUFFMANAGER__
#define __IONBUFFMANAGER__

#include <iostream>
#include <vector>
#include <pthread.h>

struct ionbuf {
    void*   addr;
    int     fd;
};

struct ionbuffs {
    struct ionbuf* buffs;
    int size;
};

class ionBuffMgr {
public:
    ionBuffMgr(char *ion_pool[], int desc[], int nmemb, int buffsize, int eissize);
    ~ionBuffMgr();
    // ��ionMgr��ȡ����M�����֡���ݺͽ��������֡����
    void  getStabilizerFrame(void **calcFrame, int *calcFd, void **stabFrame, int *stabFd);
    //  M������ɺ����
    void  calcMCmp(void *calcFrame);
    //  �������������
    void  stabilizerCmp(void *stabFrame);
    //  ��ionMgr��ȡresize���������
    int   getResizeFrame(void **resizeFrame, int *fd);
    //  resize��ɺ����
    void  resizeCmp(void *resizeFrame);     

private:
    struct ionbuffs ions;               //ion �����
    int resizeSize;                     //resize����ʹ�õĻ�������
    int eisSize;                        //eis����ʹ�õĻ�������
    int calcIdx;                        //M�����֡����
    int stabIdx;                        //��������֡����
    int resiIdx;                        //resize�����֡����
    pthread_mutex_t frameMutex;         //��lockedBuff�����Ļ�����
    std::vector<void *> lockedBuff;     //�������ʹ�õ�buffer��ַ�����л������
	
    int  buffLocked(void *ptr);
    void addPtrToLockedBuff(void *ptr);
    void delPtrFromLockedBuff(void *ptr);
};

#endif
