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
    // 从ionMgr获取计算M矩阵的帧数据和进行稳像的帧数据
    void  getStabilizerFrame(void **calcFrame, int *calcFd, void **stabFrame, int *stabFd);
    //  M计算完成后调用
    void  calcMCmp(void *calcFrame);
    //  稳像计算完后调用
    void  stabilizerCmp(void *stabFrame);
    //  从ionMgr获取resize的输出缓存
    int   getResizeFrame(void **resizeFrame, int *fd);
    //  resize完成后调用
    void  resizeCmp(void *resizeFrame);     

private:
    struct ionbuffs ions;               //ion 缓存池
    int resizeSize;                     //resize可以使用的缓存数量
    int eisSize;                        //eis可以使用的缓存数量
    int calcIdx;                        //M计算的帧索引
    int stabIdx;                        //稳像计算的帧索引
    int resiIdx;                        //resize计算的帧索引
    pthread_mutex_t frameMutex;         //对lockedBuff加锁的互斥量
    std::vector<void *> lockedBuff;     //存放正在使用的buffer地址，进行互斥管理
	
    int  buffLocked(void *ptr);
    void addPtrToLockedBuff(void *ptr);
    void delPtrFromLockedBuff(void *ptr);
};

#endif
