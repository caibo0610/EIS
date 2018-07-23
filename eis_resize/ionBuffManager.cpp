#include "ionBuffManager.hpp"

//#define PRLN(fmt, args...) fprintf(stderr, "[ION MGR][%15.15s]" fmt "\n", __FUNCTION__, args)
#define PRLN(fmt, args...) 


////////////////////////////////////////////////////////////////////////////////
//--��������
////////////////////////////////////////////////////////////////////////////////
/**********************************************************
* �������ƣ�ionBuffMgr()
* ����������ionBuffMgr���캯��
* ���������char *ion_pool[]     	--  ion�����
*           int nmemb      			--  ��������
*           int resizesize      	--  resize���仺������
*           int eissize      		--  eis��������
* �� �� ֵ����
**********************************************************/
ionBuffMgr::ionBuffMgr(char *ion_pool[], int desc[], int nmemb, int resizesize, int eissize) 
{
    if (resizesize + eissize > nmemb || resizesize < 0 || eissize < 0) 
	{
		fprintf(stderr, "ion buff size error\n");
		abort();
	}
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //��ʼ����Ա����
    ions.size = nmemb;
	ions.buffs = (struct ionbuf*)malloc(sizeof(struct ionbuf) * nmemb);
	for (int i = 0; i < nmemb; i++) 
	{
		ions.buffs[i].addr = ion_pool[i];
		ions.buffs[i].fd   = desc[i];
	}
    this->eisSize = eissize;
	this->resizeSize = resizesize;
	this->resiIdx = 0;
	this->calcIdx = 0;
	this->stabIdx = nmemb - eissize + 1;

	pthread_mutex_init(&frameMutex, NULL);

}

/**********************************************************
* �������ƣ�~ionBuffMgr()
* ����������ionBuffMg��������
* �� �� ֵ����
**********************************************************/
ionBuffMgr::~ionBuffMgr()
{
	free(ions.buffs);
	pthread_mutex_destroy(&frameMutex);
}

/**********************************************************
* �������ƣ�getStabilizerFrame()
* ������������ȡ���������֡����
* ���������void **calcFrame     	--  ���M�������֡
*           int *calcFd      		--  ���calcFrame��������
*           void **stabFrame     	--  �������֡
*           int *stabFd     		--  stabFrame��������
* �� �� ֵ����
**********************************************************/
void  ionBuffMgr::getStabilizerFrame(void **calcFrame, int *calcFd, void **stabFrame, int *stabFd)
{
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //��������������������
	pthread_mutex_lock(&frameMutex);
	int calcidx = calcIdx + 1;
	int stabidx = stabIdx + 1;
	calcidx %= ions.size;
	stabidx %= ions.size;

	
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //��ѯ�������ĵ�ַ�Ƿ�ʹ�ã���ʹ���򷵻�NULL
	//            L
	//   |0|1|2|3|4|5|6|7|8|9|
	//          R A     S
	//          C 
	// L--lock  R---resize using   A---affine using  S---stab using  C---clac M using
	//  C indx  == R indx return NULL
	if (calcIdx == resiIdx || buffLocked(ions.buffs[calcidx].addr) != -1) 
	{
		*calcFrame = NULL;
		*stabFrame = NULL;
		PRLN("++++ %p locked by resize ++++ stabIdx:%d", ions.buffs[calcidx].addr, calcidx);
	}
	else 
	{
	    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++
	    //δ��ʹ��������ֵ���£������õ��ĵ�ַ����lockedBuff��������
		calcIdx = calcidx;
		stabIdx = stabidx;
		*calcFrame = ions.buffs[calcidx].addr;
		*calcFd = ions.buffs[calcidx].fd;
		*stabFrame = ions.buffs[stabidx].addr;
		*stabFd = ions.buffs[stabidx].fd;
		addPtrToLockedBuff(*stabFrame);
		addPtrToLockedBuff(*calcFrame);
		// PRLN("++++ %p ++++ calcIdx:%d", *calcFrame, calcIdx);
		// PRLN("++++ %p ++++ stabIdx:%d", *stabFrame, stabIdx);
	}
	pthread_mutex_unlock(&frameMutex);
}

/**********************************************************
* �������ƣ�calcMCmp()
* �����������ͷż�����ɵĻ����ַ
* ���������void *calcFrame     	--  M�������֡��ַ
* �� �� ֵ����
**********************************************************/
void  ionBuffMgr::calcMCmp(void *calcFrame)
{
	pthread_mutex_lock(&frameMutex);
	delPtrFromLockedBuff(calcFrame);
	PRLN("---- %p ---- calc unlocked", calcFrame);
	pthread_mutex_unlock(&frameMutex);
}

/**********************************************************
* �������ƣ�stabilizerCmp()
* �����������ͷż�����ɵĻ����ַ
* ���������void *stabFrame     	--  �������֡��ַ
* �� �� ֵ����
**********************************************************/
void  ionBuffMgr::stabilizerCmp(void *stabFrame)
{
	pthread_mutex_lock(&frameMutex);
	delPtrFromLockedBuff(stabFrame);
	PRLN("---- %p ---- stab unlocked", stabFrame);
	pthread_mutex_unlock(&frameMutex);
}

/**********************************************************
* �������ƣ�stabilizerCmp()
* ������������ȡresize������������
* ���������void **resizeFrame     	--  ���resize�Ļ����ַ
*           int *fd      			--  ���resizeFrame��������
* �� �� ֵ����
**********************************************************/
int  ionBuffMgr::getResizeFrame(void **resizeFrame, int *fd)
{
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //��������������resize��������1
	pthread_mutex_lock(&frameMutex);
	int idx = resiIdx + 1;
	idx %= ions.size;

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //��ѯresiIdx���ĵ�ַ�Ƿ�����
	if (buffLocked(ions.buffs[idx].addr) != -1) 
	{
		*resizeFrame = NULL;
		PRLN("++++ %p locked by stab ++++ resize:%d", ions.buffs[idx].addr, idx);
	}
	else 
	{
	
	    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++
	    //δ��������resiIdx��ֵ�޸ģ��ٽ���ַ������ʾ����ʹ��
		resiIdx = idx;
		*resizeFrame = ions.buffs[idx].addr;
		*fd = ions.buffs[idx].fd;
		addPtrToLockedBuff(*resizeFrame);
		// PRLN("++++ %p ++++ resiIdx:%d", *resizeFrame, resiIdx);
	}
	pthread_mutex_unlock(&frameMutex);
	return idx;
}

/**********************************************************
* �������ƣ�stabilizerCmp()
* �����������ͷż�����ɵĻ����ַ
* ���������void *stabFrame     	--  �������֡��ַ
* �� �� ֵ����
**********************************************************/
void  ionBuffMgr::resizeCmp(void *resizeFrame)
{
	pthread_mutex_lock(&frameMutex);
	delPtrFromLockedBuff(resizeFrame);
	PRLN("---- %p ---- resize unlocked", resizeFrame);
	pthread_mutex_unlock(&frameMutex);
}

/**********************************************************
* �������ƣ�buffLocked()
* ������������ѯptrָ��ĵ�ַ�Ƿ�����
* ���������void *ptr     	--  ��ѯ��ַ
* �� �� ֵ��-1 	            --  ����
*           ����            --  δ����
**********************************************************/
int ionBuffMgr::buffLocked(void *ptr) 
{
	for(uint i = 0; i < lockedBuff.size(); ++i) 
	{
		if (lockedBuff[i] == ptr) return i;
	}
	return -1;
}


/**********************************************************
* �������ƣ�addPtrToLockedBuff()
* ������������ʹ�õĵ�ַ����lockedBuff����
* ���������void *ptr     	--  ����ʹ�õĻ����ַ
* �� �� ֵ����
**********************************************************/
void ionBuffMgr::addPtrToLockedBuff(void *ptr) 
{
	if (buffLocked(ptr) == -1) lockedBuff.push_back(ptr);
}


/**********************************************************
* �������ƣ�delPtrFromLockedBuff()
* ������������lockedBuff���Ƴ� ptr ��ָ��ĵ�ַ
* ���������void *ptr     	--  ��Ҫ�����ĵ�ַ
* �� �� ֵ����
**********************************************************/
void ionBuffMgr::delPtrFromLockedBuff(void *ptr) 
{
	int idx = buffLocked(ptr);
	if (idx != -1) lockedBuff.erase(lockedBuff.begin() + idx);
}
