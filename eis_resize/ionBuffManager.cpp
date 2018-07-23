#include "ionBuffManager.hpp"

//#define PRLN(fmt, args...) fprintf(stderr, "[ION MGR][%15.15s]" fmt "\n", __FUNCTION__, args)
#define PRLN(fmt, args...) 


////////////////////////////////////////////////////////////////////////////////
//--函数定义
////////////////////////////////////////////////////////////////////////////////
/**********************************************************
* 函数名称：ionBuffMgr()
* 功能描述：ionBuffMgr构造函数
* 输入参数：char *ion_pool[]     	--  ion缓存池
*           int nmemb      			--  缓存数量
*           int resizesize      	--  resize分配缓存数量
*           int eissize      		--  eis分配数量
* 返 回 值：无
**********************************************************/
ionBuffMgr::ionBuffMgr(char *ion_pool[], int desc[], int nmemb, int resizesize, int eissize) 
{
    if (resizesize + eissize > nmemb || resizesize < 0 || eissize < 0) 
	{
		fprintf(stderr, "ion buff size error\n");
		abort();
	}
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //初始化成员变量
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
* 函数名称：~ionBuffMgr()
* 功能描述：ionBuffMg析构函数
* 返 回 值：无
**********************************************************/
ionBuffMgr::~ionBuffMgr()
{
	free(ions.buffs);
	pthread_mutex_destroy(&frameMutex);
}

/**********************************************************
* 函数名称：getStabilizerFrame()
* 功能描述：获取进行稳像的帧缓存
* 输入参数：void **calcFrame     	--  输出M矩阵计算帧
*           int *calcFd      		--  输出calcFrame的描述符
*           void **stabFrame     	--  稳像计算帧
*           int *stabFd     		--  stabFrame的描述符
* 返 回 值：无
**********************************************************/
void  ionBuffMgr::getStabilizerFrame(void **calcFrame, int *calcFd, void **stabFrame, int *stabFd)
{
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //互斥量加锁，增加索引
	pthread_mutex_lock(&frameMutex);
	int calcidx = calcIdx + 1;
	int stabidx = stabIdx + 1;
	calcidx %= ions.size;
	stabidx %= ions.size;

	
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //查询索引处的地址是否被使用，被使用则返回NULL
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
	    //未被使用则将索引值更新，并将拿到的地址加入lockedBuff将其锁定
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
* 函数名称：calcMCmp()
* 功能描述：释放计算完成的缓存地址
* 输入参数：void *calcFrame     	--  M矩阵计算帧地址
* 返 回 值：无
**********************************************************/
void  ionBuffMgr::calcMCmp(void *calcFrame)
{
	pthread_mutex_lock(&frameMutex);
	delPtrFromLockedBuff(calcFrame);
	PRLN("---- %p ---- calc unlocked", calcFrame);
	pthread_mutex_unlock(&frameMutex);
}

/**********************************************************
* 函数名称：stabilizerCmp()
* 功能描述：释放计算完成的缓存地址
* 输入参数：void *stabFrame     	--  稳像计算帧地址
* 返 回 值：无
**********************************************************/
void  ionBuffMgr::stabilizerCmp(void *stabFrame)
{
	pthread_mutex_lock(&frameMutex);
	delPtrFromLockedBuff(stabFrame);
	PRLN("---- %p ---- stab unlocked", stabFrame);
	pthread_mutex_unlock(&frameMutex);
}

/**********************************************************
* 函数名称：stabilizerCmp()
* 功能描述：获取resize计算的输出缓存
* 输入参数：void **resizeFrame     	--  输出resize的缓存地址
*           int *fd      			--  输出resizeFrame的描述符
* 返 回 值：无
**********************************************************/
int  ionBuffMgr::getResizeFrame(void **resizeFrame, int *fd)
{
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //互斥量加锁，将resize的索引加1
	pthread_mutex_lock(&frameMutex);
	int idx = resiIdx + 1;
	idx %= ions.size;

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //查询resiIdx处的地址是否被锁定
	if (buffLocked(ions.buffs[idx].addr) != -1) 
	{
		*resizeFrame = NULL;
		PRLN("++++ %p locked by stab ++++ resize:%d", ions.buffs[idx].addr, idx);
	}
	else 
	{
	
	    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++
	    //未被锁定则将resiIdx的值修改，再将地址加锁表示正在使用
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
* 函数名称：stabilizerCmp()
* 功能描述：释放计算完成的缓存地址
* 输入参数：void *stabFrame     	--  稳像计算帧地址
* 返 回 值：无
**********************************************************/
void  ionBuffMgr::resizeCmp(void *resizeFrame)
{
	pthread_mutex_lock(&frameMutex);
	delPtrFromLockedBuff(resizeFrame);
	PRLN("---- %p ---- resize unlocked", resizeFrame);
	pthread_mutex_unlock(&frameMutex);
}

/**********************************************************
* 函数名称：buffLocked()
* 功能描述：查询ptr指向的地址是否被锁定
* 输入参数：void *ptr     	--  查询地址
* 返 回 值：-1 	            --  锁定
*           其他            --  未锁定
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
* 函数名称：addPtrToLockedBuff()
* 功能描述：将使用的地址加入lockedBuff锁定
* 输入参数：void *ptr     	--  正在使用的缓存地址
* 返 回 值：无
**********************************************************/
void ionBuffMgr::addPtrToLockedBuff(void *ptr) 
{
	if (buffLocked(ptr) == -1) lockedBuff.push_back(ptr);
}


/**********************************************************
* 函数名称：delPtrFromLockedBuff()
* 功能描述：从lockedBuff中移除 ptr 所指向的地址
* 输入参数：void *ptr     	--  需要解锁的地址
* 返 回 值：无
**********************************************************/
void ionBuffMgr::delPtrFromLockedBuff(void *ptr) 
{
	int idx = buffLocked(ptr);
	if (idx != -1) lockedBuff.erase(lockedBuff.begin() + idx);
}
