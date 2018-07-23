/*********版权所有（C）2013，武汉高德红外股份有限公司***************
* 文件名称：fakeeis.cpp
* 文件标识：稳像库库入口
* 当前版本：V1.0
* 创建作者：谢伯勇
* 创建日期：2018年2月5日
*******************************************************************/
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>
#include <semaphore.h>
#include <errno.h>
#include <string.h>
#include <CL/cl.h>
#include <exception>
#include <sys/stat.h>
#include <sys/types.h>
#include <thread>
#include <queue>
#include <sys/time.h> 
#include <time.h>
#include "TimeMeasure.hpp"
#include "msm_media_info.h"
#include "oclexe.hpp"
#include "calcM.hpp"
#include "ionBuffManager.hpp"
#include "yuvresize.h"
#include "common.h"
extern "C" {
#include "fakeeis.h"
}

#define EIS_ON 		0x00
#define EIS_OFF  	0x01

#define atomic_inc(x) __sync_add_and_fetch((x),1)
#define atomic_dec(x) __sync_sub_and_fetch((x),1)
#define atomic_add(x,y)	__sync_add_and_fetch((x),(y))
#define atomic_sub(x,y)	__sync_sub_and_fetch((x),(y))
#define atomic_lock(x) while(x != 0); atomic_inc(&x)
#define atomic_unlock(x) atomic_dec(&x)

__attribute_used__ static inline uint64_t gettimestamp_ns() 
{
	uint64_t val;
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	val = ts.tv_sec;
	val*= 1000000000;
	val+= ts.tv_nsec;
	return val;
}

////////////////////////////////////////////////////////////////////////////////
//--数据结构定义
////////////////////////////////////////////////////////////////////////////////
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//log数据结构体
struct eislog 
{
	float affine;
	float calcM;
	int   frame;
	float M[6];
}; 

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//帧数据结构体
struct frame_info 
{
	char *   data;
	int      fd;
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//全局数据结构体
struct imgproc 
{
	struct img_res      res_in;
	struct img_res      res_out;

	int 				outbufs_cnt;
	char**				outbufs;
	int*				outbufsfd;
	int					outbuf_idx;

	void *              usrdata;
	pfn_completion_cb_t comp_cb;
	pfn_completion_cb_app_t comp_cb_app;
	pfn_free_cb_t       free_cb;
	sem_t               sem_save;
	sem_t               sem_main;
	sem_t               sem_affine;
	bool                bquit;
	
	// the following are C++ members, need to new and delete.
	std::thread *                    work_thrd;
	std::thread *                    gpu_thrd;

	Affine*				affine;
	ionBuffMgr* 		ionMgr;
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//yuv图像信息结构体
struct venus_frame 
{
	unsigned int y_stride;
	unsigned int y_scanlines;
	unsigned int c_stride;
	unsigned int c_scanlines;
	char *		 y; // Y start
	char *		 c; // CrCb start
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//GPU异步执行结构体
struct async_affine 
{
	int idx;
	int out_fd;
	void *outbuf;
	void *userdata;
	pfn_completion_cb_t comp_cb;

	cv::Mat M;
	cv::Mat img;
	int ion_fd;

};


////////////////////////////////////////////////////////////////////////////////
//--声明全局变量
////////////////////////////////////////////////////////////////////////////////
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//eis开关
uint8_t g_eis_switch = EIS_ON;
uint8_t g_quence; 

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//异步数据队列
std::queue<struct async_affine *> affine_queue;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//异步队列互斥锁
pthread_mutex_t affineMutex = PTHREAD_MUTEX_INITIALIZER;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//分辨率修改互斥锁
pthread_mutex_t resMutex = PTHREAD_MUTEX_INITIALIZER;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
static struct imgproc *ip_img = NULL;

/**********************************************************
* 函数名称：_fill_venus_frame()
* 功能描述：获取yuv图像信息
* 输入参数：struct venus_frame *fr    	--  yuv信息结构体
*           struct img_res *res     	--  图像宽高结构体
*           char *buf				 	--  图像缓存指针
* 输出参数：无
* 返 回 值：无
**********************************************************/
static inline void _fill_venus_frame(struct venus_frame *fr, struct img_res *res, char *buf) 
{
	fr->y_stride    = VENUS_Y_STRIDE(COLOR_FMT_NV12, res->width);
	fr->y_scanlines = VENUS_Y_SCANLINES(COLOR_FMT_NV12, res->height);
	fr->c_stride    = VENUS_UV_STRIDE(COLOR_FMT_NV12, res->width);
	fr->c_scanlines = VENUS_UV_SCANLINES(COLOR_FMT_NV12, res->height);
	fr->y           = buf;
	fr->c           = fr->y + (fr->y_stride * fr->y_scanlines);
}




/*
static void eislog_save(struct eislog *log)
{
	if (fpLog != NULL) {
		fprintf(fpLog, "%d, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
		log->frame, log->calcM, log->affine, log->M[0], log->M[1], log->M[2], log->M[3], log->M[4], log->M[5]);
	}
}

static void createLog(void) 
{	
	printf("create eis log\r\n");
	char path[256];

	time_t file_t;
    time(&file_t);
    struct tm *p_new_file_t = gmtime(&file_t);

	mkdir("/media/internal/data/eislog", 0755);
	sprintf(path, "/media/internal/data/eislog/%04d%02d%02d%02d%02d%02d.csv", 1900+p_new_file_t->tm_year, 1+p_new_file_t->tm_mon, p_new_file_t->tm_mday, p_new_file_t->tm_hour, p_new_file_t->tm_min, p_new_file_t->tm_sec);
	fpLog = fopen(path, "w");
	fprintf(fpLog, "frame, calcM, affine, M\n");
}
*/


/**********************************************************
* 函数名称：do_image_processing()
* 功能描述：计算M矩阵
* 输入参数：struct imgproc *ip    --  全局信息指针
* 输出参数：无
* 返 回 值：无
**********************************************************/
static int do_image_processing(struct imgproc *ip)
{
	static int count = 0, frame = 0;
	struct venus_frame fr, to;
	void *calcFrame, *stabFrame;
	int  calcFd, stabFd;

	pthread_mutex_lock(&resMutex);
	struct img_res in = ip->res_in;
	struct img_res out = ip->res_out;
	pthread_mutex_unlock(&resMutex);

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //从ionMgr获取计算M矩阵的帧地址和计算稳像的帧地址
	ip->ionMgr->getStabilizerFrame(&calcFrame, &calcFd, &stabFrame, &stabFd);
	if (calcFrame == NULL) return -1;
	frame++;

	_fill_venus_frame(&fr, &in, (char *)calcFrame);
	_fill_venus_frame(&to, &out, ip->outbufs[ip->outbuf_idx]);

	if(EIS_ON == g_eis_switch && (in.width != out.width || in.height != out.height))
	{
	    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++
	    //Affine对象没创建则创建Affine对象
		if (ip->affine == NULL) 
		{
			ip->affine = new Affine(in.width, in.height, out.width, out.height);
		}
		struct async_affine *af = new struct async_affine;

		//struct eislog *log = new struct eislog;

		TimeMeasure tm;
		tm.recordTime("calc_M");
		//使用从ionMgr获取的calcFrame地址计算M矩阵
		cv::Mat yuv_img(fr.y_scanlines+fr.c_scanlines, fr.y_stride, CV_8UC1, (char *)calcFrame);
		cv::Mat M = calcM(yuv_img, in.height, in.width, out.height, out.width);
		ip->ionMgr->calcMCmp(calcFrame);
		//前3帧不进行稳像
		if (++count < 3) 
		{
			ip->ionMgr->stabilizerCmp(stabFrame);
			return -1;
		}

		//填充结构体
		af->outbuf = ip->outbufs[ip->outbuf_idx];
		af->out_fd = ip->outbufsfd[ip->outbuf_idx];
		af->idx = ip->outbuf_idx;

		af->userdata = ip->usrdata;
		af->comp_cb = ip->comp_cb;

		af->ion_fd = stabFd;
		af->M = M.clone();
		af->img = cv::Mat(fr.y_scanlines+fr.c_scanlines, fr.y_stride, CV_8UC1, (char *)stabFrame);

		tm.pauseRecord("calc_M");

		/*
		log->frame = frame;
		log->calcM = tm.diffMsWithRecord("calc_M");
		memcpy(log->M, M.data, sizeof(float)*6);
		*/
		
		float calcM_t = tm.diffMsWithRecord("calc_M");
		// 将结构体放入队列
		pthread_mutex_lock(&affineMutex);
		affine_queue.push(af);
		//log_queue.push(log);
		pthread_mutex_unlock(&affineMutex);
		sem_post(&ip->sem_affine);

		if (calcM_t > 33) 
		{
			iplog("[ION MGR] %s", tm.diffMsStringWithRecord("calc_M").c_str());
		}

		ip->outbuf_idx++;
		ip->outbuf_idx %= ip->outbufs_cnt;

	}
	else 
	{
		//关闭稳像为裁剪模式
		//关闭稳像时，如果affine_queue队列不为空，则清空队列，释放所有未处理的帧
		while (!affine_queue.empty()) 
		{
			struct async_affine *af = affine_queue.front();
			affine_queue.pop();
			ip->ionMgr->stabilizerCmp((void *)af->img.data);
			delete af;
		}
		// 关闭稳像后停止GPU
		if (ip->affine != NULL) 
		{
			ip->affine->oclExeStop();
		}

		if((in.width == out.width) && (in.height == out.height)) 
		{
			int height = in.height;
			int width  = in.width;
			memcpy(to.y, stabFrame, VENUS_BUFFER_SIZE(COLOR_FMT_NV12, width, height));
		}
		else
		{
			int offset_w = 0, offset_h = 0;
			offset_w = ((in.width - out.width)/2) & (~0x1);
			offset_h = ((in.height - out.height)/2) & (~0x1);

			int height = out.height;
			int width  = out.width;

			if (ip->bquit) return -1;
			TimeMeasure tm;
			tm.recordTime("memcpy time");

			// 复制y分量
			for (int i=0; i < height; i++) 
			{
				memcpy_neon16((int8_t *)&to.y[to.y_stride * i], (int8_t *)&fr.y[fr.y_stride * (offset_h+i) + offset_w], width);
			}
			// 复制uv分量
			for (int i=0; i< height/2; i++) 
			{
				memcpy_neon16((int8_t *)&to.c[to.y_stride * i], (int8_t *)&fr.c[fr.y_stride * (offset_h/2+i) + offset_w], width);
			}

			if (tm.diffMsWithRecord("memcpy time") > 30.0)
			{
				printf("------%s--------\n", tm.diffMsStringWithRecord("memcpy time").c_str());
			}
		}

		//裁剪完成释放从ionMgr获取的帧地址
		ip->ionMgr->calcMCmp(calcFrame);
		ip->ionMgr->stabilizerCmp(stabFrame);
		ip->comp_cb(ip->usrdata, ip->outbuf_idx, 0);
		ip->outbuf_idx++;
		ip->outbuf_idx %= ip->outbufs_cnt;
	}
	return 0;
}

/**********************************************************
* 函数名称：affine_gpu()
* 功能描述：使用GPU对图像进行仿射变换
* 输入参数：struct imgproc *ip    --  全局信息指针
* 输出参数：无
* 返 回 值：无
**********************************************************/
static void affine_gpu(struct imgproc *ip)
{
	static int count = 0;
	TimeMeasure tm;
	//createLog();
	while(!ip->bquit) 
	{
		sem_wait(&ip->sem_affine);
		while(!ip->bquit) 
		{
			if (!affine_queue.empty() && g_eis_switch == EIS_ON) 
			{
				tm.recordTime("affine");
				//从队列中取出async_affine结构体
				pthread_mutex_lock(&affineMutex);
				struct async_affine *af = affine_queue.front();
				affine_queue.pop();
				//struct eislog *log = log_queue.front();
				//log_queue.pop();
				pthread_mutex_unlock(&affineMutex);
				//执行GPU计算
				ip->affine->oclExeRun(af->img.data, af->ion_fd, af->outbuf, af->out_fd, (float*)af->M.data);
				af->comp_cb(af->userdata, af->idx, 0);
				if (tm.diffMsWithRecord("affine") > 50) 
				{
					fprintf(stderr, "[ION MGR] M : [");
					for(int i = 0; i < 9 ; ++i) 
					{
						fprintf(stderr, "%.2f, ", ((float*)af->M.data)[i]);
					}
					fprintf(stderr, "]\n");
					iplog("[ION MGR] %s frame:%dst", tm.diffMsStringWithRecord("affine").c_str(), ++count);
				}
				//释放执行完后的指针
				ip->ionMgr->stabilizerCmp((void *)af->img.data);
				delete af;
				/*
				if (log != NULL) 
				{
					log->affine = tm.diffMsWithRecord("affine");
					eislog_save(log);
					delete log;
				}
				*/
			}
			else 
			{
				break;
			}
			
		}
	}
}

/**********************************************************
* 函数名称：main_thread()
* 功能描述：主线程
* 输入参数：struct imgproc *ip    --  全局信息指针
* 输出参数：无
* 返 回 值：无
**********************************************************/
static void main_thread(struct imgproc *ip) 
{
	videostabframe_set();
	while (1) 
	{
		sem_wait(&ip->sem_main);
		if (ip->bquit) break;
		while (do_image_processing(ip) == 0);
	}
}


/**********************************************************
* 函数名称：process_image()
* 功能描述：帧数据接口
* 输入参数：void *phandle    	--  全局信息结构体体
*           char *yuv_buf     	--  图像缓存指针
*           uint64_t timestamp	--  图像时间戳
*           int buffd			--  图像缓存描述符
* 输出参数：无
* 返 回 值：无
**********************************************************/
extern "C"
int process_image(void *phandle, char *yuv_buf, uint64_t timestamp, int buffd) 
{
	struct imgproc *ip = (struct imgproc *)phandle;
	if (ip->ionMgr == NULL) 
	{
		ip->ionMgr = (ionBuffMgr *)getIonMgr();
	}
	//通知主线程
	sem_post(&ip->sem_main); //notify the work thread.

	return 0;
}

/**********************************************************
* 函数名称：create_instance()
* 功能描述：eis库初始化接口
* 输入参数：无
* 输出参数：无
* 返 回 值：无
**********************************************************/
extern "C"
void * create_instance()
{
	struct imgproc *ip = (struct imgproc *)calloc(1, sizeof(*ip));
	ip_img = ip;
	if (sem_init(&ip->sem_main, 0, 0)) 
	{
		iplog("sem_init() failed, %d: %s", errno, strerror(errno));
		return NULL;
	}
	if (sem_init(&ip->sem_affine, 0, 0)) 
	{
		iplog("sem_init() failed, %d: %s", errno, strerror(errno));
		return NULL;
	}
	if (sem_init(&ip->sem_save, 0, 0)) 
	{
		iplog("sem_init() failed, %d: %s", errno, strerror(errno));
		return NULL;
	}
	
	ip->bquit = false;
	ip->work_thrd = new std::thread(main_thread, ip);
	ip->gpu_thrd = new std::thread(affine_gpu, ip);

	return (void*)ip;
}

/**********************************************************
* 函数名称：destroy_instance()
* 功能描述：eis库销毁接口
* 输入参数：void *phandle    	--  全局信息结构体体
* 输出参数：无
* 返 回 值：无
**********************************************************/
extern "C"
void destroy_instance(void *phandle) 
{
	struct imgproc *ip = (struct imgproc *)phandle;

	if (!phandle) return;

	if (ip->work_thrd) 
	{
		// notify the work thread that we are going to quit.
		ip->bquit = true;
		sem_post(&ip->sem_main);
		sem_post(&ip->sem_affine);
		// wait for end of the work thread.
		ip->work_thrd->join();
		ip->gpu_thrd->join();
		// finally delete the std::thread object.
		delete ip->work_thrd;
	}
	while (!affine_queue.empty()) 
	{
		struct async_affine *af = affine_queue.front();
		affine_queue.pop();
		ip->ionMgr->stabilizerCmp((void *)af->img.data);
		delete af;
	}
	if (ip->affine != NULL) 
	{
		ip->affine->oclExeStop();
	}
	free(ip->outbufs);
	free(ip->outbufsfd);
	free(phandle);
}

/**********************************************************
* 函数名称：configure()
* 功能描述：eis库图像尺寸配置接口
* 输入参数：void *phandle    	--  全局信息结构体体
*           int nmemb     		--  图像缓存数量
*           img_res in			--  图像输入尺寸
*           img_res out			--  图像输出尺寸
* 输出参数：无
* 返 回 值：无
**********************************************************/
extern "C"
int configure(void *phandle, int nmemb, img_res in, img_res out) 
{

	pthread_mutex_lock(&resMutex);
	struct imgproc *ip = (struct imgproc *)phandle;
	ip->res_in  = in;
	ip->res_out = out;
	pthread_mutex_unlock(&resMutex);
	fprintf(stderr, "[EIS] [%s] change resolution to (%d %d)\n", __FUNCTION__, out.width, out.height);

	return 0;
}

/**********************************************************
* 函数名称：set_output_buffer_pool()
* 功能描述：eis库图像输出缓存配置接口
* 输入参数：void *phandle    		--  全局信息结构体体
*           char *bufpool[]    		--  图像缓存池
*           int buffd[]				--  图像缓存池对应的描述符
*           int nmemb				--  图像缓存数量
* 输出参数：无
* 返 回 值：无
**********************************************************/
extern "C"
void set_output_buffer_pool(void *phandle, char *bufpool[], int buffd[], int nmemb) 
{
	struct imgproc *ip = (struct imgproc *)phandle;
	int i;
	ip->outbufs_cnt = nmemb;
	ip->outbufs = (char **)malloc(sizeof(char *) * nmemb);
	ip->outbufsfd = (int *)malloc(sizeof(int) * nmemb);
	for (i=0; i<nmemb; i++) 
	{
		ip->outbufs[i] = bufpool[i];
		ip->outbufsfd[i] = buffd[i];
	}
}

/**********************************************************
* 函数名称：set_free_callback()
* 功能描述：设置完成回调
* 输入参数：void *phandle    				--  全局信息结构体体
*           pfn_completion_cb_t pfn_cb    	--  回调函数
* 输出参数：无
* 返 回 值：无
**********************************************************/
extern "C"
void set_completion_callback(void *phandle, pfn_completion_cb_t pfn_cb) 
{
	struct imgproc *ip = (struct imgproc *)phandle;
	ip->comp_cb = pfn_cb;
}

/**********************************************************
* 函数名称：set_completion_callback_APP()
* 功能描述：回调函数设置
* 输入参数：void *phandle    						--  全局信息结构体体
*           pfn_completion_cb_app_t pfn_cb    		--  回调函数指针
* 输出参数：无
* 返 回 值：无
**********************************************************/
extern "C"
void set_completion_callback_APP(void *phandle, pfn_completion_cb_app_t pfn_cb) 
{
	struct imgproc *ip = (struct imgproc *)phandle;
	ip->comp_cb_app = pfn_cb;
}

/**********************************************************
* 函数名称：set_free_callback()
* 功能描述：设置释放回调
* 输入参数：void *phandle    				--  全局信息结构体体
*           pfn_free_cb_t pfn_cb    		--  回调函数指针
* 输出参数：无
* 返 回 值：无
**********************************************************/
extern "C"
void set_free_callback(void *phandle, pfn_free_cb_t pfn_cb) 
{
	struct imgproc *ip = (struct imgproc *)phandle;
	ip->free_cb = pfn_cb;
}

/**********************************************************
* 函数名称：set_callback_data()
* 功能描述：设置回调数据
* 输入参数：void *phandle    		--  全局信息结构体体
*           void *priv   			--  
* 输出参数：无
* 返 回 值：无
**********************************************************/
extern "C"
void set_callback_data(void *phandle, void *priv) 
{
	struct imgproc *ip = (struct imgproc *)phandle;
	ip->usrdata = priv;
}

/**********************************************************
* 函数名称：impr_eis_process_appdata()
* 功能描述：eis库图像输出缓存配置接口
* 输入参数：void *phandle    				--  全局信息结构体体
*           struct impr_cs_data_eis *d    	--  app传入的数据
* 输出参数：无
* 返 回 值：无
**********************************************************/
extern "C"
int8_t impr_eis_process_appdata(void *phandle, struct impr_cs_data_eis *d)
{
	struct imgproc *ip = (struct imgproc *)phandle;
	struct impr_cs_data_eis Callback_Data= {0};
	Callback_Data.data = new uint8_t[8];
		
   	int8_t  rtn_flage = FAILED;
	rtn_flage  = inputData_analysis(d->size,d->data);

	int8_t rt = FAILED;
	rt = outputData_pack(&Callback_Data.size,Callback_Data.data);
	if(SUCCESS==rt&&Callback_Data.size == 8)
	{
	    ip->comp_cb_app(ip->usrdata,&Callback_Data);
	}
	
	delete[] Callback_Data.data;
	Callback_Data.data = NULL;
	
    if(FAILED==rtn_flage)
    {
        iplog("WARNNING!!!inputData_analysis failed!rtn_flage is %d",rtn_flage);
	    return FAILED;
    }
	return SUCCESS;
}

/**********************************************************
* 函数名称：eis_control()
* 功能描述：eis开关控制接口
* 输入参数：int open   		--  开关控制
* 输出参数：无
* 返 回 值：无
**********************************************************/
extern "C"
void eis_control(int open) 
{
	g_eis_switch = open == 1 ? EIS_ON : EIS_OFF;
	fprintf(stderr, "[EIS] eis %s\n", open == 1 ? "ON" : "OFF");
}

/**********************************************************
* 函数名称：set_resolution()
* 功能描述：eis库图像分辨率设置接口
* 输入参数：img_res in    		--  输入尺寸
*           img_res out    		--  输出尺寸
* 输出参数：无
* 返 回 值：无
**********************************************************/
extern "C"
void set_resolution(img_res in, img_res out)
{
	pthread_mutex_lock(&resMutex);
	ip_img->res_in  = in;
	ip_img->res_out = out;
	pthread_mutex_unlock(&resMutex);
	fprintf(stderr, "[EIS] [%s] change resolution to (%d %d)\n", __FUNCTION__, out.width, out.height);
}


