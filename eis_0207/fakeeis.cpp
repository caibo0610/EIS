/*********��Ȩ���У�C��2013���人�ߵº���ɷ����޹�˾***************
* �ļ����ƣ�fakeeis.cpp
* �ļ���ʶ�����������
* ��ǰ�汾��V1.0
* �������ߣ�л����
* �������ڣ�2018��2��5��
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
//--���ݽṹ����
////////////////////////////////////////////////////////////////////////////////
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//log���ݽṹ��
struct eislog 
{
	float affine;
	float calcM;
	int   frame;
	float M[6];
}; 

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//֡���ݽṹ��
struct frame_info 
{
	char *   data;
	int      fd;
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//ȫ�����ݽṹ��
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
//yuvͼ����Ϣ�ṹ��
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
//GPU�첽ִ�нṹ��
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
//--����ȫ�ֱ���
////////////////////////////////////////////////////////////////////////////////
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//eis����
uint8_t g_eis_switch = EIS_ON;
uint8_t g_quence; 

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//�첽���ݶ���
std::queue<struct async_affine *> affine_queue;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//�첽���л�����
pthread_mutex_t affineMutex = PTHREAD_MUTEX_INITIALIZER;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//�ֱ����޸Ļ�����
pthread_mutex_t resMutex = PTHREAD_MUTEX_INITIALIZER;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
static struct imgproc *ip_img = NULL;

/**********************************************************
* �������ƣ�_fill_venus_frame()
* ������������ȡyuvͼ����Ϣ
* ���������struct venus_frame *fr    	--  yuv��Ϣ�ṹ��
*           struct img_res *res     	--  ͼ���߽ṹ��
*           char *buf				 	--  ͼ�񻺴�ָ��
* �����������
* �� �� ֵ����
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
* �������ƣ�do_image_processing()
* ��������������M����
* ���������struct imgproc *ip    --  ȫ����Ϣָ��
* �����������
* �� �� ֵ����
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
    //��ionMgr��ȡ����M�����֡��ַ�ͼ��������֡��ַ
	ip->ionMgr->getStabilizerFrame(&calcFrame, &calcFd, &stabFrame, &stabFd);
	if (calcFrame == NULL) return -1;
	frame++;

	_fill_venus_frame(&fr, &in, (char *)calcFrame);
	_fill_venus_frame(&to, &out, ip->outbufs[ip->outbuf_idx]);

	if(EIS_ON == g_eis_switch && (in.width != out.width || in.height != out.height))
	{
	    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++
	    //Affine����û�����򴴽�Affine����
		if (ip->affine == NULL) 
		{
			ip->affine = new Affine(in.width, in.height, out.width, out.height);
		}
		struct async_affine *af = new struct async_affine;

		//struct eislog *log = new struct eislog;

		TimeMeasure tm;
		tm.recordTime("calc_M");
		//ʹ�ô�ionMgr��ȡ��calcFrame��ַ����M����
		cv::Mat yuv_img(fr.y_scanlines+fr.c_scanlines, fr.y_stride, CV_8UC1, (char *)calcFrame);
		cv::Mat M = calcM(yuv_img, in.height, in.width, out.height, out.width);
		ip->ionMgr->calcMCmp(calcFrame);
		//ǰ3֡����������
		if (++count < 3) 
		{
			ip->ionMgr->stabilizerCmp(stabFrame);
			return -1;
		}

		//���ṹ��
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
		// ���ṹ��������
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
		//�ر�����Ϊ�ü�ģʽ
		//�ر�����ʱ�����affine_queue���в�Ϊ�գ�����ն��У��ͷ�����δ�����֡
		while (!affine_queue.empty()) 
		{
			struct async_affine *af = affine_queue.front();
			affine_queue.pop();
			ip->ionMgr->stabilizerCmp((void *)af->img.data);
			delete af;
		}
		// �ر������ֹͣGPU
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

			// ����y����
			for (int i=0; i < height; i++) 
			{
				memcpy_neon16((int8_t *)&to.y[to.y_stride * i], (int8_t *)&fr.y[fr.y_stride * (offset_h+i) + offset_w], width);
			}
			// ����uv����
			for (int i=0; i< height/2; i++) 
			{
				memcpy_neon16((int8_t *)&to.c[to.y_stride * i], (int8_t *)&fr.c[fr.y_stride * (offset_h/2+i) + offset_w], width);
			}

			if (tm.diffMsWithRecord("memcpy time") > 30.0)
			{
				printf("------%s--------\n", tm.diffMsStringWithRecord("memcpy time").c_str());
			}
		}

		//�ü�����ͷŴ�ionMgr��ȡ��֡��ַ
		ip->ionMgr->calcMCmp(calcFrame);
		ip->ionMgr->stabilizerCmp(stabFrame);
		ip->comp_cb(ip->usrdata, ip->outbuf_idx, 0);
		ip->outbuf_idx++;
		ip->outbuf_idx %= ip->outbufs_cnt;
	}
	return 0;
}

/**********************************************************
* �������ƣ�affine_gpu()
* ����������ʹ��GPU��ͼ����з���任
* ���������struct imgproc *ip    --  ȫ����Ϣָ��
* �����������
* �� �� ֵ����
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
				//�Ӷ�����ȡ��async_affine�ṹ��
				pthread_mutex_lock(&affineMutex);
				struct async_affine *af = affine_queue.front();
				affine_queue.pop();
				//struct eislog *log = log_queue.front();
				//log_queue.pop();
				pthread_mutex_unlock(&affineMutex);
				//ִ��GPU����
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
				//�ͷ�ִ������ָ��
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
* �������ƣ�main_thread()
* �������������߳�
* ���������struct imgproc *ip    --  ȫ����Ϣָ��
* �����������
* �� �� ֵ����
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
* �������ƣ�process_image()
* ����������֡���ݽӿ�
* ���������void *phandle    	--  ȫ����Ϣ�ṹ����
*           char *yuv_buf     	--  ͼ�񻺴�ָ��
*           uint64_t timestamp	--  ͼ��ʱ���
*           int buffd			--  ͼ�񻺴�������
* �����������
* �� �� ֵ����
**********************************************************/
extern "C"
int process_image(void *phandle, char *yuv_buf, uint64_t timestamp, int buffd) 
{
	struct imgproc *ip = (struct imgproc *)phandle;
	if (ip->ionMgr == NULL) 
	{
		ip->ionMgr = (ionBuffMgr *)getIonMgr();
	}
	//֪ͨ���߳�
	sem_post(&ip->sem_main); //notify the work thread.

	return 0;
}

/**********************************************************
* �������ƣ�create_instance()
* ����������eis���ʼ���ӿ�
* �����������
* �����������
* �� �� ֵ����
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
* �������ƣ�destroy_instance()
* ����������eis�����ٽӿ�
* ���������void *phandle    	--  ȫ����Ϣ�ṹ����
* �����������
* �� �� ֵ����
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
* �������ƣ�configure()
* ����������eis��ͼ��ߴ����ýӿ�
* ���������void *phandle    	--  ȫ����Ϣ�ṹ����
*           int nmemb     		--  ͼ�񻺴�����
*           img_res in			--  ͼ������ߴ�
*           img_res out			--  ͼ������ߴ�
* �����������
* �� �� ֵ����
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
* �������ƣ�set_output_buffer_pool()
* ����������eis��ͼ������������ýӿ�
* ���������void *phandle    		--  ȫ����Ϣ�ṹ����
*           char *bufpool[]    		--  ͼ�񻺴��
*           int buffd[]				--  ͼ�񻺴�ض�Ӧ��������
*           int nmemb				--  ͼ�񻺴�����
* �����������
* �� �� ֵ����
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
* �������ƣ�set_free_callback()
* ����������������ɻص�
* ���������void *phandle    				--  ȫ����Ϣ�ṹ����
*           pfn_completion_cb_t pfn_cb    	--  �ص�����
* �����������
* �� �� ֵ����
**********************************************************/
extern "C"
void set_completion_callback(void *phandle, pfn_completion_cb_t pfn_cb) 
{
	struct imgproc *ip = (struct imgproc *)phandle;
	ip->comp_cb = pfn_cb;
}

/**********************************************************
* �������ƣ�set_completion_callback_APP()
* �����������ص���������
* ���������void *phandle    						--  ȫ����Ϣ�ṹ����
*           pfn_completion_cb_app_t pfn_cb    		--  �ص�����ָ��
* �����������
* �� �� ֵ����
**********************************************************/
extern "C"
void set_completion_callback_APP(void *phandle, pfn_completion_cb_app_t pfn_cb) 
{
	struct imgproc *ip = (struct imgproc *)phandle;
	ip->comp_cb_app = pfn_cb;
}

/**********************************************************
* �������ƣ�set_free_callback()
* ���������������ͷŻص�
* ���������void *phandle    				--  ȫ����Ϣ�ṹ����
*           pfn_free_cb_t pfn_cb    		--  �ص�����ָ��
* �����������
* �� �� ֵ����
**********************************************************/
extern "C"
void set_free_callback(void *phandle, pfn_free_cb_t pfn_cb) 
{
	struct imgproc *ip = (struct imgproc *)phandle;
	ip->free_cb = pfn_cb;
}

/**********************************************************
* �������ƣ�set_callback_data()
* �������������ûص�����
* ���������void *phandle    		--  ȫ����Ϣ�ṹ����
*           void *priv   			--  
* �����������
* �� �� ֵ����
**********************************************************/
extern "C"
void set_callback_data(void *phandle, void *priv) 
{
	struct imgproc *ip = (struct imgproc *)phandle;
	ip->usrdata = priv;
}

/**********************************************************
* �������ƣ�impr_eis_process_appdata()
* ����������eis��ͼ������������ýӿ�
* ���������void *phandle    				--  ȫ����Ϣ�ṹ����
*           struct impr_cs_data_eis *d    	--  app���������
* �����������
* �� �� ֵ����
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
* �������ƣ�eis_control()
* ����������eis���ؿ��ƽӿ�
* ���������int open   		--  ���ؿ���
* �����������
* �� �� ֵ����
**********************************************************/
extern "C"
void eis_control(int open) 
{
	g_eis_switch = open == 1 ? EIS_ON : EIS_OFF;
	fprintf(stderr, "[EIS] eis %s\n", open == 1 ? "ON" : "OFF");
}

/**********************************************************
* �������ƣ�set_resolution()
* ����������eis��ͼ��ֱ������ýӿ�
* ���������img_res in    		--  ����ߴ�
*           img_res out    		--  ����ߴ�
* �����������
* �� �� ֵ����
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


