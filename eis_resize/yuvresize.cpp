/*********��Ȩ���У�C��2013���人�ߵº���ɷ����޹�˾***************
* �ļ����ƣ�yuvresize.cpp
* �ļ���ʶ�����ſ����
* ��ǰ�汾��V1.0
* �������ߣ�л����
* �������ڣ�2018��2��5��
*******************************************************************/
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <CL/cl.h>
#include <CL/cl_ext.h>
#include <semaphore.h>
#include <thread>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>
#include "ionBuffManager.hpp"
#include "TimeMeasure.hpp"
#include "yuvresize.h"
#include "msm_media_info.h"


////////////////////////////////////////////////////////////////////////////////
//--���ݽṹ����
////////////////////////////////////////////////////////////////////////////////
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//֡���ݽṹ��
struct frame_info 
{
	char *   data;
	uint64_t timestamp;
	int      fd;
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//ȫ�ִ���ṹ��
struct imgproc{
	cl_program          clprogram;
	cl_device_id        cldev;
	cl_context          clctx;
    cl_command_queue    clcmdq;
    cl_kernel           clkernel;
    unsigned int *      resize_table;
    size_t              in_real_size;
    size_t              out_real_size;
    int                 out_uv_offset;
    int                 out_stride;
    int                 in_stride;
    int                 in_uv_offset;
	struct c2d_res      res_in;
	struct c2d_res      res_out;
	void *              usrdata;
	pfn_completion_cb_t comp_cb;
	sem_t               sem_event;
	bool                bquit;
	// the following are C++ members, need to new and delete.
	std::thread *                    work_thrd;
    std::queue<struct frame_info*> * input_queue; // to keep input buffers
    ionBuffMgr         *buffMgr;
};


////////////////////////////////////////////////////////////////////////////////
//--����ȫ�ֱ���
////////////////////////////////////////////////////////////////////////////////
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//֡������ڻ�����
static pthread_spinlock_t frameLock = 0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//ȫ��ion����������ָ��
static ionBuffMgr* ionMgr = NULL; 

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//frame���ʱ���¼
//TimeMeasure frameTm;

/**********************************************************
* �������ƣ�getIonMgr()
* ������������ȡion����������
* �� �� ֵ����
**********************************************************/
void* getIonMgr(void) 
{
    return (void *)ionMgr;
}

/**********************************************************
* �������ƣ�init_resize_gpu()
* ������������ʼ��GPU
* ���������struct imgproc *ip     --  ȫ�ֽṹָ��
* �� �� ֵ����
* ����˵������
**********************************************************/
void init_resize_gpu(struct imgproc *ip)
{
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //����GPU��Ҫ��һЩ���ݽṹ
    int ret;
    cl_platform_id platform;
	ret = clGetPlatformIDs(1, &platform, NULL);
	ret = clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &ip->cldev, NULL);
	ip->clctx = clCreateContext(NULL, 1, &ip->cldev,NULL, NULL, &ret);
	ip->clcmdq = clCreateCommandQueue(ip->clctx, ip->cldev, 0, &ret);

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//��ȡCLԴ��    
	char *kernelSource;
    std::ifstream kernelFile;
    kernelFile.open("/home/data/vresize.cl");
	std::stringstream  strBuf;
	strBuf << kernelFile.rdbuf();
	std::string strKernel(strBuf.str());
	size_t len = strKernel.length();
	kernelSource = (char *)malloc(sizeof(char)*(len+1));
	strKernel.copy(kernelSource, len, 0);
	kernelSource[len] = '\0';

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//����CLԴ��
    char opts[256];
//  OUT_STRIDE OUT_Y_SCANLINE 
//  IN_STRIDE  IN_Y_SCANLINE
//  OUT_UV_OFFSET
//  IN_UV_OFFSET
//  HSCALE VSCALE
#define VECTOR 4
    float hscale = ip->res_in.w * 1.f / ip->res_out.w;
    float vscale = ip->res_in.h * 1.f / ip->res_out.h;
    sprintf(opts, "-DOUT_STRIDE=%d -DIN_STRIDE=%d -DOUT_UV_OFFSET=%d -DIN_UV_OFFSET=%d -DHSCALE=%.2ff -DVSCALE=%.2ff"
                   " -DOUT_W=%d -DOUT_H=%d -DIN_W=%d -DIN_H=%d",
                    ip->out_stride / VECTOR, ip->in_stride,ip->out_uv_offset, ip->in_uv_offset, hscale, vscale, 
                    ip->res_out.w, ip->res_out.h, ip->res_in.w, ip->res_in.h);
    rslog("%s", opts);
	ip->clprogram = clCreateProgramWithSource(ip->clctx, 1, (const char**)&kernelSource, NULL, &ret);
    ret = clBuildProgram(ip->clprogram, 0, NULL, opts, NULL, NULL);
    if (ret < 0) 
	{
		size_t logsize;
		clGetProgramBuildInfo(ip->clprogram, ip->cldev, CL_PROGRAM_BUILD_LOG, 0, NULL, &logsize);
		char *log = (char *)malloc(logsize*sizeof(char));
		clGetProgramBuildInfo(ip->clprogram, ip->cldev, CL_PROGRAM_BUILD_LOG, logsize, log, NULL);
		fprintf(stderr, "log : \n%s\n", log);
		free(log);
	}

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//ͨ��Դ�봴��kernel����
    ip->clkernel = clCreateKernel(ip->clprogram, "resize", &ret);
    free(kernelSource);
}

/**********************************************************
* �������ƣ�release_gpu()
* �����������ͷ�GPU���ɵĶ���
* ���������struct imgproc *ip     --  ȫ�ֽṹָ��
* �� �� ֵ����
* ����˵������
**********************************************************/
void release_gpu(struct imgproc *ip)
{
	clReleaseKernel(ip->clkernel);
	clReleaseCommandQueue(ip->clcmdq);
	clReleaseContext(ip->clctx);
	clReleaseDevice(ip->cldev);
	clReleaseProgram(ip->clprogram);
}

/**********************************************************
* �������ƣ�c2d_gpu_processing()
* �����������������ͼ���������
* ���������struct imgproc *ip     --  ȫ�ֽṹָ��
*           void *outbuf	       --  ��������ַ
*           int outfd			   --  ��������ַ�ļ�������
* �� �� ֵ����
* ����˵������
**********************************************************/
void c2d_gpu_processing(struct imgproc *ip, void *outbuf, int outfd)
{
    int ret;
    cl_mem mem_in_obj = NULL;
    cl_mem mem_out_obj = NULL;
    size_t device_page_size = 0;
    cl_mem_ion_host_ptr  ionin  = {0};
    cl_mem_ion_host_ptr  ionout  = {0};

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //��ȡ�����е�frame����
	pthread_spin_lock(&frameLock);
    struct frame_info *finfo = ip->input_queue->front();
    ip->input_queue->pop();
	pthread_spin_unlock(&frameLock);
    

    TimeMeasure tm;
    tm.recordTime("create buffer time");
    clGetDeviceInfo(ip->cldev, CL_DEVICE_PAGE_SIZE_QCOM, sizeof(device_page_size), &device_page_size, NULL);
    if((int)finfo->data % device_page_size) 
	{
        rslog("Host pointer must be aligned to device_page_size!");
    }
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //����GPU����buffer����
    ionin.ext_host_ptr.allocation_type    = CL_MEM_ION_HOST_PTR_QCOM;
    ionin.ext_host_ptr.host_cache_policy  = CL_MEM_HOST_UNCACHED_QCOM;
    ionin.ion_filedesc                    = finfo->fd;
    ionin.ion_hostptr                     = finfo->data;
    mem_in_obj = clCreateBuffer(ip->clctx, CL_MEM_USE_HOST_PTR | CL_MEM_EXT_HOST_PTR_QCOM, ip->in_real_size, &ionin, &ret);
    
    ionout.ext_host_ptr.allocation_type    = CL_MEM_ION_HOST_PTR_QCOM;
    ionout.ext_host_ptr.host_cache_policy  = CL_MEM_HOST_UNCACHED_QCOM;
    ionout.ion_filedesc                    = outfd;
    ionout.ion_hostptr                     = outbuf;
    mem_out_obj = clCreateBuffer(ip->clctx, CL_MEM_USE_HOST_PTR | CL_MEM_EXT_HOST_PTR_QCOM, ip->out_real_size, &ionout, NULL);

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //�����ں˲���
    clSetKernelArg(ip->clkernel, 0, sizeof(cl_mem), &mem_in_obj);
    clSetKernelArg(ip->clkernel, 1, sizeof(cl_mem), &mem_out_obj);
    
    tm.recordTime("exe time");
    
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //ִ���ں˺�����ͼ���������
    size_t globalsize[2] = {(size_t)ip->res_out.w / VECTOR, (size_t)ip->res_out.h};
    ret = clEnqueueNDRangeKernel(ip->clcmdq, ip->clkernel, 2, NULL, globalsize, NULL, 0, NULL, NULL);
    clFinish(ip->clcmdq);

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //�ͷŴ����Ĺ���buffer
    clReleaseMemObject(mem_in_obj);
    clReleaseMemObject(mem_out_obj);
    delete finfo;
    if (ret < 0) 
	{
        fprintf(stderr, "[RESIZE] eis resize error - %d\n", ret);
    }
    if (tm.diffMsWithRecord("exe time") > 30.0) 
	{
        rslog("resize gpu exe time---%s---_\n", tm.diffMsStringWithRecord("exe time").c_str());
    }
}


/**********************************************************
* �������ƣ�main_thread()
* �����������������frame����ѭ������
* ���������struct imgproc *ip     --  ȫ�ֽṹָ��
* �� �� ֵ����
* ����˵������
**********************************************************/
void main_thread(struct imgproc *ip)
{
    void *frame;
    int fd;
    rslog("main thread enter>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //��ʼ��GPU
    init_resize_gpu(ip);

    while (!ip->bquit)  
	{
		//+++++++++++++++++++++++++++++++++++++++++++++++++++++++
	    //�ȴ�frame����ӿ�֪ͨ
        sem_wait(&ip->sem_event);
		if (ip->bquit) break;
        if (ip->input_queue->empty()) continue;
        
		//+++++++++++++++++++++++++++++++++++++++++++++++++++++++
	    //��ionMgr�����ȡ�������
        int idx = ip->buffMgr->getResizeFrame(&frame, &fd);
        if (frame == NULL) {
            fprintf(stderr, "[ION MGR] ---------eis busy--------- \n");
            continue;
        }
		
		//+++++++++++++++++++++++++++++++++++++++++++++++++++++++
	    //�������Ŵ���
        c2d_gpu_processing(ip, frame, fd);
		
		//+++++++++++++++++++++++++++++++++++++++++++++++++++++++
	    //֪ͨionMgr�����������
        ip->buffMgr->resizeCmp(frame);

		//+++++++++++++++++++++++++++++++++++++++++++++++++++++++
	    //�ص�֪ͨCS
        ip->comp_cb(ip->usrdata, idx, 0);
    }
    release_gpu(ip);
}


/**********************************************************
* �������ƣ�c2d_process_image()
* ����������ͼ��֡��ں���
* ���������void *phandle	       --  ȫ�ֽṹָ��
*           char *yuv_buf	       --  yuvͼ�񻺴��ַ
*           uint64_t timestamp	   --  yuvͼ��ʱ���
*           int ion_desc	       --  yuvͼ�񻺴��ַ�ļ�������
* �� �� ֵ��0					   -- �ɹ�
* 			����				   -- ʧ��
* ����˵������
**********************************************************/
extern "C"
int  c2d_process_image(void *phandle, char *yuv_buf, uint64_t timestamp, int ion_desc)
{

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //����֡���ݵ�frame_info�ṹ��
    //static int count = 0 ;
	struct imgproc *ip = (struct imgproc *)phandle;
    struct frame_info * finfo = new struct frame_info;
	finfo->data = yuv_buf;
	finfo->timestamp = timestamp;
    finfo->fd = ion_desc;

    
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //��ӡ<25ms��>35ms��֡
    //++count;
    /*
    if (frameTm.diffMsWithRecord("frame_interval") < 25 || frameTm.diffMsWithRecord("frame_interval") > 35) 
	{
        fprintf(stderr, "[ION MGR] resize recv frame %d | %f\n", count, frameTm.diffMsWithRecord("frame_interval"));
    }
    frameTm.recordTime("frame_interval");
    */


	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //��������֡���ݷ������
	pthread_spin_lock(&frameLock);
    ip->input_queue->push(finfo);
    while (ip->input_queue->size() > 4)
    {
        fprintf(stderr, "[ION MGR]+++++++resize busy, skip this frame++++++++++++\n");
        struct frame_info *finfo = ip->input_queue->front();
        ip->input_queue->pop();
        delete finfo;
    }
	pthread_spin_unlock(&frameLock);

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //֪ͨ���߳̽��д���
    sem_post(&ip->sem_event); //notify the work thread.
    
	return 0;
}


/**********************************************************
* �������ƣ�c2d_create_instance()
* ����������resize��ʼ��
* �����������
* �� �� ֵ��void *                -- ȫ�ֽṹָ��
* ����˵������
**********************************************************/
extern "C"
void *c2d_create_instance()
{

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //����ȫ�ֽṹָ�룬���̣߳�GPU�߳�
    struct imgproc *ip = (struct imgproc *)calloc(1, sizeof(*ip));
	if (sem_init(&ip->sem_event, 0, 0)) 
	{
		rslog("sem_init() failed, %d: %s", errno, strerror(errno));
		return NULL;
	}
    ip->bquit = false;
	ip->input_queue = new std::queue<struct frame_info *>();
    ip->work_thrd = new std::thread(main_thread, ip);

	return (void*)ip;
}


/**********************************************************
* �������ƣ�c2d_destroy_instance()
* ����������resize����
* ���������void *phandle              -- ȫ�ֽṹָ��
* �� �� ֵ����
* ����˵������
**********************************************************/
extern "C"
void c2d_destroy_instance(void *phandle)
{
    // rslog("1111111111111111111111111111");
    struct imgproc *ip = (struct imgproc *)phandle;
    
    if (!phandle) return;

    if (ip->work_thrd) 
	{
        // notify the work thread that we are going to quit.
        ip->bquit = true;
        sem_post(&ip->sem_event);
        // wait for end of the work thread.
        ip->work_thrd->join();
        // finally delete the std::thread object.
        delete ip->work_thrd;
    }

    if (ip->input_queue) 
    {
        while (!ip->input_queue->empty()) 
		{
            struct frame_info *finfo = ip->input_queue->front();
            ip->input_queue->pop();
            delete finfo;
        }
        delete ip->input_queue;
    }
    delete ip->buffMgr;
    free(phandle);
}


/**********************************************************
* �������ƣ�c2d_configure()
* ����������resize��������
* ���������void *phandle	       --  ȫ�ֽṹָ��
*           int bufs_in		       --  ͼ�񻺴����
*           struct c2d_res in	   --  ����ͼ���С
*           struct c2d_res out     --  ���ͼ���С
* �� �� ֵ����
* ����˵������
**********************************************************/
extern "C"
int  c2d_configure(void *phandle, int bufs_in, struct c2d_res in, struct c2d_res out)
{
    struct imgproc *ip = (struct imgproc *)phandle;
	ip->res_in  = in;
    ip->res_out = out;


	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //����yuvͼ���һЩƫ�����ʹ�С
    int in_stride = VENUS_Y_STRIDE(COLOR_FMT_NV21, in.w);
    int in_y_scanline = VENUS_Y_SCANLINES(COLOR_FMT_NV21, in.h);
    int in_uv_scanline = VENUS_UV_SCANLINES(COLOR_FMT_NV21, in.h);

    int out_stride = VENUS_Y_STRIDE(COLOR_FMT_NV21, out.w);
    int out_y_scanline = VENUS_Y_SCANLINES(COLOR_FMT_NV21, out.h);
    int out_uv_scanline = VENUS_UV_SCANLINES(COLOR_FMT_NV21, out.h);

    ip->in_real_size = (in_y_scanline+in_uv_scanline)*in_stride;
    ip->out_real_size = (out_y_scanline+out_uv_scanline)*out_stride;
    ip->in_stride = in_stride;
    ip->out_stride = out_stride;
    ip->out_uv_offset = out_stride * out_y_scanline;
    ip->in_uv_offset = in_stride * in_y_scanline;

    return 0;
}

/**********************************************************
* �������ƣ�c2d_set_output_buffer_pool()
* ����������resize��������������
* ���������void *phandle	       --  ȫ�ֽṹָ��
*           char *ion_pool[]	   --  ���ion�����
*           int desc[]    		   --  ���ion����ض�Ӧ�ļ�������
*           int nmemb		       --  �������
* �� �� ֵ����
* ����˵������
**********************************************************/
extern "C"
void c2d_set_output_buffer_pool(void *phandle, char *ion_pool[], int desc[], int nmemb)
{
    //����buffer��С����ionMgr�ĳ�ʼ������
	int resizeBuffLenIdx[] = {-1, -1, -1, -1, -1, 2, 3, 3, 4, 4, 5, 5, 5};
	int eisBuffLenIdx[]    = {-1, -1, -1, -1, -1, 3, 3, 4, 4, 5, 5, 6, 3};
    struct imgproc *ip = (struct imgproc *)phandle;

	ip->buffMgr = new ionBuffMgr(ion_pool, desc, nmemb, resizeBuffLenIdx[nmemb], eisBuffLenIdx[nmemb]);
    ionMgr = ip->buffMgr;
}

/**********************************************************
* �������ƣ�c2d_set_completion_callback()
* �������������ûص�����
* �� �� ֵ����
* ����˵������
**********************************************************/
extern "C"
void c2d_set_completion_callback(void *phandle, pfn_completion_cb_t pfn_cb)
{
    struct imgproc *ip = (struct imgproc *)phandle;
	ip->comp_cb = pfn_cb;
}

/**********************************************************
* �������ƣ�c2d_set_callback_data()
* �������������ûص�����
* �� �� ֵ����
* ����˵������
**********************************************************/
extern "C"
void c2d_set_callback_data(void *phandle, void *priv)
{
    struct imgproc *ip = (struct imgproc *)phandle;
	ip->usrdata = priv;
}


