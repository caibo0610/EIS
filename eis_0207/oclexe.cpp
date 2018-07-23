#include "oclexe.hpp"
#include <CL/cl.h>
#include <CL/cl_ext.h>
#include <pthread.h>
#include <semaphore.h>
#include <iostream>
#include "TimeMeasure.hpp"


/**********************************************************
* 函数名称：clCreateGPUKernels()
* 功能描述：初始化GPU
* 输入参数：无
* 返 回 值：无
**********************************************************/
void Affine::clCreateGPUKernels()
{
	cl_platform_id platform;
	ret = clGetPlatformIDs(1, &platform, NULL);
	ret = clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &clDev, NULL);

	clCtx = clCreateContext(NULL, 1, &clDev,NULL, NULL, &ret);
	clCmdQ = clCreateCommandQueue(clCtx, clDev, 0, &ret);

	char *kernelSource;
    std::ifstream kernelFile;
    kernelFile.open("/home/data/affine.cl");
	std::stringstream  strBuf;
	strBuf << kernelFile.rdbuf();
	std::string strKernel(strBuf.str());
	size_t len = strKernel.length();
	kernelSource = (char *)malloc(sizeof(char)*(len+1));
	strKernel.copy(kernelSource, len, 0);
	kernelSource[len] = '\0';
	
	clProgram = clCreateProgramWithSource(clCtx, 1, (const char**)&kernelSource, NULL, &ret);

	char buff[200] = "";
	sprintf(buff, "-DDST_STEP=%d -DDST_SCANLINE=%d -DDST_COLS=%d "
							  "-DSRC_STEP=%d -DSRC_SCANLINE=%d -DSRC_COLS=%d -DXOFFSET=%d -DYOFFSET=%d",
	eisOutStride / VECTOR, eisOutYscanLine, eisOutWidth, eisInStride, eisInYscanLine, eisInWidth, xOffset, yOffset);

	ret = clBuildProgram(clProgram, 0, NULL, buff, NULL, NULL);
	if (ret < 0) 
	{
		size_t logsize;
		clGetProgramBuildInfo(clProgram, clDev, CL_PROGRAM_BUILD_LOG, 0, NULL, &logsize);
		char *log = (char *)malloc(logsize*sizeof(char));
		clGetProgramBuildInfo(clProgram, clDev, CL_PROGRAM_BUILD_LOG, logsize, log, NULL);
		printf("log : \n%s\n", log);
		free(log);
		return;
	}

    clKernelAffine = clCreateKernel(clProgram, "warpAffine", &ret);
	free(kernelSource);
}

/**********************************************************
* 函数名称：executeKernel()
* 功能描述：GPU放射变换
* 输入参数：unsigned char *src        --  原始图像地址
*           int iondesc      		  --  原图像地址描述符
*           void* dst      			  --  目标地址
*           int outiondesc      	  --  目标地址描述符
*           float* M		     	  --  M矩阵地址
* 返 回 值：无
**********************************************************/
void Affine::executeKernel(unsigned char *src, int iondesc, void* dst, int outiondesc, float* M)
{
	gpu_cmp = false;
    TimeMeasure tm;
	tm.recordTime("kernel_exe");
	// double t = cv::getTickCount();
	cl_mem_ion_host_ptr  ionin  = {0};
	//创建输入图像GPU内存对象
	ionin.ext_host_ptr.allocation_type    = CL_MEM_ION_HOST_PTR_QCOM;
    ionin.ext_host_ptr.host_cache_policy  = CL_MEM_HOST_UNCACHED_QCOM;//CL_MEM_HOST_UNCACHED_QCOM;
    ionin.ion_filedesc                    = iondesc;
    ionin.ion_hostptr                     = src;
    cl_mem objSrc = clCreateBuffer(clCtx, CL_MEM_USE_HOST_PTR | CL_MEM_EXT_HOST_PTR_QCOM, eisInSize, &ionin, &ret);
	
	//创建输出图像GPU内存对象
	cl_mem_ion_host_ptr  ionout  = {0};
	ionout.ext_host_ptr.allocation_type    = CL_MEM_ION_HOST_PTR_QCOM;
    ionout.ext_host_ptr.host_cache_policy  = CL_MEM_HOST_UNCACHED_QCOM;
    ionout.ion_filedesc                    = outiondesc;
    ionout.ion_hostptr                     = dst;
    cl_mem objDst = clCreateBuffer(clCtx, CL_MEM_USE_HOST_PTR | CL_MEM_EXT_HOST_PTR_QCOM, eisOutSize, &ionout, &ret);

	//将M矩阵写入GPU内存
	clEnqueueWriteBuffer(clCmdQ, objM, CL_TRUE, 0, 9*sizeof(float), M, 0, NULL, NULL);

	//设置内核参数
    clSetKernelArg(clKernelAffine, 0, sizeof(cl_mem), &objSrc);
    clSetKernelArg(clKernelAffine, 1, sizeof(cl_mem), &objDst);
    clSetKernelArg(clKernelAffine, 2, sizeof(cl_mem), &objM);

	//执行内核函数
    size_t globalsize[2] = { (size_t)eisOutWidth / VECTOR, (size_t)eisOutHeight };
    ret = clEnqueueNDRangeKernel(clCmdQ, clKernelAffine, 2, NULL, globalsize, NULL, 0, NULL, NULL);
    clFinish(clCmdQ);

	//释放GPU内存对象
	clReleaseMemObject(objSrc);
	clReleaseMemObject(objDst);

	gpu_cmp = true;
	
    if (ret < 0)
	{
        fprintf(stderr, "[RESIZE] eis resize error - %d\n", ret);
    }
	if (tm.diffMsWithRecord("kernel_exe") > 50) 
	{
    	iplog("%s", tm.diffMsStringWithRecord("kernel_exe").c_str());
	}
}

/**********************************************************
* 函数名称：oclExeRun()
* 功能描述：GPU放射变换外部接口
* 输入参数：unsigned char *src        --  原始图像地址
*           int iondesc      		  --  原图像地址描述符
*           void* dst      			  --  目标地址
*           int outiondesc      	  --  目标地址描述符
*           float* M		     	  --  M矩阵地址
* 返 回 值：无
**********************************************************/
int Affine::oclExeRun(unsigned char *src, int iondesc, void* dst, int outiondesc, float *M)
{
	TimeMeasure tm;
	if (!gpu_cmp) return -1;

	// tm.recordTime("M_");
	//计算M的逆矩阵
	float D = M[0] * M[4] - M[1] * M[3];
	D = D != 0 ? 1. / D : 0;
	float A11 = M[4] * D, A22 = M[0] * D;
	M[0] = A11; M[1] *= -D;
	M[3] *= -D; M[4] = A22;
	float b1 = -M[0] * M[2] - M[1] * M[5];
	float b2 = -M[3] * M[2] - M[4] * M[5];
	M[2] = b1; M[5] = b2;

	//如果是开始启动，则创建M矩阵的GPU内存对象
	if (runCount == 0) 
	{
		clCreateGPUKernels();
		objM = clCreateBuffer(clCtx, CL_MEM_READ_ONLY | CL_MEM_ALLOC_HOST_PTR, 9*sizeof(float), NULL, &ret);
	}
	//执行核函数
	executeKernel(src, iondesc, dst, outiondesc, M);
	runCount++;
	return 0;
}

/**********************************************************
* 函数名称：oclExeStop()
* 功能描述：停止GPU仿射变换
* 输入参数：无
* 返 回 值：无
**********************************************************/
void Affine::oclExeStop(void)
{
	if (runCount == 0 || !gpu_cmp) return;

	clReleaseMemObject(objM);

	clReleaseKernel(clKernelAffine);
	clReleaseCommandQueue(clCmdQ);
	clReleaseContext(clCtx);
	clReleaseDevice(clDev);
	clReleaseProgram(clProgram);
	runCount = 0;
}




