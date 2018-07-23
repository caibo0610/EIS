#ifndef __OCLEXE_HPP
#define __OCLEXE_HPP

#include <opencv2/opencv.hpp>
#include <CL/cl.h>
#include <CL/cl_ext.h>

#define EIS_DEBUG

#ifdef EIS_DEBUG
#define iplog(fmt, args...) \
do { \
	struct timeval now; \
	gettimeofday(&now, NULL); \
	fprintf(stderr, "[EIS][%5lu.%06lu] %s() %d: " fmt "\n", \
			now.tv_sec, now.tv_usec, __FUNCTION__, __LINE__, ##args); \
} while (0)
#else
#define iplog(fmt, args...)
#endif

#define VECTOR		4
// #define GPU_IN_BUFF_SIZE    5
// #define GPU_OUT_BUFF_SIZE   2
// #define EIS_OUT_WIDTH       1920
// #define EIS_OUT_HEIGHT      1080
// #define EIS_OUT_STRIDE      1920
// #define EIS_OUT_YSCANLINE	1088
// #define EIS_OUT_UVOFFST     (EIS_OUT_YSCANLINE*EIS_OUT_STRIDE)
// #define EIS_OUT_UV_HEIGHT   (EIS_OUT_HEIGHT/2)
// #define EIS_OUT_UV_SZIE     (544*EIS_OUT_STRIDE)
// #define EIS_OUT_SIZE        (EIS_OUT_UVOFFST+EIS_OUT_UV_SZIE)


// static int EIS_IN_HEIGHT	=	1296
// static int EIS_IN_YSCANLINE	=	1312
// static int EIS_IN_STRIDE	=   2304
// static int EIS_IN_WIDTH     =	2304
// static int EIS_IN_SIZE      =	(2304*1968)

// #define X_OFFSET	48 	// ((2304-1920) / 4 / 2)
// #define Y_OFFSET	108	// (1296-1080) / 2

class Affine {
public:
	Affine(int inW, int inH, int outW, int outH) {
		gpu_cmp = true;
		runCount = 0;

		eisInWidth = inW;
		eisInHeight = inH;
		eisOutWidth = outW;
		eisOutHeight= outH;

		eisInYscanLine = ((int)(ceil(eisInHeight/32.0f))) * 32;
		eisInStride    = ((int)(ceil(eisInWidth/128.0f))) * 128;
		eisOutYscanLine = ((int)(ceil(eisOutHeight/32.0f))) * 32;
		eisOutStride    = ((int)(ceil(eisOutWidth/128.0f))) * 128;

		eisInSize  = (eisInYscanLine+eisInYscanLine/2) * eisInStride;
		eisOutSize = (eisOutYscanLine+eisOutYscanLine/2) * eisOutStride;

		xOffset = ((eisInWidth-eisOutWidth) / 4 / 2);
		yOffset = (eisInHeight-eisOutHeight) / 2;

		fprintf(stderr, "[EIS] ^^^^^^^^^^^^^ %d %d %d %d ^^^^^^^^^^\n", inW, inH, outW, outH);
	}
	~Affine();

	int  oclExeRun(unsigned char *src, int iondesc, void* dst, int outiondesc, float *M);
	void oclExeStop(void);

private:

	void clCreateGPUKernels();
	void executeKernel(unsigned char *src, int iondesc, void* dst, int outiondesc, float* M);
	//=========================================
	//	Variable
	//=========================================
	int runCount;								//���д���
	volatile bool gpu_cmp;						//gpu�Ƿ�������
	int eisOutStride;							//���ͼ��ʵ�ʿ��
	int eisOutYscanLine;						//���ͼ��y����ʵ�ʸ߶�
	int eisOutWidth;							//���ͼ���
	int eisOutHeight;							//���ͼ���
	int eisInStride;							//����ͼ��ʵ�ʿ��
	int eisInYscanLine;						//����ͼ��y����ʵ�ʸ߶�
	int eisInWidth;							//����ͼ���
	int eisInHeight;							//����ͼ���
	int xOffset;								//�ü�xƫ����
	int yOffset;								//�ü�yƫ����
	int eisInSize;								//����ͼ���ܴ�С
	int eisOutSize;							//���ͼ���ܴ�С
	
	//
	// cl object
	//
	cl_kernel clKernelAffine;					//GPU  kernel����
	cl_program clProgram;						//GPU  program����
	cl_device_id clDev;							//GPU  device����
	cl_context clCtx;							//GPU  context����
	cl_command_queue clCmdQ;					//GPU  cmdqueue����
	cl_int ret;
	cl_mem objM;								//M����GPU�ڴ����
};


/**********************************************************
* �������ƣ�memcpy_neon()
* ����������neon����
* ���������int8_t *dst       --  Ŀ���ַ
*           int8_t *src       --  Դ��ַ
*           size_t size	      --  �����ֽ���
* �� �� ֵ����
**********************************************************/
inline void memcpy_neon(int8_t *dst, int8_t *src, size_t size)
{
	int remainder = size % 16;  //pic size % 16 = 0
	int count_neon = size >> 4;
	while(count_neon--) {
		vst1q_s8(dst, vld1q_s8(src));
		dst += 16;
		src += 16;
	}
	memcpy(dst, src, remainder);
}

/**********************************************************
* �������ƣ�memcpy_neon16()
* ����������16�ֽڶ���neon����
* ���������int8_t *dst       --  Ŀ���ַ
*           int8_t *src       --  Դ��ַ
*           size_t size	      --  �����ֽ���
* �� �� ֵ����
**********************************************************/
inline void memcpy_neon16(int8_t *dst, int8_t *src, size_t size)
{
	int count_neon = size >> 4;
	while(count_neon--) {
		vst1q_s8(dst, vld1q_s8(src));
		dst += 16;
		src += 16;
	}
}

/**********************************************************
* �������ƣ�memcpy_neon8()
* ����������8�ֽڶ���neon����
* ���������int8_t *dst       --  Ŀ���ַ
*           int8_t *src       --  Դ��ַ
*           size_t size	      --  �����ֽ���
* �� �� ֵ����
**********************************************************/
inline void memcpy_neon8(int8_t *dst, int8_t *src, size_t size)
{
	int count_neon = size >> 3;
	while(count_neon--) {
		vst1q_s8(dst, vld1q_s8(src));
		dst += 8;
		src += 8;
	}
}

#endif
