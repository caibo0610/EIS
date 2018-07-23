#define _SCL_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <omp.h>
#include "stabilizer.hpp"
#include <string>  
#include <iostream>
#include "TimeMeasure.hpp"

using namespace std;
using namespace cv;
using namespace videostab;

OnePassStabilizer stabilizer;
imStabkData g_imStab_data;


void videostabframe_set()
{
	double estPara = 0.1;
	Ptr<MotionEstimatorRansacL2> est = makePtr<MotionEstimatorRansacL2>(MM_AFFINE);
												
	RansacParams ransac = est->ransacParams();
	ransac.size = 3;
	ransac.thresh = 5;
	ransac.eps = 0.5;
	est->setRansacParams(ransac);
	est->setMinInlierRatio(estPara);


	Ptr<KeypointBasedMotionEstimator> motionEstBuilder =
		makePtr<KeypointBasedMotionEstimator>(est); 

	int radius_pass = 9;
	
	stabilizer.setMotionFilter(makePtr<GaussianMotionFilter>(radius_pass));
	
	int radius = 3;
	// bool incl_constr = false;
	stabilizer.setMotionEstimator(motionEstBuilder);
	stabilizer.setRadius(radius);
	stabilizer.setBorderMode(BORDER_REPLICATE);
}


Mat calcM(Mat &img, int i_h, int i_w, int o_h, int o_w)
{
	// TimeMeasure tm;
	// tm.recordTime("calc_M");
	int offset_w = 0, offset_h = 0;
	offset_w = ((i_w - o_w)/2) & (~0x1);
	offset_h = ((i_h - o_h)/2) & (~0x1);
	Mat y_frame = img(Rect(offset_w, offset_h, o_w, o_h));
	Mat M = stabilizer.NextFrame(y_frame);
	// std::cout << tm.diffMsStringWithRecord("calc_M") << std::endl;
	return M;
}
