#include <vector>
#include <stdexcept>
#include <iostream>
#include <ctime>
#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include "printf_data.h"


//#define RETURN_IMG
//#define DEBUG_LOG

//#include "opencv2/core/private.hpp"

inline float sqr(float x) { return x * x; }

template <typename T> inline T& at(int idx, std::vector<T> &items)
{
	return items[cv::borderInterpolate(idx, static_cast<int>(items.size()), cv::BORDER_WRAP)];
}

template <typename T> inline const T& at(int idx, const std::vector<T> &items)
{
	return items[cv::borderInterpolate(idx, static_cast<int>(items.size()), cv::BORDER_WRAP)];
}

float median(std::vector<float> v);
void match_template_uchar(cv::Mat frame, cv::Mat img, float &simi);