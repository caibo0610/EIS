#ifndef __CALCM_HPP
#define __CALCM_HPP

#include <opencv2/core.hpp>

void videostabframe_set();
cv::Mat calcM(cv::Mat &img, int i_h, int i_w, int o_h, int o_w);

#endif
