#include "videostab_util.hpp"

void match_template_uchar(cv::Mat frame, cv::Mat img, float &simi)
{
	float sum = 0.0f;
	float sum1 = 0.0f;
	float sum2 = 0.0f;
	for (int i = 0; i<frame.cols; i++)
	{
		for (int j = 0; j<frame.rows; j++)
		{
			sum = sum + float(frame.at<uchar>(i, j))*float(img.at<uchar>(i, j));
			sum1 = sum1 + float(frame.at<uchar>(i, j))*float(frame.at<uchar>(i, j));
			sum2 = sum2 + float(img.at<uchar>(i, j))*float(img.at<uchar>(i, j));
		}
	}
	simi = float(sum / (sqrt(sum1*sum2)));
}
float median(std::vector<float> v)
{
	int n = floor(double(v.size() / 2));
	nth_element(v.begin(), v.begin() + n, v.end());
	return v[n];
}