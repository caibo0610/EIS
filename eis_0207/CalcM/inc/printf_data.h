#pragma once
#include <stdio.h>

/* 跟踪算法数据 */
typedef struct
{
	int frameID;

	double keypoints_detect_time;//角点检测耗时
	double keypoints_match_time;//角点匹配耗时
	double M_compute_time; //M矩阵计算耗时
	double stabilizeframe_time; //稳像耗时
	double M_mean_time; //稳像时求M矩阵平均值耗时
	double warpAffine_time;;//仿射变换耗时
	
	cv::Mat M_mat;
	cv::Mat M_init_Mat;
	std::vector<cv::Point2f>pointsPrevGood;
	std::vector<cv::Point2f>pointsGood;

	std::vector<cv::Point2f>lk_pointsPrevGood;
	std::vector<cv::Point2f>lk_pointsGood;

	std::vector<int>points_num;
	bool filter_match_flag;
	int detect_points_num;
	int lk_points_num;
	int filter_points_num;
	int false_return_num;
	int IsMat_cnt;
	int curPos;
	int curStabilizedPos;

} imStabkData;

extern imStabkData g_imStab_data;
