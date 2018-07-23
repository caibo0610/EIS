#pragma once
#include <stdio.h>

/* �����㷨���� */
typedef struct
{
	int frameID;

	double keypoints_detect_time;//�ǵ����ʱ
	double keypoints_match_time;//�ǵ�ƥ���ʱ
	double M_compute_time; //M��������ʱ
	double stabilizeframe_time; //�����ʱ
	double M_mean_time; //����ʱ��M����ƽ��ֵ��ʱ
	double warpAffine_time;;//����任��ʱ
	
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
