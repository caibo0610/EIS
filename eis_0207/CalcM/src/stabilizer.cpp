#include <opencv2/highgui.hpp>
#include "stabilizer.hpp"
#include <string.h>
#include <fstream>
#include <stdio.h>

namespace cv
{
namespace videostab
{
StabilizerBase::StabilizerBase()
{
    setMotionEstimator(makePtr<KeypointBasedMotionEstimator>(makePtr<MotionEstimatorRansacL2>()));
    setRadius(15);
    setBorderMode(BORDER_REPLICATE);
}

void StabilizerBase::reset()
{
    frameSize_ = Size(0, 0);
    curPos_ = -1;
    curStabilizedPos_ = -1;
	gray_frames_.clear();
    motions_.clear();
	motions_flag_.clear();

#ifdef RETURN_IMG
	frames_.clear();
#endif
}

void StabilizerBase::setUp()
{
	frameSize_ = nextgrayframe_.size();
	int cacheSize = 2 * radius_ + 1;
	gray_frames_.resize(cacheSize);
	motions_.resize(cacheSize);
	motions_flag_.resize(cacheSize);

#ifdef 	RETURN_IMG
	frames_.resize(cacheSize);
#endif

	for (int i = -radius_; i < 0; ++i)
	{
		at(i, motions_) = Mat::eye(3, 3, CV_32F);
		at(i, motions_flag_) = 0;
		at(i, gray_frames_) = nextgrayframe_;

#ifdef RETURN_IMG
		at(i, frames_) = nextframe_;
#endif
	}

	at(0, gray_frames_) = nextgrayframe_;

#ifdef 	RETURN_IMG
	at(0, frames_) = nextframe_;
#endif
}

bool StabilizerBase::doOneIteration()
{
#ifdef DEBUG_LOG
	g_imStab_data.M_mat = cv::Mat::eye(3,3,CV_32F);
	g_imStab_data.M_init_Mat = cv::Mat::eye(3, 3, CV_32F);
	g_imStab_data.curPos = 0;
	g_imStab_data.curStabilizedPos = 0;
#endif

	if (!nextgrayframe_.empty())
	{
		curPos_++;

		g_imStab_data.curPos = curPos_;

		if (curPos_ > 0)
		{
			at(curPos_, gray_frames_) = nextgrayframe_;
#ifdef RETURN_IMG
			at(curPos_, frames_) = nextframe_;
#endif
			at(curPos_ - 1, motions_) = estimateMotion();
			if (IsMat)
			{
				at(curPos_ - 1, motions_flag_) = 1;
			}
			else
			{
				at(curPos_ - 1, motions_flag_) = 0;
			}
			
			if (curPos_ >= radius_)
			{
				curStabilizedPos_ = curPos_ - radius_;

#ifdef DEBUG_LOG
				g_imStab_data.curStabilizedPos = curStabilizedPos_;
#endif

				int false_cnt = 0;
				for (int i = -radius_; i <= radius_; i++)
				{
					int flag = at(curStabilizedPos_+i, motions_flag_);
					if (flag == 0)
					{
						false_cnt++;
					}
				}

#ifdef DEBUG_LOG
				g_imStab_data.IsMat_cnt = false_cnt;
#endif

				if (false_cnt > 2*radius_*0.1 || !IsMat)
				{
					stabilizationMotion_ = cv::Mat::eye(3, 3, CV_32F);
#ifdef DEBUG_LOG
					g_imStab_data.M_mat = stabilizationMotion_;
#endif
				}
				else
				{
					double M_mean_t = getTickCount();
					stabilizationMotion_ = estimateStabilizationMotion();
					M_mean_t = (getTickCount() - M_mean_t) / getTickFrequency() * 1000;
#ifdef DEBUG_LOG
					g_imStab_data.M_mean_time = M_mean_t;
					g_imStab_data.M_mat = stabilizationMotion_;
#endif				
				}

#ifdef RETURN_IMG

				stabilizeFrame();
#endif
				
			}
		}
		else
		{
			setUp();
		}
		return true;
	}

	return false;
}

OnePassStabilizer::OnePassStabilizer()
{
	detector_ = cv::FastFeatureDetector::create(40);
	detect_num = 8;
	match_points = std::vector<std::vector<cv::Point2f>>(detect_num);
    setMotionFilter(makePtr<GaussianMotionFilter>());


    detect_img1s = std::vector<cv::Mat>(detect_num, Mat());
	detect_img2s = std::vector<cv::Mat>(detect_num, Mat());

	key_points = std::vector<std::vector<KeyPoint>>(detect_num);
	detect_points = std::vector<std::vector<cv::Point2f>>(detect_num);
	boxes = std::vector<cv::Rect>(detect_num);
	boxes[0] = cv::Rect(200, 200, 100, 100); //第一行第一列
	boxes[1] = cv::Rect(800, 200, 100, 100); //第一行第二列
	boxes[2] = cv::Rect(1600, 200, 100, 100);//第一行第三列

	boxes[3] = cv::Rect(200, 500, 100, 100); //第二行第一列
	//boxes[4] = cv::Rect(800, 500, 60, 60); //第二行第二列
	boxes[4] = cv::Rect(1600, 500, 100, 100); //第二行第三列

	boxes[5] = cv::Rect(200, 800, 100, 100); //第三行第一列
	boxes[6] = cv::Rect(800, 800, 100, 100); //第三行第二列
	boxes[7] = cv::Rect(1600, 800, 100, 100);//第三行第三列

    reset();
}

void OnePassStabilizer::setUp()
{
	for (int i = 0; i < detect_num; i++)
	{
		detect_img1s[i] = nextgrayframe_(boxes[i]).clone();
	}

	StabilizerBase::setUp();
}

Mat OnePassStabilizer::NextFrame(Mat& grayframe)
{
	StabilizerBase::nextgrayframe_ = grayframe;
#ifdef RETURN_IMG
	StabilizerBase::nextframe_ = grayframe;
#endif

	bool processed;
	processed = doOneIteration();

	if (!processed)
	{
#ifdef RETURN_IMG
		return Mat();
#else
		return Mat::eye(3, 3, CV_32F);
#endif
	}

	if (processed && curStabilizedPos_ == -1)
	{
#ifdef RETURN_IMG
		return Mat();
#else
		return Mat::eye(3, 3, CV_32F);
#endif
	}

#ifdef RETURN_IMG
	return stabilizedFrames_;
#else
	return stabilizationMotion_;
#endif
}

Mat OnePassStabilizer::NextFrame(Mat& frame,Mat& grayframe)
{
	StabilizerBase::nextgrayframe_ = grayframe;
#ifdef RETURN_IMG
	StabilizerBase::nextframe_ = frame;
#endif

	bool processed;
	processed = doOneIteration();

	if (!processed)
	{
#ifdef RETURN_IMG
		return Mat();
#else
		return Mat::eye(3, 3, CV_32F);
#endif
	}

	if (processed && curStabilizedPos_ == -1)
	{
#ifdef RETURN_IMG
		return Mat();
#else
		return Mat::eye(3, 3, CV_32F);
#endif
	}

#ifdef RETURN_IMG
	return stabilizedFrames_;
#else
	return stabilizationMotion_;
#endif
}


void OnePassStabilizer::reset()
{
    StabilizerBase::reset();
}

Mat OnePassStabilizer::estimateMotion()
{
	cv::Mat frame1 = at(curPos_, gray_frames_);
	IsMat = true;
	bool match_flag = Match_points(frame1);

	if (match_flag)
	{
		cv::Mat M0 = motionEstimator_->estimate(pointsPrevGood_, pointsGood_, &IsMat);
		return M0;
	}
	else
	{
		IsMat = false;
		return cv::Mat::eye(3, 3, CV_32F);
	}
}

Mat OnePassStabilizer::estimateStabilizationMotion()
{
    return motionFilter_->stabilize(curStabilizedPos_, motions_, std::make_pair(0, curPos_));
}

bool OnePassStabilizer::Match_points(cv::Mat& track_img)
{
#ifdef DEBUG_LOG
	g_imStab_data.points_num = std::vector<int>(detect_num, 0);
	g_imStab_data.detect_points_num = 0;
	g_imStab_data.lk_points_num = 0;
	g_imStab_data.filter_points_num = 0;
	g_imStab_data.pointsGood.clear();
	g_imStab_data.pointsPrevGood.clear();
	g_imStab_data.lk_pointsGood.clear();
	g_imStab_data.lk_pointsPrevGood.clear();
#endif

	omp_set_num_threads(4);

	if (true || pointsGood_.size() < 80)
	{
		pointsPrev_.clear();
		points_.clear();
		// find keypoints
		for (int i = 0; i < detect_num; i++)
		{
			match_points[i].clear();
			key_points[i].clear();
			detect_points[i].clear();
			//detect_img1s[i] = detect_img(boxes[i]).clone();
			detect_img2s[i] = track_img(boxes[i]).clone();
			detector_->detect(detect_img1s[i], key_points[i]);

			if (key_points[i].size() > 0)
			{
				filterPts(key_points[i], detect_points[i], 20);

				for (unsigned int j = 0; j < detect_points[i].size(); j++)
				{
					cv::Point2f pt;
					pt.x = detect_points[i][j].x + boxes[i].x;
					pt.y = detect_points[i][j].y + boxes[i].y;
					pointsPrev_.push_back(pt);
				}
			}
		}
	}
	else
	{
		pointsPrev_.clear();
		points_.clear();
		for (int i = 0; i < detect_num; i++)
		{
			detect_points[i].clear();
			detect_points[i] = match_points[i];
			match_points[i].clear();
			//detect_img1s[i] = detect_img(boxes[i]).clone();
			detect_img2s[i] = track_img(boxes[i]).clone();
			for (unsigned int j = 0; j < detect_points[i].size(); j++)
			{
				cv::Point2f pt;
				pt.x = detect_points[i][j].x + boxes[i].x;
				pt.y = detect_points[i][j].y + boxes[i].y;
				pointsPrev_.push_back(pt);
			}
		}
	}


	if (pointsPrev_.size() < 80)
	{
		for (int i = 0; i < detect_num; i++)
		{
			swap(detect_img1s[i], detect_img2s[i]);
		}
		return false;
	}

#ifdef DEBUG_LOG
	for (int i = 0; i < detect_num; i++)
	{
		g_imStab_data.points_num[i] = detect_points[i].size();
	}
	g_imStab_data.detect_points_num = pointsPrev_.size();
#endif

	cv::TermCriteria term_criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 20, 0.03);
	cv::Size window_size = cv::Size(4, 4);
	int level = 3;
	std::vector<uchar>status_flag;
	status_.clear();
	for (int i = 0; i < detect_num; i++)
	{
		if (detect_points[i].size()>0)
		{
			status_flag.clear();
			my_calcOpticalFlowPyrLK(detect_img1s[i], detect_img2s[i], detect_points[i], match_points[i], status_flag, noArray(), window_size, level, term_criteria);
			for (unsigned int j = 0; j < match_points[i].size(); j++)
			{
				cv::Point2f pt;
				pt.x = match_points[i][j].x + boxes[i].x;
				pt.y = match_points[i][j].y + boxes[i].y;
				points_.push_back(pt);
				status_.push_back(status_flag[j]);
			}
		}

		swap(detect_img1s[i], detect_img2s[i]);
	}
	// leave good correspondences only

	pointsPrevGood_.clear(); pointsPrevGood_.reserve(points_.size());
	pointsGood_.clear(); pointsGood_.reserve(points_.size());

	for (size_t i = 0; i < points_.size(); ++i)
	{
		if (status_[i])
		{
			pointsPrevGood_.push_back(pointsPrev_[i]);
			pointsGood_.push_back(points_[i]);
		}
	}

#ifdef DEBUG_LOG
	g_imStab_data.lk_points_num = pointsPrevGood_.size();
	g_imStab_data.lk_pointsGood = pointsGood_;
	g_imStab_data.lk_pointsPrevGood = pointsPrevGood_;
#endif

	if (pointsPrevGood_.size() < 3)
	{
		return false;
	}

	bool filter_flag = filter_Match(pointsPrevGood_, pointsGood_);

#ifdef DEBUG_LOG
	g_imStab_data.filter_match_flag = filter_flag;
#endif
	if (!filter_flag)
	{
		return false;
	}

#ifdef DEBUG_LOG
	g_imStab_data.filter_points_num = pointsGood_.size();

	g_imStab_data.pointsGood = pointsGood_;
	g_imStab_data.pointsPrevGood = pointsPrevGood_;

	// cv::Mat show_img = track_img.clone();
	// std::vector<double>dis_points;
	// for (unsigned int i = 0; i < pointsPrevGood_.size(); i++)
	// {
	// 	int scale = 1;
	//  	cv::circle(show_img, pointsPrevGood_[i], 1, cv::Scalar(0, 0, 255), 1);
	//  	cv::circle(show_img, pointsGood_[i], 1, cv::Scalar(255, 0, 255), 1);
	//  	dis_points.push_back(norm(pointsPrevGood_[i] - pointsGood_[i]));
	//  	cv::line(show_img, cv::Point2f(pointsPrevGood_[i].x*scale, pointsPrevGood_[i].y * scale), cv::Point2f(pointsGood_[i].x * scale, pointsGood_[i].y * scale), cv::Scalar(0, 0, 255), 2);
	// }

	// cv::namedWindow("show_img", WINDOW_NORMAL);
	// cv::imshow("show_img", show_img);
	// cv::waitKey(10);

	// std::cout << pointsPrev_.size() << std::endl;
	// std::cout << "detect is: " << keypoints_detect_t << std::endl;
	// std::cout << "match is : " << keypoints_match_t << std::endl;
#endif
	return true;
}

bool OnePassStabilizer::filterPts(std::vector<KeyPoint>& in_points, std::vector<cv::Point2f>& out_points, int points_num_)
{
	unsigned int points_num = points_num_;
	if (in_points.size() > points_num)
	{
		int p_times = in_points.size() / points_num;
		for (unsigned int i = 0; i < points_num; i++)
		{
			unsigned int index = i*p_times;
			if (index < in_points.size())
			{
				out_points.push_back(in_points[i*p_times].pt);
			}
		}
	}
	else
	{
		for (unsigned int i = 0; i < in_points.size(); i++)
		{
			out_points.push_back(in_points[i].pt);
		}
	}

	return true;
}

bool OnePassStabilizer::filter_Match(std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2)
{
#ifdef DEBUG_LOG
	g_imStab_data.false_return_num = 0;
#endif

	std::vector<float>distance_x(points1.size(), 0.0f);
	std::vector<float>distance_y(points1.size(), 0.0f);
	double var_x_sum = 0.0f;
	double mean_x_sum = 0.0f;
	double var_y_sum = 0.0f;
	double mean_y_sum = 0.0f;
	for (unsigned int i = 0; i < points1.size(); i++)
	{
		float dis_x = points1[i].x - points2[i].x;
		float dis_y = points1[i].y - points2[i].y;
		distance_x[i] = dis_x;
		distance_y[i] = dis_y;

		mean_x_sum += distance_x[i];
		mean_y_sum += distance_y[i];
	}
	double mean_x = mean_x_sum / points1.size();
	double mean_y = mean_y_sum / points1.size();


	float median_x = median(distance_x);
	float median_y = median(distance_y);

	if (abs(median_x - mean_x) > 4 || abs(median_y - mean_y) > 4)
	{
#ifdef DEBUG_LOG
		g_imStab_data.false_return_num = 1;
#endif
		return false;
	}


	mean_x_sum = 0;
	mean_y_sum = 0;

	unsigned int k = 0;
	for (unsigned int i = 0; i < points1.size(); i++)
	{
		if (abs(distance_x[i] - median_x) < 4 && abs(distance_y[i] - median_y) < 4)
		{
			mean_x_sum += distance_x[i];
			mean_y_sum += distance_y[i];

			points1[k] = points1[i];
			points2[k] = points2[i];
			distance_x[k] = distance_x[i];
			distance_y[k] = distance_y[i];

			k++;
		}
	}


	if (k > points1.size() / 3 && k > 40)
	{
		points1.resize(k);
		points2.resize(k);
		distance_x.resize(k);
		distance_y.resize(k);
		mean_x = mean_x_sum / points1.size();
		mean_y = mean_y_sum / points1.size();
		var_x_sum = 0;
		var_y_sum = 0;
		for (unsigned int i = 0; i < points1.size(); i++)
		{
			var_x_sum += (distance_x[i] - mean_x)*(distance_x[i] - mean_x);
			var_y_sum += (distance_y[i] - mean_y)*(distance_y[i] - mean_y);
		}
		double var_x = sqrtf(var_x_sum / points1.size());
		double var_y = sqrtf(var_y_sum / points1.size());

		if (var_x > 2 || var_y > 2)
		{
#ifdef DEBUG_LOG
			g_imStab_data.false_return_num = 2;
#endif
			return false;
		}
		else
		{
			return true;
		}
	}

#ifdef DEBUG_LOG
	g_imStab_data.false_return_num = 3;
#endif
	return false;
}

#ifdef RETURN_IMG
void StabilizerBase::stabilizeFrame()
{

	double warpAffine_t = getTickCount();

	cv::Mat preProcessedFrame_ = at(curStabilizedPos_, frames_);

	// apply stabilization transformation

	warpAffine(
		preProcessedFrame_, stabilizedFrames_,
		stabilizationMotion_(Rect(0, 0, 3, 2)), frameSize_, INTER_LINEAR, borderMode_);

	warpAffine_t = (getTickCount() - warpAffine_t) / getTickFrequency() * 1000;

#ifdef DEBUG_LOG
	g_imStab_data.warpAffine_time = warpAffine_t;
#endif
}
#endif

} // namespace videostab
} // namespace cv
