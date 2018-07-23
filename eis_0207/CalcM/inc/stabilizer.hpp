#include <vector>
#include <ctime>
#include "motion_stabilizing.hpp"

namespace cv
{
namespace videostab
{

class CV_EXPORTS StabilizerBase
{
public:

	Mat nextgrayframe_;
#ifdef RETURN_IMG
	Mat nextframe_;
#endif
	
    virtual ~StabilizerBase() {}
    void setRadius(int val) { radius_ = val; }
    int radius() const { return radius_; }

    void setMotionEstimator(Ptr<ImageMotionEstimatorBase> val) { motionEstimator_ = val; }
    Ptr<ImageMotionEstimatorBase> motionEstimator() const { return motionEstimator_; }

    void setBorderMode(int val) { borderMode_ = val; }
    int borderMode() const { return borderMode_; }


protected:
    StabilizerBase();
    void reset();
    bool doOneIteration();
	virtual void setUp();
    virtual Mat estimateMotion() = 0;
    virtual Mat estimateStabilizationMotion() = 0;

    Ptr<ImageMotionEstimatorBase> motionEstimator_;
    int radius_;
    float trimRatio_;
    int borderMode_;

    Size frameSize_;
    int curPos_;
    int curStabilizedPos_;
	cv::Mat stabilizationMotion_;
	std::vector<Mat> gray_frames_;
    std::vector<Mat> motions_; // motions_[i] is the motion from i-th to i+1-th frame
	std::vector<int>motions_flag_;
	bool IsMat;
#ifdef RETURN_IMG
	void stabilizeFrame();
	std::vector<Mat> frames_;
	Mat stabilizedFrames_;
#endif
};

class CV_EXPORTS OnePassStabilizer : public StabilizerBase
{
public:
    OnePassStabilizer();
    void setMotionFilter(Ptr<MotionFilterBase> val) { motionFilter_ = val; }
    Ptr<MotionFilterBase> motionFilter() const { return motionFilter_; }

	Mat NextFrame(Mat& grayframe);
	Mat NextFrame(Mat& frame, Mat& grayframe);

    virtual void reset();

	Ptr<FastFeatureDetector> detector_;
	bool Match_points(Mat& track_img);
	bool filter_Match(std::vector<Point2f>& points1, std::vector<Point2f>& points2);
	bool filterPts(std::vector<KeyPoint>& points1, std::vector<Point2f>& points2, int points_num_ = 50);
	int detect_num;
	std::vector<uchar> status_;
	std::vector<Point2f> pointsPrev_, points_;
	std::vector<Point2f> pointsPrevGood_, pointsGood_;
	std::vector<std::vector<Point2f>> match_points;

	std::vector<cv::Mat>detect_img1s;
	std::vector<cv::Mat>detect_img2s;
	std::vector<std::vector<KeyPoint>>key_points;
	std::vector<std::vector<cv::Point2f>>detect_points;
	std::vector<cv::Rect>boxes;

protected:
    virtual Mat estimateMotion();
    virtual Mat estimateStabilizationMotion();
	virtual void setUp();

    Ptr<MotionFilterBase> motionFilter_;

};

} // namespace videostab
} // namespace cv
