#ifndef OPENCV_VIDEOSTAB_GLOBAL_MOTION_HPP
#define OPENCV_VIDEOSTAB_GLOBAL_MOTION_HPP

#include <vector>
#include <fstream>
#include "optical_flow.hpp"
#include "motion_core.hpp"
#include "printf_data.h"
#include "omp.h"

namespace cv
{
namespace videostab
{

CV_EXPORTS Mat estimateGlobalMotionLeastSquares(
        InputOutputArray points0, InputOutputArray points1, int model = MM_AFFINE,
        float *rmse = 0);


CV_EXPORTS Mat estimateGlobalMotionRansac(
        InputArray points0, InputArray points1, int model = MM_AFFINE,
        const RansacParams &params = RansacParams::default2dMotion(MM_AFFINE),
        float *rmse = 0, int *ninliers = 0);


class CV_EXPORTS MotionEstimatorBase
{
public:
    virtual ~MotionEstimatorBase() {}

    virtual void setMotionModel(MotionModel val) { motionModel_ = val; }

    virtual MotionModel motionModel() const { return motionModel_; }

    virtual Mat estimate(InputArray points0, InputArray points1, bool *ok = 0) = 0;

protected:
    MotionEstimatorBase(MotionModel model) { setMotionModel(model); }

private:
    MotionModel motionModel_;
};


class CV_EXPORTS MotionEstimatorRansacL2 : public MotionEstimatorBase
{
public:
    MotionEstimatorRansacL2(MotionModel model = MM_AFFINE);

    void setRansacParams(const RansacParams &val) { ransacParams_ = val; }
    RansacParams ransacParams() const { return ransacParams_; }

    void setMinInlierRatio(float val) { minInlierRatio_ = val; }
    float minInlierRatio() const { return minInlierRatio_; }

    virtual Mat estimate(InputArray points0, InputArray points1, bool *ok = 0);

private:
    RansacParams ransacParams_;
    float minInlierRatio_;
};


class CV_EXPORTS ImageMotionEstimatorBase
{
public:
    virtual ~ImageMotionEstimatorBase() {}

    virtual void setMotionModel(MotionModel val) { motionModel_ = val; }
    virtual MotionModel motionModel() const { return motionModel_; }

    virtual Mat estimate(std::vector<cv::Point2f>& pointsPrevGood_, std::vector<cv::Point2f>& pointsGood_,bool *ok = 0) = 0;

protected:
    ImageMotionEstimatorBase(MotionModel model) { setMotionModel(model); }

private:
    MotionModel motionModel_;
};


class CV_EXPORTS KeypointBasedMotionEstimator : public ImageMotionEstimatorBase
{
public:
    KeypointBasedMotionEstimator(Ptr<MotionEstimatorBase> estimator);

    virtual void setMotionModel(MotionModel val) { motionEstimator_->setMotionModel(val); }
    virtual MotionModel motionModel() const { return motionEstimator_->motionModel(); }

    virtual Mat estimate(std::vector<cv::Point2f>& pointsPrevGood_, std::vector<cv::Point2f>& pointsGood_,bool *ok = 0);

private:
    Ptr<MotionEstimatorBase> motionEstimator_;
};


CV_EXPORTS Mat getMotion(int from, int to, const std::vector<Mat> &motions);

} // namespace videostab
} // namespace cv

#endif
