#include "motion_stabilizing.hpp"

namespace cv
{
namespace videostab
{
void MotionFilterBase::stabilize(
        int size, const std::vector<Mat> &motions, std::pair<int,int> range, Mat *stabilizationMotions)
{
    for (int i = 0; i < size; ++i)
        stabilizationMotions[i] = stabilize(i, motions, range);
}

void GaussianMotionFilter::setParams(int _radius, float _stdev)
{
    radius_ = _radius;
    stdev_ = _stdev > 0.f ? _stdev : std::sqrt(static_cast<float>(_radius));

    float sum = 0;
    weight_.resize(2*radius_ + 1);
    for (int i = -radius_; i <= radius_; ++i)
        sum += weight_[radius_ + i] = std::exp(-i*i/(stdev_*stdev_));
    for (int i = -radius_; i <= radius_; ++i)
        weight_[radius_ + i] /= sum;
}

Mat GaussianMotionFilter::stabilize(int idx, const std::vector<Mat> &motions, std::pair<int, int> range)
{
	const Mat &cur = at(idx, motions);
	Mat res = Mat::zeros(cur.size(), cur.type());
	float sum = 0.f;
	int iMin = std::max(idx - radius_, range.first);
	int iMax = std::min(idx + radius_, range.second);
	for (int i = iMin; i <= iMax; ++i)
	{
		//res += weight_[radius_ + i - idx] * getMotion(i, idx, motions);
		res += weight_[radius_ + i - idx] * getMotion(idx, i, motions);
		sum += weight_[radius_ + i - idx];
	}
	return sum > 0.f ? res / sum : Mat::eye(cur.size(), cur.type());
}

} // namespace videostab
} // namespace cv
