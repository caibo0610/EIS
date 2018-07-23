#include <opencv2/highgui.hpp>
#include "global_motion.hpp"

using namespace std;

//#include "opencv2/core/private.cuda.hpp"

namespace cv
{
namespace videostab
{

// does isotropic normalization
static Mat normalizePoints(int npoints, Point2f *points)
{
    float cx = 0.f, cy = 0.f;
    for (int i = 0; i < npoints; ++i)
    {
        cx += points[i].x;
        cy += points[i].y;
    }
    cx /= npoints;
    cy /= npoints;

    float d = 0.f;
    for (int i = 0; i < npoints; ++i)
    {
        points[i].x -= cx;
        points[i].y -= cy;
        d += std::sqrt(sqr(points[i].x) + sqr(points[i].y));
    }
    d /= npoints;

    float s = std::sqrt(2.f) / d;
    for (int i = 0; i < npoints; ++i)
    {
        points[i].x *= s;
        points[i].y *= s;
    }

    Mat_<float> T = Mat::eye(3, 3, CV_32F);
    T(0,0) = T(1,1) = s;
    T(0,2) = -cx*s;
    T(1,2) = -cy*s;
    return T;
}


static Mat estimateGlobMotionLeastSquaresTranslation(
        int npoints, Point2f *points0, Point2f *points1, float *rmse)
{
    Mat_<float> M = Mat::eye(3, 3, CV_32F);
    for (int i = 0; i < npoints; ++i)
    {
        M(0,2) += points1[i].x - points0[i].x;
        M(1,2) += points1[i].y - points0[i].y;
    }
    M(0,2) /= npoints;
    M(1,2) /= npoints;

    if (rmse)
    {
        *rmse = 0;
        for (int i = 0; i < npoints; ++i)
            *rmse += sqr(points1[i].x - points0[i].x - M(0,2)) +
                     sqr(points1[i].y - points0[i].y - M(1,2));
        *rmse = std::sqrt(*rmse / npoints);
    }

    return M;
}


static Mat estimateGlobMotionLeastSquaresTranslationAndScale(
        int npoints, Point2f *points0, Point2f *points1, float *rmse)
{
    Mat_<float> T0 = normalizePoints(npoints, points0);
    Mat_<float> T1 = normalizePoints(npoints, points1);

    Mat_<float> A(2*npoints, 3), b(2*npoints, 1);
    float *a0, *a1;
    Point2f p0, p1;

    for (int i = 0; i < npoints; ++i)
    {
        a0 = A[2*i];
        a1 = A[2*i+1];
        p0 = points0[i];
        p1 = points1[i];
        a0[0] = p0.x; a0[1] = 1; a0[2] = 0;
        a1[0] = p0.y; a1[1] = 0; a1[2] = 1;
        b(2*i,0) = p1.x;
        b(2*i+1,0) = p1.y;
    }

    Mat_<float> sol;
    solve(A, b, sol, DECOMP_NORMAL | DECOMP_LU);

    if (rmse)
        *rmse = static_cast<float>(norm(A*sol, b, NORM_L2) / std::sqrt(static_cast<double>(npoints)));

    Mat_<float> M = Mat::eye(3, 3, CV_32F);
    M(0,0) = M(1,1) = sol(0,0);
    M(0,2) = sol(1,0);
    M(1,2) = sol(2,0);

    return T1.inv() * M * T0;
}

static Mat estimateGlobMotionLeastSquaresRotation(
        int npoints, Point2f *points0, Point2f *points1, float *rmse)
{
    Point2f p0, p1;
    float A(0), B(0);
    for(int i=0; i<npoints; ++i)
    {
        p0 = points0[i];
        p1 = points1[i];

        A += p0.x*p1.x + p0.y*p1.y;
        B += p0.x*p1.y - p1.x*p0.y;
    }

    // A*sin(alpha) + B*cos(alpha) = 0
    float C = std::sqrt(A*A + B*B);
    Mat_<float> M = Mat::eye(3, 3, CV_32F);
    if ( C != 0 )
    {
        float sinAlpha = - B / C;
        float cosAlpha = A / C;

        M(0,0) = cosAlpha;
        M(1,1) = M(0,0);
        M(0,1) = sinAlpha;
        M(1,0) = - M(0,1);
    }

    if (rmse)
    {
        *rmse = 0;
        for (int i = 0; i < npoints; ++i)
        {
            p0 = points0[i];
            p1 = points1[i];
            *rmse += sqr(p1.x - M(0,0)*p0.x - M(0,1)*p0.y) +
                     sqr(p1.y - M(1,0)*p0.x - M(1,1)*p0.y);
        }
        *rmse = std::sqrt(*rmse / npoints);
    }

    return M;
}

static Mat  estimateGlobMotionLeastSquaresRigid(
        int npoints, Point2f *points0, Point2f *points1, float *rmse)
{
    Point2f mean0(0.f, 0.f);
    Point2f mean1(0.f, 0.f);

    for (int i = 0; i < npoints; ++i)
    {
        mean0 += points0[i];
        mean1 += points1[i];
    }

    mean0 *= 1.f / npoints;
    mean1 *= 1.f / npoints;

    Mat_<float> A = Mat::zeros(2, 2, CV_32F);
    Point2f pt0, pt1;

    for (int i = 0; i < npoints; ++i)
    {
        pt0 = points0[i] - mean0;
        pt1 = points1[i] - mean1;
        A(0,0) += pt1.x * pt0.x;
        A(0,1) += pt1.x * pt0.y;
        A(1,0) += pt1.y * pt0.x;
        A(1,1) += pt1.y * pt0.y;
    }

    Mat_<float> M = Mat::eye(3, 3, CV_32F);

    SVD svd(A);
    Mat_<float> R = svd.u * svd.vt;
    Mat tmp(M(Rect(0,0,2,2)));
    R.copyTo(tmp);

    M(0,2) = mean1.x - R(0,0)*mean0.x - R(0,1)*mean0.y;
    M(1,2) = mean1.y - R(1,0)*mean0.x - R(1,1)*mean0.y;

    if (rmse)
    {
        *rmse = 0;
        for (int i = 0; i < npoints; ++i)
        {
            pt0 = points0[i];
            pt1 = points1[i];
            *rmse += sqr(pt1.x - M(0,0)*pt0.x - M(0,1)*pt0.y - M(0,2)) +
                     sqr(pt1.y - M(1,0)*pt0.x - M(1,1)*pt0.y - M(1,2));
        }
        *rmse = std::sqrt(*rmse / npoints);
    }

    return M;
}


static Mat estimateGlobMotionLeastSquaresSimilarity(
        int npoints, Point2f *points0, Point2f *points1, float *rmse)
{
    Mat_<float> T0 = normalizePoints(npoints, points0);
    Mat_<float> T1 = normalizePoints(npoints, points1);

    Mat_<float> A(2*npoints, 4), b(2*npoints, 1);
    float *a0, *a1;
    Point2f p0, p1;

    for (int i = 0; i < npoints; ++i)
    {
        a0 = A[2*i];
        a1 = A[2*i+1];
        p0 = points0[i];
        p1 = points1[i];
        a0[0] = p0.x; a0[1] = p0.y; a0[2] = 1; a0[3] = 0;
        a1[0] = p0.y; a1[1] = -p0.x; a1[2] = 0; a1[3] = 1;
        b(2*i,0) = p1.x;
        b(2*i+1,0) = p1.y;
    }

    Mat_<float> sol;
    solve(A, b, sol, DECOMP_NORMAL | DECOMP_LU);

    if (rmse)
        *rmse = static_cast<float>(norm(A*sol, b, NORM_L2) / std::sqrt(static_cast<double>(npoints)));

    Mat_<float> M = Mat::eye(3, 3, CV_32F);
    M(0,0) = M(1,1) = sol(0,0);
    M(0,1) = sol(1,0);
    M(1,0) = -sol(1,0);
    M(0,2) = sol(2,0);
    M(1,2) = sol(3,0);

    return T1.inv() * M * T0;
}


static Mat estimateGlobMotionLeastSquaresAffine(
        int npoints, Point2f *points0, Point2f *points1, float *rmse)
{
    Mat_<float> T0 = normalizePoints(npoints, points0);
    Mat_<float> T1 = normalizePoints(npoints, points1);

    Mat_<float> A(2*npoints, 6), b(2*npoints, 1);
    float *a0, *a1;
    Point2f p0, p1;

    for (int i = 0; i < npoints; ++i)
    {
        a0 = A[2*i];
        a1 = A[2*i+1];
        p0 = points0[i];
        p1 = points1[i];
        a0[0] = p0.x; a0[1] = p0.y; a0[2] = 1; a0[3] = a0[4] = a0[5] = 0;
        a1[0] = a1[1] = a1[2] = 0; a1[3] = p0.x; a1[4] = p0.y; a1[5] = 1;
        b(2*i,0) = p1.x;
        b(2*i+1,0) = p1.y;
    }

    Mat_<float> sol;
    solve(A, b, sol, DECOMP_NORMAL | DECOMP_LU);

    if (rmse)
        *rmse = static_cast<float>(norm(A*sol, b, NORM_L2) / std::sqrt(static_cast<double>(npoints)));

    Mat_<float> M = Mat::eye(3, 3, CV_32F);
    for (int i = 0, k = 0; i < 2; ++i)
        for (int j = 0; j < 3; ++j, ++k)
            M(i,j) = sol(k,0);

    return T1.inv() * M * T0;
}


Mat estimateGlobalMotionLeastSquares(
        InputOutputArray points0, InputOutputArray points1, int model, float *rmse)
{
    //CV_INSTRUMENT_REGION()

    CV_Assert(model <= MM_AFFINE);
    CV_Assert(points0.type() == points1.type());
    const int npoints = points0.getMat().checkVector(2);
    CV_Assert(points1.getMat().checkVector(2) == npoints);

    typedef Mat (*Impl)(int, Point2f*, Point2f*, float*);
    static Impl impls[] = { estimateGlobMotionLeastSquaresTranslation,
                            estimateGlobMotionLeastSquaresTranslationAndScale,
                            estimateGlobMotionLeastSquaresRotation,
                            estimateGlobMotionLeastSquaresRigid,
                            estimateGlobMotionLeastSquaresSimilarity,
                            estimateGlobMotionLeastSquaresAffine };

    Point2f *points0_ = points0.getMat().ptr<Point2f>();
    Point2f *points1_ = points1.getMat().ptr<Point2f>();

    return impls[model](npoints, points0_, points1_, rmse);
}


Mat estimateGlobalMotionRansac(
        InputArray points0, InputArray points1, int model, const RansacParams &params,
        float *rmse, int *ninliers)
{
    //CV_INSTRUMENT_REGION()

    CV_Assert(model <= MM_AFFINE);
    CV_Assert(points0.type() == points1.type());
    const int npoints = points0.getMat().checkVector(2);
    CV_Assert(points1.getMat().checkVector(2) == npoints);

    if (npoints < params.size)
        return Mat::eye(3, 3, CV_32F);

    const Point2f *points0_ = points0.getMat().ptr<Point2f>();
    const Point2f *points1_ = points1.getMat().ptr<Point2f>();
    const int niters = params.niters();

    // current hypothesis
    std::vector<int> indices(params.size);
    std::vector<Point2f> subset0(params.size);
    std::vector<Point2f> subset1(params.size);

    // best hypothesis
    std::vector<Point2f> subset0best(params.size);
    std::vector<Point2f> subset1best(params.size);
    Mat_<float> bestM;
    int ninliersMax = -1;

    RNG rng(0);
    Point2f p0, p1;
    float x, y;

    for (int iter = 0; iter < niters; ++iter)
    {
        for (int i = 0; i < params.size; ++i)
        {
            bool ok = false;
            while (!ok)
            {
                ok = true;
                indices[i] = static_cast<unsigned>(rng) % npoints;
                for (int j = 0; j < i; ++j)
                    if (indices[i] == indices[j])
                        { ok = false; break; }
            }
        }
        for (int i = 0; i < params.size; ++i)
        {
            subset0[i] = points0_[indices[i]];
            subset1[i] = points1_[indices[i]];
        }

        Mat_<float> M = estimateGlobalMotionLeastSquares(subset0, subset1, model, 0);

        int numinliers = 0;
        for (int i = 0; i < npoints; ++i)
        {
            p0 = points0_[i];
            p1 = points1_[i];
            x = M(0,0)*p0.x + M(0,1)*p0.y + M(0,2);
            y = M(1,0)*p0.x + M(1,1)*p0.y + M(1,2);
            if (sqr(x - p1.x) + sqr(y - p1.y) < params.thresh * params.thresh)
                numinliers++;
        }
        if (numinliers >= ninliersMax)
        {
            bestM = M;
            ninliersMax = numinliers;
            subset0best.swap(subset0);
            subset1best.swap(subset1);
        }
    }

    if (ninliersMax < params.size)
        // compute RMSE
        bestM = estimateGlobalMotionLeastSquares(subset0best, subset1best, model, rmse);
    else
    {
        subset0.resize(ninliersMax);
        subset1.resize(ninliersMax);
        for (int i = 0, j = 0; i < npoints && j < ninliersMax ; ++i)
        {
            p0 = points0_[i];
            p1 = points1_[i];
            x = bestM(0,0)*p0.x + bestM(0,1)*p0.y + bestM(0,2);
            y = bestM(1,0)*p0.x + bestM(1,1)*p0.y + bestM(1,2);
            if (sqr(x - p1.x) + sqr(y - p1.y) < params.thresh * params.thresh)
            {
                subset0[j] = p0;
                subset1[j] = p1;
                j++;
            }
        }
        bestM = estimateGlobalMotionLeastSquares(subset0, subset1, model, rmse);
    }

    if (ninliers)
        *ninliers = ninliersMax;

    return bestM;
}


MotionEstimatorRansacL2::MotionEstimatorRansacL2(MotionModel model)
    : MotionEstimatorBase(model)
{
    setRansacParams(RansacParams::default2dMotion(model));
    setMinInlierRatio(0.1f);
}

Mat MotionEstimatorRansacL2::estimate(InputArray points0, InputArray points1, bool *ok)
{

    CV_Assert(points0.type() == points1.type());
    const int npoints = points0.getMat().checkVector(2);
    CV_Assert(points1.getMat().checkVector(2) == npoints);

    // find motion

    int ninliers = 0;
    Mat_<float> M;

    if (motionModel() != MM_HOMOGRAPHY)
        M = estimateGlobalMotionRansac(
                points0, points1, motionModel(), ransacParams_, 0, &ninliers);
    else
    {
        std::vector<uchar> mask;
        M = findHomography(points0, points1, mask, LMEDS);
        for (int i  = 0; i < npoints; ++i)
            if (mask[i]) ninliers++;
    }

    // check if we're confident enough in estimated motion

    if (ok) *ok = true;
    if (static_cast<float>(ninliers) / npoints < minInlierRatio_)
    {
        M = Mat::eye(3, 3, CV_32F);
        if (ok) *ok = false;
    }

    return M;
}

KeypointBasedMotionEstimator::KeypointBasedMotionEstimator(Ptr<MotionEstimatorBase> estimator)
    : ImageMotionEstimatorBase(estimator->motionModel()), motionEstimator_(estimator)
{
}


Mat KeypointBasedMotionEstimator::estimate(std::vector<cv::Point2f> &pointsPrevGood_, std::vector<cv::Point2f> &pointsGood_,bool *ok)
{

	cv::Mat M = motionEstimator_->estimate(pointsPrevGood_, pointsGood_, ok);
	return M;
}

Mat getMotion(int from, int to, const std::vector<Mat> &motions)
{
    Mat M = Mat::eye(3, 3, CV_32F);
    if (to > from)
    {
        for (int i = from; i < to; ++i)
            M = at(i, motions) * M;
    }
    else if (from > to)
    {
        for (int i = to; i < from; ++i)
            M = at(i, motions) * M;
        M = M.inv();
    }
    return M;
}



} // namespace videostab
} // namespace cv
