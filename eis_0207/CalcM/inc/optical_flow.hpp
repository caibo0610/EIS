/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009-2011, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#ifndef OPENCV_VIDEOSTAB_OPTICAL_FLOW_HPP
#define OPENCV_VIDEOSTAB_OPTICAL_FLOW_HPP

#include "videostab_util.hpp"
#include "opencv2/core/hal/intrin.hpp"

namespace cv
{
namespace videostab
{
	typedef short deriv_type;

	struct LKTrackerInvoker : ParallelLoopBody
	{
		LKTrackerInvoker(const Mat& _prevImg, const Mat& _prevDeriv, const Mat& _nextImg,
			const Point2f* _prevPts, Point2f* _nextPts,
			uchar* _status, float* _err,
			Size _winSize, TermCriteria _criteria,
			int _level, int _maxLevel, int _flags, float _minEigThreshold);

		void operator()(const Range& range) const;

		const Mat* prevImg;
		const Mat* nextImg;
		const Mat* prevDeriv;
		const Point2f* prevPts;
		Point2f* nextPts;
		uchar* status;
		float* err;
		Size winSize;
		TermCriteria criteria;
		int level;
		int maxLevel;
		int flags;
		float minEigThreshold;
	};

	void my_LKTracker(const Mat& prevImg, const Mat& prevDeriv, const Mat& nextImg,
		const Point2f* prevPts, Point2f* nextPts,
		uchar* status, float* err,
		Size winSize, TermCriteria& criteria,
		int level, int maxLevel, int flags, float minEigThreshold, int points_num);

	int my_buildOpticalFlowPyramid(InputArray img, OutputArrayOfArrays pyramid,
		Size winSize, int maxLevel, bool withDerivatives = true,
		int pyrBorder = BORDER_REFLECT_101,
		int derivBorder = BORDER_CONSTANT,
		bool tryReuseInputImage = true);

    void my_calcOpticalFlowPyrLK(InputArray prevImg, InputArray nextImg,
            InputArray prevPts, InputOutputArray nextPts,
            OutputArray status, OutputArray err,
            Size winSize, int maxLevel,
            TermCriteria& criteria,
            int flags = 0, double minEigThreshold = 1e-4);

} // namespace videostab
} // namespace cv

#endif
