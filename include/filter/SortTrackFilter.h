#ifndef SORTTRACKFILTER_H
#define SORTTRACKFILTER_H

#include "filter\TrackFilter.h"

namespace easytracker {
	class SortTrackFilter :public TrackFilter
	{
	public:
		SortTrackFilter(cv::Rect2f& rect);
		virtual ~SortTrackFilter();

	public:
		virtual cv::Rect Update(cv::Rect rect, bool dataCorrect);

		virtual void setParamter(KalmanType type, float deltaTime, float accelNoiseMag);

	private:
		void CreateLinearFilter(cv::Rect_<float>& rect);

		virtual cv::Rect GetRectPrediction();

	private:
		KalmanType m_type;
		std::unique_ptr<cv::KalmanFilter> m_sortLinearKalman;
		bool m_initialized;

		static const size_t MIN_INIT_VALS = 1;
	};
}

#endif // !SORTTRACKFILTER_H



