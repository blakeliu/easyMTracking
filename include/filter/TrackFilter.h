#ifndef TRACK_FULTER_H
#define TRACK_FULTER_H

#include"config.h"

namespace easytracker {
	class TrackFilter
	{
	public:
		TrackFilter(cv::Rect2f& rect);
		virtual ~TrackFilter();

		virtual cv::Rect Update(cv::Rect rect, bool dataCorrect);

		virtual cv::Rect GetRectPrediction();

		virtual void setParamter(KalmanType type, float deltaTime, float accelNoiseMag);

	protected:
		std::deque<cv::Rect> m_initialRects;
		cv::Rect_<float> m_lastRectResult;
		bool m_initialized;
	};

}

#endif // !TRACK_FULTER_H


