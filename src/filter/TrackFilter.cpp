#include "filter\TrackFilter.h"


namespace easytracker {
	TrackFilter::TrackFilter(cv::Rect2f& rect){
		m_lastRectResult = rect;
		m_initialRects.push_back(rect);
	}


	TrackFilter::~TrackFilter()
	{
	}

	cv::Rect TrackFilter::Update(cv::Rect rect, bool dataCorrect) {
		return cv::Rect(static_cast<int>(m_lastRectResult.x), static_cast<int>(m_lastRectResult.y),
			static_cast<int>(m_lastRectResult.width), static_cast<int>(m_lastRectResult.height));

	}

	cv::Rect TrackFilter::GetRectPrediction(){
		return cv::Rect(static_cast<int>(m_lastRectResult.x), static_cast<int>(m_lastRectResult.y),
			static_cast<int>(m_lastRectResult.width), static_cast<int>(m_lastRectResult.height));
	}

	void TrackFilter::setParamter(KalmanType type, float deltaTime, float accelNoiseMag) {

	}
}