#include "filter\SortTrackFilter.h"

namespace easytracker {

	SortTrackFilter::SortTrackFilter(cv::Rect2f& rect) :
		TrackFilter(rect),
		m_initialized(false)
	{
		
		
	}


	SortTrackFilter::~SortTrackFilter()
	{
	}


	cv::Rect SortTrackFilter::Update(cv::Rect rect, bool dataCorrect) {
		if (m_initialized){
			cv::Mat measurement(4, 1, CV_32FC(1));
			if (!dataCorrect)
			{
				measurement.at<float>(0) = m_lastRectResult.x + m_lastRectResult.width / 2;  // update using prediction
				measurement.at<float>(1) = m_lastRectResult.y + m_lastRectResult.height / 2;;
				measurement.at<float>(2) = m_lastRectResult.width * m_lastRectResult.height;
				measurement.at<float>(3) = m_lastRectResult.width / m_lastRectResult.height;
			}
			else
			{
				measurement.at<float>(0) = static_cast<float>(rect.x) + static_cast<float>(rect.width)/2;  // update using measurements
				measurement.at<float>(1) = static_cast<float>(rect.y) + static_cast<float>(rect.height/2);
				measurement.at<float>(2) = static_cast<float>(rect.width * rect.height);
				measurement.at<float>(3) = static_cast<float>(rect.width) / static_cast<float>(rect.height);
			}
			// Correction
			cv::Mat estimated;
			switch (m_type)
			{
			case KalmanType::KalmanLinear:
			{
				estimated = m_sortLinearKalman->correct(measurement);

				float w = sqrt(estimated.at<float>(2)*estimated.at<float>(3));
				if (std::isnan(w)){
					break;
				}
				float h = estimated.at<float>(2) / w;
				m_lastRectResult.x = estimated.at<float>(0) - w / 2;   //update using measurements
				m_lastRectResult.y = estimated.at<float>(1) - h / 2;
				m_lastRectResult.width = w;
				m_lastRectResult.height = h;
				break;
			}

			case KalmanType::KalmanUnscented:
			case KalmanType::KalmanAugmentedUnscented:
				estimated = m_sortLinearKalman->correct(measurement);
				m_lastRectResult.x = estimated.at<float>(0);   //update using measurements
				m_lastRectResult.y = estimated.at<float>(1);
				m_lastRectResult.width = estimated.at<float>(2);
				m_lastRectResult.height = estimated.at<float>(3);
				std::cerr << "UnscentedKalmanFilter was disabled in CMAKE! Set KalmanLinear in constructor." << std::endl;
				break;
			}
		}
		else {
			if (dataCorrect)
			{
				m_lastRectResult.x = static_cast<float>(rect.x);
				m_lastRectResult.y = static_cast<float>(rect.y);
				m_lastRectResult.width = static_cast<float>(rect.width);
				m_lastRectResult.height = static_cast<float>(rect.height);
			}
		}
		return cv::Rect(static_cast<int>(m_lastRectResult.x), static_cast<int>(m_lastRectResult.y),
			static_cast<int>(m_lastRectResult.width), static_cast<int>(m_lastRectResult.height));

	}


	cv::Rect SortTrackFilter::GetRectPrediction() {
		if (m_initialized) {
			cv::Mat prediction;
			switch (m_type)
			{
			case KalmanType::KalmanLinear:
				prediction = m_sortLinearKalman->predict();
				break;
			case KalmanType::KalmanUnscented:
			case KalmanType::KalmanAugmentedUnscented:
				prediction = m_sortLinearKalman->predict();
				std::cerr << "UnscentedKalmanFilter was disabled in CMAKE! Set KalmanLinear in constructor." << std::endl;
				break;
			default:
				break;
			}
			float w = sqrt(prediction.at<float>(2)*prediction.at<float>(3));
			float h = prediction.at<float>(2) / w;
			m_lastRectResult.x = prediction.at<float>(0) - w / 2;   //update using measurements
			m_lastRectResult.y = prediction.at<float>(1) - h / 2;
			m_lastRectResult.width = w;
			m_lastRectResult.height = h;
		}

		return cv::Rect(static_cast<int>(m_lastRectResult.x), static_cast<int>(m_lastRectResult.y),
			static_cast<int>(m_lastRectResult.width), static_cast<int>(m_lastRectResult.height));

	}

	void SortTrackFilter::setParamter(KalmanType type, float deltaTime, float accelNoiseMag) {
		m_type = type;
		if (!m_initialized) {
			switch (m_type)
			{
			case KalmanType::KalmanLinear:
				CreateLinearFilter(m_lastRectResult);
				break;

			case KalmanType::KalmanUnscented:

				std::cerr << "UnscentedKalmanFilter was disabled in CMAKE! Set KalmanLinear in constructor." << std::endl;
				break;

			case KalmanType::KalmanAugmentedUnscented:
				std::cerr << "AugmentedUnscentedKalmanFilter was disabled in CMAKE! Set KalmanLinear in constructor." << std::endl;
				break;
			}
		}
	}

	void SortTrackFilter::CreateLinearFilter(cv::Rect_<float>& rect) {
		
		m_sortLinearKalman = std::make_unique<cv::KalmanFilter>(7, 4);
		//state transition matrix (A)
		m_sortLinearKalman->transitionMatrix = (cv::Mat_<float>(7, 7) <<
			1, 0, 0, 0, 1, 0, 0,
			0, 1, 0, 0, 0, 1, 0,
			0, 0, 1, 0, 0, 0, 1,
			0, 0, 0, 1, 0, 0, 0,
			0, 0, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 0, 1);
		// measurement matrix (H)
		cv::setIdentity(m_sortLinearKalman->measurementMatrix);

		//measurement noise covariance matrix (R)  R[2:,2:] *= 10.
		m_sortLinearKalman->measurementNoiseCov = (cv::Mat_<float>(4, 4) <<
			1, 0, 0, 0, 
			0, 1, 0, 0,
			0, 0, 10, 0,
			0, 0, 0, 10);


		//posteriori error estimate covariance matrix (P(k)): P(k)=(I-K(k)*H)*P'(k)
		//P[4:,4:] *= 1000. P *= 10.
		m_sortLinearKalman->errorCovPost= (cv::Mat_<float>(7, 7) <<
			10, 0, 0, 0, 0, 0, 0,
			0, 10, 0, 0, 0, 0, 0,
			0, 0, 10, 0, 0, 0, 0,
			0, 0, 0, 10, 0, 0, 0,
			0, 0, 0, 0, 10000, 0, 0,
			0, 0, 0, 0, 0, 10000, 0,
			0, 0, 0, 0, 0, 0, 10000);

		//process noise covariance matrix (Q)
		m_sortLinearKalman->processNoiseCov = (cv::Mat_<float>(7, 7) <<
			1, 0, 0, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0, 0,
			0, 0, 0, 1, 0, 0, 0,
			0, 0, 0, 0, 0.01f, 0, 0,
			0, 0, 0, 0, 0, 0.01f, 0,
			0, 0, 0, 0, 0, 0, 0.0001f);

		//center x, center y, area, aspect ratio(w/h)
		m_sortLinearKalman->statePost.at<float>(0) = rect.x + rect.width/2;
		m_sortLinearKalman->statePost.at<float>(1) = rect.y + rect.height/2;
		m_sortLinearKalman->statePost.at<float>(2) = rect.width * rect.height;
		m_sortLinearKalman->statePost.at<float>(3) = rect.width / rect.height;
		m_sortLinearKalman->statePost.at<float>(4) = 0;// rect.x + rect.width / 2;
		m_sortLinearKalman->statePost.at<float>(5) = 0;// rect.y + rect.height / 2;
		m_sortLinearKalman->statePost.at<float>(6) = 0;// rect.width * rect.height;

		/*m_sortLinearKalman->statePre.at<float>(0) = rect.x + rect.width / 2;
		m_sortLinearKalman->statePre.at<float>(1) = rect.y + rect.height / 2;
		m_sortLinearKalman->statePre.at<float>(2) = rect.width * rect.height;
		m_sortLinearKalman->statePre.at<float>(3) = rect.width / rect.height;*/

		m_initialized = true;
	}
}