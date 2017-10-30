#include "filter\TKalmanFilter.h"



namespace easytracker {
	TKalmanFilter::TKalmanFilter(cv::Rect2f& rect):
		m_initialized(false),
		TrackFilter(rect)
	{
		setParamter(KalmanType::KalmanLinear, 0.2f, 0.5f);
	}


	TKalmanFilter::~TKalmanFilter(){
	}

	cv::Rect TKalmanFilter::Update(cv::Rect rect, bool dataCorrect){
		if (!m_initialized) {
			if (m_initialRects.size() < MIN_INIT_VALS) {
				if (dataCorrect){
					m_initialRects.push_back(rect);
				}
			}
			if (m_initialRects.size() == MIN_INIT_VALS){
				std::vector<Point_t> initialPoints;
				Point_t averageSize(0, 0);
				for (const auto& r : m_initialRects){
					initialPoints.push_back(Point_t(static_cast<float>(r.x), static_cast<float>(r.y)));
					averageSize.x += r.width;
					averageSize.y += r.height;
				}
				averageSize.x /= MIN_INIT_VALS;
				averageSize.y /= MIN_INIT_VALS;

				float kx = 0;
				float bx = 0;
				float ky = 0;
				float by = 0;
				get_lin_regression_params(initialPoints, 0, MIN_INIT_VALS, kx, bx, ky, by);
				cv::Rect_<float> rect0(kx * (MIN_INIT_VALS - 1) + bx, ky * (MIN_INIT_VALS - 1) + by, averageSize.x, averageSize.y);
				Point_t pointv0(kx, ky);

				switch (m_type)
				{
				case KalmanType::KalmanLinear:
					CreateLinearFilter(rect0, pointv0);
					break;

				case KalmanType::KalmanUnscented:
					CreateLinearFilter(rect0, pointv0);
					std::cerr << "UnscentedKalmanFilter was disabled in CMAKE! Set KalmanLinear in constructor." << std::endl;
					break;

				case KalmanType::KalmanAugmentedUnscented:
					CreateLinearFilter(rect0, pointv0);
					std::cerr << "AugmentedUnscentedKalmanFilter was disabled in CMAKE! Set KalmanLinear in constructor." << std::endl;
					break;
				}
			}
		}


		if (m_initialized){
			cv::Mat measurement(4, 1, CV_32FC(1));
			if (!dataCorrect)
			{
				measurement.at<float>(0) = m_lastRectResult.x;  // update using prediction
				measurement.at<float>(1) = m_lastRectResult.y;
				measurement.at<float>(2) = m_lastRectResult.width;
				measurement.at<float>(3) = m_lastRectResult.height;
			}
			else
			{
				measurement.at<float>(0) = static_cast<float>(rect.x);  // update using measurements
				measurement.at<float>(1) = static_cast<float>(rect.y);
				measurement.at<float>(2) = static_cast<float>(rect.width);
				measurement.at<float>(3) = static_cast<float>(rect.height);
			}
			// Correction
			cv::Mat estimated;
			switch (m_type)
			{
			case KalmanType::KalmanLinear:
				estimated = m_linearKalman->correct(measurement);

				m_lastRectResult.x = estimated.at<float>(0);   //update using measurements
				m_lastRectResult.y = estimated.at<float>(1);
				m_lastRectResult.width = estimated.at<float>(2);
				m_lastRectResult.height = estimated.at<float>(3);
				break;

			case KalmanType::KalmanUnscented:
			case KalmanType::KalmanAugmentedUnscented:
				estimated = m_linearKalman->correct(measurement);
				m_lastRectResult.x = estimated.at<float>(0);   //update using measurements
				m_lastRectResult.y = estimated.at<float>(1);
				m_lastRectResult.width = estimated.at<float>(2);
				m_lastRectResult.height = estimated.at<float>(3);
				std::cerr << "UnscentedKalmanFilter was disabled in CMAKE! Set KalmanLinear in constructor." << std::endl;
				break;
			}
		}
		else{
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

	cv::Rect TKalmanFilter::GetRectPrediction() {
		if (m_initialized){
			cv::Mat prediction;
			switch (m_type)
			{
			case KalmanType::KalmanLinear:
				prediction = m_linearKalman->predict();
				break;
			case KalmanType::KalmanUnscented:
			case KalmanType::KalmanAugmentedUnscented:
				prediction = m_linearKalman->predict();
				std::cerr << "UnscentedKalmanFilter was disabled in CMAKE! Set KalmanLinear in constructor." << std::endl;
				break;
			default:
				break;
			}
			m_lastRectResult = cv::Rect_<float>(prediction.at<float>(0), prediction.at<float>(1), 
				prediction.at<float>(2), prediction.at<float>(3));
		}

		return cv::Rect(static_cast<int>(m_lastRectResult.x), static_cast<int>(m_lastRectResult.y),
			static_cast<int>(m_lastRectResult.width), static_cast<int>(m_lastRectResult.height));
	}

	void TKalmanFilter::setParamter(KalmanType type , float deltaTime , float accelNoiseMag ){
		m_type = type;
		m_deltaTime = deltaTime;
		m_accelNoiseMag = accelNoiseMag;

	}

	void TKalmanFilter::CreateLinearFilter(cv::Rect_<float>rect, Point_t point) {
		// We don't know acceleration, so, assume it to process noise.
		// But we can guess, the range of acceleration values thich can be achieved by tracked object.
		// Process noise. (standard deviation of acceleration: m/s^2)
		// shows, woh much target can accelerate.

		//4 state variables (x, y, width, height, dx, dy), 4 measurements (x, y, width, height)
		m_linearKalman = std::make_unique<cv::KalmanFilter>(6, 4, 0);
		// Transition cv::Matrix
		m_linearKalman->transitionMatrix = (cv::Mat_<float>(6, 6) <<
			1, 0, 0, 0, m_deltaTime, 0,
			0, 1, 0, 0, 0, m_deltaTime,
			0, 0, 1, 0, 0, 0,
			0, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 1);

		// init...
		m_linearKalman->statePre.at<float>(0) = rect.x;      // x
		m_linearKalman->statePre.at<float>(1) = rect.y;      // y
		m_linearKalman->statePre.at<float>(2) = rect.width;  // width
		m_linearKalman->statePre.at<float>(3) = rect.height; // height
		m_linearKalman->statePre.at<float>(4) = point.x;     // dx
		m_linearKalman->statePre.at<float>(5) = point.y;     // dy

		m_linearKalman->statePost.at<float>(0) = rect.x;
		m_linearKalman->statePost.at<float>(1) = rect.y;
		m_linearKalman->statePost.at<float>(2) = rect.width;
		m_linearKalman->statePost.at<float>(3) = rect.height;

		cv::setIdentity(m_linearKalman->measurementMatrix);

		m_linearKalman->processNoiseCov = (cv::Mat_<float>(6, 6) <<
			pow(m_deltaTime, 4.) / 4., 0, 0, 0, pow(m_deltaTime, 3.) / 2., 0,
			0, pow(m_deltaTime, 4.) / 4., 0, 0, pow(m_deltaTime, 3.) / 2., 0,
			0, 0, pow(m_deltaTime, 4.) / 4., 0, 0, 0,
			0, 0, 0, pow(m_deltaTime, 4.) / 4., 0, 0,
			pow(m_deltaTime, 3.) / 2., 0, 0, 0, pow(m_deltaTime, 2.), 0,
			0, pow(m_deltaTime, 3.) / 2., 0, 0, 0, pow(m_deltaTime, 2.));


		m_linearKalman->processNoiseCov *= m_accelNoiseMag;

		setIdentity(m_linearKalman->measurementNoiseCov, cv::Scalar::all(0.1));

		setIdentity(m_linearKalman->errorCovPost, cv::Scalar::all(.1));

		m_initialized = true;

	}
}