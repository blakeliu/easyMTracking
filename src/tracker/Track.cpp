#include "tracker\Track.h"


namespace easytracker {
	CTrack::CTrack(const Region& region, size_t trackID):
		m_curTrackID(trackID),
		m_lastRegion(region),
		m_predictionPoint(Point_t(region.center().x, region.center().y)),
		m_skippedFrames(0),
		m_trackFilter(nullptr){
	}


	CTrack::~CTrack(){
		if (nullptr != m_trackFilter){
			delete m_trackFilter;
			m_trackFilter = nullptr;
		}
	}

	bool CTrack::createTrackFilter(TrackFilterType filterType, float deltaTime , float accelNoiseMag) {
		bool result = false;
		switch (filterType)
		{
		case easytracker::Kalman:
			m_trackFilter = new TKalmanFilter(m_lastRegion.rect());
			m_trackFilter->setParamter(KalmanType::KalmanLinear, deltaTime, accelNoiseMag);
			
			result = true;
			break;
		case easytracker::SORTKalman:
			m_trackFilter = new SortTrackFilter(m_lastRegion.rect());
			m_trackFilter->setParamter(KalmanType::KalmanLinear, deltaTime, accelNoiseMag);
			result = true;
			break;
		case easytracker::KCF:
			break;
		default:
			break;
		}
		m_trajectoies.push_back(m_lastRegion.center(), m_lastRegion.center());
		return result;
	}

	void CTrack::prediction(void) {
		m_predictionRect = m_trackFilter->GetRectPrediction();
	}


	void CTrack::Update(const Region& region, bool dataCorrect, size_t max_trace_length, cv::Mat& currFrame) {
		m_trackFilter->GetRectPrediction();

		
		m_predictionRect = m_trackFilter->Update(region.rect(), dataCorrect);
		

		if (m_predictionRect.width < 2){
			m_predictionRect.width = 2;
		}
		if (m_predictionRect.x < 0){
			m_predictionRect.x = 0;
		}
		else if (m_predictionRect.x + m_predictionRect.width > currFrame.cols - 1)
		{
			m_predictionRect.x = currFrame.cols - 1 - m_predictionRect.width;
		}

		if (m_predictionRect.height < 2){
			m_predictionRect.height = 2;
		}
		if (m_predictionRect.y < 0){
			m_predictionRect.y = 0;
		}
		else if (m_predictionRect.y + m_predictionRect.height > currFrame.rows - 1)
		{
			m_predictionRect.y = currFrame.rows - 1 - m_predictionRect.height;
		}

		m_predictionPoint = (m_predictionRect.tl() + m_predictionRect.br()) / 2;


		if (dataCorrect){
			m_lastRegion = region;
			m_trajectoies.push_back(m_predictionPoint, region.center());
		}
		else{
			m_trajectoies.push_back(m_predictionPoint);
		}

		if (m_trajectoies.size() > max_trace_length){
			m_trajectoies.pop_front(m_trajectoies.size() - max_trace_length);
		}
	}


	float CTrack::calculateCost(DistType type, const cv::Rect2f& r) {
		float dist;
		switch (type)
		{
		case easytracker::DistCenters:
			dist = calculateDist(Point_t(r.x, r.y));
			break;
		case easytracker::DistRects:
			dist = calculateDist(r);
			break;
		case easytracker::DistIOU:
			dist = calculateIOU(r);
			break;
		default:
			break;
		}
		return dist;
	}

	float CTrack::calculateDist(const cv::Rect2f& r) {
		std::array<float, 4> diff;
		diff[0] = m_predictionPoint.x - static_cast<int>(m_lastRegion.rect().width) / 2 - r.x;
		diff[1] = m_predictionPoint.y - static_cast<int>(m_lastRegion.rect().height) / 2 - r.y;
		diff[2] = static_cast<float>(m_lastRegion.rect().width - r.width);
		diff[3] = static_cast<float>(m_lastRegion.rect().height - r.height);

		float dist = 0;
		for (size_t i = 0; i < diff.size(); ++i)
		{
			dist += diff[i] * diff[i];
		}
		return sqrtf(dist);

	}

	float CTrack::calculateDist(const Point_t& p) {
		Point_t diff = m_predictionPoint - p;
		return sqrtf(diff.x * diff.x + diff.y * diff.y);
	}

	float CTrack::calculateIOU(const cv::Rect2f& r) {
		cv::Rect2f rr(getLastRegion());

		float intArea = (r & rr).area();
		float unionArea = r.area() + rr.area() - intArea;

		return 1 - intArea / unionArea;
	}
}