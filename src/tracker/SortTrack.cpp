#include "tracker\SortTrack.h"


namespace easytracker {
	CSortTrack::CSortTrack(const Region& region, size_t trackID)
		:CTrack(region, trackID)
	{
	}


	CSortTrack::~CSortTrack()
	{
	}


	void CSortTrack::Update(const Region& region, bool dataCorrect, size_t max_trace_length, cv::Mat& currFrame) {
		m_predictionRect = m_trackFilter->Update(region.rect(), dataCorrect);


		if (m_predictionRect.width < 2) {
			m_predictionRect.width = 2;
		}
		if (m_predictionRect.x < 0) {
			m_predictionRect.x = 0;
		}
		else if (m_predictionRect.x + m_predictionRect.width > currFrame.cols - 1)
		{
			m_predictionRect.x = currFrame.cols - 1 - m_predictionRect.width;
		}

		if (m_predictionRect.height < 2) {
			m_predictionRect.height = 2;
		}
		if (m_predictionRect.y < 0) {
			m_predictionRect.y = 0;
		}
		else if (m_predictionRect.y + m_predictionRect.height > currFrame.rows - 1)
		{
			m_predictionRect.y = currFrame.rows - 1 - m_predictionRect.height;
		}

		m_predictionPoint = (m_predictionRect.tl() + m_predictionRect.br()) / 2;


		if (dataCorrect) {
			m_lastRegion = region;
			m_trajectoies.push_back(m_predictionPoint, region.center());
		}
		else {
			m_trajectoies.push_back(m_predictionPoint);
		}

		if (m_trajectoies.size() > max_trace_length) {
			m_trajectoies.pop_front(m_trajectoies.size() - max_trace_length);
		}
	}

	float CSortTrack::calculateIOU(const cv::Rect2f& r) {
		cv::Rect2f rr(getLastRegion());

		float intArea = (r & rr).area();
		float unionArea = r.area() + rr.area() - intArea;

		return intArea / unionArea;
	}
}