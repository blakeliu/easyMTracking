#ifndef OPEN_TRACKER_CONFIG_H
#define OPEN_TRACKER_CONFIG_H
#include <string>
#include <vector>
#include <deque>
#include <array>
#include <memory>
#include <opencv2/opencv.hpp>


namespace easytracker{
	class Region
	{
	public:
		Region()
		{}
		Region(const cv::Rect2f& rect) :
			m_rect(rect),
			m_center(rect.x + 0.5f*rect.width, rect.y + 0.5f*rect.height)
		{}

		Region(cv::Rect2f& rect) :
			m_rect(rect),
			m_center(rect.x + 0.5f*rect.width, rect.y + 0.5f*rect.height)
		{}

		~Region()
		{}

		void append(cv::Point2f point){
			m_points.push_back(point);
		}

		cv::Rect2f& rect(void){
			return m_rect;
		}

		const cv::Rect2f& rect(void) const{
			return m_rect;
		}

		cv::Point2f& center(void) {
			return m_center;
		}

		const cv::Point2f& center(void) const{
			return m_center;
		}

		std::vector<cv::Point2f>& points(void){
			return m_points;
		}

	private:
		cv::Rect2f m_rect;
		cv::Point2f m_center;
		std::vector<cv::Point2f> m_points;
	};

	typedef std::vector<Region> Regions_t;
	typedef cv::Point_<float> Point_t;

	enum DistType
	{
		DistCenters = 0,
		DistRects = 1,
		DistIOU = 2
	};

	enum KalmanType
	{
		KalmanLinear = 0,
		KalmanUnscented = 1,
		KalmanAugmentedUnscented
	};

	enum TrackFilterType
	{
		Kalman=0,
		SORTKalman=1,
		KCF
	};

	enum MatchType
	{
		MatchHungrian = 0,
		MatchBipart = 1
	};
}


#endif // !OPEN_TRACKER_CONFIG_H
