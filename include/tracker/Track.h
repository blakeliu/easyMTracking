#ifndef TRACK_H
#define TRACK_H

#include "filter\TKalmanFilter.h"
#include "filter\SortTrackFilter.h"

/*
// \brief track for signle object
//
*/
namespace easytracker {
	struct TrajectoryPoint
	{
		///
		/// \brief TrajectoryPoint
		///
		TrajectoryPoint()
			: m_hasRaw(false)
		{
		}

		///
		/// \brief TrajectoryPoint
		/// \param prediction
		///
		TrajectoryPoint(const Point_t& prediction)
			:
			m_hasRaw(false),
			m_prediction(prediction)
		{
		}

		///
		/// \brief TrajectoryPoint
		/// \param prediction
		/// \param raw
		///
		TrajectoryPoint(const Point_t& prediction, const Point_t& raw)
			:
			m_hasRaw(true),
			m_prediction(prediction),
			m_raw(raw)
		{
		}

		bool m_hasRaw;
		Point_t m_prediction;
		Point_t m_raw;
	};
	
	
	class Trajectories
	{
	public:
		const Point_t& operator[](size_t i)const{
			return m_tracePoints[i].m_prediction;
		}

		Point_t& operator[](size_t i) {
			return m_tracePoints[i].m_prediction;
		}

		const TrajectoryPoint& at(size_t i)const {
			return m_tracePoints[i];
		}

		TrajectoryPoint& at(size_t i) {
			return m_tracePoints[i];
		}

		size_t size() const{
			return m_tracePoints.size();
		}

		void push_back(const Point_t& prediction) {
			m_tracePoints.push_back(TrajectoryPoint(prediction));
		}

		void push_back(const Point_t& prediction, const Point_t& raw) {
			m_tracePoints.push_back(TrajectoryPoint(prediction,raw));
		}

		void pop_front(size_t count){
			if (count < size()){
				m_tracePoints.erase(m_tracePoints.begin(), m_tracePoints.begin() + count);
			}
			else{
				m_tracePoints.clear();
			}
		}

		size_t GetRawCount(size_t lastPeriod) const
		{
			size_t res = 0;

			size_t i = 0;
			if (lastPeriod < m_tracePoints.size())
			{
				i = m_tracePoints.size() - lastPeriod;
			}
			for (; i < m_tracePoints.size(); ++i)
			{
				if (m_tracePoints[i].m_hasRaw)
				{
					++res;
				}
			}

			return res;
		}

	private:
		std::deque<TrajectoryPoint> m_tracePoints;
	};



	class CTrack
	{
	public:
		CTrack(const Region& region, size_t trackID);
		virtual ~CTrack();

	public:
		///
		/// \brief create calman filter
		bool createTrackFilter(TrackFilterType filterType, float deltaTime, float accelNoiseMag);

		///
		/// \brief prediction
		void prediction(void);

		///
		/// \brief Update
		/// \param region
		/// \param dataCorrect
		/// \param max_trace_length
		/// \param prevFrame
		/// \param currFrame
		///
		virtual void Update(const Region& region, bool dataCorrect, size_t max_trace_length, cv::Mat& currFrame);


		float calculateCost(DistType type, const cv::Rect2f& r);

		
		cv::Rect& getLastRegion(void) {
			return m_predictionRect;
		}

		Trajectories& trajectories(void) {
			return m_trajectoies;
		}

		inline size_t trackID(void) {
			return m_curTrackID;
		}

	protected:
		float calculateDist(const cv::Rect2f& r);

		float calculateDist(const Point_t& p);

		float calculateIOU(const cv::Rect2f& r);


	public:
		Region m_lastRegion;
		size_t m_curTrackID;
		size_t m_skippedFrames;

	protected:
		Point_t m_predictionPoint;
		cv::Rect m_predictionRect;
		Trajectories m_trajectoies;
		TrackFilter* m_trackFilter;
		bool m_filterObjectSize;
	};
}

#endif