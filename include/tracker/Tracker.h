#ifndef TRACKER_H
#define TRACKER_H
#include "tracker\Track.h"

namespace easytracker {
	/*
	// \brief multi-target tracker
	//
	*/
	class CTracker
	{
	public:
		CTracker(TrackFilterType tfType, MatchType mathType, DistType distType, float dist_thresh_, size_t maximum_allowed_skipped_frames_ = 2,
			size_t max_trace_length_=30);
		virtual ~CTracker();

		
	public:
		virtual void update(const Regions_t& rects, cv::Mat& img);

		bool tracksRobust(size_t trackID, int minTraceSize, float minRawRatio, cv::Size2f sizeRatio);


		inline void setKalmanParmater(KalmanType type = KalmanType::KalmanLinear, float deltaTime = 0.2, float accelNoiseMag = 0.5) {
			m_kalamType = type;
			m_kalmanDeltaTime = deltaTime;
			m_kalmanAccelNoiseMag = accelNoiseMag;
		}

		inline TrackFilterType getTrackFilterType() {
			return m_trackFilterType;
		}

		inline size_t getTracksNums(void){
			return m_tracks.size();
		}

		inline Trajectories& getTrajectories(size_t trackID) {
			return m_tracks[trackID]->trajectories();
		}

		inline cv::Rect& getLastRegion(size_t trackID) {
			return m_tracks[trackID]->getLastRegion();
		}

		inline size_t getTrackID(size_t trackID) {
			return m_tracks[trackID]->trackID();
		}
		
	protected:
		std::vector<std::unique_ptr<CTrack>> m_tracks;
		TrackFilterType m_trackFilterType;
		DistType m_distType;
		MatchType m_matchType;
		KalmanType m_kalamType;
		float m_kalmanDeltaTime;
		float  m_kalmanAccelNoiseMag;

		// exceeding this threshold, then this pair is not considered in the assignment problem.
		float m_costThresh;
		size_t m_maximum_allowed_skipped_frames;
		size_t m_max_trace_length;

		size_t m_nextTrackID;
		cv::Mat m_prevFrame;
	};
}
#endif // !TRACKER_H



