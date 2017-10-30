#include "tracker\Tracker.h"
#include "association\HungarianAlg.h"


namespace easytracker {
	CTracker::CTracker(TrackFilterType tfType, MatchType matchType, DistType distType, float dist_thresh_,
		size_t maximum_allowed_skipped_frames_,
		size_t max_trace_length_):
		m_trackFilterType(tfType),
		m_distType(distType),
		m_matchType(matchType),
		m_costThresh(dist_thresh_),
		m_maximum_allowed_skipped_frames(maximum_allowed_skipped_frames_),
		m_max_trace_length(max_trace_length_),
		m_nextTrackID(0)
	{
	}


	CTracker::~CTracker()
	{
	}

	void CTracker::update(const Regions_t& detections, cv::Mat& img)
	{
		if (m_tracks.size()==0){
			for (size_t i = 0; i < detections.size(); i++){
				m_tracks.push_back(std::make_unique<CTrack>(detections[i], m_nextTrackID++));
				m_tracks[i]->createTrackFilter(m_trackFilterType, m_kalmanDeltaTime, m_kalmanAccelNoiseMag);
			}
		}


		size_t N = m_tracks.size();		// tracks
		size_t M = detections.size();	// detectives
		assignments_t assignment(N, -1); // appointments

		if (!m_tracks.empty()){
			distMatrix_t Cost(N*M);
			float maxCost = 0;

			for (size_t i = 0; i < m_tracks.size(); i++)
			{
				for (size_t j = 0; j < detections.size(); j++)
				{
					//m_tracks[i]->prediction();
					auto rt = detections[j].rect();
					auto dist = m_tracks[i]->calculateCost(m_distType, rt);
					Cost[i + j * N] = dist;
					if (dist > maxCost)
					{
						maxCost = dist;
					}
				}
			}
				
			if (m_matchType == MatchType::MatchHungrian)
			{
				AssignmentProblemSolver APS;
				APS.Solve(Cost, N, M, assignment, AssignmentProblemSolver::optimal);
			}

			// -----------------------------------
			// clean assignment from pairs with large distance
			// -----------------------------------
			for (size_t i = 0; i < assignment.size(); i++)
			{
				if (assignment[i] != -1)
				{
					if (Cost[i + assignment[i] * N] > m_costThresh)
					{
						assignment[i] = -1;
						m_tracks[i]->m_skippedFrames++;
					}
				}
				else
				{
					// If track have no assigned detect, then increment skipped frames counter.
					m_tracks[i]->m_skippedFrames++;
				}
			}

			// -----------------------------------
			// If track didn't get detects long time, remove it.
			// -----------------------------------
			for (int i = 0; i < static_cast<int>(m_tracks.size()); i++)
			{
				if (m_tracks[i]->m_skippedFrames > m_maximum_allowed_skipped_frames)
				{
					m_tracks.erase(m_tracks.begin() + i);
					assignment.erase(assignment.begin() + i);
					i--;
				}
			}
		}

		// -----------------------------------
		// Search for unassigned detects and start new tracks for them.
		// -----------------------------------
		for (size_t i = 0; i < detections.size(); ++i)
		{
			if (find(assignment.begin(), assignment.end(), i) == assignment.end())
			{
				m_tracks.push_back(std::make_unique<CTrack>(detections[i], m_nextTrackID++));
				m_tracks[m_tracks.size()-1]->createTrackFilter(m_trackFilterType, m_kalmanDeltaTime, m_kalmanAccelNoiseMag);
			}
		}

		// Update Kalman Filters state

		for (size_t i = 0; i < assignment.size(); i++)
		{
			// If track updated less than one time, than filter state is not correct.

			if (assignment[i] != -1) // If we have assigned detect, then update using its coordinates,
			{
				m_tracks[i]->m_skippedFrames = 0;
				m_tracks[i]->Update(detections[assignment[i]], true, m_max_trace_length, img);
			}
			else				     // if not continue using predictions
			{
				m_tracks[i]->Update(Region(), false, m_max_trace_length, img);
			}
		}
		img.copyTo(m_prevFrame);
		
	}


	bool CTracker::tracksRobust(size_t trackID, int minTraceSize, float minRawRatio, cv::Size2f sizeRatio){
		Trajectories trace = getTrajectories(trackID);
		cv::Rect lastRegion =getLastRegion(trackID);
		bool flag = trace.size() > static_cast<size_t>(minTraceSize);
		flag &= trace.GetRawCount(trace.size() - 1) / static_cast<float>(trace.size()) > minRawRatio;
		if (sizeRatio.width + sizeRatio.height > 0)
		{
			float sr = lastRegion.width / static_cast<float>(lastRegion.height);
			if (sizeRatio.width > 0)
			{
				flag &= (sr > sizeRatio.width);
			}
			if (sizeRatio.height > 0)
			{
				flag &= (sr < sizeRatio.height);
			}
		}

		return flag;
	}
}