#include "tracker\SortTracker.h"
#include "tracker\SortTrack.h"
#include "association\HungarianAlg.h"



namespace easytracker {
	CSortTracker::CSortTracker(TrackFilterType tfType, MatchType mathType, DistType distType, float cost_thresh_,
		size_t maximum_allowed_skipped_frames_ ,
		size_t max_trace_length_):
		CTracker(tfType, mathType, distType, cost_thresh_, maximum_allowed_skipped_frames_,
			max_trace_length_)
	{
	}


	CSortTracker::~CSortTracker()
	{
	}

	void CSortTracker::update(const Regions_t& detections, cv::Mat& img) {
		

		size_t N = m_tracks.size();		// tracks
		size_t M = detections.size();	// detectives
		assignments_t assignment(N, -1); // appointments

		if (!m_tracks.empty()) {
			distMatrix_t Cost(N*M);

			for (size_t i = 0; i < m_tracks.size(); i++)
			{
				m_tracks[i]->prediction();
				for (size_t j = 0; j < detections.size(); j++)
				{
					auto rt = detections[j].rect();
					auto dist = m_tracks[i]->calculateCost(m_distType, rt);
					Cost[i + j * N] = dist;
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
					if (1.0f - Cost[i  + assignment[i] * N] < m_costThresh)
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

		if (!m_tracks.empty())
		{
			// -----------------------------------
			// Search for unassigned detects and start new tracks for them.
			// -----------------------------------
			for (size_t i = 0; i < detections.size(); ++i)
			{
				if (find(assignment.begin(), assignment.end(), i) == assignment.end())
				{
					m_tracks.push_back(std::make_unique<CTrack>(detections[i], m_nextTrackID++));
					m_tracks[m_tracks.size() - 1]->createTrackFilter(m_trackFilterType, m_kalmanDeltaTime, m_kalmanAccelNoiseMag);
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
		}


		if (m_tracks.empty()) {
			for (size_t i = 0; i < detections.size(); i++) {
				m_tracks.push_back(std::make_unique<CSortTrack>(detections[i], m_nextTrackID++));
				m_tracks[i]->createTrackFilter(m_trackFilterType, m_kalmanDeltaTime, m_kalmanAccelNoiseMag);
			}
		}


		
		img.copyTo(m_prevFrame);
	}
}