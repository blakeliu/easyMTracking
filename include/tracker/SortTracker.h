#ifndef SORTTRACKER_H
#define SORTTRACKER_H

#include "tracker\Tracker.h"

namespace easytracker {
	class CSortTracker : public CTracker
	{
	public:
		CSortTracker(TrackFilterType tfType, MatchType mathType, DistType distType, float cost_thresh_, size_t maximum_allowed_skipped_frames_ = 2,
			size_t max_trace_length_=30);
		virtual ~CSortTracker();
	public:
		virtual void update(const Regions_t& rects, cv::Mat& img);
	};
}


#endif // !SORTTRACKER_H



