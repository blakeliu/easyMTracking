#ifndef SORTTRACK_H
#define SORTTRACK_H

#include "tracker\Track.h"
namespace easytracker {
	class CSortTrack : public CTrack
	{
	public:
		CSortTrack(const Region& region, size_t trackID);
		virtual ~CSortTrack();

	public:	
		///
		/// \brief Update
		/// \param region
		/// \param dataCorrect
		/// \param max_trace_length
		/// \param prevFrame
		/// \param currFrame
		///
		virtual void Update(const Region& region, bool dataCorrect, size_t max_trace_length, cv::Mat& currFrame);

	protected:
		float calculateIOU(const cv::Rect2f& r);

	};
}


#endif // !SORTTRACK_H




