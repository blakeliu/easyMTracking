#ifndef SEQUENCEDETECOR_H
#define SEQUENCEDETECOR_H

#include "MotionDetector.h"
namespace easytracker {
	class VideoSortDetector :
		public MotionDetector
	{
	public:
		VideoSortDetector(bool showLogs, bool debug, int startFrame, int endFrame, int colorNums);
		virtual ~VideoSortDetector();

	public:
		virtual bool InitTracker(cv::Mat frame);

		virtual void ProcessFrame(cv::Mat& frame);

		void ProcessSequence(std::string infile, std::string outfile);

	private:
		size_t m_frameID;
	};

}

#endif // !SEQUENCEDETECOR_H



