#ifndef SEQSORTEDETECOR_H
#define SEQSORTEDETECOR_H

#include "MotionDetector.h"
namespace easytracker {
	class SeqSortDetector :
		public MotionDetector
	{
	public:
		SeqSortDetector(bool showLogs, bool debug, int startFrame, int endFrame, int colorNums);
		virtual ~SeqSortDetector();

	public:
		virtual bool InitTracker(cv::Mat frame);

		virtual void ProcessFrame(cv::Mat& frame);

		void ProcessSequence(std::string& infile, std::string& outfile);

		void setDetsPath(std::string path) {
			m_seqDetsPath = path;
		}

	private:
		void loadDets(std::string file);

		inline std::string intToString(int i) {
			char buff[10];
			sprintf(buff, "%06d", i);
			return std::string(buff);
		}

	private:
		size_t m_frameID;
		std::string m_seqDetsPath;
		std::vector<Regions_t> m_curSequenceDets;
	};

}

#endif // !SEQUENCEDETECOR_H



