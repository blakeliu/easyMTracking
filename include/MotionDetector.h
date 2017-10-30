#ifndef MOTION_DETECTOR_H
#define MOTION_DETECTOR_H
#include"detector\Detector.h"
#include"tracker\Tracker.h"

namespace easytracker {
	class MotionDetector
	{
	public:
		MotionDetector(bool showLogs, bool debug,int startFrame,int endFrame, int colorNums);
		virtual ~MotionDetector();

	public:
		virtual bool InitTracker(cv::Mat frame);

		virtual void ProcessFrame(cv::Mat& frame);

		void DrawData(cv::Mat& frame, int framesCounter, int currTime);

		void ProcessVideo(std::string infile, std::string outfile);

		void setDebug(bool value);

		void setMinTraceSize(int size);

		void setDetectorType(int type);

		void setDetectorParam(std::string netfile, std::string weightfile, std::string objnamesfile, std::string filternames);
		

	private:
		void DrawTrace(cv::Mat& frame, Trajectories& trace, cv::Rect& rect, size_t& trackID, int resize=1);
	protected:
		bool m_showLogs;
		bool m_debug;
		int m_minTraceSize;
		int m_detectorType;
		std::string m_netFile;
		std::string m_weightFile;
		std::string m_objectNamesFile;
		std::string m_filterObjNames;
		std::unique_ptr<CDetector> m_detector;
		std::unique_ptr<CTracker> m_tracker;
	protected:
		int m_startFrame;
		int m_endFrame;
		std::vector<cv::Scalar> m_colors;
	};

}

#endif // !MOTION_DETECTOR_H


