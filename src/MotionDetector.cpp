#include "MotionDetector.h"

namespace easytracker {

	MotionDetector::MotionDetector(bool showLogs, bool debug, int startFrame, int endFrame, int colorNums=30):
		m_showLogs(showLogs),
		m_debug(debug),
		m_startFrame(startFrame),
		m_endFrame(endFrame)
	{
		m_colors.push_back(cv::Scalar(255, 0, 0));
		m_colors.push_back(cv::Scalar(0, 255, 0));
		m_colors.push_back(cv::Scalar(0, 0, 255));
		m_colors.push_back(cv::Scalar(255, 255, 0));
		m_colors.push_back(cv::Scalar(0, 255, 255));
		m_colors.push_back(cv::Scalar(255, 0, 255));
		m_colors.push_back(cv::Scalar(255, 127, 255));
		m_colors.push_back(cv::Scalar(127, 0, 255));
		m_colors.push_back(cv::Scalar(127, 0, 127));
	}


	MotionDetector::~MotionDetector()
	{
	}


	bool MotionDetector::InitTracker(cv::Mat frame){
		
		m_tracker = std::make_unique<CTracker>(TrackFilterType::Kalman, 
												MatchType::MatchHungrian,
												DistType::DistRects, 
												0.8f, 
												2, 
												30);
		if (m_tracker->getTrackFilterType() == TrackFilterType::Kalman)
		{
			m_tracker->setKalmanParmater(KalmanType::KalmanLinear, 0.2f, 0.1f);
		}

		return true;
	}

	void MotionDetector::ProcessFrame(cv::Mat& frame){
		const Regions_t regions = m_detector->detectObject(frame);
		m_tracker->update(regions, frame);
	}

	void MotionDetector::ProcessVideo(std::string infile, std::string outfile) {
		cv::VideoWriter writer;

		if (infile.empty()){
			std::cerr << "infile: "<< infile<<" is empty!" << std::endl;
			return;
		}

		cv::VideoCapture capture(infile);
		if (!capture.isOpened()){
			std::cerr << "Can't open " << infile << std::endl;
			return;
		}
		cv::namedWindow("Video");
		cv::Mat frame;

		capture.set(cv::CAP_PROP_POS_FRAMES, m_startFrame);

		int m_fps = std::max(1, cvRound(capture.get(cv::CAP_PROP_FPS)));

		capture >> frame;

		if (!InitTracker(frame))
		{
			return;
		}

		int k = 0;

		double freq = cv::getTickFrequency();

		int64 allTime = 0;

		bool manualMode = false;
		if (m_debug)
		{
			manualMode = true;
		}

		int framesCounter = m_startFrame + 1;
		while (k != 27)
		{
			capture >> frame;
			if (frame.empty())
			{
				break;
			}
			//cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

			if (!writer.isOpened())
			{
				writer.open(outfile, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), capture.get(cv::CAP_PROP_FPS), frame.size(), true);
			}

			int64 t1 = cv::getTickCount();

			ProcessFrame(frame);

			int64 t2 = cv::getTickCount();

			allTime += t2 - t1;
			int currTime = cvRound(1000 * (t2 - t1) / freq);

			DrawData(frame, framesCounter, currTime);

			cv::imshow("Video", frame);

			int waitTime = manualMode ? 0 : std::max<int>(1, 1000 / m_fps - currTime);
			k = cv::waitKey(waitTime);

			if (k == 'm' || k == 'M')
			{
				manualMode = !manualMode;
			}

			if (writer.isOpened())
			{
				writer << frame;
			}

			++framesCounter;
			if (m_endFrame && framesCounter > m_endFrame)
			{
				break;
			}
		}

		std::cout << "work time = " << (allTime / freq) << std::endl;
		cv::waitKey(0);
	}

	void MotionDetector::DrawData(cv::Mat& frame, int framesCounter, int currTime){

		if (m_showLogs)
		{
			std::cout << "Frame " << framesCounter << ": tracks = " << m_tracker->getTracksNums() << ", time = " << currTime << std::endl;
		}

		for (size_t i=0; i< m_tracker->getTracksNums();i++)
		{
			if (m_tracker->tracksRobust(i, m_minTraceSize,    // minimal trajectory size
				0.6f,                        // minimal ratio raw_trajectory_points / trajectory_lenght
				cv::Size2f(0.1f, 8.0f))      // min and max ratio: width / height
				)
			{
				Trajectories trace = m_tracker->getTrajectories(i);
				cv::Rect rt = m_tracker->getLastRegion(i);
				size_t id = m_tracker->getTrackID(i);
				DrawTrace(frame, trace, rt, id, 1);
			}
		}


	}

	void MotionDetector::DrawTrace(cv::Mat& frame, Trajectories& trace, cv::Rect& rect, size_t& trackID, int resize) {
		auto ResizeRect = [&](const cv::Rect& r) -> cv::Rect
		{
			return cv::Rect(resize * r.x, resize * r.y, resize * r.width, resize * r.height);
		};
		auto ResizePoint = [&](const cv::Point& pt) -> cv::Point
		{
			return cv::Point(resize * pt.x, resize * pt.y);
		};

		cv::rectangle(frame, ResizeRect(rect), cv::Scalar(0, 255, 0), 1, CV_AA);

		cv::Scalar cl = m_colors[trackID % m_colors.size()];

		if (m_debug){
			std::cout << "track ID:  " << trackID << " ,color: " << cl << std::endl;
		}
		

		for (size_t j = 0; j < trace.size() - 1; ++j)
		{
			const TrajectoryPoint& pt1 = trace.at(j);
			const TrajectoryPoint& pt2 = trace.at(j + 1);

			cv::line(frame, ResizePoint(pt1.m_prediction), ResizePoint(pt2.m_prediction), cl, 1, CV_AA);

			if (m_debug) {
				std::cout << "trace " << j << " : " << pt1.m_prediction.x << " " << pt1.m_prediction.y << std::endl;
			}

			/*if (!pt2.m_hasRaw)
			{
				cv::circle(frame, ResizePoint(pt2.m_prediction), 4, cl, 1, CV_AA);
			}*/
		}

		if (m_debug) {
			std::cout << "track ID:  " << trackID << " done!!! \n" << std::endl;
		}
	}

	void MotionDetector::setDebug(bool value) {
		m_debug = value;
	}

	void MotionDetector::setMinTraceSize(int size) {
		m_minTraceSize = size;
	}

	void MotionDetector::setDetectorType(int type) {
		m_detectorType = type;
	}

	void MotionDetector::setDetectorParam(std::string netfile, std::string weightfile, std::string objnamesfile, std::string filternames){
		m_netFile = netfile;
		m_weightFile = weightfile;
		m_objectNamesFile = objnamesfile;
		m_filterObjNames = filternames;
	}
}