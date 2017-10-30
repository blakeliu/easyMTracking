#include "VideoSortDetector.h"
#include "tracker\SortTracker.h"
#include "detector\YoloDetector.h"
#include "util\Utils.h"
namespace easytracker {


	VideoSortDetector::VideoSortDetector(	bool showLogs,
										bool debug,
										int startFrame, 
										int endFrame, 
										int colorNums):
		MotionDetector(
			showLogs,
			debug,
			startFrame, 
			endFrame,  
			colorNums),
		m_frameID(0)
	{
	}


	VideoSortDetector::~VideoSortDetector()
	{
		/*if (m_detector != nullptr)
		{
			delete m_detector;
			m_detector = nullptr;
		}*/
	}

	bool VideoSortDetector::InitTracker(cv::Mat frame){
		m_tracker = std::make_unique<CSortTracker>(TrackFilterType::SORTKalman,
			MatchType::MatchHungrian,
			DistType::DistIOU,
			0.3f,
			2,         //skip max frames
			10);

		m_tracker->setKalmanParmater(KalmanType::KalmanLinear, 0.2f, 0.1f);

		m_detector = std::make_unique<CYoloDetector>(m_netFile, m_weightFile);
		m_detector->loadObjectsNames(m_objectNamesFile);
		m_detector->filterObjectsNames(m_filterObjNames);

		return true;
	}

	void VideoSortDetector::ProcessFrame(cv::Mat& frame){
		Regions_t region;
		switch (m_detectorType)
		{
		case 0: //
		{
			
			break;
		}

		case 1://yolo
		{
			region = m_detector->detectObject(frame);
		}
		default:
			break;
		}

		m_tracker->update(region, frame);
	}

	void VideoSortDetector::ProcessSequence(std::string infile, std::string outfile)
	{
		cv::VideoCapture capture(infile);
		if (!capture.isOpened())
		{
			std::cerr << "Can't open " << infile << std::endl;
			return;
		}

		capture.set(cv::CAP_PROP_POS_FRAMES, m_startFrame);

		int fps = std::max(1, cvRound(capture.get(cv::CAP_PROP_FPS)));

	
		if (!InitTracker(cv::Mat()))
		{
			return ;
		}

		int framesCounter = 1;
		int waitTime = 1;
		if (m_debug) {
			waitTime = 0;
		}
		cv::VideoWriter writer;
		int64 allTime = 0;
		double freq = cv::getTickFrequency();
		cv::Mat frame;
		int k = 0;

		while (k != 27)
		{
			capture >> frame;
			if (frame.empty())
			{
				break;
			}

			if (!writer.isOpened() && !outfile.empty())
			{
				writer.open(outfile, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, frame.size(), true);
				if (!writer.isOpened()) {
					std::cerr << "Could not open the output video file for write\n";
					break;
				}
			}

			int64 t1 = cv::getTickCount();

			ProcessFrame(frame);

			int64 t2 = cv::getTickCount();

			allTime += t2 - t1;
			int currTime = cvRound(1000 * (t2 - t1) / freq);

			DrawData(frame, framesCounter, currTime);

			std::string frametxt = "frame " + std::to_string(framesCounter);
			cv::putText(frame, frametxt, cvPoint(0, 25), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 255));

			cv::imshow("Video", frame);

			cv::waitKey(waitTime);


			if (writer.isOpened()) {
				writer.write(frame);
			}

			++framesCounter;
		}

		std::cout << "work time = " << (allTime / freq) << std::endl;
		cv::waitKey(0);
		cv::destroyWindow("Video");
	}

}
