#include "SequenceSortDetector.h"
#include "tracker\SortTracker.h"
#include "detector\YoloDetector.h"
#include "util\Utils.h"

namespace easytracker {


	SeqSortDetector::SeqSortDetector(	bool showLogs,
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


	SeqSortDetector::~SeqSortDetector()
	{
		/*if (m_detector != nullptr)
		{
			delete m_detector;
			m_detector = nullptr;
		}*/
	}

	bool SeqSortDetector::InitTracker(cv::Mat frame){

		m_tracker = std::make_unique<CSortTracker>(TrackFilterType::SORTKalman,
			MatchType::MatchHungrian,
			DistType::DistIOU,
			0.3f,
			2,         //skip max frames
			10);

		m_tracker->setKalmanParmater(KalmanType::KalmanLinear, 0.2f, 0.1f);

		switch (m_detectorType)
		{
		case 0: //
		{
			break;
		}

		case 1://yolo
		{
			m_detector = std::make_unique<CYoloDetector>(m_netFile, m_weightFile);
			m_detector->loadObjectsNames(m_objectNamesFile);
			m_detector->filterObjectsNames(m_filterObjNames);
		}
		default:
			break;
		}

		

		return true;
	}

	void SeqSortDetector::ProcessFrame(cv::Mat& frame){
		Regions_t region;
		switch (m_detectorType)
		{
		case 0: //
		{
			region = m_curSequenceDets[m_frameID];
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

	void SeqSortDetector::ProcessSequence(std::string& infile, std::string& outfile)
	{
		//load sequence dets
		std::string seqfile = m_seqDetsPath + infile.substr(infile.find_last_of("/"))+"/det.txt";
		std::string seqName = infile.substr(infile.find_last_of("/")+1);
		loadDets(seqfile);

		int framesCounter = 1;
		int waitTime = 1;
		if (m_debug){
			waitTime = 0;
		}

		if (!InitTracker(cv::Mat()))
		{
			return ;
		}

		
		int stop = 0;
		int64 allTime = 0;
		double freq = cv::getTickFrequency();
		cv::Mat frame;

		double fps = 25.0;
		cv::VideoWriter writer;
		
		for (size_t i = 0; i < m_curSequenceDets.size(); i++){
			m_frameID = i;

			frame = cv::imread(infile + "/img1/" + intToString(static_cast<int>(i)+1) + ".jpg");

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
			cv::putText(frame, frametxt, cvPoint(0, 25), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0,0,255));

			if (writer.isOpened()) {
				writer.write(frame);
			}

			cv::imshow("Video", frame);
			
			cv::waitKey(waitTime);

			++framesCounter;
		}
		std::cout << "work time = " << (allTime / freq) << std::endl;
		cv::waitKey(0);

		cv::destroyWindow("Video");
	}

	void SeqSortDetector::loadDets(std::string path) {
		std::ifstream file(path);// (path.c_str(), std::ios::in);
		assert(file.is_open());

		std::string line;
		int frame = 1;
		int curid = 1;
		Regions_t framedet;
		float x, y, w, h;

		while (getline(file,line)){
			std::vector<std::string>strs = Utils::stringSplit(line, ',');
			curid = atoi(strs[0].c_str());

			x = round(static_cast<float>(atof(strs[2].c_str())));
			y = round(static_cast<float>(atof(strs[3].c_str())));
			w = round(static_cast<float>(atof(strs[4].c_str())));
			h = round(static_cast<float>(atof(strs[5].c_str())));

			if (curid == frame)
			{	
				framedet.push_back(Region(cv::Rect2f(x, y, w, h)));
			}
			else {
				m_curSequenceDets.push_back(framedet);
				framedet.clear();
				framedet.push_back(Region(cv::Rect2f(x, y, w, h)));
				frame++;
			}
		}

		file.close();
	}
}
