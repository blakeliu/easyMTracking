#include "VideoSortDetector.h"
#include "util\Utils.h"

using namespace easytracker;

static void Help()
{
	printf("\nExamples of the easy tracking algorithm\n"
		"Usage: \n"
		"          ./easyTracker  [-t]=<0> \n\n"
	);
}

const char* keys =
{
	"{ t  tracker          |0  | tracker type: 0-video sort kalman, 1- seq sort kalman | }"
	"{ d  debug           |0  | debug : 0-yes, 1-false | }"
	"{ mint  minTraceSize |8  | min trace size > 2 | }"
	"{ in   inFile        |   |input video| }"
	"{ out  outFile       |   |write processed video| }"
	"{ seq  sequences     |PETS09-S2L1 |process sequences| }"
	"{ det  detector      |0  |detector type: 0-identified_seq, 1-yolov2 |}"
	"{ net  netfile       |  |dl net file |}"
	"{ weight  weightfile |  |dl net weight file |}"
	"{ objnamefile  objectnamefile |voc.names  |data sets object names |}"
	"{ filname  filtername |person  | custom made object names like: person,car,bicycle,bus | }"
};


int main(int argc, char** argv) {

	cv::CommandLineParser parser(argc, argv, keys);
	int trackerType = parser.get<int>("tracker");
	int debugFlag = parser.get<int>("debug");
	int minTraceSize = parser.get<int>("minTraceSize");
	std::string inFile = parser.get<std::string>("in");
	std::string outFile = parser.get<std::string>("out");
	std::string seqStrs= parser.get<std::string>("seq");
	int detectorType = parser.get<int>("detector");
	std::string weightfile = parser.get<std::string>("weightfile");
	std::string netfile = parser.get<std::string>("netfile");
	std::string objnamefile = parser.get<std::string>("objectnamefile");
	std::string filtername = parser.get<std::string>("filtername");


	std::vector<std::string> sequences= Utils::stringSplit(seqStrs, ',');
	std::string datasetpath = "D:/datasets/tracking/2DMOT2015/train";
	std::string seqdetpath = "E:/cv/tracking/mutil-tracking/sort/sort/data";


	switch (trackerType)
	{
	case 0:
	{	VideoSortDetector videoSortDetecotDemo(true, false, 0, -1, 30);

		videoSortDetecotDemo.setDebug(debugFlag == 1 ? true : false);
		videoSortDetecotDemo.setMinTraceSize(minTraceSize);
		videoSortDetecotDemo.setDetectorType(detectorType);
		videoSortDetecotDemo.setDetectorParam(netfile, weightfile, objnamefile, filtername);


		videoSortDetecotDemo.ProcessSequence(inFile, outFile);
		
		break; 
	}
	default:
		break;
	}

	return 0;
}