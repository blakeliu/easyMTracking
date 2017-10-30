#ifndef YOLODETECTOR_H
#define YOLODETECTOR_H

#include "detector\Detector.h"
#define OPENCV
#include "detector\yolo\yolo_v2_class.hpp"

#ifdef OPENCV
#include <opencv2/opencv.hpp>			// C++
#include "opencv2/core/version.hpp"
#include "opencv2/videoio/videoio.hpp"

#ifdef _DEBUG
#pragma comment(lib, "opencv_world330.lib")  
#else
#pragma comment(lib, "opencv_world330.lib")  
#endif // _DEBUG

#endif

namespace easytracker {
	class CYoloDetector :public CDetector
	{
	public:
		CYoloDetector(std::string netfile, std::string weightfile);
		virtual ~CYoloDetector();
	public:
		virtual Regions_t detectObject(cv::Mat& src);
	private:
		Detector* m_yoloV2Detector;
	};
}
#endif // !YOLODETECTOR_H
