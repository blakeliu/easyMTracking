#include "detector\Detector.h"
#include "util\Utils.h"

namespace easytracker {
	CDetector::CDetector(std::string netfile, std::string weightfile)
		:m_netFile(netfile),
		m_weightFile(weightfile)
	{
	}


	CDetector::~CDetector()
	{
	}

	Regions_t CDetector::detectObject(cv::Mat& src)
	{
		Regions_t rt;
		//todo
		return rt;
	}

	void CDetector::loadObjectsNames(std::string& file){
		m_allObjNames = Utils::loadObjectsNames(file);
	}

	void CDetector::filterObjectsNames(std::string& file){
		m_customObjNames = Utils::stringSplit(file, ',');
	}

	void CDetector::loadSeqDets(std::string path)
	{
		std::ifstream file(path.c_str(), std::ios::in);

		//todo file>>xx

		file.close();
	}

	void CDetector::setminObjectSize(cv::Size size){
		m_minObjectSize = size;
	}

	void CDetector::setmaxObjectSize(cv::Size size){
		m_minObjectSize = size;
	}
}