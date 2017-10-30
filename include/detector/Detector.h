#ifndef OPEN_TRACKER_DETECTOR
#define OPEN_TRACKER_DETECTOR

#include "config.h"

namespace easytracker {
	class CDetector
	{
	public:
		CDetector(std::string netfile, std::string weightfile);
		virtual ~CDetector();
		virtual  Regions_t detectObject(cv::Mat& src);

		void loadObjectsNames(std::string& file);

		void filterObjectsNames(std::string& file);

		void loadSeqDets(std::string path);

		void setminObjectSize(cv::Size size);

		void setmaxObjectSize(cv::Size size);
	protected:
		std::string m_netFile;
		std::string m_weightFile;
		std::vector<std::string> m_allObjNames;
		std::vector<std::string> m_customObjNames;
	private:
		Regions_t m_regions;
		cv::Size m_minObjectSize;
		cv::Size m_maxObjectSize;
	};
}
#endif // !1



