#include "detector\YoloDetector.h"


namespace easytracker {
	CYoloDetector::CYoloDetector(std::string netfile, std::string weightfile)
		:CDetector(netfile, weightfile)
	{
		m_yoloV2Detector = new Detector(netfile, weightfile);
	}


	CYoloDetector::~CYoloDetector()
	{
		if (m_yoloV2Detector != nullptr)
		{
			delete m_yoloV2Detector;
			m_yoloV2Detector = nullptr;
		}
	}

	Regions_t CYoloDetector::detectObject(cv::Mat& src) {
		Regions_t regions;

		std::vector<bbox_t> result_vec = m_yoloV2Detector->detect(src);

		std::vector<std::string>::iterator result;
		float x=0.f, y=0.f, w=0.f, h=0.f;
		//filter detected results
		for (auto &i : result_vec)
		{
			if (m_allObjNames.size() > i.obj_id)
			{
				std::string obj_name = m_allObjNames[i.obj_id];
				result = std::find(m_customObjNames.begin(), m_customObjNames.end(), obj_name);
				if (result != m_customObjNames.end())
				{
					x = round(static_cast<float>(i.x));
					y = round(static_cast<float>(i.y));
					w = round(static_cast<float>(i.w));
					h = round(static_cast<float>(i.h));
					regions.push_back(Region(cv::Rect2f(x, y, w, h)));
					std::cout << "obj_id = " << i.obj_id << ",  x = " << i.x << ", y = " << i.y
						<< ", w = " << i.w << ", h = " << i.h
						<< std::setprecision(3) << ", prob = " << i.prob << std::endl;
				}
			}
		}

		return regions;
	}

}
