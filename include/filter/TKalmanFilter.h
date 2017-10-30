#ifndef TKALMAN_FILTER_H
#define TKALMAN_FILTER_H

#include "filter\TrackFilter.h"

namespace easytracker {

	template<typename T, typename CONT> void get_lin_regression_params(
		const CONT& in_data,
		size_t start_pos,
		size_t in_data_size,
		T& kx, T& bx, T& ky, T& by){
		T m1(0.), m2(0.);
		T m3_x(0.), m4_x(0.);
		T m3_y(0.), m4_y(0.);

		const T el_count = static_cast<T>(in_data_size - start_pos);
		for (size_t i = start_pos; i < in_data_size; ++i){
			m1 += i;
			m2 += sqr(i);

			m3_x += in_data[i].x;
			m4_x += i * in_data[i].x;

			m3_y += in_data[i].y;
			m4_y += i * in_data[i].y;
		}
		T det_1 = 1.f / (el_count * m2 - sqr(m1));

		m1 *= -1.;

		kx = det_1 * (m1 * m3_x + el_count * m4_x);
		bx = det_1 * (m2 * m3_x + m1 * m4_x);

		ky = det_1 * (m1 * m3_y + el_count * m4_y);
		by = det_1 * (m2 * m3_y + m1 * m4_y);
	}

	template<class T> inline
		T sqr(T val)
	{
		return val * val;
	}

	class TKalmanFilter :public TrackFilter
	{
	public:
		TKalmanFilter(cv::Rect2f& rect);
		virtual ~TKalmanFilter();
	public:
		virtual cv::Rect Update(cv::Rect rect, bool dataCorrect);

		virtual cv::Rect GetRectPrediction();

		virtual void setParamter(KalmanType type,float deltaTime, float accelNoiseMag);

	private:
		void CreateLinearFilter(cv::Rect_<float>rect, Point_t point);

	private:
		KalmanType m_type;
		static const size_t MIN_INIT_VALS = 4;
		std::unique_ptr<cv::KalmanFilter> m_linearKalman;
		bool m_initialized;
		float m_deltaTime;
		float m_accelNoiseMag;
	};
}

#endif // !TKALMAN_FILTER_H

