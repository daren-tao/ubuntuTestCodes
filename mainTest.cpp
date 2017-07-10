#include "mainHeader.h"
#include "opencv2/legacy/legacy.hpp"

#define CAM_WIDTH	320
#define CAM_HEIGHT	240


int main(int argc, char** argv)
{
	clock_t start;
	cv::Mat frame_left, frame_right;

	start = clock();
	
	frame_left	= cv::imread("./out/left_0.jpg");
	frame_right	= cv::imread("./out/right_0.jpg");

	StereoMatch matcher;
	matcher.init(frame_left.cols, frame_left.rows, "./out/calib_paras.xml");
	
	matcher.m_BM.state->minDisparity = 0;
	matcher.m_BM.state->numberOfDisparities = 224;
	matcher.m_BM.state->SADWindowSize = 19;
	matcher.m_BM.state->textureThreshold = 10;
	matcher.m_BM.state->disp12MaxDiff = -1;
	matcher.m_BM.state->preFilterSize = 17;
	matcher.m_BM.state->uniquenessRatio = 25;
	matcher.m_BM.state->speckleRange = 32;
	matcher.m_BM.state->speckleWindowSize = 100;
	
	std::cout << "initial spends : " << double(clock() - start) / CLOCKS_PER_SEC << " s" << std::endl;
	
	start = clock();
	cv::Mat disparity, pointsCloud;
	double obst = getDisparity_ObstacleDist(frame_left, frame_right, disparity, pointsCloud, matcher);
	std::cout << "distance = " << obst << std::endl;

//	cv::namedWindow("disparity");
	matcher.getDisparityImage(disparity, disparity);
	
	std::cout << "get disparity spends : " << double(clock() - start) / CLOCKS_PER_SEC << " s" << std::endl;
	
	cv::resize(disparity, disparity, cv::Size(disparity.cols / DESCALE, disparity.rows / DESCALE));
	cv::imshow("disparity", disparity);
	cv::waitKey(0);
	cv::destroyAllWindows();
	

	return 0;
}