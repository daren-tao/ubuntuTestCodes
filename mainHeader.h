#pragma once

#include <iostream>
#include <time.h>

#include "opencv2/opencv.hpp"

#include "StereoMatch.h"
#include "PointCloudAnalyzer.h"

#define DESCALE		1


double getDisparity_ObstacleDist(
	cv::Mat& lFrame, cv::Mat& rFrame, 
	cv::Mat& disparity, cv::Mat& pointCloud,StereoMatch& m_stereoMatcher)
{
	double m_ObjectDistance = 0.0;
	cv::Mat lcImage, rcImage;
	PointCloudAnalyzer pointCloudAnalyzer;

	clock_t start = clock();
	m_stereoMatcher.bmMatch(lFrame, rFrame, disparity, lcImage, rcImage);
	std::cout << "block match algo takes " << double(clock() - start) / CLOCKS_PER_SEC << " s" << std::endl;

	start = clock();
	m_stereoMatcher.getPointClouds(disparity, pointCloud);
	std::cout << "get points cloud takes " << double(clock() - start) / CLOCKS_PER_SEC << " s" << std::endl;

	start = clock();
	std::vector<PointCloudAnalyzer::ObjectInfo> objectInfos;
	pointCloudAnalyzer.detectNearObject(lcImage, pointCloud, objectInfos);
	std::cout << "detect obstacle takes " << double(clock() - start) / CLOCKS_PER_SEC << " s" << std::endl;

	if (!objectInfos.empty())
	{
	//	double fl = m_stereoMatcher.m_FL;
		m_ObjectDistance = objectInfos[0].distance; 
		m_ObjectDistance = (int)(m_ObjectDistance * 10000) / 10000.;

	//	double m_ObjectHeight = objectInfos[0].boundRect.height * objectInfos[0].distance / fl;
	//	m_ObjectHeight = (int)(m_ObjectHeight * 10000) / 10000.;

	//	double m_ObjectWidth = objectInfos[0].boundRect.width * objectInfos[0].distance / fl; 
	//	m_ObjectWidth = (int)(m_ObjectWidth * 10000) / 10000.;

	//	double m_ObjectDisparity = disparity.at<short>(objectInfos[0].nearest) / 16.;
	}

	return m_ObjectDistance;
}
