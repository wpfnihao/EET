/**
 * @file mbtEdgeTracker.cpp
 * @brief The implementation of the mbtEdgeTracker class.
 * @author Pengfei Wu - wpfnihao@gmail.com
 * @version 1.0
 * @date 2013-05-28
 */

#include "mbtEdgeTracker.h"

// opencv2 headers
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//#define  VISP2_7

mbtEdgeTracker::mbtEdgeTracker()
:tracker()  // init the vpMbEdgeTracker
{
}

void
mbtEdgeTracker::initialize(std::string config_file, std::string model_name, std::string init_file)
{
	this->cad_name = model_name;

	size_t numOfscales = 2;
	std::vector<bool> scales;
	for (size_t i = 0; i < numOfscales; i++)
		scales.push_back(true);

	tracker.setScales(scales);

	tracker.loadConfigFile(config_file);
	tracker.getCameraParameters(cam);
	//tracker.setDisplayFeatures(true);
	//tracker.setOgreVisibilityTest(false);
	tracker.loadModel(model_name);

	display.init(curImg);
	tracker.initClick(curImg, init_file);
	tracker.getPose(cMo);
}

bool 
mbtEdgeTracker::pubRst(cv::Mat& rst, cv::Rect& box)
{
	tracker.display(curImg, cMo, cam, vpColor::red, 2);
	return false;
}

void 
mbtEdgeTracker::retrieveImage(const cv::Mat& img)
{
	// can the cv::Mat automatically converted to iplImage?
	vpImageConvert::convert(img, curImg);
}

void 
mbtEdgeTracker::track(void)
{
	vpDisplay::display(curImg);
	try
	{
#ifdef VISP2_7
		tracker.setPose(curImg, cMo);
#else
		// this step will read the init file, so it runs relatively slow.
		tracker.reInitModel(curImg, cad_name.c_str(), cMo);
#endif
		tracker.track(curImg);
		tracker.getPose(cMo);
	}
	catch(...)
	{
		std::cout<<"error caught"<<std::endl;
	}
	tracker.display(curImg, cMo, cam, vpColor::red, 2);
	vpDisplay::displayFrame(curImg, cMo, cam, 0.025, vpColor::none, 3);
	vpDisplay::flush(curImg);
	vpTime::wait(40);
}
