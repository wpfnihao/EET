/**
 * @file mbtEdgeKltTracker.cpp
 * @brief the wrap of the vpMbEdgeKltTracker class
 * @author Pengfei Wu - wpfnihao@gmail.com
 * @version 1.0
 * @date 2013-05-29
 */

#include "mbtEdgeKltTracker.h"

// opencv2 headers
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>


mbtEdgeKltTracker::mbtEdgeKltTracker()
:tracker()  // init the vpMbEdgeKltTracker
{
}

void
mbtEdgeKltTracker::initialize(std::string config_file, std::string model_name, std::string init_file)
{
	tracker.loadConfigFile(config_file);
	tracker.getCameraParameters(cam);
	tracker.setDisplayFeatures(true);
	tracker.setOgreVisibilityTest(true);
	tracker.loadModel(model_name);

	display.init(curImg);
	tracker.initClick(curImg, init_file);
	tracker.getPose(cMo);
}

bool 
mbtEdgeKltTracker::pubRst()
{
	tracker.display(curImg, cMo, cam, vpColor::red, 2);
	return false;
}

void 
mbtEdgeKltTracker::retrieveImage(const cv::Mat& img)
{
	// can the cv::Mat automatically converted to iplImage?
	vpImageConvert::convert(img, curImg);
}

void 
mbtEdgeKltTracker::track(void)
{
	vpDisplay::display(curImg);
	tracker.track(curImg);
	tracker.getPose(cMo);
	tracker.display(curImg, cMo, cam, vpColor::red, 2);
	vpDisplay::displayFrame(curImg, cMo, cam, 0.025, vpColor::none, 3);
	vpDisplay::flush(curImg);
	vpTime::wait(40);
}
