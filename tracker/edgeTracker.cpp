#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImageIo.h>
#include <visp/vpMbEdgeTracker.h>
#include <visp/vpMbEdgeKltTracker.h>
#include <visp/vpMbKltTracker.h>

#include <opencv2/opencv.hpp>
#include <visp/vpImageConvert.h>

int main()
{
	cv::VideoCapture cap("/home/sai/Desktop/cracker.avi"); // open the default video file
	cv::Mat curImg;
	cap >> curImg;
	vpImage<unsigned char> I;
	vpImageConvert::convert(curImg,I);

	vpCameraParameters cam;
	vpHomogeneousMatrix cMo;
#if defined(VISP_HAVE_X11)
	vpDisplayX display(I,100,100,"Model-based edge tracker");;
#elif defined(VISP_HAVE_GDI)
	vpDisplayGDI display(I,100,100,"Model-based edge tracker");;
#else
	std::cout << "No image viewer is available..." << std::endl;
#endif
	vpMbEdgeTracker tracker;
	//vpMbEdgeKltTracker tracker;
	//vpMbKltTracker tracker;
	tracker.loadConfigFile("../config.xml");
	tracker.setDisplayFeatures(true);
	tracker.setOgreVisibilityTest(false);
	tracker.loadModel("../config.cao");
	tracker.initClick(I, "../config.init");
	while(1){
		cap >> curImg;
		vpImageConvert::convert(curImg,I);

		vpDisplay::display(I);
		tracker.track(I);
		tracker.getPose(cMo);
		tracker.getCameraParameters(cam);
		tracker.display(I, cMo, cam, vpColor::red, 2);
		vpDisplay::displayFrame(I, cMo, cam, 0.025, vpColor::none, 3);
		vpDisplay::flush(I);
		if (vpDisplay::getClick(I, false))
			break;
		vpTime::wait(40);
	}
#ifdef VISP_HAVE_XML2
	vpXmlParser::cleanup();
#endif
#ifdef VISP_HAVE_COIN
	SoDB::finish();
#endif
}
