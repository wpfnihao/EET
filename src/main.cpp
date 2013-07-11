// opencv headers
#include "opencv2/opencv.hpp"

// standard headers
#include <iostream>
#include <vector>
#include <fstream>

// visp headers
#include <visp/vpPose.h>
#include <visp/vpImage.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpPoseFeatures.h>

// user headers
#include "kltFbTracker.h"
#include "mbtEdgeTracker.h"
#include "condensPose.h"
#include "mbtEdgeKltTracker.h"
#include "mbtKltTracker.h"

using namespace cv;

enum tracking_status
{
	TRACKING,
	LOST,
	INITIAL
};

// global statuses maintained
::tracking_status status = ::INITIAL;	
int rows = 720;
int cols = 1280;
std::string init_file = "config.init";
std::string model_name = "config.cao";
std::string config_file = "config.xml";
std::vector<cv::Point3f> initP;
cv::Mat processedImg;
cv::Rect TrackerWindow;
vpCameraParameters cam;
vpHomogeneousMatrix cMo;
vpPoseVector poseVector;
int numberOfParticles = 5;

// some functions used
/**
 * @brief  	Init the model from the *.init file, only used for the cube tracking
 * 			initClick in the ViSP software is referred for the IO operations used here
 *  		fstream is used here
 */
void 
getInitPoints(void)
{
	std::fstream finit;	
	finit.open(init_file.c_str(), std::ios::in);
	if (finit.fail())
	{
		std::cout << "cannot read " << init_file << std::endl;
		throw vpException(vpException::ioError, "cannot read init file for model initialization!");
	}

	//file parser
	//number of points
	//X Y Z
	//X Y Z
	 
	double X,Y,Z ;

	unsigned int n ;
	finit >> n ;
	std::cout << "number of points " << n << std::endl ;
	for (unsigned int i=0 ; i < n ; i++)
	{
		finit >> X ;
		finit >> Y ;
		finit >> Z ;
		// NOTE: X,Y,Z are small float variables in meter, do NOT use int to save them
		cv::Point3f curP(X, Y, Z);
		// initialize the initP, which is the member variable of the class
		initP.push_back(curP); // (X,Y,Z)
	}

	finit.close();
}

/**
 * @brief 			use the vpMbEdgeTracker to initialize the tracker
 *
 * @param srcImg 	the sensor_msgs of ROS topic, it will be transfered to vpImage in the function
 */
void 
initializeTracker(const cv::Mat& srcImg)
{
	// the vpImg is a mono channel image
	vpImage<unsigned char> vpImg;
	// the operator:=() will allocate the memory automatically
	vpImageConvert::convert(srcImg, vpImg);

	// We only use the tracker in visp to initialize the program
	vpMbEdgeTracker initializer;
	// NOTE:the params can also be retrieved from the sensor_msgs::CameraInfo using the visp_bridge package
	initializer.loadConfigFile(config_file);		
	initializer.getCameraParameters(cam);
	initializer.loadModel(model_name);
	initializer.setCovarianceComputation(true);

	//initial the display
	//these steps are not shown in the manual document
	vpDisplayX display;
	display.init(vpImg);

	initializer.initClick(vpImg, init_file);
	// obtain the initial pose of the model
	cMo = initializer.getPose();
	poseVector.buildFrom(cMo);

	//clean up
	vpXmlParser::cleanup();
}

// the program entry point
int main(int, char**)
{

	// capture frames
	VideoCapture cap(0); // open the default camera
	cap.set(CV_CAP_PROP_FRAME_WIDTH, cols);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, rows);
	if(!cap.isOpened()) // check if we succeeded
		return -1;

	// declare trackers
	kltFbTracker fbTracker;	
	mbtEdgeTracker meTracker;	
	mbtEdgeKltTracker meKltTracker;	
	mbtKltTracker kltTracker;

	// declare condens
	condensPose particle(numberOfParticles);

	// for display
	namedWindow("video",CV_WINDOW_AUTOSIZE);

	for(;;)
	{
		// retrieve image from the camera
		Mat curImg;
		cap >> curImg; // get a new frame from camera
		processedImg = curImg;


		//tracking based on different status
		if (status == ::LOST || status == ::INITIAL)
		{
			// init the model from file
			// only requires when initializing
			if (status == ::INITIAL)
			{
				// get some basic info about the video
				cv::Mat grayImg;
				cv::cvtColor(curImg, grayImg, CV_BGR2GRAY);
				rows = grayImg.rows;
				cols = grayImg.cols;

				// parse the .init file, and get the init points
				getInitPoints();
				// get the info of the cube from the init points
				//hlTracker.initModel(initP);
				//fbTracker.initModel(initP);
				//fbTracker.init(curImg);

				kltTracker.retrieveImage(curImg);
				kltTracker.initialize(config_file, model_name, init_file);
				//meTracker.retrieveImage(curImg);
				//meTracker.initialize(config_file, model_name, init_file);
			}

			initializeTracker(curImg);
			//fbTracker.retrieveImage(curImg);
			//fbTracker.initialize(cam, cMo, poseVector, rows, cols);


			// initialize condens
			//particle.init(cMo, 0.001, 0.00001);
			

			//finish the initialization step and start to track
			status = ::TRACKING;
		}	
		else if (status == ::TRACKING)
		{
			//std::vector<vpHomogeneousMatrix> vecOfPose;
			//particle.getPose(vecOfPose);

			//for (int i = 0; i < numberOfParticles; i++)
			//{
			//	vpHomogeneousMatrix& cMo_ = vecOfPose[i];
			//	fbTracker.getPose(cMo_);
			//	fbTracker.retrieveImage(curImg);
			//	fbTracker.track();
			//	fbTracker.pubPose(cMo_);
			//	if(fbTracker.pubRst(processedImg, TrackerWindow))
			//		status = ::LOST;

				//meTracker.getPose(cMo_);
				//meTracker.retrieveImage(curImg);
				//meTracker.track();
				//meTracker.pubPose(cMo_);
				kltTracker.retrieveImage(curImg);
				kltTracker.track();
				kltTracker.pubRst();
			//}
			//particle.setPredict(vecOfPose);
			//particle.findConfidence(curImg, fbTracker.getPolygon(), vecOfPose, cam);
			//particle.getMeasure(cMo);
			//particle.update();
		}

		// display the result
		imshow("video", processedImg);
		if(waitKey(1) >= 0) break;
	}

	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}
