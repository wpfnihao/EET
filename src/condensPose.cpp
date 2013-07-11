/**
 * @file condensPose.cpp
 * @brief the implementation of the condensPose class
 * @author Pengfei Wu - wpfnihao@gmail.com
 * @version 1.0
 * @date 2013-06-14
 */

#include "condensPose.h"

condensPose::condensPose(int np)
:nParticles(np)	
{
	// the dimension of pose
	dim = 6;
	condens = cvCreateConDensation(dim, dim, np);

	// TODO: additional init steps here
	//
	// init the DynamMatr, since the prediction is not given by the DynamMatr, here we just simply set DynamMatr as identity matrix
	for (int i = 0; i < dim; i++)
		for (int j = 0; j < dim; j++)
		{
			if (i == j)
				condens->DynamMatr[i * dim + j] = 1.0;
			else
				condens->DynamMatr[i * dim + j] = 0.0;
		}
}

void
condensPose::init(vpHomogeneousMatrix& cMo, float rotPerturb, float transPerturb)
{
	vpPoseVector pv;
	pv.buildFrom(cMo);
	float minRange[] = {
		pv[0] - rotPerturb, 
		pv[1] - rotPerturb, 
		pv[2] - rotPerturb, 
		pv[3] - transPerturb, 
		pv[4] - transPerturb, 
		pv[5] - transPerturb};
	float maxRange[] = {
		pv[0] + rotPerturb, 
		pv[1] + rotPerturb, 
		pv[2] + rotPerturb, 
		pv[3] + transPerturb, 
		pv[4] + transPerturb, 
		pv[5] + transPerturb};
	CvMat LB, UB;
	cvInitMatHeader(&LB, this->dim, 1, CV_32FC1, minRange);
	cvInitMatHeader(&UB, this->dim, 1, CV_32FC1, maxRange);

	cvConDensInitSampleSet(condens, &LB, &UB);
}

void
condensPose::findConfidence(
		cv::Mat& img, 
		vpMbtPolygon* pyg,
		const std::vector<vpHomogeneousMatrix>& cMo,
		const vpCameraParameters& cam)
{
	// TODO: many parameters should be tuned here
	// step1: generate the edge distance map
	//
	cv::Mat gray_image, smoothed_image, edges_image, inverted_edges_image, distance_image;
	// gray scale
	cv::cvtColor(img, gray_image, CV_RGB2GRAY);
	
	//smoothing
	cv::GaussianBlur(gray_image, smoothed_image, cv::Size(5, 5), 3);

	cv::Mat sobel_x, sobel_y, gradient;
	cv::Sobel(gray_image, sobel_x, CV_32FC1, 1, 0);
	cv::Sobel(gray_image, sobel_y, CV_32FC1, 0, 1);
	cv::multiply(sobel_x, sobel_x, sobel_x);
	cv::multiply(sobel_y, sobel_y, sobel_y);
	cv::add(sobel_x, sobel_y, gradient);
	cv::pow(gradient, 0.5, gradient);
	distance_image = gradient;

	//do canny edge detector
	//cv::Canny(smoothed_image, edges_image, 30, 50);

	//invert values
	//cv::bitwise_not(edges_image, inverted_edges_image);

	//calculate the distance transform
	//distance_image = cv::Mat::eye(inverted_edges_image.rows, inverted_edges_image.cols, CV_32FC1);
	//cv::distanceTransform(inverted_edges_image, distance_image, CV_DIST_L2, 3);
	// test only
	cv::Mat dist;
	cv::normalize(distance_image, dist, 0.0, 1.0, cv::NORM_MINMAX);
	cv::imshow("distance", dist);

	// step2: measure the confidence
	// too complicated
	
	int numOfFaces = 6;
	int numOfPtsPerLine = 10;
	std::vector<float> sums;
	float mins = 0;
	for (int i = 0; i < nParticles; i++)
	{
		std::vector<cv::Point> edgePoints;
		// step2.1 find the visible face and project the corners
		// step2.2 generate the points to measure confidence
		for (int j = 0; j < numOfFaces; j++)	
		{
			if (pyg[j].isVisible(cMo[i]))
			{
				for (int k = 0; k < 4; k++)
				{
					vpPoint vp1 = pyg[j].p[k],
							vp2 = pyg[j].p[(k + 1) % 4];

					vp1.changeFrame(cMo[i]);
					vp1.project();
					vp2.changeFrame(cMo[i]);
					vp2.project();

					double u1, v1, u2, v2;
					vpMeterPixelConversion::convertPoint(cam, vp1.get_x(), vp1.get_y(), u1, v1);
					vpMeterPixelConversion::convertPoint(cam, vp2.get_x(), vp2.get_y(), u2, v2);
					for (int l = 0; l < numOfPtsPerLine; l++)
					{
						float alpha = l / numOfPtsPerLine;
						cv::Point p;
						p.x = alpha * u1 + (1 - alpha) * u2;
						p.y = alpha * v1 + (1 - alpha) * v2;
						edgePoints.push_back(p);
					}
				}
			}
		}

		// step2.3 mean value of the confidence
		float sum = 0;
		for (size_t j = 0; j < edgePoints.size(); j++)
		{
			sum += distance_image.at<float>(edgePoints[j]);
		}
		sum /= edgePoints.size();
		if (sum > mins)
			mins = sum;
		sums.push_back(sum);
	}
	for (int i = 0; i < nParticles; i++)	
		condens->flConfidence[i] = sums[i] - mins;
}

void 
condensPose::setPredict(const std::vector<vpHomogeneousMatrix>& cMo)
{
	for (int i = 0; i < nParticles; i++)
	{
		vpPoseVector pv;
		pv.buildFrom(cMo[i]);
		for (int j = 0; j < dim; j++)
		{
			condens->flSamples[i][j] = pv[j];
		}
	}
}

void 
condensPose::getPose(std::vector<vpHomogeneousMatrix>& cMo)
{
	for (int i = 0; i < nParticles; i++)
	{
		vpPoseVector pv;
		for (int j = 0; j < dim; j++)
		{
			pv[j] = condens->flSamples[i][j];
		}
		vpHomogeneousMatrix HM;
		HM.buildFrom(pv);
		cMo.push_back(HM);
	}
}

void 
condensPose::getMeasure(vpHomogeneousMatrix& cMo)
{
	vpPoseVector pv;
	for (int i = 0; i < dim; i++)
		pv[i] = condens->State[i];
	measurement.buildFrom(pv);	
	cMo = this->measurement;
}
