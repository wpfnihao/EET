/**
 * @file condensPose.h
 * @brief this is the head file for the wrapper of the condensation (particle filter) method
 * 				the condensation method is used to estimate the pose 
 * @author Pengfei Wu - wpfnihao@gmail.com
 * @version 1.0
 * @date 2013-06-14
 */

#include <opencv2/legacy/legacy.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpPoseFeatures.h>
#include <visp/vpMbtPolygon.h>

class condensPose
{
	private:
		CvConDensation* condens;
		int nParticles;
		int dim;
		vpHomogeneousMatrix measurement;

	public:
		/**
		 * @brief the constructor
		 *
		 * @param np :number of particles
		 */
		condensPose(int np);
		
		/**
		 * @brief init the condens based on the initial cMo (pose)
		 *
		 * @param cMo the initial pose
		 */
		void init(vpHomogeneousMatrix& cMo, float rotPerturb, float transPerturb);

		/**
		 * @brief output the measured pose obtained by particle filter
		 *
		 * @param cMo vpHomogeneousMatrix 
		 */
		void getMeasure(vpHomogeneousMatrix& cMo);

		/**
		 * @brief set the predicted samples
		 *
		 * @param cMo vector of vpHomogeneousMatrix (pose)
		 */
		void setPredict(const std::vector<vpHomogeneousMatrix>& cMo);

		/**
		 * @brief use the current frame to measure the confidence of each pose
		 *
		 * @param img current frame
		 */
		void findConfidence(
				cv::Mat& img, 
				vpMbtPolygon* pyg,
				const std::vector<vpHomogeneousMatrix>& cMo,
				const vpCameraParameters& cam);

		/**
		 * @brief update the particles based on the samples and confidence
		 */
		inline void update(void)
		{
			cvConDensUpdateByTime(this->condens);
		}
		/**
		 * @brief output the re-sampled pose
		 *
		 * @param cMo
		 */
		void getPose(std::vector<vpHomogeneousMatrix>& cMo);
	protected:
};
