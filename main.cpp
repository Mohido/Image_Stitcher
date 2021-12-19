#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <exception>
#include <algorithm>
#include "functions.h"
#include <thread> 

#include "non_normalized_homography.h"
#include "normalized_homography.h"
#include "ransac_homography.h"


#define WINDOW_NAME "image stitcher"
#define RANSAC_ITERATIONS_COUNT 400
#define RANSAC_INLIER_THRESHOLD 4 // 3 and 4 are good threshold for inliers 


/*Uncommenting any of these will change how the project runs.*/
//#define INFO_LOG										/*For logging into the console the debug and info logs*/
//#define SHOW_PREPROCESS
//#define NON_NORMALIZED_HOMOGRAPHY
//#define NORMALIZED_HOMOGRAPHY
//#define RANSAC_NORMALIZED_HOMOGRAPHY



void task1(Mapper first10, Mat firstImage, Mat secondImage, int index);
void task2(Mapper first10, Mat firstImage, Mat secondImage, int index);
void task3(Mapper featurePoints, Mat firstImage, Mat secondImage, int iterations, double threshold, int index);



/// <summary>
/// ELABORATION:
///		 In this project, we are creating an image stitcher application (Homography estimator). Homography is the projection
///		 Matrix of an image plane into another image plane. We define 3 methods for implementing this algorithm. 
///		 First, We preprocess the data by getting the Featured Points in both images. Note: We used ORB OpenCV model 
///		 Second, we try with the non-normalized linear estimation. It is the easiest to implement and naive.
///		 Third, we try to improve the first method by adding normalization terms (Matrices). Normalization will decrease the
///				approximation errors during the matrix formation.
///		 Finally, we robustify the second technique by adding the RANSAC algorithm which helps in choosing the best 
///				parameters for fitting the model (Our Homography approximator).
/// 
/// REMARKS AND NOTES:
///		Uncomment any of the sections if you want to try seeing the result. Running the code will output 9 different files
///		Each file determines a frame. We are using multithreading to improve the performance of outputing the 
///		stitched images. The main() is dependent on the given resources files while the other classes do not.
///		Uncoment the macros for each stage to see the output.
/// 
/// 
/// USEFULL REFERENCES:
///		http://cg.elte.hu/index.php/computer-vision/
///		https://web.archive.org/web/20150929063658/http://www.ele.puc-rio.br/~visao/Topicos/Homographies.pdf
///		http://6.869.csail.mit.edu/fa12/lectures/lecture13ransac/lecture13ransac.pdf
/// </summary>
/// <returns>0</returns>
int main() {
	/*Variables area*/
	std::vector<std::thread> threadPool;
	std::vector<ImageFeatureMatch> featuresMaps;
	std::vector<std::pair<cv::Mat, cv::Mat>> imagePairs = readTaskImages();	// Loading the images from the res file.
	
	// Creating a window
	cv::namedWindow(WINDOW_NAME, cv::WINDOW_AUTOSIZE);

	// A datatype for storing the matches of the 2 images
	featuresMaps.reserve(imagePairs.size());
	for (std::pair<cv::Mat, cv::Mat>& pr : imagePairs) {
		featuresMaps.push_back(ImageFeatureMatch(pr.first, pr.second));
	}


//#define SHOW_PREPROCESS
#ifdef SHOW_PREPROCESS /*Shows the feature mapping between the frist 2 images.*/
	
	/*Testing the ImageFeatureMatching for each and ever single image. Press any button to go to the next images.*/
	for (int i = 0; i < imagePairs.size(); i++) {
		ImageFeatureMatch temp = featuresMaps[i];
		std::pair<cv::Mat, cv::Mat> pr = imagePairs[i];

		cv::Mat output_image;
		for (int i = 0; i < temp.matches.size(); i++)
		{
			std::cout << temp.matches[i].distance << ", ";
		}
		std::cout << std::endl;


		cv::drawMatches(
			pr.first, temp.keypointsBaseImage,
			pr.second, temp.keypointsTargetImage,
			temp.matches,
			output_image);

		cv::imshow(WINDOW_NAME, output_image);
		cv::waitKey(0);
	}
#endif // SHOW_PREPROCESS

//#define NON_NORMALIZED_HOMOGRAPHY
#ifdef NON_NORMALIZED_HOMOGRAPHY  // Create Homography files using the Non normalized method
	threadPool.clear();
	for (int i = 0; i < featuresMaps.size(); i++) {
		Mapper first10(featuresMaps.at(i).matchingPoints.begin(), featuresMaps.at(i).matchingPoints.begin() + 11);
		threadPool.push_back(std::thread(task1, first10, imagePairs[i].first, imagePairs[i].second, i));
	}

	for (int i = 0; i < featuresMaps.size(); i++) {
		threadPool[i].join();
	}
#endif // NON_NORMALIZED_HOMOGRAPHY

//#define NORMALIZED_HOMOGRAPHY
#ifdef NORMALIZED_HOMOGRAPHY
	threadPool.clear();
	for (int i = 0; i < featuresMaps.size(); i++) {
		Mapper firstFeatures(featuresMaps.at(i).matchingPoints.begin(), featuresMaps.at(i).matchingPoints.begin() + 11);
		threadPool.push_back(std::thread(task2, firstFeatures, imagePairs[i].first, imagePairs[i].second, i));
	}

	for (int i = 0; i < threadPool.size(); i++) {
		threadPool[i].join();
	}
#endif // NORMALIZED_HOMOGRAPHY


//#define RANSAC_NORMALIZED_HOMOGRAPHY
#ifdef RANSAC_NORMALIZED_HOMOGRAPHY
	threadPool.clear();
	for (int i = 0; i < featuresMaps.size(); i++) {
		threadPool.push_back(std::thread(task3, featuresMaps.at(0).matchingPoints, imagePairs[i].first, imagePairs[i].second, RANSAC_ITERATIONS_COUNT, RANSAC_INLIER_THRESHOLD, i));
	}

	for (int i = 0; i < threadPool.size(); i++) {
		threadPool[i].join();
	}
#endif // RANSAC_NORMALIZED_HOMOGRAPHY

	return 0;

}






void task1(Mapper first10, Mat firstImage, Mat secondImage, int index) {
	NNHomography nonNor(WINDOW_NAME, first10, false);
	nonNor.projectAndSave(firstImage, secondImage, index);
	std::cout << "Finished NON Normalized image: " << index << std::endl;
}

void task2(Mapper first10, Mat firstImage, Mat secondImage, int index) {
	NormalizedHomography nonNor(WINDOW_NAME, first10, false);
	nonNor.projectAndSave(firstImage, secondImage, index);
	std::cout << "Finished Normalized image: " << index << std::endl;
}

void task3(Mapper featurePoints, Mat firstImage, Mat secondImage, int iterations, double threshold, int index) {
	RANSACHomography hom(WINDOW_NAME, featurePoints, false, iterations, threshold);
	hom.projectAndSave(firstImage, secondImage, index);
	std::cout << "Finished Ransac Normalized image: " << index << std::endl;
}
