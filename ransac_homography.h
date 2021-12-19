#pragma once
#include "Homography.h";
#include <algorithm>
#include "normalized_homography.h"


/// <summary>
/// </summary>
class RANSACHomography : public Homography {
public:
    RANSACHomography(const std::string& windowName, const Mapper& mappingPoints, bool showWindow = true,
                        unsigned int iterations = 200, double threshold = 1 ) {
        this->m_windowName = windowName;
        this->m_mappingPoints = mappingPoints;
        this->m_showWindow = showWindow;
        this->m_threshold = threshold;
        this->m_iterations = iterations;
        this->calculate();
    }

    cv::Mat calculate(const Mapper& pointPairs);
    cv::Mat calculate() { return this->calculate(this->m_mappingPoints); }

private:
    double m_threshold;
    unsigned int m_iterations;
    Mapper m_inliers; // the inliers points after running the algorithm.

};




///////////////////////////
//////////////////////////////////////////// Homography Function Definitions ////////////////////////////////////////////
//////////////////////////



cv::Mat RANSACHomography::calculate(const Mapper& pointPairs) {
	for (int i = 0; i < this->m_iterations; i++) {
		/*Generating 4 random indices*/
		cv::Mat mat(1, 4, CV_32S);
		int low = 0;
		int high = pointPairs.size()-1;
		randu(mat, cv::Scalar(low), cv::Scalar(high));

		/*Check if the indices are unique*/
		std::vector<int> vec = mat.clone();
		std::sort(vec.begin(), vec.end());
		for (int j = 0; j < vec.size() - 1; j++) {
			if (vec.at(j) == vec.at(j + 1)) {
				i--;
				continue;
			}
		}

		/*Gathering the point maps*/
		Mapper chosenSamples(vec.size());
		for (int j = 0; j < vec.size(); j++) {
			chosenSamples.at(j) = pointPairs.at(vec.at(j));
		}

		/*Fitting the model*/
		NormalizedHomography nh(this->m_windowName, chosenSamples, false);
		Mat H = nh.getHomography();

		/*Calculating the number of inliers*/
		vector<pair<Point2f, Point2f>> curInliers;
		for (int j = 0; j < pointPairs.size(); j++) {
			Point2f v = pointPairs.at(j).second - transformPoint(pointPairs.at(j).first, H);
			double dist = sqrt(v.x * v.x + v.y * v.y);
			if (dist < this->m_threshold) {
				curInliers.push_back(pointPairs.at(j));
			}
		}

		/*Checking if current fit is better*/
		if (curInliers.size() > this->m_inliers.size()) {
			this->m_inliers.clear();
			this->m_inliers = curInliers;
		}

#ifdef INFO_LOG
		cout << "Random points matrix: " << endl;
		cout << mat << endl;
		cout << endl;
		for (int j = 0; j < vec.size(); j++)
			cout << vec[j] << ", ";
		cout << endl;
#endif

	} // ransac loop

	/*Fitting the model*/
	NormalizedHomography nh(this->m_windowName, this->m_inliers, false);
	this->m_homography = nh.getHomography();
	return this->m_homography;
}
