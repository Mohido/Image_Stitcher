#pragma once
/*Open CV stuff*/
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

/*Standard Library*/
#include <vector>
#include <string>
#include <iostream>

/*Project Utils*/
#include "functions.h"



//////////////////////////////////////////// TYPE MAPPINGS (ABSTRACT DATA TYPES) ////////////////////////////////////////////

typedef std::vector <std::pair <cv::Point2f, cv::Point2f>> Mapper;

////////////////////////////////


//////////////////////////////////////////// Homography Interface ////////////////////////////////////////////


/*
* Homography class interface. Every Homography implementation must extend from this class.
* 
*/
class Homography {
public:
	virtual cv::Mat calculate() = 0;
	virtual cv::Mat calculate(const Mapper& mappingPoints) = 0;
    cv::Mat& projectAndSave(const cv::Mat&, const cv::Mat&, int);

    cv::Mat getHomography() {return this->m_homography;}
protected:
	Mapper m_mappingPoints;
	std::string m_windowName;
	bool m_showWindow;		
	cv::Mat m_homography;	// stores the last homography that were calculated.
};


////////////////////////////////


//////////////////////////////////////////// Homography Function Definitions ////////////////////////////////////////////


cv::Mat& Homography::projectAndSave(const cv::Mat& firstImage, const cv::Mat& secondImage, int imageIndex = 0) {
    cv::Mat transformedImage = cv::Mat::zeros(1.5 * firstImage.size().height,
        2.0 * firstImage.size().width,
        firstImage.type());

    transformImage(secondImage, transformedImage, cv::Mat::eye(3, 3, CV_32F), true);
    transformImage(firstImage, transformedImage, this->m_homography, true);

    if (this->m_showWindow) {
        cv::imshow(this->m_windowName, transformedImage);
        cv::waitKey();
    }
    std::string outputImageName = "HOMOGRAPHY_" + std::to_string(imageIndex) + ".png";
    cv::imwrite(outputImageName, transformedImage);
    return transformedImage;
}