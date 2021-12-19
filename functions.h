#pragma once
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>


using namespace cv;
using namespace std;



/// <summary>
///     An abstraction over the Feature mappings. It takes 2 images and by using the ORB model provided for 
///     free by OpenCV, it generates a set of Feature maps and holds other very useful information. However, for 
///     this project, the most useful variable is the matchingPoints since it defines the set of points in image 'a'
///     and their corresponding points in image 'b'
/// 
/// USEFULL REFERENCES:
///     https://github.com/santosderek/Brute-Force-Matching-using-ORB-descriptors/blob/master/src/main.cpp
/// </summary>
struct ImageFeatureMatch {
    std::vector<cv::KeyPoint> keypointsBaseImage, keypointsTargetImage;

    // Find descriptors.
    cv::Mat descriptorsBaseImage, descriptorsTargetImage;

    // Vector where matches will be stored.
    std::vector<cv::DMatch> matches;
    
    // vector of points matched...
    std::vector <std::pair <cv::Point2f, cv::Point2f> > matchingPoints;


    /*How things go: Keypoints -> descriptors -> DMatches -> matchingPoints*/
    ImageFeatureMatch(Mat& baseImage, Mat& targetImage) {
        /*Computing the keypoints*/
        cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
        detector->detect(baseImage, this->keypointsBaseImage);
        detector->detect(targetImage, this->keypointsTargetImage);
        detector.release();

        /*Computing the descriptors*/
        cv::Ptr<cv::DescriptorExtractor> extractor = cv::ORB::create();
        extractor->compute(baseImage, this->keypointsBaseImage, this->descriptorsBaseImage);
        extractor->compute(targetImage, this->keypointsTargetImage, this->descriptorsTargetImage);
        extractor.release();

        // Create Brute-Force Matcher. Other Algorithms are 'non-free'.
        cv::BFMatcher brue_force_matcher = cv::BFMatcher(cv::NORM_HAMMING, true);

        // Find matches and store in matches vector.
        brue_force_matcher.match((const cv::OutputArray)this->descriptorsBaseImage, (const cv::OutputArray)this->descriptorsTargetImage, this->matches);

        // Sort them in order of their distance. The less distance, the better.
        std::sort(this->matches.begin(), this->matches.end(), [](cv::DMatch& a, cv::DMatch& b) { return a.distance < b.distance; });
        this->matchingPoints.resize(this->matches.size());

        /*Filling the pixels matches*/
        for (int i = 0; i < this->matches.size(); i++) {
            int baseInd = this->matches[i].queryIdx;
            this->matchingPoints[i].first = this->keypointsBaseImage[baseInd].pt;

            auto targetInd = this->matches[i].trainIdx;
            this->matchingPoints[i].second = this->keypointsTargetImage[targetInd].pt;
        }
    }
};


/// <summary>
/// Helper function for getting the type of the OpenCV into a string.
/// </summary>
/// <param name="type">OpenCV defined matrix types. E.G: CV_32F</param>
/// <returns>The type as a string</returns>
std::string type2str(int type) {
    string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch (depth) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
    }

    r += "C";
    r += (chans + '0');

    return r;
}


/// <summary>
///     Reads the data from the attached file. It is dependent to an absolute path on my machine. 
///     If you donwloaded the project, please change the path to the attached resource directory.
/// 
/// NOTE:
///     File inputs would have the following pattern
///        Dev1_Image_w960_h600_fn1000
///        Dev2_Image_w960_h600_fn1000
///
///        Dev1_Image_w960_h600_fn1001
///        Dev2_Image_w960_h600_fn1001
/// </summary>
/// <returns></returns>
vector<pair<Mat, Mat>> readTaskImages() {
    string directoryName = "D:/University/semester_5/3dCV/Project2/ImageStitcher/res/";
    
    vector<pair<Mat, Mat>> ret;
    for (int i = 0; i < 10; i++) {
        string left = directoryName + "Dev1_Image_w960_h600_fn100" + to_string(i) + ".jpg";
        string right = directoryName + "Dev2_Image_w960_h600_fn100" + to_string(i) + ".jpg";

        Mat leftImg  = imread(left);
        Mat rightImg = imread(right);
        if (leftImg.empty() || rightImg.empty()) {
            std::cout << "can't find image at: " << left << std::endl;
            continue;
        }

        ret.push_back({ rightImg,leftImg});
    }

    return ret;
}



/// <summary>
///     From the given set of points, it creates a normalization matrix. Normalization matrices can be used later
///     to encode/decode the data for the ease of computing and avoiding large rounding errors.
/// USEFULL REFERENCES:
///     https://web.archive.org/web/20150929063658/
///     http://www.ele.puc-rio.br/~visao/Topicos/Homographies.pdf
/// </summary>
/// <param name="pointPairs">Pair of points (Feature map)</param>
/// <param name="imageIdx">Which set of points to take from the given feature map (0 means from a normalization matrix from the feature map domain)</param>
/// <returns>Normalization matrix</returns>
Mat getNormalizer(const vector<pair<Point2f, Point2f> >& pointPairs, int imageIdx = 0) {
    /*Getting the mean values*/
    float sumX = 0;
    float sumY = 0;
    for (unsigned int i = 0; i < pointPairs.size(); i++) {
        sumX += (imageIdx == 0)? pointPairs[i].first.x : pointPairs[i].second.x;
        sumY += (imageIdx == 0)? pointPairs[i].first.y : pointPairs[i].second.y;
    }

    /*Mean of X and Y*/
    const float meanX = sumX / (float)pointPairs.size();
    const float meanY = sumY / (float)pointPairs.size();

    /*Forming the scaler in the normalization matrix*/
    float den = 0;
    for (unsigned int i = 0; i < pointPairs.size(); i++) {
        float x = (imageIdx == 0) ? pointPairs[i].first.x : pointPairs[i].second.x;
        float y = (imageIdx == 0) ? pointPairs[i].first.y : pointPairs[i].second.y;

        den += sqrt((x - meanX)*(x - meanX) + (y - meanY)*(y - meanY));
    }

    const float scaler = sqrt(2 * pointPairs.size()) / den;

    Mat T = Mat::eye(3, 3, CV_32F);
    T.at<float>(2, 2) = 1.0 / scaler;
    T.at<float>(1, 2) = -meanY;
    T.at<float>(0, 2) = -meanX;

    return scaler * T;
}


/// <summary>
///     Trnasform a Point2F by the given transfromation matrix.
/// </summary>
/// <param name="p">The desired point to transform</param>
/// <param name="normalizer">The transformatoin matrix</param>
/// <returns>Transformed point</returns>
Point2f transformPoint(const Point2f& p, const Mat& transformer ) {
    /*Homogenous representation of the point.*/
    Mat p1(3, 1, CV_32F);
    p1.at<float>(0, 0) = p.x;
    p1.at<float>(1, 0) = p.y;
    p1.at<float>(2, 0) = 1;

    /*Normalizing the point with the normalizer matrix*/
    Mat p1n = transformer * p1;  // [u, v, 1]
    p1n = (1.0 / p1n.at<float>(2, 0)) * p1n;

    // Note that by default if you apply the normalization matrix. The depth value yould remain 1
    return Point2f(p1n.at<float>(0, 0), p1n.at<float>(1, 0)); 
}




/// <summary>
///     Helper function taken from the course materials. It helps in transforming the image with regard to the 
///     transformation matrix, and the prespective projection.
/// </summary>
/// <param name="origImg">Input image plane</param>
/// <param name="newImage">Image plane of projection</param>
/// <param name="tr">Transformation matrix of the orig image</param>
/// <param name="isPerspective">perspective or orthographic</param>
void transformImage(Mat origImg, Mat& newImage, Mat tr, bool isPerspective) {
    Mat invTr = tr.inv();
    const int WIDTH = origImg.cols;
    const int HEIGHT = origImg.rows;

    const int newWIDTH = newImage.cols;
    const int newHEIGHT = newImage.rows;

    for (int x = 0; x < newWIDTH; x++)
        for (int y = 0; y < newHEIGHT; y++) {
            Mat pt(3, 1, CV_32F);
            pt.at<float>(0, 0) = x;
            pt.at<float>(1, 0) = y;
            pt.at<float>(2, 0) = 1.0;

            Mat ptTransformed = invTr * pt;
            if (isPerspective) ptTransformed = (1.0 / ptTransformed.at<float>(2, 0)) * ptTransformed;

            int newX = round(ptTransformed.at<float>(0, 0));
            int newY = round(ptTransformed.at<float>(1, 0));

            if ((newX >= 0) && (newX < WIDTH) && (newY >= 0) && (newY < HEIGHT)) newImage.at<Vec3b>(y, x) = origImg.at<Vec3b>(newY, newX);
        }
}