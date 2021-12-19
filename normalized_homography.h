#pragma once
#include "Homography.h";




/// <summary>
/// Normalized Homography. It is used to calculate the homography of images with respect to the 
/// normalization of points. Note that getHomography() will return the full homography. If you want 
/// to return the normalized Homography you can call getHomographyNormalized()
/// </summary>
class NormalizedHomography : public Homography {
public:
    NormalizedHomography(const std::string& windowName, const Mapper& mappingPoints, bool showWindow = true) {
        this->m_windowName = windowName; 
        this->m_mappingPoints = mappingPoints;
        this->m_showWindow = showWindow;
        this->calculate();
    }
    cv::Mat calculate(const Mapper& pointPairs);
    cv::Mat calculate() {return this->calculate(this->m_mappingPoints);}

    cv::Mat getHomography() { return this->m_homography; }
    cv::Mat getHomographyNormalized() { return this->H_; }
private:
    cv::Mat T, T_, H_; // Normalizers matrices
};




///////////////////////////
//////////////////////////////////////////// Homography Function Definitions ////////////////////////////////////////////
//////////////////////////



cv::Mat NormalizedHomography::calculate(const Mapper& pointPairs) {
    /*Creating the Normalization Matrix for the feature points.*/
    pair<Mat, Mat> normalizers = { getNormalizer(pointPairs) , getNormalizer(pointPairs, 1) };
    T = normalizers.first;
    T_ = normalizers.second;

    const int ptsNum = pointPairs.size();
    Mat A(2 * ptsNum, 9, CV_32F);

    // Filling up the Matrix A
    for (int i = 0; i < ptsNum; i++) {
        Point2f p1 = transformPoint(pointPairs[i].first, normalizers.first);

        /*Preparing "A" for homogrophy least squared method.*/
        float u1 = p1.x;//pointPairs[i].first.x;
        float v1 = p1.y;//pointPairs[i].first.y;

        Point2f p2 = transformPoint(pointPairs[i].second, normalizers.second);
        float u2 = p2.x;
        float v2 = p2.y;

        A.at<float>(2 * i, 0) = u1;
        A.at<float>(2 * i, 1) = v1;
        A.at<float>(2 * i, 2) = 1.0f;
        A.at<float>(2 * i, 3) = 0.0f;
        A.at<float>(2 * i, 4) = 0.0f;
        A.at<float>(2 * i, 5) = 0.0f;
        A.at<float>(2 * i, 6) = -u2 * u1;
        A.at<float>(2 * i, 7) = -u2 * v1;
        A.at<float>(2 * i, 8) = -u2;

        A.at<float>(2 * i + 1, 0) = 0.0f;
        A.at<float>(2 * i + 1, 1) = 0.0f;
        A.at<float>(2 * i + 1, 2) = 0.0f;
        A.at<float>(2 * i + 1, 3) = u1;
        A.at<float>(2 * i + 1, 4) = v1;
        A.at<float>(2 * i + 1, 5) = 1.0f;
        A.at<float>(2 * i + 1, 6) = -v2 * u1;
        A.at<float>(2 * i + 1, 7) = -v2 * v1;
        A.at<float>(2 * i + 1, 8) = -v2;

    }

    // Getting the Eigen vector and values of matrix AT A
    Mat eVecs(9, 9, CV_32F), eVals(9, 9, CV_32F);
    eigen(A.t() * A, eVals, eVecs);

    // Calculating the homogrophy matrix
    Mat H(3, 3, CV_32F);
    for (int i = 0; i < 9; i++)
        H.at<float>(i / 3, i % 3) = eVecs.at<float>(8, i);

    //Normalize:
    H = H * (1.0 / H.at<float>(2, 2));

#ifdef INFO_LOG
    std::cout << A << std::endl;
    std::cout << eVals << std::endl;
    std::cout << eVecs << std::endl;
    std::cout << H << std::endl;
#endif
    this->H_ = H;
    this->m_homography = T_.inv() * H * T;
    return this->m_homography;
}
