#pragma once
#include "Homography.h";

/// <summary>
/// Non Normalized Homography. It is used to calculate the images homogrophy without considering 
/// normalizing the set of feature points.
/// </summary>
class NNHomography : public Homography {
public:
    NNHomography(const std::string& windowName, const Mapper& mappingPoints, bool showWindow = true) {
        this->m_windowName = windowName;
        this->m_mappingPoints = mappingPoints;
        this->m_showWindow = showWindow;
        this->calculate();
    }


    cv::Mat calculate() {
        return this->calculate(this->m_mappingPoints);
    }

    cv::Mat calculate(const Mapper& pointPairs) {
        const int ptsNum = pointPairs.size();
        cv::Mat A(2 * ptsNum, 9, CV_32F);

        // Filling up the Matrix A
        for (int i = 0; i < ptsNum; i++) {
            float u1 = pointPairs[i].first.x;
            float v1 = pointPairs[i].first.y;

            float u2 = pointPairs[i].second.x;
            float v2 = pointPairs[i].second.y;

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
        cv::Mat eVecs(9, 9, CV_32F), eVals(9, 9, CV_32F);
        eigen(A.t() * A, eVals, eVecs);

        // Calculating the homogrophy matrix
        cv::Mat H(3, 3, CV_32F);
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

        this->m_homography = H;
        return H;
    }

};