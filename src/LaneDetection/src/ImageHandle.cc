// ImageHandle.cc

#include "../include/ImageHandle.h"

#include <iostream>

ImageHandle::ImageHandle() {
    parameters_._dev_num = 0;

    std::vector<cv::Point2f> origin;
    std::vector<cv::Point2f> distortion;

    cv::Point2f origin_tmp;
    origin_tmp.x = 426;
    origin_tmp.y = 720;
    origin.push_back(origin_tmp);
    origin_tmp.x = 580;
    origin_tmp.y = 480;
    origin.push_back(origin_tmp);
    origin_tmp.x = 650;
    origin_tmp.y = 480;
    origin.push_back(origin_tmp);
    origin_tmp.x = 853;
    origin_tmp.y = 720;
    origin.push_back(origin_tmp);

    cv::Point2f distortion_tmp;
    distortion_tmp.x = 426;
    distortion_tmp.y = 720.0;
    distortion.push_back(distortion_tmp);
    distortion_tmp.x = 426;
    distortion_tmp.y = 0.0;
    distortion.push_back(distortion_tmp);
    distortion_tmp.x = 853;
    distortion_tmp.y = 0;
    distortion.push_back(distortion_tmp);
    distortion_tmp.x = 853;
    distortion_tmp.y = 720.0;
    distortion.push_back(distortion_tmp);

    parameters_._perspec.first = origin;
    parameters_._perspec.second = distortion;
    parameters_.filter_min = 150;
    parameters_.filter_max = 255;
}

ImageHandle::ImageHandle(ImageHandle_Paras paras) {
    parameters_ = paras;
}

ImageHandle::~ImageHandle() {
    img_= nullptr;
}

void ImageHandle::setImage(cv::Mat *image) {
    img_ = image;
}

void ImageHandle::hls_filtering(double min, double max, char channel) {
    cv::Mat image_hls;
    cv::cvtColor(*img_, image_hls, cv::COLOR_RGB2HLS);

    // 进行颜色过滤
    cv::Mat img_singleChannel;
    std::vector<cv::Mat> channels;
    cv::split(*img_, channels);

    switch(channel){
        case 'h':
            img_singleChannel = channels[0];
        case 'l':
            img_singleChannel = channels[1];
        case 's':
            img_singleChannel = channels[2];
    }
    
    int type = img_singleChannel.type();

    // 过滤
    /*
    std::cout << "Image size---------------->" << std::endl;
    std::cout << "rows: " << img_singleChannel.rows << " ";
    std::cout << "cols: " << img_singleChannel.cols << std::endl;
    */

    // cv::imshow("img_singleChannel", img_singleChannel);
    // cv::waitKey(0);
    img_filter_ = cv::Mat(cv::Size(img_singleChannel.cols, img_singleChannel.rows), CV_8UC1, cv::Scalar(0));
    // cv::imshow("zero matrix", img_filter_);
    // cv::waitKey(0);
    int k = 0;
    for(int i = 0; i < img_filter_.rows; i++) {
        for(int j = 0; j < img_filter_.cols; j++) {
            // std::cout << img_singleChannel.at<uchar>(i, j) << " ";
            // std::cout << img_singleChannel.at<uchar>(i, j) << std::endl;
            if(img_singleChannel.at<uchar>(i, j) > min 
                && img_singleChannel.at<uchar>(i, j) < max) {
                img_filter_.at<uchar>(i, j) = 255;
                k++;
            }
        }
    }

    // TODO: Debug结束后删除
    // cv::imshow("hls threshold", img_filter_);
    // cv::waitKey(0);
}

void ImageHandle::computePerspecMatrix() {
    perspective_matrix_ = cv::getPerspectiveTransform(parameters_._perspec.first, parameters_._perspec.second);

    // TODO: Debug之后删除
    // std::cout << "Perspective Matrix:" << std::endl;
    // std::cout << perspective_matrix_ << std::endl;

    // 将原图透视变换
    cv::warpPerspective(img_filter_, img_perspec_,perspective_matrix_, cv::Size(img_filter_.cols, img_filter_.rows), cv::INTER_LINEAR);

    // cv::imshow("src", img_filter_);
    // cv::waitKey(0);
    // cv::imshow("Perspective", img_perspec_);
    // cv::waitKey(0);
}

cv::Mat ImageHandle::handleImage() {
    hls_filtering(200, 255, 's');
    computePerspecMatrix();

    return img_perspec_;
}