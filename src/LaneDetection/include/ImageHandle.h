/***********************************************************************
 * @file ImageHandle.h
     IMAGEHANDLE
 * @brief   header file
 * @history
 * Date       Version Author    description
 * ========== ======= ========= =======================================
 * 2019-11-22 V1.0    Kaiyu Lei   Create
 *
 * @Copyright (C)  2019  .cdWFVCEL. all right reserved
***********************************************************************/
#ifndef __IMAGEHANDLE_h__
#define __IMAGEHANDLE_h__

#ifdef __IMAGEHANDLE_h__GLOBAL
    #define __IMAGEHANDLE_h__EXTERN 
#else
    #define __IMAGEHANDLE_h__EXTERN extern
#endif

#include <iostream>
#include <utility>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/StdVector>

// TODO: 定义更多的阈值过滤方法
#define COLOR_THRESHOLD 0

// 透视变换
typedef std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>> PerspecPoints;
typedef cv::Mat PerspecMatrix;

// 参数
typedef struct ImageHandle_Paras {
    PerspecPoints _perspec;           // 透视变换
    int _dev_num;                     // 相机设备名
    int filter_min;                   // 阈值过滤的最小值
    int filter_max;                   // 阈值过滤的最大值
};

/*
 *@breif: Image handle class, to put a filter on a particular threshold in an image. 
 *@Date: 2019-11-22 17:06:05
*/
class ImageHandle {
public:
    /*
     *@breif: Create an image handler 
     *@Date: 2019-11-22 17:09:32
    */
    ImageHandle();
    ImageHandle(int method);
    ImageHandle(ImageHandle_Paras parameters);
    ImageHandle(const ImageHandle& image_handle) = delete;
    ImageHandle& operator=(const ImageHandle& image_handle) = delete;
    ~ImageHandle();

    /*
     *@breif: recieve an image
     *@param: image pointer
     *@return: none
     *@Date: 2019-11-22 17:13:33
    */
    void setImage(cv::Mat *image);

   /*
    *@breif: handle an image
    *@param: picture which should be handled
    *@return: picture which has been handled
    *@Date: 2019-11-22 17:40:46
   */
    cv::Mat handleImage();

    // TEST private functions
    void TEST() {
        hls_filtering(200, 255, 's');
        hls_filtering(200, 255, 'h');
        hls_filtering(200, 255, 'l');

        computePerspecMatrix();
    }

private:
    /*
     *@breif: comput perspective matrix
     *@param: 
     *@return: 
     *@Date: 2019-11-22 19:51:32
    */
    void computePerspecMatrix();

    /*
     *@breif: Project image with perspective matrix
     *@param: perspective matrix
     *@return: 
     *@Date: 2019-11-22 19:49:12
    */
    void project(PerspecMatrix matrix);

    /*
     *@breif: filtring image with hls colors information
     *@param: member in class img_
              threashold values(minimun and maxium) of "l" channel
     *@return: 
     *@Date: 2019-11-22 19:41:08
    */
    void hls_filtering(double min, double max, char channel = 's');

    cv::Mat getPerspectMatrix();
    cv::Mat getTranformImage();

private:
    cv::Mat *img_;
    cv::Mat img_filter_;
    cv::Mat img_perspec_;
    ImageHandle_Paras parameters_;
    PerspecMatrix perspective_matrix_;
};


#endif // __IMAGEHANDLE_h__
