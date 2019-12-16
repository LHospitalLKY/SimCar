/***********************************************************************
 * @file LANEFITTING.h
     LANEFITTING
 * @brief   header file
 * @history
 * Date       Version Author    description
 * ========== ======= ========= =======================================
 * 2019-11-22 V1.0    Leikai Yu   Create
 *
 * @Copyright (C)  2019  .cdWFVCEL. all right reserved
***********************************************************************/
#ifndef __LANEFITTING_h__
#define __LANEFITTING_h__

#ifdef __LANEFITTING_h__GLOBAL
    #define __LANEFITTING_h__EXTERN 
#else
    #define __LANEFITTING_h__EXTERN extern
#endif

#include <iostream>
#include <vector>
#include <random>

#include "CurveFit.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Eigen>

typedef struct LaneFitParas {
    int nwindows;
    int margin;
    int minpix;
};

class LaneFitting {
public:
    LaneFitting();
    LaneFitting(LaneFitParas &fit_paras);
    ~LaneFitting() = default;

public:
    /*
     *@breif: Fine the lane in the image coordinate
     *@param: member image_pres_ in class
     *@return: coefficients of quadratic function in image coordinate
     *@Date: 2019-11-22 19:24:55
    */
    bool findLane(cv::Mat image_pres);

    /*
     *@breif: fit the lane curve in real world
     *@param: members in class:
                1. image_pres_
                2. left_x_, left_y, right_x_, right_y_
                3. meters per pixel in x and y
     *@return: if fit the lane success
     *@Date: 2019-11-22 19:28:46
    */
    double laneFitting(cv::Mat image_pres);

    void showLane();

private:
    /*
     *@breif: 计算阈值过滤后以底边为基准的直方图，找到左右辆车道的base位置 
     *@param: 
     *@return: 状态量，1表示左右都找到了，0表示左右都没找到
     *@Date: 2019-12-05 09:59:49
    */
    int findLaneBase();

    /*
     *@breif: 通过滑动窗口来计算车道线拟合函数 
     *@param: 
     *@return: 
     *@Date: 2019-12-05 10:03:57
    */
    int sildeWindows();

private:
    cv::Mat image_pres_;           // 投影变换之后的图像
    Eigen::MatrixXd *image_eigen_;   // Eigen格式的图像
    // 二次函数的参数
    std::vector<double> left_abc_image_;
    std::vector<double> right_abc_image_;
    std::vector<double> left_abc_world_;
    std::vector<double> right_abc_world_;
    // 左侧车道线控制点
    std::vector<double> left_x_;
    std::vector<double> left_y_;
    // 右侧车道线控制点
    std::vector<double> right_x_;
    std::vector<double> right_y_;

    int nwindows_;                // 滑动窗口的数量
    int margin_;                  // 滑动窗口的宽度
    int minpix_;                  // 滑动窗口中最小的像素点

    // 图像的长度和高度
    int column_;
    int rows_;

    double xm_per_pix_;           // x方向每个像素对应的长度
    double ym_per_pix_;           // y方向每个像素对应的长度

    double curvature_;               // 曲率
    double theta_;                   // 偏移角度
    double distance_from_center_;    // 距离中心的距离
};

#endif // __LANEFITTING_h__
