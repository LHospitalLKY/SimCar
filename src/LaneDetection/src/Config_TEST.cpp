//
// Created by LHospital
//

#include "../include/ImageHandle.h"
#include "../include/LaneFitting.h"

#include <iostream>
#include <chrono>
#include <libconfig.h++>

using namespace libconfig;

int main(int argc, char *argv[]) {

    Config conf;
    char filename[] = "src/LaneDetection/conf/paramters.conf";
    try {
        conf.readFile(filename);
    } catch(const FileIOException &fioex) {
        std::cout << "Error to READ file: " << filename << std::endl;
        return(EXIT_FAILURE);
    } catch(const ParseException &pex) {
        std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine()
              << " - " << pex.getError() << std::endl;
        return(EXIT_FAILURE);
    }

    int image_height, image_width;
    image_height = conf.lookup("image_size.height");
    image_width = conf.lookup("image_size.width");

    // 1. 读取透视变换的点
    std::vector<cv::Point2f> origin;
    std::vector<cv::Point2f> distortion;
    // 源点
    cv::Point2f origin_tmp;
    const Setting &origin_points = conf.lookup("origin_points");
    for(int i = 0; i < origin_points.getLength(); i++) {
        origin_points[i].lookupValue("x", origin_tmp.x);
        origin_points[i].lookupValue("y", origin_tmp.y);
        origin.push_back(origin_tmp);
        std::cout << "Point: " << origin_tmp.x << " " << origin_tmp.y << std::endl;
    }
    // 投影点
    cv::Point2f distortion_tmp;
    const Setting &distortion_points = conf.lookup("distortion_points");
    for(int i = 0; i < distortion_points.getLength(); i++) {
        distortion_points[i].lookupValue("x", distortion_tmp.x);
        distortion_points[i].lookupValue("y", distortion_tmp.y);
        distortion.push_back(distortion_tmp);
        std::cout << "Point: " << distortion_tmp.x << " " << distortion_tmp.y << std::endl;
    }
    
    // 2. 读取阈值过滤的上下界
    int threshold_min = conf.lookup("threshold_min");
    int threshold_max = conf.lookup("threshold_max");

    std::cout << "image size: " << image_height << " " << image_width << std::endl;

    // 构建ImageHandle_Paras
    ImageHandle_Paras paras;
    paras._perspec.first = origin;
    paras._perspec.second = distortion;
    paras._dev_num = -1;
    paras.filter_min = threshold_min;
    paras.filter_max = threshold_max;

    // 进行图像预处理
    ImageHandle image_handle(paras);
    std::string image_path = conf.lookup("test_image");
    cv::Mat lane = cv::imread(image_path);
    cv::imshow("origin", lane);
    cv::waitKey(0);
    image_handle.setImage(&lane);

    cv::Mat distorted_lane = image_handle.handleImage();
    cv::imshow("distorted", distorted_lane);
    cv::waitKey(0);

    // 车道拟合
    // 构建LaneFitParas
    LaneFitParas fit_paras;
    fit_paras.nwindows = conf.lookup("nwindows");
    fit_paras.margin = conf.lookup("margin");
    fit_paras.minpix = conf.lookup("minpix");
    auto lane_fit_start = std::chrono::high_resolution_clock::now();
    LaneFitting lane_fit(fit_paras);
    lane_fit.laneFitting(distorted_lane);
    auto lane_fit_end = std::chrono::high_resolution_clock::now();

    auto lane_fit_millise = std::chrono::duration_cast<std::chrono::milliseconds>(lane_fit_end - lane_fit_start);
    std::cout<<"use "<< lane_fit_millise.count()<<" millise\n";

    lane_fit.showLane();

    std::cout << "Hello World!" << std::endl;
    return 1;

}
