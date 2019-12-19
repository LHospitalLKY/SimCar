//
// Created by LHospital
//

#include "../include/ImageHandle.h"
#include "../include/LaneFitting.h"

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <libconfig.h++>

using namespace libconfig;

void config(Config &conf, ImageHandle_Paras *ih_paras, LaneFitParas *lf_paras);

int main(int argc, char *argv[]) {

	ros::init(argc, argv, "node_laneDetection");	//初始化ROS，节点名命名为node_a，节点名必须保持唯一
	ros::NodeHandle n;	//实例化节点, 节点进程句柄
	ros::Publisher pub = n.advertise<std_msgs::Float32>("/offset_from_lanecenter", 1000);	//告诉系统要发布话题了，话题名为“str_message”，类型为std_msgs::String，缓冲队列为1000。
    
	ros::Rate loop_rate(1000);	//设置发送数据的频率为10Hz
    //ros::ok()返回false会停止运行，进程终止。
	
	// 读取配置信息
	Config conf;
	ImageHandle_Paras ih_paras;
	LaneFitParas lf_paras;
	config(conf, &ih_paras, &lf_paras);
	ImageHandle image_handle(ih_paras);
	LaneFitting lane_fit(lf_paras);
	int rows = conf.lookup("image_size.height");
    int cols = conf.lookup("image_size.width");
	// 读取视频或摄像头
	cv::VideoCapture cap;
    
	std::string type = conf.lookup("type");
	if(type == "video") {
		std::string vedio_path = conf.lookup("test_video");
		cap.open(vedio_path);
	}
	else if(type == "camera") {
		// TODO: 补充完整
        int camera_num = conf.lookup("camera_num");
        cap.set(cv::CAP_PROP_FRAME_WIDTH, cols);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, rows);
        cap.open(camera_num);
	}

    double err = 0;
	cv::Mat video_frame;
	cv::Mat resize_frame;
    while(ros::ok())
	{
		// 读取摄像头
		cap >> video_frame;
        if(video_frame.empty()) {
            std::cout << "Vedio finished!" << std::endl;
            break;
        }
		if(type == "video") {
            cv::resize(video_frame, resize_frame, cv::Size(cols, rows));
        }
        if(type == "camera") {
            resize_frame = video_frame.clone();
        }
		// std::cout << vedio_frame.size() << std::endl;
		// 处理图像

        // 显示图像
        // cv::imshow("vedio", resize_frame);
        // cv::waitKey(5);

		image_handle.setImage(&resize_frame);
		// cv::imshow("origin", resize_frame);
		cv::Mat distorted_lane = image_handle.handleImage();
		// cv::imshow("distorted_lane", distorted_lane);
		// cv::waitKey(0);
		// 拟合
		err = lane_fit.laneFitting(distorted_lane);
		lane_fit.showLane();

		std_msgs::Float32 msg;
		msg.data = err;
		
        ROS_INFO_STREAM("offset is publishing: " << msg.data);

		pub.publish(msg);	//向话题“str_message”发布消息
		ros::spinOnce();	//不是必须，若程序中订阅话题则必须，否则回掉函数不起作用。
		loop_rate.sleep();	//按前面设置的10Hz频率将程序挂起
	}
 
	return 0;

}

void config(Config &conf, ImageHandle_Paras *ih_paras, LaneFitParas *lf_paras) {
    char filename[] = "src/LaneDetection/conf/paramters.conf";
    try {
        conf.readFile(filename);
    } catch(const FileIOException &fioex) {
        std::cout << "Error to READ file: " << filename << std::endl;
        // return(EXIT_FAILURE);
    } catch(const ParseException &pex) {
        std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine()
              << " - " << pex.getError() << std::endl;
        // return(EXIT_FAILURE);
    }

    int image_height, image_width;
    image_height = conf.lookup("image_size.height");
    image_width = conf.lookup("image_size.width");

	// 构造ImageHandle_Paras
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

    ih_paras -> _perspec.first = origin;
    ih_paras -> _perspec.second = distortion;
    ih_paras -> _dev_num = -1;
    ih_paras -> filter_min = threshold_min;
    ih_paras -> filter_max = threshold_max;

    lf_paras -> nwindows = conf.lookup("nwindows");
    lf_paras -> margin = conf.lookup("margin");
    lf_paras -> minpix = conf.lookup("minpix");

	std::cout << "image size: " << image_height << " " << image_width << std::endl;
	std::cout << "filter: " << ih_paras -> filter_min << " " << ih_paras -> filter_max << std::endl;
	std::cout << "nwindows: " << lf_paras -> nwindows << std::endl;
	std::cout << "margin: " << lf_paras -> margin << std::endl;
	std::cout << "minpix: " << lf_paras -> minpix << std::endl;
	
}