// LaneFitting.cc

#include "../include/LaneFitting.h"

LaneFitting::LaneFitting() {
    // 采用默认的参数
    nwindows_ = 7;
    margin_ = 100;
    minpix_ = 25;

    xm_per_pix_ = 30. / 180;
    ym_per_pix_ = 3.7 / 220;
}

LaneFitting::LaneFitting(LaneFitParas &fit_paras) {
    nwindows_ = fit_paras.nwindows;
    margin_ = fit_paras.margin;
    minpix_ = fit_paras.minpix;

    xm_per_pix_ = 30. / 180;
    ym_per_pix_ = 3.7 / 220;
}

// TODO: 将这部分结构化
bool LaneFitting::findLane(cv::Mat image_pres) {
    image_pres_ = image_pres;
    column_ = image_pres_.cols;
    rows_ = image_pres_.rows;

    std::cout << "image size: " << column_ << " " << rows_ << std::endl;

    // 清空队列
    left_x_.clear();
    left_y_.clear();
    right_x_.clear();
    right_y_.clear();

    // 转为eigen
    // Eigen::MatrixXd image(rows_, column_);
    image_eigen_ = new Eigen::MatrixXd(rows_, column_);
    cv::cv2eigen(image_pres_, *image_eigen_);
    
    // 检查image_eigen_中是否有非零元素
    /*
    int none_zero = 0;
    for(int i = 0; i < rows_; i++) {
        for(int j = 0; j < column_; j++) {
            if((*image_eigen_)(i, j) > 0)
                none_zero++;
        }
    }*/

    // TODO: Debug之后删除
    // std::cout << "num of none zero: " << none_zero << std::endl;
    
    // 计算左右base
    int left_base, right_base;
    int left_max_hist, right_max_hist;
    // 按列求和
    Eigen::VectorXd histogram;
    histogram = image_eigen_ -> colwise().sum();
    // std::cout << "Histogram" << histogram << std::endl;

    // 将图像一分为二
    int mid = histogram.size()/2;
    Eigen::VectorXd left_histogram, right_histogram;
    left_histogram = histogram.segment(0, mid);
    right_histogram = histogram.segment(mid, mid);
    // TODO: Debug之后删除
    // std::cout << "left_histogram size: " << left_histogram.size() << " ";
    // std::cout << "right_histogram size: " << right_histogram.size() << std::endl;

    left_max_hist = 0;
    for(int i = 0; i < left_histogram.size(); i++) {
        if(left_histogram[i] > left_max_hist) {
            left_max_hist = left_histogram[i];
            left_base = i;
        }
    }
    right_max_hist = 0;
    for(int i = 0; i < right_histogram.size(); i++) {
        if(right_histogram[i] > right_max_hist) {
            right_max_hist = right_histogram[i];
            right_base = i;
        }
    }
    right_base += mid;

    // TODO: Debug之后删除
    // std::cout << "left base: " << left_base << "value: " << left_max_hist << " ";
    // std::cout << "right base: " << right_base << "value: " << right_max_hist << std::endl;

    // 滑动窗口
    // 滑动窗口的高度
    int window_height = image_pres_.rows/nwindows_;
    // 初始框的位置，以框的中心为基准
    int left_win_x_curr = left_base;
    int right_win_x_curr = right_base;
    int win_y_low, win_y_high;
    int left_win_x_low, left_win_x_high;
    int right_win_x_low, right_win_x_high;
    // 连续未在windows中找到足够数量的点的次数
    int left_empty = 0;
    int right_empty = 0;
    for(int i = 0; i < nwindows_; i++) {
        // 计算出当前框的位置
        win_y_low = rows_ - (i + 1) * window_height;
        win_y_high = rows_ - (i) * window_height;

        left_win_x_low = std::max(0, left_win_x_curr - margin_);
        left_win_x_high = std::min(column_, left_win_x_curr + margin_);
        right_win_x_low = std::max(0, right_win_x_curr - margin_);
        right_win_x_high = std::min(column_, right_win_x_curr + margin_);

        // 将方框中的点填入左右车道标志点队列中
        // 创建临时列表
        std::vector<double> left_x_tmp, left_y_tmp;
        std::vector<double> right_x_tmp, right_y_tmp;
        // 左侧
        int num_left = 0;
        for(int i = win_y_low; i < win_y_high; i++) {
            for(int j = left_win_x_low; j < left_win_x_high; j++) {
                if((*image_eigen_)(i, j) > 0) {
                    // 若当前位置存在未被过滤的点
                    left_x_tmp.push_back(j);
                    left_y_tmp.push_back(i);
                    num_left++;
                }
            }
        }

        int num_right = 0;
        for(int i = win_y_low; i < win_y_high; i++) {
            for(int j = right_win_x_low; j < right_win_x_high; j++) {
                if((*image_eigen_)(i, j) > 0) {
                    // 若当前位置存在未被过滤的点
                    right_x_tmp.push_back(j);
                    right_y_tmp.push_back(i);
                    num_right++;
                }
            }
        }

        // 更新左右的窗口中心
        // 左侧
        if(num_left > minpix_ && left_empty < nwindows_/2) {
            // 若当前窗口中非零点的数量大于minpix_
            // 将这些点加入队列，并求出下一个窗口的中心
            for(int i = 0; i < num_left; i++) {
                left_x_.push_back(left_x_tmp[i]);
                left_y_.push_back(left_y_tmp[i]);
            }
            left_win_x_curr = std::accumulate(left_x_tmp.begin(), left_x_tmp.end(), 0)/num_left;
            if(left_empty > 0)
                left_empty--;
        } else {
            left_empty++;
        }
        if(num_right > minpix_ && right_empty < nwindows_/2) {
            // 若当前窗口中非零点的数量大于minpix_
            // 将这些点加入队列，并求出下一个窗口的中心
            for(int i = 0; i < num_right; i++) {
                right_x_.push_back(right_x_tmp[i]);
                right_y_.push_back(right_y_tmp[i]);
            }
            right_win_x_curr = std::accumulate(right_x_tmp.begin(), right_x_tmp.end(), 0)/num_right;
            if(right_empty > 0)
                right_empty--;
        } else {
            right_empty++;
        }

        // TODO: Debug结束后删去
        // 左右各自找到的点的数量
/*
        std::cout << "window " << i << "----------" << "\n";
        printf("left lane find %d points\n", num_left);
        printf("right lane fine %d points\n", num_right);
        // 最终的框的位置为：
        std::cout << "left window: " << "\n";
        printf("(%d, %d) ", left_win_x_low, win_y_high);
        printf("(%d, %d) ", left_win_x_high, win_y_high);
        printf("(%d, %d) ", left_win_x_high, win_y_low);
        printf("(%d, %d) \n", left_win_x_low, win_y_low);
        std::cout << "right window: " << "\n";
        printf("(%d, %d) ", right_win_x_low, win_y_high);
        printf("(%d, %d) ", right_win_x_high, win_y_high);
        printf("(%d, %d) ", right_win_x_high, win_y_low);
        printf("(%d, %d) \n", right_win_x_low, win_y_low);
        // 左右empty的值
        std::cout << "left empty: " << left_empty << "\n";
        std::cout << "right empty: " << right_empty << std::endl;
*/

    }
    
    // 查看有多少个点
    // std::cout << "<------Loop finished!-----_>\n";
    // std::cout << "Num of left lane points: " << left_x_.size() << " " << left_y_.size() << " ";
    // std::cout << "Num of right lane points: " << right_x_.size() << " " << right_y_.size() << std::endl;

    // 若有一边没有找到点
    if(left_x_.size() != 0 && right_x_.size() == 0) {
        // 右侧未能找到车道线
        std::cout << "No RIGHT LANE" << std::endl;
        for(int i = 0; i < left_x_.size(); i++) {
            right_x_.push_back((column_ / 2.) + left_x_[i]);
            right_y_.push_back(left_y_[i]);
        }
    }
    if(left_x_.size() == 0 && right_x_.size() != 0) {
        // 左侧未能找到车道线
        std::cout << "No LEFT LANE" << std::endl;
        for(int i = 0; i < right_x_.size(); i++) {
            left_x_.push_back(right_x_[i] - (column_ / 2.));
            left_y_.push_back(right_y_[i]);
        }
    }
    if(left_x_.size() == 0 && right_x_.size() == 0) {
        std::cout << "No LANE here!" << std::endl;
        return false;
    }

    return true;
}

double LaneFitting::laneFitting(cv::Mat image_pres) {
    bool find_lane_success = findLane(image_pres);
    if(find_lane_success != true) {
        std::cout << "No Line here!" << std::endl;
        return 0;
    }
    // 清空abc队列
    left_abc_image_.clear();
    right_abc_image_.clear();
    // 1. 从左右控制点中随机选取固定数量的拟合点
    // 图像系
    std::vector<double> left_x_short, left_y_short;
    std::vector<double> right_x_short, right_y_short;
    // 图像映射到带长度单位的量
    std::vector<double> w_left_x_short, w_left_y_short;
    std::vector<double> w_right_x_short, w_right_y_short;
    std::vector<int> random_index_left, random_index_right;

    std::default_random_engine e;
    std::uniform_int_distribution<int> u_left(0, (int)left_x_.size());
    std::uniform_int_distribution<int> u_right(0, (int)right_y_.size());
    for(int i = 0; i < 30; i++) {
        random_index_left.push_back(u_left(e));
        random_index_right.push_back(u_right(e));
        // std::cout << "------> index: " << random_index_left[i] << " " << random_index_right[i] << std::endl;

        // 将随机选取的点插入到拟合队列中
        left_x_short.push_back(left_x_[random_index_left[i]]);
        left_y_short.push_back(left_y_[random_index_left[i]]);
        right_x_short.push_back(right_x_[random_index_right[i]]);
        right_y_short.push_back(right_y_[random_index_right[i]]);
        // std::cout << "------> point: " << left_x_short[i] << " " << left_y_short[i] << std::endl;
        /*
        w_left_x_short.push_back(left_x_[random_index_left[i]]*xm_per_pix_);
        w_left_y_short.push_back(left_y_[random_index_left[i]]*ym_per_pix_);
        w_right_x_short.push_back(right_x_[random_index_right[i]]*xm_per_pix_);
        w_right_y_short.push_back(right_y_[random_index_right[i]]*ym_per_pix_);
        */
    }
    // TODO: Debug结束之后删除
    /*
    std::cout << "random index left : " << "(len: " << left_x_short.size() << ")\n";
    for(int i = 0; i < 100; i++) {
        std::cout << random_index_left[i] << std::endl;
    }
    std::cout << "random index right : " << "(len: " << right_x_short.size() << ")\n";
    for(int i = 0; i < 100; i++) {
        std::cout << random_index_right[i] << std::endl;
    }*/

    // 3. 拟合左右车道线
    // TODO: 在实际中，若无需可视化，则只算world中的拟合曲线就好
    QuadFitting qf_left(&left_y_short, &left_x_short);
    qf_left.solve(left_abc_image_);
    // qf_left.set(&w_left_y_short, &w_left_x_short);
    // qf_left.solve(left_abc_world_);
    
    QuadFitting qf_right(&right_y_short, &right_x_short);
    qf_right.solve(right_abc_image_);
    // qf_right.set(&w_right_y_short, &w_right_x_short);
    // qf_right.solve(right_abc_world_);

    // 如果左右车道靠的太近
    double d_left, d_right;
    d_left = left_abc_image_[0] * rows_ * rows_ + left_abc_image_[1] * rows_ + left_abc_image_[2];
    d_right = right_abc_image_[0] * rows_ * rows_ + right_abc_image_[1] * rows_ + right_abc_image_[2];
    
    if(d_right - d_left < 50) {
        if((d_right + d_left)/2. <= column_/2.) {
            right_abc_image_[2] = left_abc_image_[2] + 2*column_/3.;
            // right_abc_world_[2] = left_abc_world_[2] + 2*column_*xm_per_pix_/3.;
        }
        else {
            left_abc_image_[2] = right_abc_image_[2] - 2*column_/3.;
            // left_abc_image_[2] = right_abc_image_[2] - 2*column_*xm_per_pix_/3.;
        }
    }

    // std::cout << "left lane: " << "\n";
    // std::cout << left_abc_image_[0] << " " << left_abc_image_[1] << " " << left_abc_image_[2] << "\n";
    // std::cout << left_abc_world_[0] << " " << left_abc_world_[1] << " " << left_abc_world_[2] << "\n";

    // std::cout << "right lane: " << "\n";
    // std::cout << right_abc_image_[0] << " " << right_abc_image_[1] << " " << right_abc_image_[2] << "\n";
    // std::cout << right_abc_world_[0] << " " << right_abc_world_[1] << " " << right_abc_world_[2] << "\n";
     
    // 4. 计算曲率、角度和中心偏移量
    // 计算更准确的车道宽度
    double lane_width_pix;
    d_left = left_abc_image_[0] * rows_ * rows_ + left_abc_image_[1] * rows_ + left_abc_image_[2];
    d_right = right_abc_image_[0] * rows_ * rows_ + right_abc_image_[1] * rows_ + right_abc_image_[2];
    lane_width_pix = d_right - d_left;
    double lane_xm_per_pix = 3.7 / lane_width_pix;
    // 图像底边中心为车辆当前位置
    double vhe_cen_pos = column_*lane_xm_per_pix / 2.;
    // 车道中心
    double lane_cen_pose = (d_right + d_left) * lane_xm_per_pix / 2.;

    distance_from_center_ = lane_cen_pose - vhe_cen_pos;

    // TODO: Debug之后删除
    std::cout << "d_left_: " << d_left << "d_right_: " << d_right << std::endl;
    std::cout << "lane_xm_per_pix: " << lane_xm_per_pix << std::endl;


    // std::cout << "distance_from_center_" << distance_from_center_ << std::endl;

    return distance_from_center_;
}

void LaneFitting::showLane() {
    cv::Mat lane(rows_, column_, CV_8UC1);
    for(int i = 0; i < rows_; i++) {
        for(int j = 0; j < column_; j++) {
            lane.at<uchar>(i, j)  = 0;
        }
    }
    for(int i = 0; i < left_x_.size(); i++) {
        lane.at<uchar>(left_y_[i], left_x_[i]) = 255;
    }
    for(int i = 0; i < right_x_.size(); i++) {
        lane.at<uchar>(right_y_[i], right_x_[i]) = 255;
    }

    cv::imshow("lane", lane);
    cv::waitKey(5);
}