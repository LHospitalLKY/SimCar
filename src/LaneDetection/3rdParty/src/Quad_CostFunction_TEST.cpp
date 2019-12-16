//
// Created by LHospital
//

#include "Quad_CostFunction.cc"

#include <bits/stdc++.h>
#include <iostream>
#include <opencv2/opencv.hpp>

void generateData(Eigen::VectorXd x, Eigen::VectorXd y);
void test1();
void test2();

int main(int argc, char *argv[]) {

    test1();
    test2();

}

// 将参数设置为a = 1, b = 3, c = 2
void generateData(Eigen::VectorXd *x, Eigen::VectorXd *y) {

    std::vector<double> X;
    std::vector<double> Y;
    int N = 100;                           // 数据点
    double w_sigma = 1.0;                  // 噪声Sigma值
    double inv_sigma = 1.0 / w_sigma;
    cv::RNG rng;                     // OpenCV随机数产生器

    for(int i = 0; i < N; i++) {
        // double random = rand()%1000/(double)1001;
        std::cout << "random: " << random;
        (*x)[i] = (double)i;
        (*y)[i] = pow((*x)[i], 2) + 3 * (*x)[i] + 2 + rng.gaussian(w_sigma*w_sigma);
    }

    std::cout << "Generate data success!";
    std::cout << "x: " << *x;
    std::cout << "y: " << *y; 
}

void test1() {

    Eigen::VectorXd x(100), y(100);
    generateData(&x, &y);

    // 定义数据
    int num_of_parameters = 3;
    int num_of_residuals = 1;
    int num_of_constants = 2;       // 一个x一个y

    Eigen::VectorXd parameters(num_of_parameters);
    Eigen::VectorXd residuals(num_of_residuals);
    Eigen::VectorXd constants(num_of_constants);

    // parameter的初值
    parameters << 0, 0, 0;
    
    for(int i = 0; i < 9; i++) {
        std::cout << "x, y: " << x(i) << " " << y(i);
        constants << y(i), x(i);
        Quad_CostFunction cost_function(&parameters, &residuals, num_of_parameters, num_of_residuals, &constants, num_of_constants);
        cost_function.computeCost();
        cost_function.computeJacobian();
    }

}
/*
TEST(ResidualFunction_TEST, RedisualFunction_TEST) {
    Eigen::VectorXd x(10), y(10);
    generateData(&x, &y);

    // 定义数据
    int num_of_parameters = 3;
    int num_of_residuals = 1;
    int num_of_constants = 2;       // 一个x一个y

    Eigen::VectorXd parameters(num_of_parameters);
    Eigen::VectorXd residuals(num_of_residuals);
    Eigen::VectorXd constants(num_of_constants);

    // parameter的初值
    parameters << 0.8, 3.7, 1.5;
    
    
    for(int i = 0; i < 10; i++) {
        std::cout << "x, y: " << x(i) << " " << y(i);
        constants << y(i), x(i);
        Quad_CostFunction cost_function(&parameters, &residuals, num_of_parameters, num_of_residuals, &constants, num_of_constants);
        ResidualFunction residual_block_0(&cost_function, 0);
        Eigen::MatrixXd H1 = residual_block_0.computeHessian();
        Eigen::VectorXd B1 = residual_block_0.computeBias();
        ResidualFunction residual_block_1(&cost_function, 1);
        Eigen::MatrixXd H2 = residual_block_1.computeHessian();
        Eigen::VectorXd B2 = residual_block_1.computeBias();
    }
}*/

void test2() {
    Eigen::VectorXd x(100), y(100);
    generateData(&x, &y);

    // 定义数据
    int num_of_parameters = 3;
    int num_of_residuals = 1;
    int num_of_constants = 2;       // 一个x一个y

    Eigen::VectorXd parameters(num_of_parameters);
    Eigen::VectorXd residuals(num_of_residuals);
    Eigen::VectorXd constants(num_of_constants);

    // parameter的初值
    parameters << 0, 0, 0;
    Quad_CostFunction *cost_function = nullptr;
    ResidualFunction *residual_block_0 = nullptr;
    std::vector<Quad_CostFunction*> cost_vector;
    GradientDesent gd(parameters, 3, 1, GN);
    for(int i = 0; i < 100; i++) {
        std::cout << "x, y: " << x(i) << " " << y(i);
        constants << y(i), x(i);
        std::cout << "Constant: " << constants;
        cost_function = new Quad_CostFunction(&parameters, &residuals, num_of_parameters, num_of_residuals, &constants, num_of_constants);

        std::cout << "constant in cost_function: " << cost_function -> constants_;
        cost_vector.push_back(cost_function);
        residual_block_0 = new ResidualFunction(cost_function, 0);
        
        gd.addResidualBlock(residual_block_0);
    }

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    gd.solve();

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    std::chrono::duration<double> time_used = std:: chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
    std::cout << "Time used: " << time_used.count() << " milliseconds." << std::endl;
}

