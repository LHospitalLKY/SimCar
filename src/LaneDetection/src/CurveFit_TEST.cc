#include "../include/CurveFit.h"

#include <bits/stdc++.h>

int main()
{
    double a=4, b=2.0, c=-1.0;         // 真实参数值
    int N = 100;                          // 数据点
    double w_sigma= 1.;                 // 噪声Sigma值

    std::default_random_engine generator;
    std::normal_distribution<double> noise(0.,w_sigma);

    // 构造 N 次观测
    std::vector<double> X;
    std::vector<double> Y;
    for (int i = 0; i < N; ++i) {

        double x = 10*i/100.0;
        double n = noise(generator);
        // 观测 y
        double y = a*x*x + b*x + c + n;
//        double y = std::exp( a*x*x + b*x + c );
        X.push_back(x);
        Y.push_back(y);
    }

    std::vector<double> abc;
    QuadFitting qf(&X, &Y);
    qf.solve(abc);

    std::cout << "abc: " << abc[0] << " ";
    std::cout << abc[1] << " ";
    std::cout << abc[2] << "\n";

    return 0;
}