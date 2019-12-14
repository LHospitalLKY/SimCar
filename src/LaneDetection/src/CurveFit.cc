
#include "../include/CurveFit.h"

bool QuadFitting::solve(std::vector<double> *abc) {
    // 设置初值
    double a = 0., b = 0., c = 0.;
    // 数据点
    int N = x_ -> size();

    if(N <= 3) {
        std::cout << "Num of x is wrong: " << N << "\n";
        return false;
    }

    // 构建problem
    Problem problem(Problem::ProblemType::GENERIC_PROBLEM);
    std::shared_ptr<CurveFittingVertex> vertex(new CurveFittingVertex());

    // 设置初始值
    vertex -> SetParameters(Eigen::Vector3d(0.0, 0.0, 0.0));
    // 向最小二乘问题中加入vertex
    problem.AddVertex(vertex);

    // 向最小二乘问题中加入edge
    for(int i = 0; i < N; ++i) {
        double x = (*x_)[i];
        double y = (*y_)[i];

        // std::cout << "x: " << x << "\n";
        // std::cout << "y: " << y << "\n";

        std::shared_ptr<CurveFittingEdge> edge(new CurveFittingEdge(x, y));
        std::vector<std::shared_ptr<Vertex>> edge_vertex;
        edge_vertex.push_back(vertex);
        edge -> SetVertex(edge_vertex);

        // 向problem中加入edge
        problem.AddEdge(edge);
    }

    problem.Solve(50);

    // TODO: Debug结束后删除
    // std::cout << "-------After optimization, we got these parameters :" << std::endl;
    // std::cout << vertex -> Parameters().transpose() << std::endl;

    a_ = vertex -> Parameters()(0);
    b_ = vertex -> Parameters()(1);
    c_ = vertex -> Parameters()(2);

    abc -> push_back(a_);
    abc -> push_back(b_);
    abc -> push_back(c_);

    return true;
}