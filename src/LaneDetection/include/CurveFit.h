/***********************************************************************
 * @file CurveFit.h
     CURVEFIT
 * @brief   header file
 * @history
 * Date       Version Author    description
 * ========== ======= ========= =======================================
 * 2019-12-09 V1.0    Kaiyu Lei   Create
 *
 * @Copyright (C)  2019  .cdWFVCEL. all right reserved
***********************************************************************/
#ifndef __CURVEFIT_h__
#define __CURVEFIT_h__

#ifdef __CURVEFIT_h__GLOBAL
    #define __CURVEFIT_h__EXTERN 
#else
    #define __CURVEFIT_h__EXTERN extern
#endif

// 二次函数拟合

#include <iostream>
#include <memory>
#include "../3rdParty/problem.h"

using namespace myslam::backend;

// 构造vertex
class CurveFittingVertex : public Vertex {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CurveFittingVertex() : Vertex(3) {}

    virtual std::string TypeInfo() const {
        return "abc";
    }
};

// 构造edge
class CurveFittingEdge : public Edge {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CurveFittingEdge(double x, double y) : Edge(1, 1, std::vector<std::string>{"abc"}) {
        x_ = x;
        y_ = y;
    }

    // 计算二次曲线模型的误差
    virtual void ComputeResidual() override {
        Vec3 abc = verticies_[0] -> Parameters();
        residual_(0) = (abc(0)*x_*x_ + abc(1)*x_ + abc(2)) - y_; 
    }

    // 计算残差对变量的雅克比
    virtual void ComputeJacobians() override {
        Vec3 abc = verticies_[0] -> Parameters();
        // double estim_y = abc(0)*x_*x_ + abc(1)*x_ + abc(2);
        // 残差为1维，参数为3维，所以Jacobian矩阵是1x3的
        Eigen::Matrix<double, 1, 3> jaco_abc;
        jaco_abc << x_*x_, x_, 1;
        jacobians_[0] = jaco_abc;
    }

    // 返回边的类型信息
    virtual std::string TypeInfo() const override {
        return "CurveFittingEdge";
    }                                                                         
public: 
    double x_, y_;
};

class QuadFitting {
public:
    QuadFitting(std::vector<double> *x, std::vector<double> *y) {
        x_ = x;
        y_ = y;
    }

    // 设置vector
    void set(std::vector<double> *x, std::vector<double> *y) {
        x_ = x;
        y_ = y;
    }

    // 求解二次曲线拟合问题
    bool solve(std::vector<double> *abc);
    
private:
    std::vector<double> *x_;
    std::vector<double> *y_;
    double a_, b_, c_;
};

#endif // __CURVEFIT_h__
