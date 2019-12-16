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

#include "../3rdParty/include/ResidualFunction.h"
#include "../3rdParty/include/GradientDesent.h"
#include "../3rdParty/include/Quad_CostFunction.h"

#include <iostream>
#include <memory>

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
    bool solve(std::vector<double> &abc);
    
private:
    std::vector<double> *x_;
    std::vector<double> *y_;
    double a_, b_, c_;
};

#endif // __CURVEFIT_h__
