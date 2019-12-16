//
// Created by LHospital
//
#ifndef _COSTFUNCTION_H
#define _COSTFUNCTION_H 

// #include <iostream>
#include <eigen3/Eigen/Dense>

// CostFunction是个虚基类，在程序中，根据不同问题自行构建不同的cost function
class CostFunction {
public:
    CostFunction(Eigen::VectorXd* parameters, Eigen::VectorXd* residuals, int num_of_parameters, int num_of_residuals, Eigen::VectorXd* constants, int num_of_constants) {
        if(parameters -> rows() == num_of_parameters){
            parameters_ = *parameters;
            num_of_parameters_ = num_of_parameters;
        }
            
        if(residuals -> rows() == num_of_residuals){
            residuals_ = *residuals;
            num_of_residuals_ = num_of_residuals;
        }
    
        if(constants -> rows() == num_of_constants) {
            constants_ = *constants;
            num_of_constants_ = num_of_constants;
        }
 
        // LOG(INFO) << "constants: " << *constants_;
        
        Jacobian_ = Eigen::MatrixXd(num_of_residuals_, num_of_parameters_);
    };

    virtual void computeCost() = 0;
    virtual void computeJacobian() = 0;
    virtual Eigen::VectorXd returnCost() {
        return residuals_;
    }
    virtual Eigen::MatrixXd returnJacobian() {
        return Jacobian_;
    }
public:
// TODO: 改回protected
    Eigen::VectorXd parameters_;
    Eigen::VectorXd residuals_;
    Eigen::MatrixXd Jacobian_;
    Eigen::VectorXd constants_;
    int num_of_parameters_;
    int num_of_residuals_;
    int num_of_constants_;
};


#endif	// _COSTFUNCTION_H