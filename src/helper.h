#ifndef helper_h
#define helper_h

#include <stdio.h>
#include <math.h>
#include <vector>
#include "Eigen-3.3/Eigen/Dense"
#include <random>
#include <cstdlib>
#include <ctime>
#include "vehicle.h"

using namespace std;
using namespace Eigen;
using Eigen::MatrixXd;
using Eigen::VectorXd;
default_random_engine generator(time(0)); //put inside the funciton will cause identical outputs


double logistic (double x){
    
    /*A function that returns a value between 0 and 1 for x in the
     range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].
     Useful for cost functions.*/
    
    return 2.0 / (1 + exp(-x)) - 1.0;
    
}

vector<double> JMT(vector<double> start, vector<double> end, double T) {
    /* Helper function for calculating JMT coefficient */
    
    vector<double> sol_vec;
    MatrixXd A(3,3);
    VectorXd b(3);
    VectorXd a3_5(3);
    
    //Recover state parameters
    double a0 = start[0];
    double a1 = start[1];
    double a2 = 0.5*start[2];
    
    //Compute c0, c1, c2
    double c0 = a0 + a1 * T + a2 * pow(T, 2.0);
    double c1 = a1 + 2 * a2 * T;
    double c2 = 2*a2;
    
    //compute LHS matrix
    A << pow(T,3), pow(T,4), pow(T,5),
    3*pow(T,2), 4*pow(T,3), 5*pow(T,4),
    6*T, 12*pow(T,2), 20*pow(T,3);
    
    //compute RHS vecotr
    b << end[0]-c0,
    end[1]-c1,
    end[2]-c2;
    
    //solve for Ax=b
    a3_5 = A.inverse() * b;
    sol_vec={a0,a1, a2, a3_5(0), a3_5(1), a3_5(2)};
    return sol_vec;
}

double poly(vector<double> coeffs, double t){
    /* Cacluate the value of polynomial */
    double total = 0.0;
    for (int i=0; i<coeffs.size(); i++){
        double c = coeffs[i];
        total += c * pow(t,i);
    }
    
    return total;
    
}

vector<double> poly_diff(vector<double> coeffs){
    /*"""
     Calculates the derivative of a polynomial and returns
     the corresponding coefficients.*/
    
    vector<double> new_cos;
    for (int i=1; i<coeffs.size(); i++){
        new_cos.push_back(i*coeffs[i]);
    }
    return new_cos;
}

vector<vector<double>> poly_diff_N(vector<double> coeffs, int N){
    /*
     Calculates the derivative of a polynomial to the Nth order
     and returns the corresponding coefficients.*/
    
    vector<vector<double>> new_cos;
    vector<double> tmp_coeffs = coeffs;
    new_cos.push_back(tmp_coeffs);
    for (int i=1; i<=N; i++){
        tmp_coeffs = poly_diff(tmp_coeffs);
        new_cos.push_back(tmp_coeffs);
    }
    return new_cos;
}



void print_vec(vector<double> vec){
    /* Helper function to print out the vector information for debuggin*/
    if (vec.size()==0){
        cout<<"[ ]"<<endl;}
    else{
        cout<<"[";
        for (int i=0; i<vec.size(); i++){
            if(i<vec.size()-1){
                cout <<vec[i]<<", ";
            }
            else{
                cout << vec[i]<<"]"<<endl;
            }
        }
        
    }
    
}


#endif /* helper_h */
