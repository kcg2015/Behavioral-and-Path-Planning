//
//  vehicle.h
//  
#ifndef vehicle_h
#define vehicle_h
#include <stdio.h>
#include <math.h>
#include <vector>
#include "Eigen-3.3/Eigen/Dense"
#include <random>
#include <cstdlib>
#include <ctime>

using namespace std;
using namespace Eigen;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class Vehicle
{
public:
    
    int ID;  // ID for each Vehicle
    int lane_no; // Current lane
    double x, y, vx, vy, v, yaw, s, d; // reading from sensor fusion
    
    vector<double> s_vec={0, 0, 0}; //{s, s_dot, s_double_dot}
    vector<double> d_vec={0, 0, 0}; //{d, d_dot, d_double_dot}
    
    vector<double> s_saved_vec={0, 0, 0}; //{s, s_dot, s_double_dot}
    vector<double> d_saved_vec={6, 0, 0}; //{d, d_dot, d_double_dot}
    
    Vehicle(){};
   ~Vehicle() {};
    
    void ego_update(vector<double> ego_info){
        /*Method for updating ego vehicle's status, information
        ego_info = {car_x, car_y, car_s, car_d, car_yaw, car_speed}; */
        
        this->ID = -1; //special id assigned to the ego vehicle, to differentiate from other vehicles
        this->x = ego_info[0];
        this->y = ego_info[1];
        this->s = ego_info[2];
        this->d = ego_info[3];
        this->yaw = ego_info[4];
        this->v = ego_info[5];
        this->lane_no = floor(d/4);
        this->s_vec[0] = this->s;
        this->d_vec[0] = this->d;
    }
    
    void ego_update_saved_vec(vector<double> s, vector<double> d){
        /*Method for updating s_saved_vec and d_saved_vec */
        this->s_saved_vec = s;
        this->d_saved_vec = d;
    }
    
    void update_sensor_fusion(vector< vector<double>> sensor_fusion, int idx) {
        /* Method for updating non-ego vehicle information*/
        this->ID = idx;
        this->x	= sensor_fusion[idx][1];
        this->y	= sensor_fusion[idx][2];
        this->vx = sensor_fusion[idx][3];
        this->vy = sensor_fusion[idx][4];
        this->s	=  sensor_fusion[idx][5];
        this->d	= sensor_fusion[idx][6];
        this->lane_no = floor(d/4);
        this->s_vec[0] = this->s;
        this->d_vec[0] = this->d;
        this->s_vec[1] =sqrt(pow(this->vx, 2) + pow(this->vy, 2));
        this->d_vec[1] = 0;
        this->s_vec[2] = 0;
        this->d_vec[2] = 0;
        
    }
    
    
   vector<double> state_in(double t) {
        
        // diplacement of s as a function of t
        double s     = this->s_vec[0] + this->s_vec[1] * t + this->s_vec[2] * t * t / 2;
        double v     = this->s_vec[1] + this->s_vec[2] * t;
        // diplacement of s as a function of t
        double d     = this->d_vec[0] + this->d_vec[1] * t + this->d_vec[2] * t * t / 2;
        double d_dot = this->d_vec[1] + this->d_vec[2] * t;
        return {s, v, this->s_vec[2], d, d_dot, this->d_vec[2]};
    }
    
    vector<double>  get_s() const {
        return s_vec;
    }
    
    vector<double> get_d() const {
        return d_vec;
    }
    
    
};


#endif /* vehicle_h */
