//
//  path.h
//  

#ifndef path_h
#define path_h

#include <stdio.h>
#include <string>
#include <map>
#include <vector>
#include "vehicle.h"
#include "helper.h"
#include "spline.h"
#include <random>
#include <ctime>
#include <math.h>
using namespace std;

const double TIME_STEP = 0.02;    // time step in unit of seconds, provided by the simulator
const double SPEED_LIMIT = 21.78; //speed limit in meter per second
const double SPEED_LIMIT_DELTA =0.02; //speed limit adjustment, when setting the ego car's target velocity
const int NUM_GOAL_POINTS_COLD_START = 100; //number of goal points for cold start (from stationary)
const int NUM_GOAL_POINTS_STRAIGHT = 15; //number of goal points for going straight
const int NUM_GOAL_POINTS_LEFT_RIGHT = 100; // number of goal points for switching to left/right lane
const double MAX_ACCEL = 9.8; // maxmal allowed acceleration, unit m/s^2
const double MAX_JERK = 9.8;  // maximal allowed jerk, unit m/s^3
const int NUM_CANDIDATE_TRAJECTORIES = 20; // number of candidate trajectories for optimization
const double TRACK_DIST = 5104.62105369568; // length of the track

// Random number generation of purturbing an candidate trajectory
default_random_engine _generator(time(0));

// Structure associated with spline
struct WP_spline{
    tk::spline X;
    tk::spline Y;
    tk::spline dX;
    tk::spline dY;
};

// Structure for next points in x-y coordinate
struct XY_NEXT_POINTS{
    vector<double> X_POINTS;
    vector<double> Y_POINTS;
    int N;
};
// Structure for next points in s-d coordinate
struct SD_NEXT_POINTS{
    vector<double> S_POINTS;
    vector<double> D_POINTS;
    int N;
};
// Structure for goal points in s-d coordinate
struct GOAL_POINTS {
    vector<double> S_GOAL;
    vector<double> D_GOAL;
    double T;
};

// Structure used in tracking the closest vehicles ahead and behind
// the ego vehicle.
struct VEH_STATE{
    int ID;
    double S;
    double S_DOT;
};

// Class for path planning related member variables and methods

class Path {
  public:
    Path(){};
    virtual ~Path(){};
    
    struct S_D {
        vector<double> S;
        vector<double> D;
    };
    
    vector<double> previous_path_x;
    vector<double> previous_path_y;
    double end_path_s;
    double end_path_d;
    
    Vehicle ego_vehicle;
    
    // Build a map (dictionary) for all the vehicles
    // other than the ego vehicle.
    
    map<int, Vehicle*> other_vehicles;

    // Keep track no. of times JMT produces infeasible
    // solutions for debugging
    int no_times_infeasible = 0;

    
    //Initialize map<int,  Vehicle*>
    void init_vehicles(){
        for (int i=0; i<12; i++){ // We know before hand there are at most 12 vehicles at a given time in the simulator
            Vehicle *veh = new Vehicle;
            other_vehicles[i] = veh;
        }
    };
    // Update non-ego vehicle information
    void other_vehicles_update(vector<vector<double>> sensor_fusion){
        for (int i=0; i<sensor_fusion.size(); i++){
            other_vehicles[i]->update_sensor_fusion(sensor_fusion, i);
          }
     };
    
    // Find the state (s, s_dot) of the cloest vehicle ahead (in the same line)
    VEH_STATE closest_vehicle_ahead_in_lane(vector<double> const &start, int ego_lane_i, map<int, Vehicle*> other_vehicles)
    
    {
        int closest_id = -1;
        double min_s_dif = 999;
        double min_s_v =0;
        
        Vehicle *veh;
        
        //Loop through all the vehicle states provided by sensor fusion
        for (int i=0; i<12; i++){
            veh = other_vehicles[i];
            vector<double> traffic_d = veh->get_d();
            int traffic_lane_i = 0;
            if (traffic_d[0] > 8) traffic_lane_i = 2;
            else if (traffic_d[0] > 4) traffic_lane_i = 1;
            
            if (ego_lane_i == traffic_lane_i) {
                vector<double> traffic_s = veh->get_s();
                float dif_s = traffic_s[0] - start[0];
                //If a car is ahead and less than the existing min_s_dif
                if ((dif_s > 0.0) && (dif_s < min_s_dif)) {
                    closest_id = i;
                    min_s_dif = dif_s;
                    min_s_v =traffic_s[1];
                }
            }
        }
        
        VEH_STATE closest;
        closest.ID = closest_id;
        closest.S = min_s_dif;
        closest.S_DOT = min_s_v;
        
        return closest;
        
    }
    
    // Find the state (s, s_dot) of the cloest vehicle ahead in all three lanes
    vector<VEH_STATE> closest_vehicle_ahead_in_lanes(vector<double> const &start,  map<int, Vehicle*> other_vehicles){
        
        vector<VEH_STATE> closest_veh_i(3);
        for (int i = 0; i < 3; i++) //Loop through three lanes 0, 1, and 2.
            closest_veh_i[i] = closest_vehicle_ahead_in_lane(start, i, other_vehicles);
        return closest_veh_i;
    }
    
    // Find the state (s, s_dot) of the cloest vehicle behind (in the same lane)
    
    VEH_STATE closest_vehicle_behind_in_lane(vector<double> const &start, int ego_lane_i, map<int, Vehicle*> other_vehicle)
    
    {
        int closest_id = -1;
        float min_s_dif = -999;
        float min_s_v =0;
        
        Vehicle *veh;
        
        for (int i=0; i<12; i++){
            
            veh = other_vehicles[i];
            vector<double> traffic_d = veh->get_d();
            int traffic_lane_i = 0;
            if (traffic_d[0] > 8) traffic_lane_i = 2;
            else if (traffic_d[0] > 4) traffic_lane_i = 1;
            
            if (ego_lane_i == traffic_lane_i) {
                vector<double> traffic_s = veh->get_s();
                float dif_s = traffic_s[0] - start[0];
                //If a car is behind and less than the existing abs(min_s_dif)
                if ((dif_s < 0.0) && (abs(dif_s) < abs(min_s_dif))) {
                    closest_id = i;
                    min_s_dif = dif_s;
                    min_s_v =traffic_s[1];
                }
            }
        }
        
        VEH_STATE closest;
        closest.ID = closest_id;
        closest.S = min_s_dif;
        closest.S_DOT = min_s_v;
        
        return closest;
    }
    
    // Find the state (s, s_dot) of the cloest vehicle behind in all three lanes
    vector<VEH_STATE> closest_vehicle_behind_in_lanes(vector<double> const &start,  map<int, Vehicle*> other_vehicles){
        
        vector<VEH_STATE> closest_veh_i(3);
        for (int i = 0; i < 3; i++)
            closest_veh_i[i] = closest_vehicle_behind_in_lane(start, i, other_vehicles);
        return closest_veh_i;
    }

    vector<double> getXY_spline(double s, double d, tk::spline const &map_x_spline,tk::spline const &map_y_spline, tk::spline const &map_dx_spline,tk::spline const &map_dy_spline){
        
        double mod_s = fmod(s, TRACK_DIST);
        double x_tmp = map_x_spline(mod_s);
        double y_tmp = map_y_spline(mod_s);
        double dx = map_dx_spline(mod_s);
        double dy = map_dy_spline(mod_s);
        double x = x_tmp + dx * d;
        double y = y_tmp + dy * d;
        
        return {x, y};
    }

    // Generate trajectory in X_Y coordinate
    XY_NEXT_POINTS generate_single_XY_trajectory(vector<double> start_s,
                                                 vector<double> start_d,
                                                 vector<double> goal_s,
                                                 vector<double> goal_d,
                                                 double t,
                                                 WP_spline &wp_spl){
        
        
        vector<double> s_coeff = JMT(start_s, goal_s, t); //JMT is defined in helper.h
        vector<double> d_coeff = JMT(start_d, goal_d, t);
        vector<double> p;
        vector<double> next_x_vals;
        vector<double> next_y_vals;
        tk::spline map_x_spline = wp_spl.X;
        tk::spline map_y_spline = wp_spl.Y;
        tk::spline map_dx_spline = wp_spl.dX;
        tk::spline map_dy_spline = wp_spl.dY;
        
        XY_NEXT_POINTS next_vals;
        int n = floor(t/TIME_STEP);
        for (int i = 0; i < n; i++) {
            
            double tmp_s = poly(s_coeff, i*TIME_STEP);
            double tmp_d = poly(d_coeff, i*TIME_STEP);
            
            p = getXY_spline(tmp_s, tmp_d, map_x_spline, map_y_spline,  map_dx_spline, map_dy_spline);
            
            next_x_vals.push_back(p[0]);
            next_y_vals.push_back(p[1]);
        }
        
        next_vals.X_POINTS = next_x_vals;
        next_vals.Y_POINTS = next_y_vals;
        
        return next_vals;
    }
    
    // Generate trajectory in s_d coordinate
   
    SD_NEXT_POINTS generate_single_SD_trajectory(vector<double> start_s,
                                                 vector<double> start_d,
                                                 vector<double> goal_s,
                                                 vector<double> goal_d,
                                                 double t){
    
        vector<double> s_coeff = JMT(start_s, goal_s, t); //JMT is defined in helper.h
        vector<double> d_coeff = JMT(start_d, goal_d, t);
        
        
        vector<vector<double>> tmp_traj= {s_coeff, d_coeff};
        vector<double> p;
        vector<double> next_x_vals;
        vector<double> next_y_vals;
        
        
        SD_NEXT_POINTS next_sd_vals;
        int n = floor(t/TIME_STEP);
        cout<<"No. of points: "<<n<<endl;
        next_sd_vals.N = n;
        for (int i = 0; i < n; i++) {
            double tmp_s = poly(s_coeff, i*TIME_STEP);
            double tmp_d = poly(d_coeff, i*TIME_STEP);
            next_sd_vals.S_POINTS.push_back(tmp_s);
            next_sd_vals.D_POINTS.push_back(tmp_d);
            }

        return next_sd_vals;
        
    }
    
    // Convert points from s-d coordinates to x_y coordinates
    XY_NEXT_POINTS convert_SD_POINTS_to_XY(SD_NEXT_POINTS sd_pts, WP_spline &wp_spl){
    
        //unpack wp_spl
        tk::spline map_x_spline = wp_spl.X;
        tk::spline map_y_spline = wp_spl.Y;
        tk::spline map_dx_spline = wp_spl.dX;
        tk::spline map_dy_spline = wp_spl.dY;
        
        XY_NEXT_POINTS next_vals;
        int n = sd_pts.N;
        vector<double> next_x_vals, next_y_vals;
        for (int i = 0; i < n; i++) {
            vector<double> p = getXY_spline(sd_pts.S_POINTS[i], sd_pts.D_POINTS[i], map_x_spline, map_y_spline, map_dx_spline, map_dy_spline);
            next_x_vals.push_back(p[0]);
            next_y_vals.push_back(p[1]);
        }
        
        next_vals.X_POINTS = next_x_vals;
        next_vals.Y_POINTS = next_y_vals;
        next_vals.N  = n;
        return next_vals;
    }
    
    
    // Generate trajectory at the start of the simulation, when is car is stationary
    
    XY_NEXT_POINTS generate_cold_start_trajectory(Vehicle &ego_veh,
                                                  WP_spline &wp_spl){
        
        double car_s = ego_veh.s;
        double car_d = ego_veh.d;
        double car_speed = ego_veh.v;
        int ego_lane = ego_veh.lane_no;
        int n = NUM_GOAL_POINTS_COLD_START;
        double t = n * TIME_STEP;
        
        double tgt_v = 0.25*MAX_ACCEL*t; // Very conservative target velocity to avoid violating acceleration constraint
        double tgt_s = car_s + 0.5 * t * (car_speed +tgt_v);
        double tgt_d = 2 + 4*ego_lane; // ego_lane: 0, tgt_d=2; ego_lane:1 tgt_d=6; ego_lane:2, tgt_d=10
        
        vector<double> start_s = {car_s, car_speed, 0};
        vector<double> start_d = {car_d, 0,  0};
        
        vector<double> goal_s = {tgt_s, tgt_v, 0};
        vector<double> goal_d = {tgt_d, 0, 0};
        
        ego_veh.ego_update_saved_vec(goal_s, goal_d);
        
        SD_NEXT_POINTS sd_pts;
        
        sd_pts = generate_single_SD_trajectory(start_s, start_d, goal_s, goal_d, t);
        
        XY_NEXT_POINTS next_vals;
        
        next_vals =convert_SD_POINTS_to_XY(sd_pts, wp_spl);
        
        return next_vals;
    }

    
    // Generate goal points when going straight

    GOAL_POINTS  generate_goal_points_straight(Vehicle &ego_veh, bool too_close, double lead_car_v){
        
        int n = NUM_GOAL_POINTS_STRAIGHT;
        double t = n * TIME_STEP; // Calculate the time duration
        double dv = n * TIME_STEP * MAX_ACCEL * 0.88;
        double dv_thd = n * TIME_STEP * MAX_ACCEL * 0.88;
        double s0 = ego_veh.s_saved_vec[0];
        
        double v0 = ego_veh.s_saved_vec[1];
        double tgt_v;
        
        if (too_close){ // if the front car is too close
            double dv2 = v0 - lead_car_v; // Calculate the velocity difference between the front car and the ego car
            if(dv2 >dv_thd){dv2=dv_thd;} // If the front car is slower, we want to match the speed of the front car
                                         // To avoid collision, however if the velocity difference is too large that the
                                         //accelerationis violated, we need to cap the delta
            tgt_v = v0 - dv2;
                   }
        else{
            tgt_v = v0 + dv;
                    }
        // Be ware of the speed limit.
        if (tgt_v>SPEED_LIMIT){tgt_v=SPEED_LIMIT-SPEED_LIMIT_DELTA;}
        
        
        double tgt_s = s0 + 0.5 * t * (v0 + tgt_v);
        int ego_lane = ego_veh.lane_no;
        double tgt_d = 2 + 4*ego_lane;
        
        vector<double> goal_s = {tgt_s, tgt_v, 0};
        vector<double> goal_d = {tgt_d, 0, 0};
        ego_veh.ego_update_saved_vec(goal_s, goal_d);
        
        GOAL_POINTS goal_pts;
        goal_pts.S_GOAL = goal_s;
        goal_pts.D_GOAL = goal_d;
        goal_pts.T = t;
        
        return goal_pts;
    
    
    }
    
    // Generate goal points when changing to right lane
    
    GOAL_POINTS  generate_goal_points_right(Vehicle &ego_veh, bool too_close, double lead_car_v){
        
        int n = NUM_GOAL_POINTS_LEFT_RIGHT;
        double t = n * TIME_STEP;
        double dv =-1.8;
        double s0 = ego_veh.s_saved_vec[0];
        double v0 = ego_veh.s_saved_vec[1];
        double tgt_v;
        
        tgt_v = v0 + dv;
        if (tgt_v>SPEED_LIMIT){tgt_v=SPEED_LIMIT-SPEED_LIMIT_DELTA;} //
        
        double tgt_s = s0 + 0.5 * t * (v0 + tgt_v);
        int ego_lane = ego_veh.lane_no;
        double tgt_d = (2 + 4*ego_lane) + 4;
        
        vector<double> goal_s = {tgt_s, tgt_v, 0};
        vector<double> goal_d = {tgt_d, 0, 0};
        ego_veh.ego_update_saved_vec(goal_s, goal_d);
        
        GOAL_POINTS goal_pts;
        goal_pts.S_GOAL = goal_s;
        goal_pts.D_GOAL = goal_d;
        goal_pts.T = t;
        
        return goal_pts;
        
        
    }
    // Generate goal points when changing to left lane
    
    GOAL_POINTS  generate_goal_points_left(Vehicle &ego_veh, bool too_close, double lead_car_v){
        
        int n = NUM_GOAL_POINTS_LEFT_RIGHT;
        double t = n * TIME_STEP;
        double dv =-1.8;
        double s0 = ego_veh.s_saved_vec[0];
        
        double v0 = ego_veh.s_saved_vec[1];
        double tgt_v;
        
        tgt_v = v0 + dv;
        
        if (tgt_v>SPEED_LIMIT){tgt_v=SPEED_LIMIT-SPEED_LIMIT_DELTA;}
        
        double tgt_s = s0 + 0.5 * t * (v0 + tgt_v);
        int ego_lane = ego_veh.lane_no;
        double tgt_d = (2 + 4*ego_lane) - 4;
        
        vector<double> goal_s = {tgt_s, tgt_v, 0};
        vector<double> goal_d = {tgt_d, 0, 0};
        ego_veh.ego_update_saved_vec(goal_s, goal_d);
        
        GOAL_POINTS goal_pts;
        goal_pts.S_GOAL = goal_s;
        goal_pts.D_GOAL = goal_d;
        goal_pts.T = t;
        
        return goal_pts;
        
    }
   //Cost for viloating speed limit
    
   double speed_limit_cost(vector<vector<double>> traj, int N) {
        
        vector<double> s = traj[0];
        vector<double> s_dot = poly_diff(s);
        vector<double> d = traj[1];
        vector<double> d_dot = poly_diff(d);
        
        for (int i=0; i<N; i++){
            double tmp_speed = abs(poly(s_dot, i*TIME_STEP));
            
            if (tmp_speed>SPEED_LIMIT+0.1)
                return 1.0; //violation
        }
        
        return 0; // No violation
    };

    
    // Cost for viloating acceleration limit
    double max_accel_cost(vector<vector<double>> traj, int N){
        
        vector<double> s = traj[0];
        vector<double> s_dot = poly_diff(s);
        vector<double> s_d_dot = poly_diff(s_dot);
        vector<double> d = traj[1];
        vector<double> d_dot = poly_diff(d);
        vector<double> d_d_dot = poly_diff(d_dot);
        
        for (int i=0; i<N; i++){
            double acc_tmp = abs(poly(s_d_dot, i*TIME_STEP))+abs(poly(d_d_dot,i*TIME_STEP));
            
            if (acc_tmp > MAX_ACCEL)
            {
                return 1.0; //violation
            }
        }
        
        return 0; // No violation
        
    };
    
    // Cost for viloating jerk limit
    double max_jerk_cost(vector<vector<double>> traj, int N){
        
        vector<double> s = traj[0];
        vector<double> s_dot = poly_diff(s);
        vector<double> s_d_dot = poly_diff(s_dot);
        vector<double> s_jerk = poly_diff(s_d_dot);
        vector<double> d = traj[1];
        vector<double> d_dot = poly_diff(d);
        vector<double> d_d_dot = poly_diff(d_dot);
        vector<double> d_jerk = poly_diff(d_d_dot);
        
        for (int i=0; i<N; i++){
            double tmp_jerk = (abs(poly(s_jerk, i*TIME_STEP))+abs(poly(d_jerk, i*TIME_STEP)));
            if (tmp_jerk> MAX_JERK)
                return 1.0;
        }
        
        return 0;
   };

   // Total acceleration cost
   double total_accel_cost(vector<vector<double>> traj, int N){
        
        double cost = 0.0;
        vector<double> s = traj[0];
        vector<double> s_dot = poly_diff(s);
        vector<double> s_d_dot = poly_diff(s_dot);
        vector<double> d = traj[1];
        vector<double> d_dot = poly_diff(d);
        vector<double> d_d_dot = poly_diff(d_dot);
        
        for (int i=0; i<N; i++){
            cost += abs(poly(s_d_dot, i*TIME_STEP)/10);
            cost += abs(poly(d_d_dot, i*TIME_STEP)/10);
        }
        
        return logistic(cost/N);
        
    }
    
    // Total jerk cost
    double total_jerk_cost(vector<vector<double>> traj, int N){
        
        double cost = 0.0;
        vector<double> s = traj[0];
        vector<double> s_dot = poly_diff(s);
        vector<double> s_d_dot = poly_diff(s_dot);
        vector<double> s_jerk = poly_diff(s_d_dot);
        vector<double> d = traj[1];
        vector<double> d_dot = poly_diff(d);
        vector<double> d_d_dot = poly_diff(d_dot);
        vector<double> d_jerk = poly_diff(d_d_dot);
        
        for (int i=0; i<N; i++){
            cost += abs(poly(s_jerk, i*TIME_STEP)/10);
            cost += abs(poly(d_jerk, i*TIME_STEP)/10);
        }
        
        return logistic(cost/N);
        
    };
    // Total cost
    
    double return_total_cost(vector<vector<double>> traj, int N){
        
        double total_cost = 0;
        double speed_cost = speed_limit_cost(traj, N);
        double accel_cost = max_accel_cost(traj, N);
        double jerk_cost = max_jerk_cost(traj, N);
        double feasibility_cost = speed_cost + accel_cost;
        double tot_accel_cost = total_accel_cost(traj, N);
        double tot_jerk_cost = total_jerk_cost(traj, N);
        
        
        // Calculate the weighted cost
        total_cost=+10 * tot_accel_cost+10 * tot_jerk_cost;
        vector<double> cost_vec = {speed_cost, accel_cost, jerk_cost,
            tot_accel_cost, tot_jerk_cost};
        cout<<"|speed|accel|jerk|tot_acc|tot_jerk|"<<endl;
        print_vec(cost_vec);
        if (feasibility_cost>0){ // if any of the speed, accelleration, jerk, and collision constraints is violate, the trajectory is deemed infeasible.
            return 1e+06;}
        else{
            return total_cost;
        }
    };
    
   
    vector<vector<double>> generate_trajectory(vector<double> const start,
                                               vector<vector<double>>all_goals, double t){
        
        vector<vector<double>> trajectory;
        vector<double> cost_trajectories;
        vector<vector<vector<double>>> trajectories;
        double min_cost =1e+06;
        int min_idx =-1;
        vector<vector<double>> min_traj={{0,0,0,0,0,0},{0,0,0,0,0,0}};
        const vector<double> start_s = {start[0], start[1], start[2]};
        const vector<double> start_d = {start[3], start[4], start[5]};
        
        int tmp_N = floor(t/TIME_STEP);
        for (int i=0; i<all_goals.size(); i++){
            vector<double> goal = all_goals[i];
            vector<double> s_goal(goal.begin(), goal.begin()+3);
            vector<double> d_goal(goal.begin()+3, goal.end());
            
            vector<double> s_coeff = JMT(start_s, s_goal, t);
            vector<double> d_coeff = JMT(start_d, d_goal, t);
            
            trajectory={s_coeff, d_coeff};
            trajectories.push_back(trajectory);
           
            double tmp_cost =return_total_cost(trajectory, tmp_N);
            cost_trajectories.push_back(tmp_cost);
            
            
            if (tmp_cost < min_cost){
                min_cost=tmp_cost;
                min_traj=trajectory;
                min_idx = i;
            }
            trajectory.clear();
            
        }
        
        // Deal with the corner cases that all the JMTs are infeasible
        if (min_idx == -1) {
            
            //Output all the information for debugging:
            no_times_infeasible +=1;
            cout<<"JMT error !"<<endl;
            cout<<endl;
            cout<<"start"<<endl;
            print_vec(start);
            cout<<endl;
            cout<<"All goals"<<endl;
            for (int i=0; i<all_goals.size(); i++){
                print_vec(all_goals[i]);
            }

            int tmp_idx = trajectories.size()-1;
            min_traj = trajectories[tmp_idx];
                   }
        return min_traj;
        
    };


    
    // Pertub goal points, all the parameters are obtained through trial-and-error
    vector<double> perturb_goal(vector<double> goal_s, vector<double>goal_d) {
        
        vector<double> new_goal={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double pct_std = 0.005; //0.005
        normal_distribution<double> _distrib_pct(0.0, pct_std);
       
        double multiplier = _distrib_pct(_generator);

        new_goal[0] = goal_s[0] - SPEED_LIMIT*TIME_STEP*(abs(multiplier));
        new_goal[1] = goal_s[1] - 8*TIME_STEP* (abs(multiplier));
        new_goal[2] = 0;
        
        multiplier = _distrib_pct(_generator) * 0.25;
        new_goal[3] = goal_d[0] + (1* multiplier);
        new_goal[4] = 0;
        new_goal[5] = 0;
        
        return new_goal;
    }
    
    // Pertub goal points to generate candidate trajectories
    vector<vector<double>> perturb_goal_N(vector<double> goal_s, vector<double> goal_d, int N){
        
        vector<vector<double>> new_goal_vec;
        
        vector<double> tmp_goal;
        for (int i=0; i<N; i++){
            tmp_goal = perturb_goal(goal_s, goal_d);
            new_goal_vec.push_back(tmp_goal);
        }
        
        return new_goal_vec;
    }
    
    // Generate the optimal trajectory in s-d coordinate
    SD_NEXT_POINTS generate_optimal_SD_trajectory(vector<double> start_s,
                                                 vector<double> start_d,
                                                 vector<double> goal_s,
                                                 vector<double> goal_d,
                                                 double t){
        
        int no_samples = NUM_CANDIDATE_TRAJECTORIES;
        cout<<"Goals.."<<endl;
        print_vec(goal_s);
        print_vec(goal_d);
        vector<double> orig_goal = {goal_s[0],goal_s[1],goal_s[2],goal_d[0],goal_d[1],goal_d[2]};
        vector<vector<double>> all_goals =perturb_goal_N(goal_s, goal_d, no_samples);
        all_goals.push_back(orig_goal);
                vector<double> start = {start_s[0], start_s[1],start_s[2], start_d[0],start_d[1],start_d[2]};
        vector<vector<double>> tmp_traj= generate_trajectory(start, all_goals, t);
        
        vector<double> s_coeff = tmp_traj[0];
        vector<double> d_coeff = tmp_traj[1];
        vector<double> p;
        vector<double> next_x_vals;
        vector<double> next_y_vals;
        
        
        SD_NEXT_POINTS next_sd_vals;
        int n = floor(t/TIME_STEP);
        cout<<"No. of points: "<<n<<endl;
        next_sd_vals.N = n;
        for (int i = 0; i < n; i++) {
            double tmp_s = poly(s_coeff, i*TIME_STEP);
            double tmp_d = poly(d_coeff, i*TIME_STEP);
            next_sd_vals.S_POINTS.push_back(tmp_s);
            next_sd_vals.D_POINTS.push_back(tmp_d);
        }
        
        return next_sd_vals;
        
    }
    
};

#endif /* path_hpp */
