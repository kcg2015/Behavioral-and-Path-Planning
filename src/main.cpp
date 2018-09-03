#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <algorithm>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "path.h"
using namespace std;

//Parameters used for behavioral planning
const double LANE_CHANGE_FRONT_GAP = 10;
const double LANE_CHANGE_REAR_GAP = 12;
const double FRONT_GAP_RATIO = 0.2;
const double LEAD_CAR_DIST1 = 120;
const double LEAD_CAR_DIST2 = 70;
const double LEAD_CAR_DIST_TOO_CLOSE =25;

int counter=0;
bool cold_start = true; //The ego car is stationary by default
bool debug_output = true; // To print out vehicle information for debugging

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map_bosch1.csv";
  
  
  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
      
    if (map_waypoints_s.size() && map_waypoints_s.back() > s) {
          break;
      }
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }
  
  tk::spline map_x_spline;
  tk::spline map_y_spline;
  tk::spline map_dx_spline;
  tk::spline map_dy_spline;
    
  map_x_spline.set_points(map_waypoints_s, map_waypoints_x);
  map_y_spline.set_points(map_waypoints_s, map_waypoints_y);
  map_dx_spline.set_points(map_waypoints_s, map_waypoints_dx);
  map_dy_spline.set_points(map_waypoints_s, map_waypoints_dy);
    
    
    
  //Start with Lane 1
  
  int lane = 1;
    
  //Have a reference velocity to target
    
  double ref_vel = 0; //mph
    
  
  //Initialize path planning object;
  Path path;
  path.init_vehicles();
  //Initialize ego vehicle object;
  Vehicle ego_veh;
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel,
               &path, &ego_veh, &map_x_spline, &map_y_spline, &map_dx_spline, &map_dy_spline](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    json msgJson;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];
            
          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

            //Update ego vehicle
            vector<double> car_info = {car_x, car_y, car_s, car_d, car_yaw, car_speed};
            ego_veh.ego_update(car_info);
            
            if (debug_output){
                vector<double> car_display_info = {car_x, car_y, car_s, car_d, car_yaw, car_speed, car_speed*0.44704};
                cout<<"|car_x|car_y|car_s|car_d|car_yaw|car_mph|car_mps|"<<endl;
                print_vec(car_display_info);
            }

            bool too_close = false;
            double lead_car_v = 100;
            vector<double> s_vec_tmp = ego_veh.s_saved_vec;
            vector<double> d_vec_tmp = ego_veh.d_saved_vec;

            
            vector<double> start1 ={car_s, car_speed*0.44704, 0,
                car_d, 0 , 0};
            
          	// Sensor fusion data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];
            int ego_lane = ego_veh.lane_no;

            
            map<int, Vehicle*> vehicles = path.other_vehicles;
            
            //Gather the information of the closest vehices ahead and behind
            vector<double> closest_veh_dist(3);
            vector<double> closest_veh_vel(3);
            vector<int> closest_veh_i(3);
            vector<double> closest_veh_behind_dist(3);
            vector<double> closest_veh_behind_vel(3);
            vector<int> closest_veh_behind_i(3);
            
            // Gather the information of the closest vehicles ahead in each lane
            vector<VEH_STATE> closest = path.closest_vehicle_ahead_in_lanes(start1, path.other_vehicles);
            for (int i=0; i<closest.size(); i++){
                VEH_STATE v_state = closest[i];
                closest_veh_i[i] = v_state.ID;
                closest_veh_dist[i] = v_state.S;
                closest_veh_vel[i] = v_state.S_DOT;
            }
            
            // Gather the information of the closest vehicles behind in each lane
            vector<VEH_STATE> closest_behind = path.closest_vehicle_behind_in_lanes(start1, path.other_vehicles);
            for (int i=0; i<closest_behind.size(); i++){
                            VEH_STATE v_state = closest_behind[i];
                            closest_veh_behind_i[i] = v_state.ID;
                            closest_veh_behind_dist[i] = v_state.S;
                            closest_veh_behind_vel[i] = v_state.S_DOT;
            }
           
            if (closest_veh_dist[ego_lane]<LEAD_CAR_DIST_TOO_CLOSE){
                too_close = true;
                lead_car_v = closest_veh_vel[ego_lane];
                
            }
            
            // For debugging, print out all the relavant information
            if (debug_output){
                cout<<"Ego car state: "<<endl;
                print_vec({start1[0], start1[1], start1[2]});
                cout<<"Ego car in lane "<<ego_lane <<", the closest vehicles ahead: "<<endl;
                print_vec(closest_veh_dist);
                cout<<"The closest vehicles behind: "<<endl;
                print_vec(closest_veh_behind_dist);
                cout<<endl;
                if (too_close){cout<<"Too close!"<<", lead car speed: "<<lead_car_v<<endl;}
            }
            
            //=========================Start of behavioral planning=======================================
            
            bool go_straight = true; // The default behavior is to go straight (stay at the current lane)
            bool change_left = false;
            bool change_right = false;
            
            //IF the closest car in front of the ego vehicle is far away (>LEAD_CAR_DIST1)
            if (closest_veh_dist[ego_lane] >= LEAD_CAR_DIST1){
                go_straight = true;
            }
            
            //IF the closest car in front of the ego vehicle is the mid range:
            if ((closest_veh_dist[ego_lane] >= LEAD_CAR_DIST2) &&
                (closest_veh_dist[ego_lane] < LEAD_CAR_DIST1))
            {
                // We look at the possible actions for each lane:
                if (ego_lane==0){//left most lane
                    // Can not change to left lane, when in Lane 0.
                    change_left = false;
                    //IF there is enough room ahead and behind at Lane 1, change to Lane 1
                    if ((closest_veh_dist[1]
                         -closest_veh_dist[ego_lane] >= LANE_CHANGE_FRONT_GAP)&&
                        (abs(closest_veh_behind_dist[1]) > LANE_CHANGE_REAR_GAP))
                    {
                        change_right = true;
                        go_straight= false;
                    }
                }//ego_lane ==0
                
                if (ego_lane==1){ //middle lane
                    
                    if (closest_veh_dist[2]> closest_veh_dist[0]){ //Lane 2 has more room
                        if ((closest_veh_dist[2]
                             -closest_veh_dist[ego_lane] >= LANE_CHANGE_FRONT_GAP)&&
                            (abs(closest_veh_behind_dist[2]) > LANE_CHANGE_REAR_GAP))
                        {
                            change_right = true;
                            change_left = false;
                            go_straight= false;
                        }
                    }
                    
                   else { // Lane 0 has more room
                        if ((closest_veh_dist[0]
                             -closest_veh_dist[ego_lane]>=LANE_CHANGE_FRONT_GAP)&&
                            (abs(closest_veh_behind_dist[0])>LANE_CHANGE_REAR_GAP))
                        {
                            change_left = true;
                            change_right = false;
                            go_straight= false;
                        }
                    }
                    
                }//ego_lane ==1
                
                if (ego_lane==2){// right lane
                    change_right = false; // Can not make a right lane change
                    
                    if ((closest_veh_dist[1]
                         -closest_veh_dist[ego_lane]>=LANE_CHANGE_FRONT_GAP)&&
                        (abs(closest_veh_behind_dist[1])>LANE_CHANGE_REAR_GAP))
                    {
                        change_left = true;
                        go_straight= false;
                    }
                    
                }
                
             }
            
            //IF the closest car in front of the ego vehicle is too close
            if (closest_veh_dist[ego_lane] <LEAD_CAR_DIST2){
                go_straight = true;;
                
                if (ego_lane==0){
                    change_left = false;
                    if ((closest_veh_dist[1]
                         -closest_veh_dist[ego_lane]>=LANE_CHANGE_FRONT_GAP*0.2)&&
                        (abs(closest_veh_behind_dist[1])>LANE_CHANGE_REAR_GAP))
                    {
                        change_right = true;
                        go_straight = false;
                        
                    }
                }
                
                if (ego_lane==1){
                    
                    if (closest_veh_dist[2]> closest_veh_dist[0]){//If Lane 2 has more room
                        if ((closest_veh_dist[2]
                             -closest_veh_dist[ego_lane]>=LANE_CHANGE_FRONT_GAP*0.2)&&
                            (abs(closest_veh_behind_dist[2])>LANE_CHANGE_REAR_GAP))
                        {change_right = true;
                         change_left = false;
                         go_straight = false;
                        }
                        
                    }
                    
                    else {// IF Lane 0 has more room
                        if ((closest_veh_dist[0]
                             -closest_veh_dist[ego_lane]>=LANE_CHANGE_FRONT_GAP *FRONT_GAP_RATIO)&&
                            (abs(closest_veh_behind_dist[0])>LANE_CHANGE_REAR_GAP))
                        {change_left = true;
                         change_right = false;
                         go_straight = false;
                        }
                        
                    }
                }
                
                if (ego_lane==2){
                    change_right = false;
                    if ((closest_veh_dist[1]
                         -closest_veh_dist[ego_lane]>=LANE_CHANGE_FRONT_GAP * FRONT_GAP_RATIO)&&
                        (abs(closest_veh_behind_dist[1])>LANE_CHANGE_REAR_GAP))
                    {
                        change_left = true;
                        go_straight = false;
                    }
                    
                }
                
            }
            if (debug_output){
                cout << "Action(s) taken: ";
                if (go_straight)
                    cout << " Go straight ";
                if (change_left)
                    cout << " Change to left ";
                if (change_right)
                    cout << " Change to right ";
                    cout << endl;
            }

            //===========================End of Behaviroal Planning =======================================
            
            
            //===========================Begin of the Path Planning =======================================
            int prev_size = previous_path_x.size();
            vector<double> next_x_vals;
            vector<double> next_y_vals;
            for (int i=0; i<previous_path_x.size(); i++){
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }
            
            path.other_vehicles_update(sensor_fusion);
            
            vector<double> start_s= {car_s, car_speed, 0};
            vector<double> start_d= {car_d, 0, 0};
            
            int n;
            double t;
            double dv;
            double tgt_v;
            double tgt_s;
            double tgt_d;
            vector<double> goal_s;
            vector<double> goal_d;
            vector<double> s_coeff;
            vector<double> d_coeff;
            bool add_new;
            
            WP_spline wp_spl;
            wp_spl.X = map_x_spline;
            wp_spl.Y = map_y_spline;
            wp_spl.dX = map_dx_spline;
            wp_spl.dY = map_dy_spline;
            
            XY_NEXT_POINTS next_vals;
            GOAL_POINTS goal_pts;
            if (cold_start){// start from intital imobility
                next_vals= path.generate_cold_start_trajectory(ego_veh, wp_spl);
                int no_points = next_vals.X_POINTS.size();
                for (int i = 0; i < no_points; i++) {
                    next_x_vals.push_back(next_vals.X_POINTS[i]);
                    next_y_vals.push_back(next_vals.Y_POINTS[i]);
                 }
                cold_start = false;
                }
            else{ // if not from cold start
            
                if (prev_size<10){ // need to generate new points
                    start_s = ego_veh.s_saved_vec;
                    start_d = ego_veh.d_saved_vec;
                    //GOAL_POINTS goal_pts;
                    if (go_straight){
                        goal_pts = path.generate_goal_points_straight(ego_veh, too_close, lead_car_v);
                    }
                    if (change_left){
                        goal_pts = path.generate_goal_points_left(ego_veh, too_close, lead_car_v);
                    }
                    if (change_right){
                        goal_pts = path.generate_goal_points_right(ego_veh, too_close, lead_car_v);
                    }
            
                    goal_s = goal_pts.S_GOAL;
                    goal_d = goal_pts.D_GOAL;
                
                    double t = goal_pts.T;
                    SD_NEXT_POINTS sd_pts;
                    if (go_straight){
                        sd_pts = path.generate_single_SD_trajectory(start_s, start_d, goal_s, goal_d, t);}
                    else{
                        sd_pts = path.generate_optimal_SD_trajectory(start_s, start_d, goal_s, goal_d, t);}
                        
                    XY_NEXT_POINTS next_vals;
                    next_vals =path.convert_SD_POINTS_to_XY(sd_pts, wp_spl);
            
                    for (int i=0; i<next_vals.N; i++){
                        next_x_vals.push_back(next_vals.X_POINTS[i]);
                        next_y_vals.push_back(next_vals.Y_POINTS[i]);
             }
           }
            
        }
        //=====================================End of Path Planning =================================================
            
            if (debug_output){
                cout<<"End next_x_vals size: "<<next_x_vals.size()<<endl;
                cout<<"==========================================="<<endl;
            }
           
        // Send the path information back to simulator
            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;
            auto msg = "42[\"control\","+ msgJson.dump()+"]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
















































































