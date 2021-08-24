#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  
  //start in lane 1
  int lane=1;
  
  //using a reference velocity to target
  double ref_vel=0.0;
  

  h.onMessage([&ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          
          //Info obtained from previous path provide by simulator
          int prev_size= previous_path_x.size();
          
          if(prev_size>0)
          {
           car_s=end_path_s;
          }
          
          //bools to deal when car is too close in the same lane and cars passing in left and right lane
          
          bool too_close=false;
          bool car_left=false;
          bool car_right=false;
          
          //find ref_v to use
          
          //using sensor fusion data to get an understanding of surrounding car's velocities and positions
          for(int i =0;i< sensor_fusion.size();i++)
          {
           
            float d = sensor_fusion[i][6];
            
//          Since d lies between 0 and 12 with d=2,4 an 6 being middle of the lane. We are trying to check where the car lies
            int present_car_lane;
            
             if(d > 0 && d < 4)
             {
               present_car_lane = 0;
             }else if(d > 4 && d < 8)
             {
               present_car_lane = 1;
             }else if(d > 8 && d < 12)
             {
               present_car_lane = 2;
             }
         
            double vx= sensor_fusion[i][3];
            double vy= sensor_fusion[i][4];
            double check_speed=sqrt(vx*vx+vy*vy);
            double check_car_s =sensor_fusion[i][5];
            
            //predicting where the car will be in the future
            check_car_s+=((double)prev_size*0.02*check_speed);
              
            //if the car in the same lane then check how far it is and if too close then flag is changed to true 
            if(present_car_lane==lane)
              {
                if(check_car_s > car_s && (check_car_s - car_s) < 30)
                	{
                   		too_close=true;
                    }
                }
              	//if the car is in the left lane, check what its path in the 30m range ahead and back
             else if (present_car_lane-lane==-1)
                {
                 	if((car_s+30)> check_car_s && (car_s-30)<check_car_s){
                         car_left=true;
                    }
                }
                //if the car is in the right lane, check what its path in the 30m range ahead and back
             else if (present_car_lane-lane==1)
                {
                    if((car_s+30)> check_car_s && (car_s-30)<check_car_s){
                         car_right=true;
                    }
                }
          }

//       std::cout<<car_left<<' '<<car_right<<' '<<too_close<<' '<<lane<<' '<<std::endl;
          
          //Logic for shifitng and keeping lanes during highway driving
          if(too_close)
           {
             if(!car_left && lane>0){
               lane--;
             }
             else if (!car_right && lane<2){
             	lane++;
             }       
            else {
                ref_vel-=1.2*.224; 
            }
          }
          
          else if( ref_vel< 49.5){
           //help increment at start slowly without leading to jerk and avoid cold_start
              ref_vel +=0.224;
            
          }
            
      	// Creating x,y waypoints points that are widely spaced and then using spline library to fit them
          vector<double> ptsx;
          vector<double> ptsy;
          
          //keep track of reference x,y and yaw states. this could be either starting point or previous paths end point
          double ref_x=car_x;
          double ref_y=car_y;
          double ref_yaw=deg2rad(car_yaw);
          
          //if previous states is almost empty then use the car as starting reference
          if(prev_size<2)
          {
          
            //Use two points that make the path tangent to the angle of car
            double prev_car_x=car_x-cos(car_yaw);
            double prev_car_y=car_y-sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          
          }
          
          //use the previous path's end point as  reference
          else
            
          {
          
            //Redefine reference state as previous path end point
            ref_x=previous_path_x[prev_size-1];
            ref_y=previous_path_y[prev_size-1];
            
            
            double ref_x_prev=previous_path_x[prev_size-2];
          	double ref_y_prev=previous_path_y[prev_size-2];
            ref_yaw=atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);
            
            
            //Use two points that make the path tanget to the previous path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
            
            
          }
          
          //In Frenet , adding evenly 30m spaced points ahead of the starting reference calculated pushed earlier
          
          vector<double> next_wp0 = getXY(car_s+30, (2+4*lane) , map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60, (2+4*lane) , map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90, (2+4*lane) , map_waypoints_s,map_waypoints_x,map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          //doing transformations to local car's coordinates so that last point of the previous path is at zero 
          //looking from car's reference frame
          
          
          for(int i=0;i<ptsx.size();i++)
          {
          	double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;
            
            ptsx[i]= (shift_x *cos(0-ref_yaw)- shift_y*sin(0-ref_yaw));
            ptsy[i]= (shift_x *sin(0-ref_yaw)+ shift_y*cos(0-ref_yaw));          
            
          }
          
          //Create a spline
          tk::spline s;
          
          // set (x,y) points to the spline
          s.set_points(ptsx,ptsy);
         
          // define the actual(x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

		
          // start with all of the previous path points from last time
		  for (int i = 0; i < prev_size; i++) {
            
  		  next_x_vals.push_back(previous_path_x[i]);
          next_y_vals.push_back(previous_path_y[i]);
            
		  }
          
          //Calculate how to break up spline points so that we travel at our desired reference velocity
                      
          double target_x=30.0;
          double target_y=s(target_x);
          double target_dist= sqrt((target_x)*(target_x)+ (target_y)*(target_y));
                      
          double x_add_on=0;
          
          //Fill up the rest of our path planner after filling it with previous points,here we will always output 50 points
                      
          for(int i=1; i<=50-previous_path_x.size(); i++ ){
          	
            double N = (target_dist/(.02*ref_vel/2.24));
            double x_point=x_add_on+(target_x)/N;
            double y_point= s(x_point);
            
            x_add_on=x_point;
            
            double x_ref=x_point;
            double y_ref=y_point;
            
            //rotate back to normal to go from local to global coordinates
            x_point=(x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point=(x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));
            
            x_point+=ref_x;
            y_point+=ref_y;
            
           	next_x_vals.push_back(x_point);
          	next_y_vals.push_back(y_point);

          }            

          json msgJson;
                      
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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