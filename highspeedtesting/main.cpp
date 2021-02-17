#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include <numeric> 

// for convenience
using nlohmann::json;
using std::string;
double prev_cte=0.0;
double sum_cte=0.0;
std::vector<double> integral_sum;
std::vector<double> speed_reduction_sum;

double p[3]={0.05,1.8,0.000008},dp[3]={0.005,0.18,0.0000008};
double best_err=200;
int param_to_tune = 0; 
int idx=0;//0=kp,1=ki,2=kd
int twiddle=1;
double throttle = 0.3;
int step=0;
double t_error=0.0;
double prev_t_error=0.0;
double sum_t_error=0.0;
double prev_steer = 0.0;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

PID speed_pid;
PID speed_reduction;


int main() {
  uWS::Hub h;

  
  PID pid;
  integral_sum.push_back(0.0);
  speed_reduction_sum.push_back(0.0);
  
  
  /**
   * TODO: Initialize the pid variable.
   */
   



	
	//pid.Init(0.15,0.9,0.00000005); //Ok for speed 40mph
   pid.Init(0.15,1.1,0.00000005); //testing
   speed_pid.Init(0.5,0.003,0.0003);
   speed_reduction.Init(0.1,0.5,0.0);
	
  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */

		  
          double tot_sum = std::accumulate(integral_sum.begin(), integral_sum.end(), 0);
          pid.UpdateError(cte,prev_cte,tot_sum);
          
          speed_reduction.UpdateError(cte,prev_cte,0.0);
          
          prev_cte = cte;
        
          if(integral_sum.size()<50)
          {
          	integral_sum.push_back(cte);
		  }
		  else
		  {
		  	integral_sum.erase(integral_sum.begin());
		  
		  	integral_sum.push_back(cte);
		  }
          steer_value =  pid.TotalError();

          if (steer_value>1)
          	steer_value = 1;
          if (steer_value<-1)
          	steer_value = -1;
          step+=1;
          
		 
          double speed_desired = 60;
          
          t_error = speed - speed_desired;
          
         speed_pid.UpdateError(t_error,prev_t_error,sum_t_error);
         double throttle_value = speed_pid.TotalError();
         prev_t_error = t_error;
         sum_t_error += t_error;
         
         double speed_reduce = speed_reduction.TotalError();
		 if(abs(cte)>=0.9)
		 {
		 	if(speed>30)
		 	{
			 
		 	//throttle_value -=0.1;
		 	throttle_value =  - abs(speed_reduce);
		 }
		 	
		 	
		 	
		 		
		 }
         std::cout<<"Throttle: "<<throttle_value<<std::endl;

        
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                    << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
//          msgJson["throttle"] = 0.3;
		  msgJson["throttle"] =	throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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
