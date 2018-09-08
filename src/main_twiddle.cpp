#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int num = 0;

//p: 0.100513 , 5.9316e-05 , 7.1
//p: 0.100513 , 0.002906 , 7.1
//double p_p = 0.001227688652457;
double p_p = 0.100513;
double p_i = 0.0493160;
double p_d = 7.1; //10.326589894591526;

//double p_p = 0.0;
//double p_i = 0.0;
//double p_d = 0.0;

//double dp_p = 1.0;
//double dp_i = 1.0;
//double dp_d = 1.0;

double dp_p = 0;
double dp_i = 0.01;
double dp_d = 0;//1.0;

//double err = std::numeric_limits<double>::infinity();
double err = 0.0;
double best_err = std::numeric_limits<double>::infinity();

int loopflag = 1; // 1-2 = p, 3-4 = i, 5-6 = d
int flag = 0; //0 = loop1, 1 = loop2

int main()
{
  uWS::Hub h;

  //int num;

  //num = 0;

  PID pid;
  // TODO: Initialize the pid variable.
  //[2.9331227688652457, 0.49316041639454505, 10.326589894591526]
  //pid.Init(2.9331227688652457, 0.49316041639454505, 10.326589894591526);
  //pid.Init(0.001227688652457, 0.00004931604163945, 10.326589894591526); // is ok 01
  //pid.Init(0.001227688652457, 0.00004931604163945, 10.326589894591526); // is ok 01
  pid.Init(p_p, p_i, p_d);


  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid.UpdateError(cte);
          pid.last_p_error = cte;
          steer_value = pid.TotalError();

          if (steer_value > 1){
              steer_value = 1;
          }

          if (steer_value < -1){
              steer_value = -1;
          }
 
          // DEBUG
          //std::cout << num << std::endl;
          //std::cout << "speed: " << speed << " angle: " << angle << std::endl;
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          //std::cout << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          //msgJson["throttle"] = 0.3;
          msgJson["throttle"] = 0.4;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          //std::cout << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);


          err += cte * cte;

          std::cout <<"***************num: "<< num <<"   err: " << err << "   best_err: "  << best_err << std::endl;

          if(num > 300){
              //std::string msg = "42[\"reset\",{}]";
              //ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              //num = 0;
              //std::cout << "------------------------------" << std::endl;

              if(loopflag >= 1 && loopflag <= 2){

                  if(flag == 0){
                      p_p += dp_p;

                      if(err < best_err){
                          best_err = err;
                          dp_p *= 1.1;
                      }               
                      else{
                          p_p = p_p - 2 * dp_p;
                      }

                      flag++;
                  }
                  else{

                      if(err < best_err){
                          best_err = err;
                          dp_p *= 1.1;
                      }    
                      else{
                          p_p += dp_p;
                          dp_p *= 0.9;
                      }                      

                      flag = 0;
                  }

                  loopflag++;

              }
              else if (loopflag >= 3 && loopflag <= 4){

                  if(flag == 0){
                      p_i += dp_i;
                      
                      if(err < best_err){
                          best_err = err;
                          dp_i *= 1.1;
                      }    
                      else{
                          p_i = p_i - 2 * dp_i;
                      }
                      
                      flag++;
                  }
                  else{
                      
                      if(err < best_err){
                          best_err = err;
                          dp_i *= 1.1;
                      }    
                      else{
                          p_i += dp_i;
                          dp_i *= 0.9;
                      }
                      
                      flag = 0;
                  }

                  loopflag++;
              }
              else if (loopflag >= 5 && loopflag <= 6){

                  if(flag == 0){
                      p_d += dp_d;
                      
                      if(err < best_err){
                          best_err = err;
                          dp_d *= 1.1;
                      }    
                      else{
                          p_d = p_d - 2 * dp_d;
                      }
                      
                      flag++;
                  }
                  else{
                      
                      if(err < best_err){
                          best_err = err;
                          dp_d *= 1.1;
                      }    
                      else{
                          p_d += dp_d;
                          dp_d *= 0.9;
                      }
                      
                      flag = 0;
                  }

                  if(loopflag == 5){
                      loopflag++;
                  }
                  else{
                      loopflag = 1;
                  }

              }


              //pid.Init(p_p + dp_p, p_i + dp_i, p_d + dp_d);
              pid.Init(p_p, p_i, p_d);
             
              std::cout << "dp:  " << dp_p + dp_i + dp_d << std::endl; 
              std::cout <<"p: " << p_p << " , "<< p_i << " , " << p_d << std::endl;
              std::cout <<"err: " << err << "   best_err: "  << best_err << std::endl;
              std::cout << std::endl;
             
              num = 0;
              err = 0.0;

              std::string msg = "42[\"reset\",{}]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

              if(dp_p + dp_i + dp_d <0.00001){
                  exit(0);
              }

          }

          num += 1;

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
