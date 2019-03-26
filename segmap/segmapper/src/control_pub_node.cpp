#include "ros/ros.h"
#include "std_msgs/String.h"
#include "map_msgs/SaveMap.h"

#include <sstream>
#include <iostream>
#include <string>

int main(int argc, char **argv){
  ros::init(argc, argv, "control_talker");
  ros::NodeHandle n("segmapper");
  std::cout << n.getNamespace() << std::endl;
  // ros::Publisher control_pub = n.advertise<std_msgs::String>("control", 10);

  // ros::Rate loop_rate(10);
  ros::Publisher save_map = n.advertise<std_msgs::String>("/segmatch/control", 10);
  while (ros::ok()){
    std_msgs::String msg;

    std::stringstream ss;
    std::string cmd, name;
    std::cout << "wait for control command: \n";
    std::cin >> cmd >> name;
    if(cmd == "quit" || cmd == "exit"){
      break;
    }else if(cmd == "save"){
      std::cout << "SAVE MAP\n";
      save_map.publish(cmd);
      std::cout << "FINISHED\n";
    }
    // else if(cmd == "save_local"){
    //   std::cout << "SAVE LOCAL MAP\n";
    //   ros::ServiceClient save_local_map;
    //   save_local_map = n.serviceClient<map_msgs::SaveMap>("save_local_map");
    //   map_msgs::SaveMap srv;
    //   srv.request.filename.data = "/home/liu/.segmap/" + name + ".pcd";
    //   std::cout << save_local_map.call(srv) << "\n";
    //   std::cout << "FINISHED\n";
    // }
    // ss << cmd;
    // msg.data = ss.str();

    // control_pub.publish(msg);
  }
  return 0;
}
