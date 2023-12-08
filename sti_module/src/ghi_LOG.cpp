#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <ctime> // time
using namespace std;
fstream f;
geometry_msgs::Pose tag1,tag2;


// ham write file 
    void writeData(geometry_msgs::Pose tag_data){ 
        ROS_WARN("ghi log file");
        string datax  = to_string(int(tag_data.position.x));
        string datay  = to_string(int(tag_data.position.y));
        string dataz  = to_string(tag_data.position.z);
        string datarx = to_string(tag_data.orientation.x);
        string datary = to_string(tag_data.orientation.y);
        string datarz = to_string(tag_data.orientation.z);
        string datarw = to_string(tag_data.orientation.w);

        // f << "\n solan = " + (string)datax;       
        // f << "\n idTag = " + (string)datay;
        // // f << "\n z= " + (string)dataz;
        // f << "\n x = " + (string)datarx;    
        // f << "\n y = " + (string)datary;       
        // f << "\n g = " + (string)datarz;
        // f << "\n" ;
        // // f << "\n   rw= " + (string)datarw;
        time_t hientai = time(0);
        string tg = ctime(&hientai);
        string thoigian = to_string(hientai);

        f << " " + (string)datax +    // so lan
             " " + (string)datay +    // 1: co tag
             " " + (string)datarx +   // x
             " " + (string)datary +   // y
             " " + (string)datarz +
             " " + tg  ;             // thoi gian 

    }
// call
    void callSub(const geometry_msgs::Pose& log){ 
        f.open("/home/stivietnam/catkin_ws/debug/data_log.txt", ios::app);
        writeData(log);
        f.close();
    }
    
int main(int argc, char **argv)
{   
    ros::init(argc, argv, "write_file");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    ros::Subscriber sub1    = n.subscribe("/data_log", 100, callSub);
    ros::spin();
    return 0;
}
