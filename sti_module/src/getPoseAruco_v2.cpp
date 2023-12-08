//author : AnhTuan 27/5/2020  
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int16.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <string>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>

using namespace std;
fstream f;


#define pi 3.141592653589793238462643383279502884
#define can2_2 0.7071067812
apriltag_ros::AprilTagDetectionArray getPosearuco; 
std_msgs::Int16 key ;
string file = "/home/stivietnam/catkin_ws/src/vector_convert/data/data_tag.txt" ;

bool sub4 = false ,sub5 = false ;

    // 5hz
    void callSub4(const apriltag_ros::AprilTagDetectionArray& pose)
    { 
        getPosearuco = pose ; 
        sub4 = true ;
        // ROS_INFO("hihi");
    }

    void callSub5(const std_msgs::Int16& keyboard){
        key = keyboard ;
        sub5 = true ;
    }

    void writeData(geometry_msgs::PoseStamped& tag_data, string name_file){ 
        f.open(name_file, ios::app);
        string tag_name = *(&tag_data.header.frame_id) ;
        string datax  = to_string(tag_data.pose.position.x);
        string datay  = to_string(tag_data.pose.position.y);
        string dataz  = to_string(tag_data.pose.position.z);
        string datarx = to_string(tag_data.pose.orientation.x);
        string datary = to_string(tag_data.pose.orientation.y);
        string datarz = to_string(tag_data.pose.orientation.z);
        string datarw = to_string(tag_data.pose.orientation.w);

        f << "\n" + tag_name;
        f << "\n    x= " + (string)datax;       
        f << "\n    y= " + (string)datay;
        f << "\n    z= " + (string)dataz;
        f << "\n   rx= " + (string)datarx;    
        f << "\n   ry= " + (string)datary;       
        f << "\n   rz= " + (string)datarz;
        f << "\n   rw= " + (string)datarw;
        f.close();

    }
    
    bool checkData(geometry_msgs::PoseStamped& tag_data, string name_file){ 
        f.open(name_file, ios::in);
        string line;
        string tag_name = *(&tag_data.header.frame_id) ;
        ROS_INFO("tag_name= %s",tag_name.c_str());
        int dem =0 ;
        while (!f.eof())
        {
            getline(f, line);
            if (line == tag_name){  
                dem ++ ; 
                f.close();
            }
            
        }
        if (dem != 0) {
            dem =0 ;
            return 1 ;
        }
        else return 0 ;
 

    }

    geometry_msgs::PoseStamped stiTransform(string map_frame, string tag_frame)
    {   
        geometry_msgs::PoseStamped      tag_data   ;
        tf::TransformBroadcaster        broadcaster;
        geometry_msgs::TransformStamped trans      ;
        tf::TransformListener           listener   ;
        tf::StampedTransform            transform  ;;
        string f_tag_frame = "f_" + tag_frame ;

        listener.waitForTransform(map_frame, tag_frame, ros::Time(), ros::Duration(1.0));
        try{
            listener.lookupTransform(map_frame, tag_frame,ros::Time(0), transform);

            trans.header.stamp            = ros::Time::now();
            trans.header.frame_id         = map_frame;
            trans.child_frame_id          =  f_tag_frame;
            trans.transform.translation.x = transform.getOrigin().x() ;                                            
            trans.transform.translation.y = transform.getOrigin().y() ;
            trans.transform.translation.z = transform.getOrigin().z() ;
            trans.transform.rotation.x    = transform.getRotation().x() ;
            trans.transform.rotation.y    = transform.getRotation().y() ;
            trans.transform.rotation.z    = transform.getRotation().z() ; 
            trans.transform.rotation.w    = transform.getRotation().w() ;                
            broadcaster.sendTransform(trans);

            tag_data.header.frame_id     = tag_frame;
            tag_data.pose.position.x     = transform.getOrigin().x() ;    
            tag_data.pose.position.y     = transform.getOrigin().y() ;
            tag_data.pose.position.z     = transform.getOrigin().z() ;
            tag_data.pose.orientation.x  = transform.getRotation().x() ;
            tag_data.pose.orientation.y  = transform.getRotation().y() ;
            tag_data.pose.orientation.z  = transform.getRotation().z() ; 
            tag_data.pose.orientation.w  = transform.getRotation().w() ;   

            return tag_data ;   
                    
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }
    }

int main(int argc, char** argv){
    // init
        ros::init(argc, argv, "getPoseArucoInMap");
        ros::NodeHandle n;
        ros::NodeHandle nh_priv("~");
        ros::Time current_time, last_vel_time;
        ros::Rate r(100.0);

    // subcriber
        ros::Subscriber subscribe4 = n.subscribe("/tag_detections_d435", 1000, callSub4);
        ros::Subscriber subscribe5 = n.subscribe("/keypub_int", 10, callSub5);

    // bien
        string map_frame,tag_name,tag_fistName,link_file;
        geometry_msgs::PoseStamped tagPose;

    // load param 
        nh_priv.param<std::string>("map_frame",map_frame,"/map");
        nh_priv.param<std::string>("tag_fistName",tag_fistName,"/tag_");
        nh_priv.param<std::string>("link_file",link_file,"");
        
        printf("\n\n                 go ENTER de luu ARUCO              \n\n");
    while(n.ok()){
        ros::spinOnce();
        current_time = ros::Time::now();

        if(sub4 == true){
            sub4 = false ;
            int sl_tag = getPosearuco.detections.size() ;
            ROS_INFO("Detection : %d tag",sl_tag );
            if ( sl_tag != 0 ){
                int id_tag = getPosearuco.detections.at(0).id.at(0);
                ROS_INFO("id_tag= %d",id_tag);
                std::string id = to_string(id_tag);
                tag_name = tag_fistName + id ;
                // stiTransform(tagPose,map_frame,tag_name);
                tagPose = stiTransform(map_frame,tag_name);
        
                if(sub5 ==true && key.data == 13 && sl_tag ==1){ // enter
                    sub5 = false ;
                        writeData(tagPose,link_file);
                        ROS_INFO("\n\n                  Da luu : %s \n",tagPose.header.frame_id.c_str());

                }
                if(sub5 ==true && key.data == 13 && sl_tag !=1){ // enter
                    ROS_WARN("\n\n       Chi luu khi Detection 1 tag !!!  \n\n");     
                    sub5 = false ;              
                }
                        
            }

        }

                         

        r.sleep();
    }
    
    return 0;
}
