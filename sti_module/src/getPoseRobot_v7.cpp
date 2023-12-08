// Author : AnhTuan 05/01/2021
// Update : AnhTuan 05/01/2021 : tu dong setpose theo id nhin thay , setpose_control : 100 , 4

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <sti_msgs/Setpose_control.h>
#include <sti_msgs/Setpose_status.h>
// file
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>
#include <jsoncpp/json/reader.h>
#include <jsoncpp/json/writer.h>

using namespace std;
fstream f;
geometry_msgs::PoseStamped tag1,tag2;
vector<string> nameTag ;
vector<int> idTag ;
vector<geometry_msgs::PoseStamped> dataTag ;

apriltag_ros::AprilTagDetectionArray getPosearuco; 
apriltag_ros::AprilTagDetectionArray getPosearuco_empty; 
geometry_msgs::PoseWithCovarianceStamped setposeRb; 
bool vel_0 = false ;
geometry_msgs::Pose positionRb ,poseloc;
bool sub4 = false ;
sti_msgs::Setpose_control sp_ctl ;
bool sub1 = false ;
bool sub2 = false ;
ros::Publisher pub2 ;

int sl_tag ;
int dem_wait = 0 ; // xoa data con luu khi tat camera


// call
    // 5hz
    void callSub1(const apriltag_ros::AprilTagDetectionArray& pose){ 
        dem_wait ++; 
        ROS_WARN("dem_wait: %d",dem_wait);
        if (dem_wait > 10 ) {
            getPosearuco = pose ; 
            if (sub1 == false ) sub1 = true ;
        }
        // ROS_INFO("haha");
    }

    void callSub2(const sti_msgs::Setpose_control& data){
        sp_ctl = data ;
        if (sub2 == false ) sub2 = true ;
    }

    void callSub3(const nav_msgs::Odometry& data){
        if( data.twist.twist.linear.x == 0 ) vel_0 = true ;
        else vel_0 = false ;
    }

    void callSub4(const geometry_msgs::Pose& data){
        positionRb = data ;
        if (sub4 == false ) sub4 = true ;
    }

    void pub_status(int status, int id_tag, vector<int> listtag){
        sti_msgs::Setpose_status stt ;
        stt.status = status ;
        stt.find_tag = id_tag ;

        for(int i=0 ; i < listtag.size(); i++ ){
            stt.tagInFile.push_back( listtag.at(i) ) ;
        }

        pub2.publish(stt);
    }

    bool check_position(geometry_msgs::Pose p1 , geometry_msgs::Pose p2){
        if (fabs(p1.position.x - p2.position.x) < 0.02 && \
            fabs(p1.position.y - p2.position.y) < 0.02 && \
            fabs(p1.orientation.x - p2.orientation.x) < 0.02 && \
            fabs(p1.orientation.y - p2.orientation.y) < 0.02    )
        {
            return true ;
        }
        else return false ;
    }

// ham read file 
    void exportData(string link_file){ 

        // cap nhat lai data
        idTag.clear();
        nameTag.clear();
        dataTag.clear();

        Json::Reader reader;
        Json::Value val;

        // open file
        f.open(link_file, ios::in);
        reader.parse(f, val);
        geometry_msgs::PoseStamped datTag ;
        string name_tag;
        int id;

        for( Json::ValueIterator itr = val.begin() ; itr != val.end() ; itr++ ) 
        {
            if(itr.key().asString().substr(0,5) == "/tag_")
            {
                name_tag = itr.key().asString();
                id = stoi(name_tag.substr(5,10).c_str());
                datTag.header.frame_id    =   name_tag ;
                datTag.pose.position.x    =   val[name_tag]["x"].asDouble();
                datTag.pose.position.y    =   val[name_tag]["y"].asDouble();
                datTag.pose.position.z    =   val[name_tag]["z"].asDouble();
                datTag.pose.orientation.x =   val[name_tag]["rx"].asDouble();
                datTag.pose.orientation.y =   val[name_tag]["ry"].asDouble();
                datTag.pose.orientation.z =   val[name_tag]["rz"].asDouble();
                datTag.pose.orientation.w =   val[name_tag]["rw"].asDouble();

                // cout << "name_tag = " << name_tag << "         id = "<< id << " " << datTag.pose.position.x <<endl;

                idTag.push_back(id);
                nameTag.push_back(name_tag);
                dataTag.push_back(datTag);
            }

        }
        f.close();
    }


int main(int argc, char** argv){
    // init
        ros::init(argc, argv, "getPoseRobot_v6");
        ros::NodeHandle n;
        ros::NodeHandle nh_priv("~"); 
        ros::Time current_time, last_vel_time;
        ros::Rate r(100.0);

    // sub - pub
        
        ros::Publisher  pub1    = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("posePoseRobotFromAruco", 100);
        ros::Publisher  setpost = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 50);
        pub2 = n.advertise<sti_msgs::Setpose_status>("setpose_status",100);
        ros::Publisher  pub3    = n.advertise<std_msgs::String>("syscommand", 100);

        ros::Subscriber subscribe1    = n.subscribe("/tag_detections_d435", 100, callSub1);
        ros::Subscriber subscribe2 = n.subscribe("/setpose_control",100,callSub2);
        ros::Subscriber subscribe3 = n.subscribe("/raw_odom",100,callSub3);
        ros::Subscriber subscribe4 = n.subscribe("/robot_pose",100,callSub4);
        
    // TF
        geometry_msgs::TransformStamped trans1, trans2, trans3, trans4;
        tf::TransformBroadcaster broadcaster1, broadcaster2, broadcaster3, broadcaster4;

        tf::TransformListener listener1,listener2,listener3,listener4;
        tf::StampedTransform transform1,transform2,transform3,transform4;
        
    // bien
        int buoc = -1 ;
        int idTarget = 0 ;
        int idTagFind = 0 ;
        int dem_filter = 0 ;
        int sl_filter = 40.0 ;
        geometry_msgs::PoseStamped dataTarget ;
        int dem_setpose = 0 ; 

    // load param 
        string map_frame,tag_name,tag_fistName,cam_name, robot_frame;
        string f_cam_name ,f_tag_name ,f_tag_fistName ,f_robot_frame;
        string link_file ; // link dan den file data_tag.txt

        nh_priv.param<std::string>("map_frame",map_frame,"/map");
        nh_priv.param<std::string>("tag_fistName",tag_fistName,"/tag_");
        nh_priv.param<std::string>("cam_name",cam_name,"/camera_aruco");
        nh_priv.param<std::string>("robot_frame",robot_frame,"/base_footprint");

        nh_priv.param<std::string>("f_cam_name",f_cam_name,"/f_camera_aruco");
        nh_priv.param<std::string>("f_tag_fistName",f_tag_fistName,"/f_tag_");
        nh_priv.param<std::string>("f_robot_frame",f_robot_frame,"/f_robot");

        nh_priv.param<std::string>("link_file",link_file,"/home/amr1-l300/catkin_ws/src/sti_module/data/data_tag.json");

        nh_priv.param<int>("sl_filter",sl_filter,40);


    // read file
        exportData(link_file);

        ROS_INFO("detection : %d Tag",int(idTag.size()));
        for (int i=0; i<idTag.size() ; i++){
            ROS_INFO("id = %d",idTag.at(i));

            ROS_INFO("%s : x= %f ,y= %f, z= %f",dataTag.at(i).header.frame_id.c_str(),\
                                                dataTag.at(i).pose.position.x,\
                                                dataTag.at(i).pose.position.y,\
                                                dataTag.at(i).pose.position.z);
            ROS_INFO("%s : rx= %f ,ry= %f, rz= %f, rw= %f \n",dataTag.at(i).header.frame_id.c_str(),\
                                                            dataTag.at(i).pose.orientation.x,\
                                                            dataTag.at(i).pose.orientation.y,\
                                                            dataTag.at(i).pose.orientation.z,\
                                                            dataTag.at(i).pose.orientation.w);
        }

        pub_status(-1,0,idTag);

    while(n.ok()){
        ros::spinOnce();
        if(sub2 == true)
        {
            exportData(link_file);
        }
        pub_status(buoc,idTagFind,idTag);

        //B-1 :wait cmt + pub id TAG dang nhin thay
        if (buoc == -1){
            ROS_INFO("buoc: %d", buoc);
            // ROS_INFO("wait cmt");
            // ROS_INFO("sl_filter: %d", sl_filter);

            if (sub2 == true && sp_ctl.setposeTag != 0){
                sub2 = false ;
                // reset
                sl_tag = 0 ;
                idTarget = idTagFind = 0 ;
                dem_wait = 0 ;

                buoc = 2 ;
                ROS_INFO("buoc: %d", buoc);
                pub_status(buoc,idTagFind,idTag);        
            }
            
        }


        //B2 :check id tag in camera
        if (buoc == 2){
            ROS_INFO("check id tag in camera");
            if ( sub1 == true){
                sub1 = false ;
                sl_tag = getPosearuco.detections.size() ;  
                ROS_WARN("b2: sl_tag: %d",sl_tag);           
                if ( sl_tag > 0){
                    idTagFind = getPosearuco.detections.at(0).id.at(0) ;
                    ROS_WARN("b2: idTagFind: %d",idTagFind);
                    // set tag nhin thay
                    if ( idTagFind > 0 ){
                        idTarget = idTagFind ;
                        tag_name = tag_fistName + to_string(idTarget) ;
                        f_tag_name = f_tag_fistName + to_string(idTarget) ;
                        buoc = 1 ;
                        ROS_WARN("idTarget: %d",idTarget);
                    }
                    else{
                        buoc = -4 ; // k nhin thay tag
                        ROS_INFO("k nhin thay tag");
                    }

                }
                else{
                    buoc = -4 ; // k nhin thay tag
                    ROS_INFO("k nhin thay tag");
                }
            }
			else{
                
				r.sleep();
				buoc = 2 ; // doi khoi dong apriltag 
			}
            pub_status(buoc,idTagFind,idTag);
            ROS_INFO("buoc: %d", buoc);
        }


        //B-4 : k nhin thay tag
        if (buoc == -4){
            ROS_INFO("k nhin thay tag");
            pub_status(buoc,idTagFind,idTag);
            ROS_INFO("buoc: %d", buoc);
            if(sp_ctl.setposeTag == 0){
                buoc = -1 ;
            }
        }

        //B1 :check id tag in file 
        // and get dataTarget
        if( buoc == 1){
            ROS_INFO("nhan lenh Ok , check file");
            bool check = false ;
            for (int i=0; i<idTag.size() ; i++){
                if ( idTarget == idTag.at(i)){ // khop tag trong file
                    dataTarget = dataTag.at(i);
                    check = true ;
                }
            }
            if (check == true ){
                buoc = 3 ;
                // ROS_INFO("check tag in cam");
            }
            else {
                buoc = -2 ;
                ROS_INFO("khong co data trong file");
            }
            pub_status(buoc,idTagFind,idTag);
            ROS_INFO("buoc: %d", buoc);
        }
        
        //B-2
        if (buoc == -2 ){
            ROS_INFO("k co data trong file");
            pub_status(buoc,idTagFind,idTag);
            ROS_INFO("buoc: %d", buoc);
            if(sp_ctl.setposeTag == 0){
                buoc = -1 ;
            }
        }

        //B3 :check k/c
        if (buoc ==3){
            ROS_INFO("check k/c");
            float kc_x = getPosearuco.detections.at(0).pose.pose.pose.position.x ;
            float kc_z = getPosearuco.detections.at(0).pose.pose.pose.position.z ;
            float kc = sqrt(kc_x*kc_x + kc_z*kc_z) ;

            if (fabs(kc) < sp_ctl.distance ){
                buoc = 4 ;
                ROS_INFO("set pose");
            }
            else{
                buoc = -5 ;
                ROS_INFO("k/x qua xa");
            }
            pub_status(buoc,idTagFind,idTag);
            ROS_INFO("buoc: %d", buoc);
        }

        //B-5 :k/x qua xa
        if (buoc == -5 ){
            ROS_INFO("k/c qua xa");
            pub_status(buoc,idTagFind,idTag);
            ROS_INFO("buoc: %d", buoc);
            if(sp_ctl.setposeTag == 0){
                buoc = -1 ;
            }
        }

        // khop tag -> tranform : /map-> /tag -> /cam -> /robot
        // loc + check vel_0 = true
        if (buoc == 4){
            pub_status(buoc,idTagFind,idTag);
            
            // map-> tag
                trans1.header.stamp = ros::Time::now();
                trans1.header.frame_id      = map_frame;
                trans1.child_frame_id       = f_tag_name;
                trans1.transform.translation.x = dataTarget.pose.position.x  ;                                         
                trans1.transform.translation.y = dataTarget.pose.position.y  ;
                trans1.transform.translation.z = dataTarget.pose.position.z  ;
                trans1.transform.rotation.x    = dataTarget.pose.orientation.x ;
                trans1.transform.rotation.y    = dataTarget.pose.orientation.y ;
                trans1.transform.rotation.z    = dataTarget.pose.orientation.z ; 
                trans1.transform.rotation.w    = dataTarget.pose.orientation.w ; 
                broadcaster1.sendTransform(trans1);

            // tf : // tag -> cam
                listener2.lookupTransform(tag_name, cam_name, ros::Time(0), transform2);
                trans2.header.stamp = ros::Time::now();
                trans2.header.frame_id = f_tag_name;
                trans2.child_frame_id  = f_cam_name;
                trans2.transform.translation.x = transform2.getOrigin().x() ;                                          
                trans2.transform.translation.y = transform2.getOrigin().y() ;
                trans2.transform.translation.z = transform2.getOrigin().z() ;
                trans2.transform.rotation.x    = transform2.getRotation().x() ;
                trans2.transform.rotation.y    = transform2.getRotation().y() ;
                trans2.transform.rotation.z    = transform2.getRotation().z() ; 
                trans2.transform.rotation.w    = transform2.getRotation().w() ; 
                broadcaster2.sendTransform(trans2);

            // tf :cam -> robot
                listener3.lookupTransform(cam_name, robot_frame, ros::Time(0), transform3);
                trans3.header.stamp = ros::Time::now();
                trans3.header.frame_id = f_cam_name;
                trans3.child_frame_id  = f_robot_frame;
                trans3.transform.translation.x = transform3.getOrigin().x() ;                                          
                trans3.transform.translation.y = transform3.getOrigin().y() ;
                trans3.transform.translation.z = transform3.getOrigin().z() ;
                trans3.transform.rotation.x    = transform3.getRotation().x() ;
                trans3.transform.rotation.y    = transform3.getRotation().y() ;
                trans3.transform.rotation.z    = transform3.getRotation().z() ; 
                trans3.transform.rotation.w    = transform3.getRotation().w() ; 
                broadcaster3.sendTransform(trans3);
        

            // tf : map -> robot
                listener4.waitForTransform(map_frame, f_robot_frame, ros::Time(), ros::Duration(1));
                try{
                    listener4.lookupTransform(map_frame, f_robot_frame, ros::Time(0), transform4);
                    trans4.header.stamp = ros::Time::now();
                    trans4.header.frame_id = map_frame;
                    trans4.child_frame_id =  "f_rb";
                    trans4.transform.translation.x = transform4.getOrigin().x() ;                                          
                    trans4.transform.translation.y = transform4.getOrigin().y() ;
                    trans4.transform.translation.z = transform4.getOrigin().z() ;
                    trans4.transform.rotation.x    = transform4.getRotation().x() ;
                    trans4.transform.rotation.y    = transform4.getRotation().y() ;
                    trans4.transform.rotation.z    = transform4.getRotation().z() ; 
                    trans4.transform.rotation.w    = transform4.getRotation().w() ; 
                    broadcaster4.sendTransform(trans4);

                    // check v=0
                    if ( vel_0 == true){
                        
                        poseloc.position.x     += transform4.getOrigin().x() ;                                          
                        poseloc.position.y     += transform4.getOrigin().y() ;
                        poseloc.orientation.z  += transform4.getRotation().z() ; 
                        poseloc.orientation.w  += transform4.getRotation().w() ; 

                        dem_filter ++ ;
                        // ROS_INFO("dem_filter= %d",dem_filter);
                    }
                    if (dem_filter == sl_filter ){
                        poseloc.position.x    = poseloc.position.x / sl_filter ; 
                        poseloc.position.y    = poseloc.position.y / sl_filter ; 
                        poseloc.orientation.x = poseloc.orientation.x / sl_filter ; 
                        poseloc.orientation.y = poseloc.orientation.y / sl_filter ; 
                        
                        std_msgs::String clearMap ;
                        clearMap.data = "reset";
                        pub3.publish(clearMap);
                        ros::Duration(0.2).sleep() ;
                        buoc = 5 ;
                    } 
                    if (dem_filter > sl_filter ){
                        dem_filter = 0 ;
                        poseloc.position.x = 0 ;
                        poseloc.position.y = 0 ; 
                        poseloc.orientation.z = 0 ;
                        poseloc.orientation.w = 0 ;
                        buoc = 4 ;
                    }
                }
                catch (tf::TransformException ex){
                    ROS_ERROR("%s",ex.what());
                }
        
        }

        // setpose  + check position 
        if (buoc == 5){
            ROS_INFO("setpose  + check position ");
            pub_status(buoc,idTagFind,idTag);
            ROS_INFO("buoc: %d", buoc);

            setposeRb.header.stamp = ros::Time::now();
            setposeRb.header.frame_id = "map";
            setposeRb.pose.pose.position.x       = poseloc.position.x     ;                                 
            setposeRb.pose.pose.position.y       = poseloc.position.y     ;
            setposeRb.pose.pose.orientation.z    = poseloc.orientation.z  ;
            setposeRb.pose.pose.orientation.w    = poseloc.orientation.w  ;
            if ( vel_0 == true ){
                setpost.publish(setposeRb);
                ROS_WARN("setposed OK");
                buoc = 51 ;
            }
            else buoc = 5 ; 
        
        }

        if (buoc == 51){
            if ( sub4 == true){ 
                sub4 = false ;
                if ( check_position(poseloc,positionRb) == true) buoc = 52 ;
                else buoc = 5 ;
            }
        }

        // setpose 3 time 
        if(buoc == 52){
            dem_setpose ++ ;
            ROS_WARN("dem_setpose= %d",dem_setpose);
            if(dem_setpose < 4){
                buoc = 4 ;
            }
            else{
                buoc = 6 ;
                dem_setpose = 0 ;
            }
        }

        if (buoc == 6){
            pub_status(buoc,idTagFind,idTag);
            // ROS_INFO("buoc: %d", buoc);
            // ROS_INFO("DONE!!");
            if(sp_ctl.setposeTag == 0){
                // reset
                sl_tag = 0 ;
                idTarget = idTagFind = 0 ;
                dem_filter = 0 ;
                getPosearuco = getPosearuco_empty ;
                dem_wait = 0 ;
                poseloc.position.x = 0 ;
                poseloc.position.y = 0 ;
                poseloc.orientation.z = 0 ;
                poseloc.orientation.w = 0 ;
                buoc = -1 ;
            }
        }


 
        r.sleep();
    }
}
