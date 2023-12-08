// Author : Phùng Quý Dương 
// Date: 15-9-2023

/*
    - Node stiControl_cpp
    - function:
        + Xử lý chung các thiết bị và chương trình con cho AGV. 
*/

#include "ros/ros.h"
#include "ros/console.h"

#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"

#include "sensor_msgs/Imu.h"

#include "sti_msgs/POWER_request.h"
#include "sti_msgs/POWER_info.h"
#include "sti_msgs/HC_request.h"
#include "sti_msgs/HC_info.h"
#include "sti_msgs/Lift_control.h"
#include "sti_msgs/Lift_status.h"
#include "sti_msgs/Move_request.h"
#include "sti_msgs/NN_cmdRequest.h"
#include "sti_msgs/NN_infoRespond.h"
#include "sti_msgs/NN_infoRequest.h"
#include "sti_msgs/Status_goal_control.h"
#include "sti_msgs/Zone_lidar_2head.h"

#include "message_pkg/Status_reconnect.h"
#include "message_pkg/App_button.h"
#include "message_pkg/Nav350_data.h"
#include "message_pkg/Status_port.h"
#include "message_pkg/Parking_request.h"
#include "message_pkg/Parking_respond.h"
#include "message_pkg/Driver_respond.h"

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <bits/stdc++.h>
#include <unistd.h>
#include <vector>


#define True 1
#define False 0
#define _USE_MATH_DEFINES

using namespace std;

class ros_control
{
    public:
    std_msgs::Int16 cancelMission_control;
    std_msgs::Int16 cancelMission_status;

    uint8_t flag_cancelMission;
    uint8_t status_cancel;

    message_pkg::Status_reconnect status_reconnect;

    sti_msgs::POWER_info main_info;
    sti_msgs::POWER_request power_request;
    double timeStampe_main;  //
    float voltage;

    geometry_msgs::Twist velPs2;

    sti_msgs::HC_info HC_info;
    sti_msgs::HC_request HC_request;
    double timeStampe_HC;

    sti_msgs::Lift_status lift_status;
    double timeStampe_OC;
    sti_msgs::Lift_control lift_control;

    message_pkg::App_button app_button;

    std_msgs::Int8 safety_NAV;

    message_pkg::Nav350_data nav350_data;

    sti_msgs::Move_request request_move;
    sti_msgs::NN_cmdRequest NN_cmdRequest;
    sti_msgs::NN_infoRespond NN_infoRespond;

    sensor_msgs::Imu imu_data;

    sti_msgs::Zone_lidar_2head zoneRobot;
    uint8_t is_readZone;

    geometry_msgs::PoseStamped robotPose_nav;
    message_pkg::Status_port status_port;

    message_pkg::Parking_respond parking_status;
    geometry_msgs::Pose parking_poseTarget;
    uint8_t is_parking_status;
    uint8_t flag_requirResetparking;
    uint8_t enb_parking;
    geometry_msgs::Pose parking_poseBefore;
    geometry_msgs::Pose parking_poseAfter;
	float parking_offset;

	uint8_t flag_requirBackward;
    uint8_t completed_backward; 
    float backward_x; 
    float backward_y;
	float backward_z;

    sti_msgs::NN_infoRequest NN_infoRequest;
    double timeStampe_server;

    sti_msgs::Status_goal_control status_goalControl;
    double timeStampe_statusGoalControl;

    message_pkg::Driver_respond driver1_respond;
    message_pkg::Driver_respond driver2_respond;
    double timeStampe_driver;

    std_msgs::Int16 task_driver;
    uint8_t taskDriver_nothing; 
    uint8_t taskDriver_resetRead; 
    uint8_t taskDriver_Read; 

    std_msgs::Bool disable_brake;

	uint8_t FrequencePubBoard;
    double pre_timeBoard;
	uint8_t mode_by_hand;
	uint8_t mode_auto; 
	uint8_t mode_operate;

    double target_x;
    double target_y;
    double target_z;
    double target_tag;

    int8_t process;
    uint8_t before_mission;
    uint8_t after_mission;

    int completed_before_mission;
    int completed_after_mission;
    int completed_move;
    int completed_moveSimple;
    int completed_moveSpecial;
    int completed_reset;
    int completed_MissionSetpose;
    int completed_checkLift;

    int flag_afterChager;
    int flag_checkLiftError;
    int flag_Auto_to_Byhand;
    int enb_move;
    int flag_read_client;
    int flag_error;
    int flag_warning;

    sti_msgs::Move_request move_req;
    string pre_mess;

    // -- Check:
    uint8_t is_get_pose;
    uint8_t is_mission;
    uint8_t is_read_prepheral;       // ps2 - sti_read
    uint8_t is_request_client;
    uint8_t is_set_pose;
    uint8_t is_zone_lidar;

    uint8_t completed_wakeup;        // khi bật nguồn báo 1 sau khi setpose xong.
    // -- Status to server:
    uint8_t statusU300L;
    uint8_t statusU300L_ok;
    uint8_t statusU300L_warning;
    uint8_t statusU300L_error;
    uint8_t statusU300L_cancelMission;
    // -- Status to detail to follow:
    uint8_t stf;
    uint8_t stf_wakeup;
    uint8_t stf_running_simple;
    uint8_t stf_stop_obstacle;
    uint8_t stf_running_speial;
    uint8_t stf_running_backward;
    uint8_t stf_performUp;
    uint8_t stf_performDown;
    // -- EMC reset
    uint8_t EMC_resetOn;
    uint8_t EMC_resetOff;
    uint8_t EMC_reset;
    // -- EMC write
    uint8_t EMC_writeOn;
    uint8_t EMC_writeOff;
    uint8_t EMC_write;
    // -- Led
    uint8_t led;
    uint8_t led_error; 			
    uint8_t led_simpleRun;
    uint8_t led_specialRun;
    uint8_t led_perform;
    uint8_t led_completed;
    uint8_t led_stopBarrier;
    // -- Mission server
    uint8_t statusTask_liftError;
    uint8_t serverMission_liftUp;
    uint8_t serverMission_liftDown;
    uint8_t serverMission_charger;
    uint8_t serverMission_unknown;
    uint8_t serverMission_liftDown_charger;
    // -- Lift task.
    uint8_t liftTask; 
    uint8_t liftUp;
    uint8_t liftDown;
    uint8_t liftStop;
    uint8_t liftResetOn;
    uint8_t liftResetOff;
    uint8_t liftReset;
    uint8_t flag_commandLift;
    uint8_t liftTask_byHand;
    // -- Speaker
    uint8_t speaker;
    uint8_t speaker_requir;
    uint8_t spk_error;
    uint8_t spk_move;
    uint8_t spk_warn;
    uint8_t spk_not;
    uint8_t spk_off;
    uint8_t enb_spk;
    // -- Charger
    uint8_t charger_on;
    uint8_t charger_off;		
    uint8_t charger_requir;
    uint8_t charger_write;
    float charger_valueOrigin;
    // -- Voltage
    uint16_t timeCheckVoltage_charger;
    uint16_t timeCheckVoltage_normal;
    uint16_t pre_timeVoltage;   
    float valueVoltage;
    uint8_t step_readVoltage;
    // -- Cmd_vel
    geometry_msgs::Twist vel_ps2;       
    // -- ps2 
    double time_ht;
    double time_tr;
    float rate_cmdvel = 10.; 
    // -- Error Type
    uint8_t error_move;
    uint8_t error_perform;
    uint8_t error_device;  

    uint8_t numberError;
    float lastTime_checkLift;
    // -- add new
    uint8_t enb_debug;
    // --
    // uint16_t listError[20];
    // uint16_t listError;
    vector<int> listError;
    uint8_t job_doing;
    // -- -- -- Su dung cho truong hop khi AGV chuyen Che do bang tay, bi keo ra khoi vi tri => AGV se chay lai.
    // -- Pose tai vi tri Ke, sac
    geometry_msgs::Pose poseWait;
    float distance_resetMission;
    uint8_t flag_resetFramework;
    // --
    uint8_t flag_stopMove_byHand;
    // --
    double timeStampe_reflectors;
    // -- add 23/12/2021:
    geometry_msgs::Pose pose_parkingRuning;
    // -- add 27/12/2021
    geometry_msgs::Pose cancelbackward_pose;
    float cancelbackward_offset;

    // -- add 18/01/2022 : sua loi di lai cac diem cu khi mat ket noi server.
    // uint16_t list_id_unknown;
    vector<int32_t> list_id_unknown;
    uint8_t flag_listPoint_ok;
    
    // -- add 19/01/2022 : Check error lost server.
    string name_card;
    string address;
    double saveTime_checkServer;
    uint8_t saveStatus_server;
    // -- add 30/03/2022 : co bao loi qua tai dong co.
    uint8_t flagError_overLoad;
    // -- add 15/04/2022
    uint8_t flag_listPointEmpty;

    // --
    uint8_t step_tryTarget;

    uint16_t time_curr;
    uint16_t delta_time;

    uint8_t enb_mission;

    ros::Publisher pub_cancelMission;
    ros::Publisher pub_requestMain;
    ros::Publisher pub_vel;
    ros::Publisher pub_HC;
    ros::Publisher pub_OC;
    ros::Publisher pub_moveReq;
    ros::Publisher pub_infoRespond;
    ros::Publisher pub_parking;
    ros::Publisher pub_taskDriver;
    ros::Publisher pub_disableBrake;

    void callback_imu(const sensor_msgs::Imu msg){
        imu_data = msg;
    }

    void nav350_callback(const message_pkg::Nav350_data msg){
        nav350_data = msg;
    }

    void safety_NAV_callback(const std_msgs::Int8 msg){
        safety_NAV = msg;
    }

    void callback_goalControl(const sti_msgs::Status_goal_control msg){
        status_goalControl = msg;
        timeStampe_statusGoalControl = ros::Time::now().toSec();
    }

    void callback_cancelMission(const std_msgs::Int16 msg){
        cancelMission_control = msg;
    }

    void callback_infoRequest(const sti_msgs::NN_infoRequest msg){
        NN_infoRequest = msg;
        timeStampe_server = ros::Time::now().toSec();
    }

    void callback_parking(const message_pkg::Parking_respond msg){
        parking_status = msg;
        is_parking_status = 1;
    }

    void callback_reconnect(const message_pkg::Status_reconnect msg){
        status_reconnect = msg;
    }

    float round_1decimal(float var)
    {
        // 37.66666 * 100 =3766.66
        // 3766.66 + .5 =3767.16    for rounding off value
        // then type cast to int so value is 3767
        // then divided by 100 so the value converted into 37.67
        float value = (int)(var * 10 + .5);
        return (float)value / 10;
    }
    float round_2decimal(float var)
    {
        // 37.66666 * 100 =3766.66
        // 3766.66 + .5 =3767.16    for rounding off value
        // then type cast to int so value is 3767
        // then divided by 100 so the value converted into 37.67
        float value = (int)(var * 100 + .5);
        return (float)value / 100;
    }

    float round_3decimal(float var)
    {
        float value = int(var * 1000 + 0.5);
        return (float)value / 1000;
    }

    float radians(float var){
        return (var*M_PI/180);
    }
    void callback_main(const sti_msgs::POWER_info msg){
        main_info = msg;
		voltage = round_2decimal(main_info.voltages);
		timeStampe_main = ros::Time::now().toSec();
    }

    void callback_hc(const sti_msgs::HC_info msg){
        HC_info = msg;
        timeStampe_HC = ros::Time::now().toSec();
    }

    void callback_oc(const sti_msgs::Lift_status msg){
        lift_status = msg;
        timeStampe_OC = ros::Time::now().toSec();
    }

    void callback_appButton(const message_pkg::App_button msg){
        app_button = msg;
    }

    void callback_cmdRequest(const sti_msgs::NN_cmdRequest msg){
        NN_cmdRequest = msg;
        timeStampe_server = ros::Time::now().toSec();
        flag_listPoint_ok = 0;
    }

    void callback_safetyZone(const sti_msgs::Zone_lidar_2head msg){
        zoneRobot = msg;
        is_readZone = 1;
    }

    void callback_port(const message_pkg::Status_port msg){
        status_port = msg;
    }

    void callback_driver1(const message_pkg::Driver_respond msg){
        driver1_respond = msg;
        timeStampe_driver = ros::Time::now().toSec();
    }

    void callback_driver2(const message_pkg::Driver_respond msg){
        driver2_respond = msg;
        timeStampe_driver = ros::Time::now().toSec();
    }

    void callback_getPose(const geometry_msgs::PoseStamped msg){
        robotPose_nav = msg;
        tf::Quaternion q(msg.pose.orientation.x, msg.pose.orientation.y, \
                            msg.pose.orientation.z, msg.pose.orientation.w);
        
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
		// quaternion1 = (msg.pose.orientation.x, msg.pose.orientation.y,\
		// 			msg.pose.orientation.z, msg.pose.orientation.w);
		// euler = tf.transformations.euler_from_quaternion(quaternion1);

		NN_infoRespond.x = round_3decimal(msg.pose.position.x);
		NN_infoRespond.y = round_3decimal(msg.pose.position.y);
		NN_infoRespond.z = round_3decimal(yaw);
    }

	// -- add 15/04/2022
	uint8_t check_listPoints(vector<int32_t> list_id){
		int count = 0;
		try{
            int size_list = list_id.size();
			for(int i = 0; i < size_list; i++){
				if (list_id[i] != 0){
					count += 1;
					ROS_INFO("Danh sach diem khac 0");
                }
            }

			if (count != 0){
				return 1;
            }
			else{
				return 0;
            }
        }
		catch(...){
			return 0;
        }
    }

	// void log_mess(string typ, string mess, float val){
	// 	// -- add new
	// 	if (enb_debug){
	// 		if (pre_mess != mess){
	// 			if (typ == "info"){
	// 				ROS_INFO(mess + ": " + string(val));
    //             }
	// 			else if(typ == "warn"){
	// 				ROS_WARN(mess + ": " + string(val));
    //             }
	// 			else{
	// 				ROS_ERROR(mess + ": " + string(val));
    //             }
    //         }
	// 		pre_mess = mess;
    //     }
    // }

	void pub_move_req(ros::Publisher pub_moveReq, uint8_t ena, sti_msgs::Move_request ls){
		sti_msgs::Move_request req_move;

		req_move.enable = ena;
		req_move.target_x = ls.target_x;
		req_move.target_y = ls.target_y;
		req_move.target_z = ls.target_z;
		req_move.tag = ls.tag;
		req_move.offset = ls.offset;
		req_move.list_id = ls.list_id;
		req_move.list_x = ls.list_x;
		req_move.list_y = ls.list_y;
		req_move.list_speed = ls.list_speed;
		req_move.mission = ls.mission;

		pub_moveReq.publish(req_move);
    }

	void pub_cmdVel(ros::Publisher pub_vel, geometry_msgs::Twist twist , float rate , double mtime){
		time_ht = mtime;

		if (time_ht - time_tr > float(1/rate)){
			time_tr = time_ht;
			pub_vel.publish(twist);
        }
    }

	void Main_pub(ros::Publisher pub_requestMain, uint8_t charge, uint8_t sound, uint8_t EMC_write, uint8_t EMC_reset){
		sti_msgs::POWER_request mai;
		mai.charge = charge;
		if (sound == 0) {
            mai.sound_on = 0;
        }
		else{
			mai.sound_on = 1;
			mai.sound_type = sound;
        }
		mai.EMC_write = EMC_write;
		mai.EMC_reset = EMC_reset;
		
		pub_requestMain.publish(mai);
    }

	uint8_t point_same_point(float x1, float y1, float z1, float x2, float y2, float z2){
		// tọa độ
		float x = x2 - x1;
		float y = y2 - y1;
		float d = sqrt(x*x + y*y);
		// góc 
        static float z;
		if (z2*z1 >= 0){
			z = z2 - z1;
        }
		else{
			z = z2 + z1;
        }
		if (d > 0.2 || abs(z) > 0.14){  // 20 cm - ~ 20*C
			return 1;
        }
		else{
			return 0;
        }
    }

	void pub_park(ros::Publisher pub_parking, uint8_t modeRun, geometry_msgs::Pose poseBefore, geometry_msgs::Pose poseTarget, float offset){
		message_pkg::Parking_request park;
		park.modeRun = modeRun;
		park.poseBefore = poseBefore;
		park.poseTarget = poseTarget;
		park.offset = offset;

		pub_parking.publish(park);
    }

	geometry_msgs::Quaternion euler_to_quaternion(float euler){
		geometry_msgs::Quaternion quat;

        tf::Quaternion odom_quat;
        odom_quat.setRPY(0,0, euler);
        odom_quat = odom_quat.normalize();
		// odom_quat = quaternion_from_euler(0, 0, euler)
		quat.x = odom_quat.x();
		quat.y = odom_quat.y();
		quat.z = odom_quat.z();
		quat.w = odom_quat.w();
		return quat;
    }

	void readbatteryVoltage(){ 
		time_curr = ros::Time::now().toSec();
		delta_time = (time_curr - pre_timeVoltage);
		if (charger_requir == charger_on){
			flag_afterChager = 1;
			if (step_readVoltage == 0){  // bat sac.
				charger_write = charger_on;
				if (delta_time > timeCheckVoltage_charger){
					pre_timeVoltage = time_curr;
					step_readVoltage = 1;
                }
            }

			else if (step_readVoltage == 1){ // tat sac va doi.
				charger_write = charger_off;
				if (delta_time > timeCheckVoltage_normal*3){
					pre_timeVoltage = time_curr;
					step_readVoltage = 2;
                }
            }

			else if (step_readVoltage == 2){ // do pin.	
				int bat = round_1decimal(main_info.voltages)*10;
				// print "charger --"
				if  (bat > 255){
					valueVoltage = 255;
                }
				else if (bat < 0){
					valueVoltage = 0;
                }
				else{
					valueVoltage = bat;
                }

				pre_timeVoltage = time_curr;
				step_readVoltage = 3;
            }

			else if(step_readVoltage == 3){ // doi.
				if (delta_time > 2){
					pre_timeVoltage = time_curr;
					step_readVoltage = 0;
                }
            }
        }
		else if(charger_requir == charger_off){
			if (flag_afterChager == 1){   // sau khi tat sac doi T s roi moi do dien ap.
				pre_timeVoltage = time_curr;
				flag_afterChager = 0;
				charger_write = charger_off;
				step_readVoltage = 0;
            }
			else{
				if (delta_time > timeCheckVoltage_normal){
					pre_timeVoltage = time_curr;
					int bat = round_1decimal(main_info.voltages)*10;
					// print "normal --"
					if  (bat > 255){
						valueVoltage = 255;
                    }
					else if( bat < 0){
						valueVoltage = 0;
                    }
					else{
						valueVoltage = int(bat);
                    }
                }
            }
        }
    }

	geometry_msgs::Twist run_maunal(){
		geometry_msgs::Twist cmd_vel;
		float val_linear = (app_button.vs_speed/100.)*0.35;
		float val_rotate = (app_button.vs_speed/100.)*0.3;

		if (app_button.bt_forwards == True){
			if (HC_info.zone_sick_ahead == 1){
				cmd_vel.linear.x = 0.0;
				cmd_vel.angular.z = 0.0;
            }
			else{
				cmd_vel.linear.x = val_linear;
				cmd_vel.angular.z = 0.0;
            }
        }

		if (app_button.bt_backwards == True){
			if (HC_info.zone_sick_behind == 2){
				cmd_vel.linear.x = 0.0;
				cmd_vel.angular.z = 0.0;
            }
			else{
				cmd_vel.linear.x = -val_linear;
				cmd_vel.angular.z = 0.0;
            }
        }

		if (app_button.bt_rotation_left == True){
			if (HC_info.zone_sick_ahead == 1 || HC_info.zone_sick_behind == 2){
				cmd_vel.linear.x = 0.0;
				cmd_vel.angular.z = 0.0;
            }				
			else{
				cmd_vel.linear.x = 0.0;
				cmd_vel.angular.z = val_rotate;
            }
        }

		if (app_button.bt_rotation_right == True){
			if (HC_info.zone_sick_ahead == 1 || HC_info.zone_sick_behind == 2){
				cmd_vel.linear.x = 0.0;
				cmd_vel.angular.z = 0.0;
            }				
			else{
				cmd_vel.linear.x = 0.0;
				cmd_vel.angular.z = -val_rotate;
            }
        }

		if (app_button.bt_stop == True){
			cmd_vel.linear.x = 0.0;
			cmd_vel.angular.z = 0.0;
        }

		if (safety_NAV.data == 1){
			geometry_msgs::Twist cmd_vel;
        }

		return cmd_vel;

    }    

	float quaternion_to_euler(geometry_msgs::Quaternion qua){
        tf::Quaternion quat(qua.x, qua.y, qua.z, qua.w);
        tf::Matrix3x3 m(quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
		return yaw;
    }

	geometry_msgs::Pose getPose_from_offset(geometry_msgs::Pose pose_in, float offset){
		geometry_msgs::Pose pose_out;
		float angle = quaternion_to_euler(pose_in.orientation);
        float angle_target;

		if (angle >= 0){
			angle_target = angle - M_PI;
        }
		else{
			angle_target = M_PI + angle;
        }

		pose_out.position.x = pose_in.position.x + cos(angle_target)*offset;
		pose_out.position.y = pose_in.position.y + sin(angle_target)*offset;

		pose_out.orientation = euler_to_quaternion(angle_target);
		return pose_out;
    }

	uint8_t detectLost_goalControl(){
		double delta_t = ros::Time::now().toSec() - timeStampe_statusGoalControl;
		if (delta_t > 1.4){
			return 1;
        }
		return 0;
    }

	uint8_t detectLost_driver(){
		double delta_t = ros::Time::now().toSec() - timeStampe_driver;
		if (delta_t > 0.8){
			return 1;
        }
		return 0;
    }

	uint8_t detectLost_hc(){
		double delta_t = ros::Time::now().toSec() - timeStampe_HC;
		if (delta_t > 0.4){
			return 1;
        }
		return 0;
    }

	uint8_t detectLost_oc(){
		double delta_t = ros::Time::now().toSec() - timeStampe_OC;
		if (delta_t > 0.4){
			return 1;
        }
		return 0;
    }

	uint8_t detectLost_main(){
		double delta_t = ros::Time::now().toSec() - timeStampe_main;
		if (delta_t > 1.0){
			return 1;
        }
		return 0;
    }

	uint8_t detectLost_nav(){
		double delta_t = (ros::Time::now() - nav350_data.header.stamp).toSec();
		if (delta_t > 0.4){
			return 1;
        }
		return 0;
    }

	uint8_t detectLost_poseRobot(){
		double delta_t = (ros::Time::now() - robotPose_nav.header.stamp).toSec();
		if (delta_t > 0.5){
			return 1;
        }
		return 0;
    }

	uint8_t detectLost_Imu(){
		double delta_t = (ros::Time::now() - imu_data.header.stamp).toSec();
		if (delta_t > 0.2){
			return 1;
        }
		return 0;
    }

	// -- add 19/01/2020
	uint8_t get_ipAuto(string name_card){ // name_card : str()
		try{
			// address = re.search(re.compile(r'(?<=inet )(.*)(?=\/)', re.M), os.popen("ip addr show {}".format(name_card) ).read()).groups()[0];
			// print ("address: ", address)
			return 1;
        }
		catch(...){
			return 0;
        }
	}
			
	uint8_t check_server(){
		uint8_t is_ip = get_ipAuto("wlo2");

		// is_ip = 1
		// time_ping = pingServer(address)
		int8_t time_ping = 0;
		if (is_ip == 1){
			if (time_ping == -1){
				return 1; // khong Ping dc server
            }
			else{
				return 0; // oki
            }
        }
		else{
			return 2; // khong lay dc IP
        }
    }

	uint8_t detectLost_server(){
		double delta_t = ros::Time::now().toSec() - timeStampe_server;
		if (delta_t > 15){
			double delta_s = ros::Time::now().toSec() - saveTime_checkServer;
			if (delta_s > 5){
				saveTime_checkServer = ros::Time::now().toSec();
				saveStatus_server = check_server();
            }

			if (saveStatus_server == 1){
				return 2;
            }
			else if (saveStatus_server == 2){
				return 3;
            }
			return 1;
        }
		return 0;
    }

	uint8_t detectLost_reflectors(){
		// -- so luong guong
		if (nav350_data.number_reflectors >= 3){ // loi mat guong
			timeStampe_reflectors = ros::Time::now().toSec();
        }

		double delta_t = ros::Time::now().toSec() - timeStampe_reflectors;
		if (delta_t > 1.2){
			return 1;
        }
		return 0;
    }

	float calculate_distance(geometry_msgs::Point p1, geometry_msgs::Point p2){ // p1, p2 | geometry_msgs/Point
		float x = p2.x - p1.x;
		float y = p2.y - p1.y;
		return sqrt(x*x + y*y); 
    }

	float calculate_angle(geometry_msgs::Quaternion qua1, geometry_msgs::Quaternion qua2){ // p1, p2 |
		float euler1 = quaternion_to_euler(qua1);
		float euler2 = quaternion_to_euler(qua2);

		float delta_angle = euler2 - euler1;
		if (abs(delta_angle) >= M_PI){
			if (delta_angle >= 0){
				delta_angle = (M_PI*2 - abs(delta_angle))*(-1);
            }
			else{
				delta_angle = M_PI*2 - abs(delta_angle);
            }
        }
		return delta_angle;
    }

	// uint8_t find_element(int value_find, int list_in[]){
	// 	int lenght = sizeof(list_in) / sizeof(int);
	// 	for(int i = 0; i < lenght; i++){
	// 		if (value_find == list_in[i]){
	// 			return 1;
    //         }
    //     }
	// 	return 0;
    // }

	vector<int> synthetic_error(){
		vector<int> listError_now;

		// -- Goal Control
		if (detectLost_goalControl() == 1){
			listError_now.push_back(282);
        }

		// -- EMG
		if (main_info.EMC_status == 1){
			listError_now.push_back(121);
        }

		// -- Va cham
		if (HC_info.vacham == 1){
			listError_now.push_back(122);
        }

		// -- lost HC
		if (detectLost_hc() == 1){
			listError_now.push_back(351);
        }

		// -- lost CAN HC
		if (HC_info.status == -1){
			listError_now.push_back(352);
        }

		// -- OC: ket noi
		if (detectLost_oc() == 1){
			listError_now.push_back(341);
        }

		// -- OC-CAN
		if (lift_status.status.data == -2){
			listError_now.push_back(343);
        }

		// -- MAIN: CAN ket noi 
		if (main_info.CAN_status == 0){
			listError_now.push_back(323);
        }

		// -- lost Main
		if (detectLost_main() == 1){
			listError_now.push_back(321);
        }

		// -- lost Nav
		if (detectLost_nav() == 1){
			listError_now.push_back(221);
        }

		// -- lost pose robot
		if (detectLost_poseRobot() == 1){
			listError_now.push_back(222);
        }

		// -- Lost Driver 1
		if (detectLost_driver() == 1){
			listError_now.push_back(251);
        }

		// -- Error Driver 1
		int summation1 = driver1_respond.alarm_all + driver1_respond.alarm_overload + driver1_respond.warning;
		if (summation1 != 0){
			listError_now.push_back(252);
        }

		// -- Lost Driver 2
		if (detectLost_driver() == 1){
			listError_now.push_back(261);
        }

		// -- Error Driver 2
		int summation2 = driver2_respond.alarm_all + driver2_respond.alarm_overload + driver2_respond.warning;
		if (summation2 != 0){
			listError_now.push_back(262);
        }

		// -- Loi mat nguong
		if (detectLost_reflectors() == 1){ // loi mat guong
			listError_now.push_back(272);
        }

		// -- Ban Nang khong bat duoc cam bien
		if (lift_status.status.data == 238 || lift_status.status.data == -1){
			listError_now.push_back(141);
        }

		// -- Loi Code Parking:
		if (parking_status.warning == 2){
			listError_now.push_back(281);
        }

		// -- Nâng kệ nhưng không có kệ.
		if (flag_checkLiftError == 1){
			listError_now.push_back(471);
        }

		// -- 19/01/2022 - Mat giao tiep voi Server 
		uint8_t sts_sr = detectLost_server();
		if (sts_sr == 1){// lost server
			listError_now.push_back(431);
        }
		else if (sts_sr == 2){ // lost server: Ping
			listError_now.push_back(432);
        }
		else if (sts_sr == 3){ // lost server: IP- wifi
			listError_now.push_back(433);
        }

		// --- Low battery
		if (voltage < 23){
			listError_now.push_back(451);
        }

		// -- Co vat can khi di chuyen giua cac diem
		if (status_goalControl.safety == 1){
			listError_now.push_back(411);
        }

		// -- Co vat can khi di chuyen parking
		if (parking_status.warning == 1){
			listError_now.push_back(412);
        }

		// -- AGV dung do da di het danh sach diem.
		if (status_goalControl.misson == 1 || status_goalControl.misson == 3){
			if (status_goalControl.complete_misson == 2){
				listError_now.push_back(441);
				// -- add 18/01/2022
				flag_listPoint_ok = 1;
            }
        }

		// -- add 15/04/2022 - Danh sach ID lenh trong. Dung tranh AGV khac
		if (flag_listPointEmpty == 1){
			listError_now.push_back(442);
        }

		return listError_now;
    }

    void tryTarget_run(){
		if (app_button.bt_tryTarget_start == 1){
			if (step_tryTarget == 0){
				step_tryTarget = 1;
				// -
				move_req.target_x = app_button.tryTarget_x;
				move_req.target_y = app_button.tryTarget_y;
				// move_req.target_z = radians(app_button.tryTarget_r);
                move_req.target_z = app_button.tryTarget_r * M_PI/180;
				move_req.offset   = app_button.tryTarget_d;
				move_req.list_id  = {1000, 0, 0, 0, 0};
				move_req.list_x   = {app_button.tryTarget_x, 0, 0, 0, 0};
				move_req.list_y   = {app_button.tryTarget_y, 0, 0, 0, 0};
				move_req.list_speed = {0, 0, 0, 0, 0};
            }
        }

		if (app_button.bt_tryTarget_reset == 1){
			step_tryTarget = 0;
			enb_move = 0;
			enb_parking = 0;
        }

		if (app_button.bt_tryTarget_stop == 1){
			enb_move = 0;
			enb_parking = 0;
        }
		
		if (step_tryTarget == 0){
			enb_move = 0;
			enb_parking = 0;
        }

		else if (step_tryTarget == 1){ // - Di chuyển đến vị trí.
			move_req.mission = 0;
			if (app_button.bt_tryTarget_stop == 1){
				enb_move = 0; // -
            }
			else{
				enb_move = 3; // -
            }

			if (status_goalControl.complete_misson == 1){  // Hoan thanh di chuyen.
				step_tryTarget = 2;
				enb_move = 0;
            }
        }

		else if (step_tryTarget == 2){ // - Đi vào điểm thao tác
			if (app_button.bt_tryTarget_stop == 1){
				enb_parking = 0;
            }
			else{
				if (app_button.ck_tryTarget_safety == 1){
					enb_parking = 1;
                }
				else{
					enb_parking = 2;
                }
            }

			parking_offset = move_req.offset;
			// -
			parking_poseBefore.position.x = move_req.target_x;
			parking_poseBefore.position.y = move_req.target_y;
			parking_poseBefore.orientation = euler_to_quaternion(move_req.target_z);
			parking_poseTarget = getPose_from_offset(parking_poseBefore, move_req.offset);

			if (parking_status.status == 51){
				enb_parking = 0;
				step_tryTarget = 3;
            }

			if (move_req.offset <= 0){
				enb_parking = 0;
				step_tryTarget = 3;
            }
        }

		else if (step_tryTarget == 3){
			enb_parking = 0;
			enb_move = 0;
        }
    }

	void resetAll_variable()
	{
		ROS_INFO("RESET variable -----");
		enb_move = 0;
		enb_parking = 0;
		completed_before_mission = 0;
		completed_after_mission = 0;
		completed_move = 0;
		completed_moveSimple = 0;
		completed_moveSpecial = 0;
		completed_backward = 0;
		completed_MissionSetpose = 0;
		completed_checkLift = 0;
		flag_checkLiftError = 0;
		enb_mission = 0;
		flag_listPoint_ok = 0;

		target_x = NN_cmdRequest.target_x;
		target_y = NN_cmdRequest.target_y;
		target_z = NN_cmdRequest.target_z;
		target_tag = NN_cmdRequest.tag;
		before_mission = NN_cmdRequest.before_mission;
		after_mission = NN_cmdRequest.after_mission;
		// -- add 12/11/2021
		flag_resetFramework = 0;
		flag_Auto_to_Byhand = 0;
		// -- add 30/03/2022 : co bao loi qua tai dong co.
		// flagError_overLoad = 0
		// -- add 15/04/2022
		flag_listPointEmpty = 0;
    }
};

int main(int argc, char **argv)
{
  
    ros::init(argc, argv, "stiControl_cpp");   // specify node names
    ros::NodeHandle n;      //create a handle to this process's node
    ros::Rate loop_rate(30);   // 10Hz

    ros_control Main_ctrl;

    // -- cancel mission --
    ros::Subscriber sub_cancelMission = n.subscribe("/cancelMission_control", 1000, &ros_control::callback_cancelMission, &Main_ctrl);
    Main_ctrl.flag_cancelMission = 0;
    Main_ctrl.status_cancel = 0;

    Main_ctrl.pub_cancelMission = n.advertise<std_msgs::Int16>("/cancelMission_status", 100);

    // -- reconnect -- 
    ros::Subscriber sub_status_reconnect = n.subscribe("/status_reconnect", 1000, &ros_control::callback_reconnect, &Main_ctrl);

    // -- MAIN - POWER
    ros::Subscriber sub_main = n.subscribe("/POWER_info", 1000, &ros_control::callback_main, &Main_ctrl);
    Main_ctrl.pub_requestMain = n.advertise<sti_msgs::POWER_request>("/POWER_request", 100);
    Main_ctrl.timeStampe_main = ros::Time::now().toSec();
    Main_ctrl.voltage = 24.5;

    Main_ctrl.pub_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    // -- HC 8.2
    ros::Subscriber sub_hc = n.subscribe("/HC_info", 1000, &ros_control::callback_hc, &Main_ctrl);
    Main_ctrl.pub_HC = n.advertise<sti_msgs::HC_request>("/HC_request", 100);
    Main_ctrl.timeStampe_HC = ros::Time::now().toSec();    

    // -- OC 82
    ros::Subscriber sub_oc = n.subscribe("/lift_status", 1000, &ros_control::callback_oc, &Main_ctrl);
    Main_ctrl.timeStampe_OC = ros::Time::now().toSec();

    Main_ctrl.pub_OC = n.advertise<sti_msgs::Lift_control>("/lift_control", 100);

    // -- App
    ros::Subscriber sub_app = n.subscribe("/app_button", 1000, &ros_control::callback_appButton, &Main_ctrl);

    // -- data safety NAV
    ros::Subscriber sub_safetyNAV = n.subscribe("/safety_NAV", 1000, &ros_control::safety_NAV_callback, &Main_ctrl);

    // -- data nav
    ros::Subscriber sub_nav350 = n.subscribe("/nav350_data", 1000, &ros_control::nav350_callback, &Main_ctrl);

    // -------------- Cac node thuat toan dieu khien.
    Main_ctrl.pub_moveReq = n.advertise<sti_msgs::Move_request>("/request_move", 100);

    // -- Communicate with Server
    ros::Subscriber sub_NNcmdRequest = n.subscribe("/NN_cmdRequest", 1000, &ros_control::callback_cmdRequest, &Main_ctrl);
    Main_ctrl.pub_infoRespond = n.advertise<sti_msgs::NN_infoRespond>("/NN_infoRespond", 100);

    // -- imu 
    ros::Subscriber sub_imudata = n.subscribe("/imu/data", 1000, &ros_control::callback_imu, &Main_ctrl);

    // -- Safety Zone
    ros::Subscriber sub_safetyZone = n.subscribe("safety_zone", 1000, &ros_control::callback_safetyZone, &Main_ctrl);
    Main_ctrl.is_readZone = 0;

    // -- Odometry - POSE
    ros::Subscriber sub_odom = n.subscribe("robotPose_nav", 1000, &ros_control::callback_getPose, &Main_ctrl);

    // -- Port physical
    ros::Subscriber sub_checkPort = n.subscribe("status_port", 1000, &ros_control::callback_port, &Main_ctrl);

    // -- parking
    ros::Subscriber sub_parking = n.subscribe("parking_respond", 1000, &ros_control::callback_parking, &Main_ctrl);
    Main_ctrl.is_parking_status = 0;
    Main_ctrl.flag_requirResetparking = 0; 

    Main_ctrl.pub_parking = n.advertise<message_pkg::Parking_request>("/parking_request", 100);    
    Main_ctrl.enb_parking = 0;
	Main_ctrl.parking_offset = 0.0;
	// -- backward
	Main_ctrl.flag_requirBackward = 0;
	Main_ctrl.completed_backward = 0;
	Main_ctrl.backward_x = 0.0;
	Main_ctrl.backward_y = 0.0;
	Main_ctrl.backward_z = 0.0;

    ros::Subscriber sub_NNinforequest = n.subscribe("NN_infoRequest", 1000, &ros_control::callback_infoRequest, &Main_ctrl);
	Main_ctrl.timeStampe_server = ros::Time::now().toSec();

    ros::Subscriber sub_goalcontrol = n.subscribe("status_goal_control", 1000, &ros_control::callback_goalControl, &Main_ctrl);
    Main_ctrl.timeStampe_statusGoalControl = ros::Time::now().toSec();

    ros::Subscriber sub_driver1respond = n.subscribe("driver1_respond", 1000, &ros_control::callback_driver1, &Main_ctrl);
    ros::Subscriber sub_driver2respond = n.subscribe("driver2_respond", 1000, &ros_control::callback_driver2, &Main_ctrl);
	Main_ctrl.timeStampe_driver = ros::Time::now().toSec();

    Main_ctrl.pub_taskDriver = n.advertise<std_msgs::Int16>("/task_driver", 100);
	// -- task driver
	Main_ctrl.taskDriver_nothing = 0;
	Main_ctrl.taskDriver_resetRead = 1;
	Main_ctrl.taskDriver_Read = 2;
	Main_ctrl.task_driver.data = Main_ctrl.taskDriver_Read;

    // -
    Main_ctrl.pub_disableBrake = n.advertise<std_msgs::Bool>("/disable_brake", 100);    
    // -- HZ	
    Main_ctrl.FrequencePubBoard = 10.;
    Main_ctrl.pre_timeBoard = 0;
    // -- Mode operate
    Main_ctrl.mode_by_hand = 1;
    Main_ctrl.mode_auto = 2;
    Main_ctrl.mode_operate = Main_ctrl.mode_by_hand;    // Lưu chế độ hoạt động.

    // -- Target
    Main_ctrl.target_x = 0.;	 // lưu tọa độ điểm đích hiện tại.			
    Main_ctrl.target_y = 0.;
    Main_ctrl.target_z = 0.;
    Main_ctrl.target_tag = 0.;

    Main_ctrl.process = -1;       // tiến trình đang xử lý
    Main_ctrl.before_mission = 0;  // nhiệm vụ trước khi di chuyển.
    Main_ctrl.after_mission = 0;   // nhiệm vụ sau khi di chuyển.

    Main_ctrl.completed_before_mission = 0;	 // Báo nhiệm vụ trước đã hoàn thành.
    Main_ctrl.completed_after_mission = 0;	 // Báo nhiệm vụ sau đã hoàn thành.
    Main_ctrl.completed_move = 0;			 	 // Báo di chuyển đã hoàn thành.
    Main_ctrl.completed_moveSimple = 0;     // bao da den dich.
    Main_ctrl.completed_moveSpecial = 0;    // bao da den aruco.
    Main_ctrl.completed_reset = 0;             // hoan thanh reset.
    Main_ctrl.completed_MissionSetpose = 0; 	//
    Main_ctrl.completed_checkLift = 0; 	// kiem tra ke co hay ko sau khi nang. 
    // -- Flag
    Main_ctrl.flag_afterChager = 0;
    Main_ctrl.flag_checkLiftError = 0; // cờ báo ko có kệ khi nâng. 
    Main_ctrl.flag_Auto_to_Byhand = 0;
    Main_ctrl.enb_move = 0;                    // cho phep navi di chuyen
    Main_ctrl.flag_read_client = 0;
    Main_ctrl.flag_error = 0;
    Main_ctrl.flag_warning = 0;
    Main_ctrl.pre_mess = "";               // lưu tin nhắn hiện tại.

    // -- Check:
    Main_ctrl.is_get_pose = 0;
    Main_ctrl.is_mission = 0;
    Main_ctrl.is_read_prepheral = 0;      // ps2 - sti_read
    Main_ctrl.is_request_client = 0;
    Main_ctrl.is_set_pose = 0;
    Main_ctrl.is_zone_lidar = 0;

    Main_ctrl.completed_wakeup = 0;        // khi bật nguồn báo 1 sau khi setpose xong.
    // -- Status to server:
    Main_ctrl.statusU300L = 0;
    Main_ctrl.statusU300L_ok = 0;
    Main_ctrl.statusU300L_warning = 1;
    Main_ctrl.statusU300L_error = 2;
    Main_ctrl.statusU300L_cancelMission = 5;	
    // -- Status to detail to follow:
    Main_ctrl.stf = 0;
    Main_ctrl.stf_wakeup = 0;
    Main_ctrl.stf_running_simple = 1;
    Main_ctrl.stf_stop_obstacle = 2;
    Main_ctrl.stf_running_speial = 3;
    Main_ctrl.stf_running_backward = 4;
    Main_ctrl.stf_performUp = 5;
    Main_ctrl.stf_performDown = 6;
    // -- EMC reset
    Main_ctrl.EMC_resetOn = 1;
    Main_ctrl.EMC_resetOff = 0;
    Main_ctrl.EMC_reset = Main_ctrl.EMC_resetOff;
    // -- EMC write
    Main_ctrl.EMC_writeOn = 1;
    Main_ctrl.EMC_writeOff = 0;
    Main_ctrl.EMC_write = Main_ctrl.EMC_writeOff;
    // -- Led
    Main_ctrl.led = 0;
    Main_ctrl.led_error = 1; 			// 1
    Main_ctrl.led_simpleRun = 2; 		// 2
    Main_ctrl.led_specialRun = 3; 	// 3
    Main_ctrl.led_perform = 4; 		// 4
    Main_ctrl.led_completed = 5;  	// 5	
    Main_ctrl.led_stopBarrier = 6;  	// 6
    // -- Mission server
    Main_ctrl.statusTask_liftError = 64; // trang thái nâng kệ nhueng ko có kệ.
    Main_ctrl.serverMission_liftUp = 65; // 1 65
    Main_ctrl.serverMission_liftDown = 66; // 2 66
    Main_ctrl.serverMission_charger = 6; // 5
    Main_ctrl.serverMission_unknown = 0;
    Main_ctrl.serverMission_liftDown_charger = 5; // 6
    // -- Lift task.
    Main_ctrl.liftTask = 0; 
    Main_ctrl.liftUp = 2;
    Main_ctrl.liftDown = 1;
    Main_ctrl.liftStop = 0;
    Main_ctrl.liftResetOn = 1;
    Main_ctrl.liftResetOff = 0;
    Main_ctrl.liftReset = Main_ctrl.liftStop;
    Main_ctrl.flag_commandLift = 0;
    Main_ctrl.liftTask_byHand = Main_ctrl.liftStop;
    // -- Speaker
    Main_ctrl.speaker = 0;
    Main_ctrl.speaker_requir = 0; // luu trang thai cua loa
    Main_ctrl.spk_error = 3;
    Main_ctrl.spk_move = 1;
    Main_ctrl.spk_warn = 2;
    Main_ctrl.spk_not = 4;		
    Main_ctrl.spk_off = 0;
    Main_ctrl.enb_spk = 1;
    // -- Charger
    Main_ctrl.charger_on = 1;
    Main_ctrl.charger_off = 0;		
    Main_ctrl.charger_requir = Main_ctrl.charger_off;
    Main_ctrl.charger_write = Main_ctrl.charger_requir;
    Main_ctrl.charger_valueOrigin = 0.2;
    // -- Voltage
    Main_ctrl.timeCheckVoltage_charger = 1800; // s => 30 minutes.
    Main_ctrl.timeCheckVoltage_normal = 60;     // s
    Main_ctrl.pre_timeVoltage = 0;   // s
    Main_ctrl.valueVoltage = 0;
    Main_ctrl.step_readVoltage = 0;
    // -- ps2 
    Main_ctrl.time_ht = ros::Time::now().toSec();
    Main_ctrl.time_tr = ros::Time::now().toSec();
    Main_ctrl.rate_cmdvel = 10.; 
    // -- Error Type
    Main_ctrl.error_move = 0;
    Main_ctrl.error_perform = 0;
    Main_ctrl.error_device = 0;  // camera(1) - MC(2) - Main(3) - SC(4)

    Main_ctrl.numberError = 0;
    Main_ctrl.lastTime_checkLift = 0.0;
    // -- add new
    Main_ctrl.enb_debug = 0;
    // --
    Main_ctrl.job_doing = 0;
    // -- -- -- Su dung cho truong hop khi AGV chuyen Che do bang tay, bi keo ra khoi vi tri => AGV se chay lai.
    // -- Pose tai vi tri Ke, sac
    Main_ctrl.distance_resetMission = 0.1;
    Main_ctrl.flag_resetFramework = 0;
    // --
    Main_ctrl.flag_stopMove_byHand = 0;
    // --
    Main_ctrl.timeStampe_reflectors = ros::Time::now().toSec();
    // -- add 27/12/2021
    Main_ctrl.cancelbackward_offset = 0.0;

    // -- add 18/01/2022 : sua loi di lai cac diem cu khi mat ket noi server.
    for(int i = 0; i<5; i++){
        Main_ctrl.list_id_unknown.push_back(0);
    }
    Main_ctrl.flag_listPoint_ok = 0;
    
    // -- add 19/01/2022 : Check error lost server.
    Main_ctrl.name_card = "wlo2";
    Main_ctrl.address = "192.168.1.40"; // "172.21.15.224"
    Main_ctrl.saveTime_checkServer = ros::Time::now().toSec();
    Main_ctrl.saveStatus_server = 0;
    // -- add 30/03/2022 : co bao loi qua tai dong co.
    Main_ctrl.flagError_overLoad = 0;
    // -- add 15/04/2022
    Main_ctrl.flag_listPointEmpty = 0;

    // --
    Main_ctrl.step_tryTarget = 0;

    while (ros::ok())
    {
        if (Main_ctrl.process == -1){ //khi moi khoi dong len
            // mode_operate = mode_hand
            usleep(200);
            Main_ctrl.mode_operate = Main_ctrl.mode_by_hand;
            Main_ctrl.led = 0;
            Main_ctrl.speaker_requir = Main_ctrl.spk_warn;
            Main_ctrl.process = 0;
            Main_ctrl.enb_parking = 0;
        }
        else if (Main_ctrl.process == 0){	// chờ cac node khoi dong xong.
            int ct = 8;
            if (ct == 8){
                Main_ctrl.process = 1;
            }
        }

        else if (Main_ctrl.process == 1){ // reset toan bo: Main - MC - OC.
            Main_ctrl.flag_error = 0;
            Main_ctrl.completed_before_mission = 0;
            Main_ctrl.completed_after_mission = 0;
            Main_ctrl.completed_move = 0;
            Main_ctrl.completed_checkLift = 0;
            Main_ctrl.liftTask = 0;
            Main_ctrl.error_device = 0;
            Main_ctrl.error_move = 0;
            Main_ctrl.error_perform = 0;
            Main_ctrl.process = 2;
        }

		else if (Main_ctrl.process == 2){  // kiem tra toan bo thiet bi ok.
			Main_ctrl.listError =  Main_ctrl.synthetic_error();
			Main_ctrl.numberError = Main_ctrl.listError.size();
			int lenght = Main_ctrl.listError.size();
			
			uint8_t count_error = 0;
			uint8_t count_warning = 0;
			for(int i = 0; i < lenght; i++){
				if (Main_ctrl.listError[i] < 400){
					count_error += 1;
                }
				else{
					count_warning += 1;
                }
            }
					
			// -- add 30/03/2022 : co bao loi qua tai dong co.
			if (count_error == 0 && count_warning == 0){ // and Main_ctrl.flagError_overLoad == 0:
				Main_ctrl.flag_error = 0;
				Main_ctrl.flag_warning = 0;
            }

			else if (count_error == 0 && count_warning > 0){
				Main_ctrl.flag_error = 0;
				Main_ctrl.flag_warning = 1;
            }

			else{
				Main_ctrl.flag_error = 1;
				Main_ctrl.flag_warning = 0;
            }

			if (Main_ctrl.flag_error == 1){
				Main_ctrl.statusU300L = Main_ctrl.statusU300L_error;
            }
			else{
				if (Main_ctrl.flag_warning == 1){
					Main_ctrl.statusU300L = Main_ctrl.statusU300L_warning;
                }
				else{
					Main_ctrl.statusU300L = Main_ctrl.statusU300L_ok;
                }
            }

			// -- ERROR
			if (Main_ctrl.app_button.bt_clearError == 1 || Main_ctrl.main_info.stsButton_reset == 1){
				if (Main_ctrl.flag_requirResetparking == 1){
					Main_ctrl.enb_parking = 0;
					Main_ctrl.flag_requirResetparking = 0;
                }

				// -- dung parking neu co loi. - thay doi
				// Main_ctrl.enb_parking = 2

				Main_ctrl.EMC_write = Main_ctrl.EMC_writeOff;
				Main_ctrl.EMC_reset = Main_ctrl.EMC_resetOn;
				// -- sent clear error
				Main_ctrl.flag_error = 0;
				Main_ctrl.error_device = 0;
				Main_ctrl.error_move = 0;
				Main_ctrl.error_perform = 0;

				Main_ctrl.flag_checkLiftError = 0;
				Main_ctrl.task_driver.data = Main_ctrl.taskDriver_resetRead;

				// -- add 30/03/2022 : co bao loi qua tai dong co.
				Main_ctrl.flagError_overLoad = 0;

				// -- xoa loi ban nang.
				if (Main_ctrl.lift_status.status.data == -1){
					Main_ctrl.liftReset = Main_ctrl.liftResetOn;
                }
				else{
					Main_ctrl.liftReset = Main_ctrl.liftResetOff;
                }
            }
			else{
				Main_ctrl.task_driver.data = Main_ctrl.taskDriver_Read;
				Main_ctrl.EMC_reset = Main_ctrl.EMC_resetOff;

				// -- Add new: 23/12: Khi mat ket Driver, EMG duoc keo len.
				// if (Main_ctrl.flag_error == 1):
				// 	if (Main_ctrl.find_element(251, Main_ctrl.listError) == 1 or Main_ctrl.find_element(261, Main_ctrl.listError) == 1):
				// 		Main_ctrl.EMC_write = Main_ctrl.EMC_writeOn
				// 	else:
				// 		Main_ctrl.EMC_write = Main_ctrl.EMC_writeOff
            }

			// if (Main_ctrl.error_device != 0 or Main_ctrl.error_perform != 0 or Main_ctrl.error_move != 0):
			// 	Main_ctrl.flag_error = 1	

			Main_ctrl.process = 3;
        }
		else if (Main_ctrl.process == 3){ // read app
			if (Main_ctrl.app_button.bt_passHand == 1){
				if (Main_ctrl.mode_operate == Main_ctrl.mode_auto){ // keo co bao dang o tu dong -> chuyen sang bang tay.
					Main_ctrl.flag_Auto_to_Byhand = 1;
                }
				Main_ctrl.mode_operate = Main_ctrl.mode_by_hand;
            }

			if (Main_ctrl.app_button.bt_passAuto == 1){
				Main_ctrl.mode_operate = Main_ctrl.mode_auto;
            }

			if (Main_ctrl.mode_operate == Main_ctrl.mode_by_hand){
				Main_ctrl.process = 30;
            }
			else if (Main_ctrl.mode_operate == Main_ctrl.mode_auto){
				Main_ctrl.process = 40;
				Main_ctrl.disable_brake.data = 0;
            }
        }
	// ------------------------------------------------------------------------------------
	// -- BY HAND:
		else if (Main_ctrl.process == 30){
			Main_ctrl.enb_move = 0;
			Main_ctrl.job_doing = 20;

			if (Main_ctrl.parking_status.status != 0){
				Main_ctrl.enb_parking = 0;
            }

			if (Main_ctrl.step_tryTarget == 0){
				// ------------------------------------------------------------
				if (Main_ctrl.flag_error == 0){
				// -- Send vel
					if (Main_ctrl.lift_status.status.data == 0 || Main_ctrl.lift_status.status.data >= 3){  // Đang thực hiện nhiệm vụ ở chế độ auto -> ko cho phép di chuyển.
						// -- Move
						Main_ctrl.pub_cmdVel(Main_ctrl.pub_vel, Main_ctrl.run_maunal(), Main_ctrl.rate_cmdvel, ros::Time::now().toSec());
                    }
					else{
                        geometry_msgs::Twist twist;
						Main_ctrl.pub_cmdVel(Main_ctrl.pub_vel, twist, Main_ctrl.rate_cmdvel, ros::Time::now().toSec());
                    }
                }
				else{ // -- Has error
                    geometry_msgs::Twist twist;
					Main_ctrl.pub_cmdVel(Main_ctrl.pub_vel, twist, Main_ctrl.rate_cmdvel, ros::Time::now().toSec());
                }

				// ------------------------------------------------------------
				// -- Lift
				if (Main_ctrl.app_button.bt_lift_up == True){
					Main_ctrl.liftTask_byHand = Main_ctrl.liftUp;
					Main_ctrl.flag_commandLift = 1;
                }

				else if (Main_ctrl.app_button.bt_lift_down == True){
					Main_ctrl.liftTask_byHand = Main_ctrl.liftDown;
					Main_ctrl.flag_commandLift = 1;
                }

				if (Main_ctrl.app_button.bt_lift_up == False && Main_ctrl.app_button.bt_lift_down == False){
					Main_ctrl.liftTask_byHand = Main_ctrl.liftStop;
					Main_ctrl.flag_commandLift = 0;
					Main_ctrl.liftTask = Main_ctrl.liftTask_byHand;
                }

				if (Main_ctrl.flag_commandLift == 1){
					Main_ctrl.liftTask = Main_ctrl.liftTask_byHand;
					if (Main_ctrl.lift_status.status.data >= 3){ // Hoàn thành
						Main_ctrl.flag_commandLift = 0;
                    }
                }
				else{
					Main_ctrl.liftTask = Main_ctrl.liftStop;
                }

				// -- Speaker
				if (Main_ctrl.app_button.bt_spk_on == True){
					Main_ctrl.enb_spk = 1;
                }

				else if (Main_ctrl.app_button.bt_spk_off == True){
					Main_ctrl.enb_spk = 0;
                }

				// -- Charger
				if (Main_ctrl.app_button.bt_chg_on == True){
					Main_ctrl.charger_requir = Main_ctrl.charger_on;
                }
				else if (Main_ctrl.app_button.bt_chg_off == True){
					Main_ctrl.charger_requir = Main_ctrl.charger_off;
                }

				// -- Keep shaft.
				Main_ctrl.disable_brake.data = Main_ctrl.app_button.bt_disableBrake;

            }
			else{
				Main_ctrl.charger_requir = Main_ctrl.charger_off;
				Main_ctrl.liftTask = Main_ctrl.liftStop;
            }

			// --
			Main_ctrl.tryTarget_run();

			// --
			if (Main_ctrl.app_button.bt_resetFrameWork == 1){
				Main_ctrl.resetAll_variable();
            }
			// ------------------------------------------------------------
			

			Main_ctrl.process = 2;
        }

	// -- RUN AUTO:
		else if (Main_ctrl.process == 40){ // -- kiem tra loi
			if (Main_ctrl.flag_error == 1){ // thay doi != 0
				Main_ctrl.job_doing = 30;
				Main_ctrl.enb_move = 0;
				Main_ctrl.enb_mission = 0;
				Main_ctrl.process = 2;
            }

			else{
				Main_ctrl.process = 41;
            }
        }

		else if (Main_ctrl.process == 41){    // kiem tra muc tieu thay doi
			if (( Main_ctrl.target_x != Main_ctrl.NN_cmdRequest.target_x ) || ( Main_ctrl.target_y != Main_ctrl.NN_cmdRequest.target_y) || ( Main_ctrl.target_z != Main_ctrl.NN_cmdRequest.target_z) || ( Main_ctrl.target_tag != Main_ctrl.NN_cmdRequest.tag)){
				if ((Main_ctrl.NN_cmdRequest.target_x < 500) && (Main_ctrl.NN_cmdRequest.target_y < 500)){
                    /*
					Không cho phép đổi lệnh khi đang thao tác:
						1, Nâng/Hạ.
						2, Đang đi vào/ra điểm thao tác.
						3, Đang đi vào/ra khỏi sạc.
                    */
					int8_t a1 = 0;
					int8_t a2 = 0;
					// - Đang thao tác Nâng/Hạ. OK
					if ((Main_ctrl.lift_status.status.data == -1) || (Main_ctrl.lift_status.status.data == 1) || (Main_ctrl.lift_status.status.data == 2)){
						a1 = 1;
						ROS_INFO("tao chay o dk 1");
                    }

					// // - Đang đi vào điểm thao tác.
					if ((Main_ctrl.parking_status.status == 41) || (Main_ctrl.parking_status.status == 51)){
						a2 = 1;
						ROS_INFO("tao chay o dk 2");
                    }

					// // - Đang đi ra khỏi điểm thao tác.
					if ((Main_ctrl.status_goalControl.misson == 2) && (Main_ctrl.parking_status.status == 2)){
						a2 = 1;
						ROS_INFO("tao chay o dk 3");
                    }
					// --
					if ((a1 == 1) || (a2 == 1)){
						Main_ctrl.job_doing = 9;
						// Main_ctrl.log_mess("warn", "Have new target but must Waiting perform done ....", 0);
                        ROS_INFO("Have new target but must Waiting perform done ....");
						Main_ctrl.process = 42;
                    }
					else{
						ROS_INFO("Quy trinh bi reset o process 41....");
						Main_ctrl.resetAll_variable();
						Main_ctrl.process = 42;
                    }
                }
				else{
					// Main_ctrl.log_mess("info", "Have new target but Not fit: X= ", Main_ctrl.NN_cmdRequest.target_x)
					// Main_ctrl.log_mess("info", "Have new target but Not fit: y= ", Main_ctrl.NN_cmdRequest.target_y)
                    ROS_INFO("Have new target but Not fit: X= %f", Main_ctrl.NN_cmdRequest.target_x);
                    ROS_INFO("Have new target but Not fit: Y= %f", Main_ctrl.NN_cmdRequest.target_y);
					Main_ctrl.process = 2;
                }
				Main_ctrl.NN_infoRespond.offset = 0; // --
            }
			else{
				// - Nếu target ko đổi mà nhiện vụ muốn thay đổi (lấy hoặc trả hàng luôn tại đó).
				if (Main_ctrl.completed_after_mission == 1){
					if (Main_ctrl.after_mission != Main_ctrl.NN_cmdRequest.after_mission){
						Main_ctrl.completed_after_mission = 0;
						// Main_ctrl.log_mess("info", "After mission change to ", Main_ctrl.NN_cmdRequest.after_mission);
                        ROS_INFO("After mission change to %d", Main_ctrl.NN_cmdRequest.after_mission);
						Main_ctrl.after_mission = Main_ctrl.NN_cmdRequest.after_mission;
                    }
                }

				Main_ctrl.process = 42;
            }
        }

		else if (Main_ctrl.process == 42){ 
			if (Main_ctrl.flag_Auto_to_Byhand == 1){
				// -- add 12/11/2021 : Chay lai quy trinh Vao Sac khi Chuuyen che do.
				if (Main_ctrl.completed_after_mission == 1){
					if (Main_ctrl.after_mission == Main_ctrl.serverMission_liftDown_charger || Main_ctrl.after_mission == Main_ctrl.serverMission_charger){ //
						float delta_distance = Main_ctrl.calculate_distance(Main_ctrl.robotPose_nav.pose.position, Main_ctrl.poseWait.position);
						if (delta_distance > Main_ctrl.distance_resetMission){
							ROS_INFO("Quy trinh bi reset o process 42....");
							Main_ctrl.resetAll_variable();
							Main_ctrl.completed_backward = 1;
                        }
                    }
                }
				else{
					// -- add 23/12/2021: Xu ly loi Dang Parking thi bi chuyen che do -> AGV van parking.
					if (Main_ctrl.completed_before_mission == 1 && Main_ctrl.completed_moveSimple == 1 && Main_ctrl.completed_moveSpecial == 0){
						float delta_distance = Main_ctrl.calculate_distance(Main_ctrl.robotPose_nav.pose.position, Main_ctrl.pose_parkingRuning.position);
						// -- phien ban 2: Xac dinh do lech giua goc cua AGV voi goc cua diem vao Tag
						float delta_angle = Main_ctrl.calculate_angle(Main_ctrl.robotPose_nav.pose.orientation, Main_ctrl.parking_poseTarget.orientation);

						if (delta_distance > 0.2 or abs(delta_angle) > Main_ctrl.radians(20)){
							Main_ctrl.completed_moveSimple = 0;
                        }
                    }
                }
				// -- add 27/12/2021: Sua loi cu di thang ra sau khi parking
				if (Main_ctrl.completed_before_mission == 1 && Main_ctrl.completed_checkLift == 1 && Main_ctrl.completed_backward == 0){
					float delta_distance = Main_ctrl.calculate_distance(Main_ctrl.robotPose_nav.pose.position, Main_ctrl.cancelbackward_pose.position);
					float delta_angle = Main_ctrl.calculate_angle(Main_ctrl.robotPose_nav.pose.orientation, Main_ctrl.cancelbackward_pose.orientation);

					if (delta_distance > Main_ctrl.cancelbackward_offset*0.85 || abs(delta_angle) > Main_ctrl.radians(20)){
						Main_ctrl.completed_backward = 1;
                    }
                }

				// -- add 18/01/2022
				if (Main_ctrl.flag_listPoint_ok == 1){
					Main_ctrl.NN_cmdRequest.list_id = Main_ctrl.list_id_unknown;
					Main_ctrl.flag_listPoint_ok = 0;
                }

				Main_ctrl.job_doing = 1;
				// thuc hien lai nhiem vu nang, ha, sac sau khi chuyen che do tu tu dong sang bang tay.
				if (Main_ctrl.completed_after_mission == 0 && Main_ctrl.completed_before_mission == 0){
					Main_ctrl.flag_Auto_to_Byhand = 0;
                }

				else if (Main_ctrl.completed_after_mission == 0 && Main_ctrl.completed_before_mission == 1){
					if (Main_ctrl.before_mission == Main_ctrl.serverMission_unknown){
						Main_ctrl.flag_Auto_to_Byhand = 0;
                    }
							
					else if (Main_ctrl.before_mission == Main_ctrl.serverMission_liftDown || Main_ctrl.before_mission == 2){ // Hạ
						Main_ctrl.liftTask = Main_ctrl.liftDown;

						if (Main_ctrl.lift_status.status.data == 3){  // Hoàn thành
							Main_ctrl.liftTask = Main_ctrl.liftStop;
							Main_ctrl.flag_Auto_to_Byhand = 0;
                        }
                    }

					else if (Main_ctrl.before_mission == Main_ctrl.serverMission_liftUp || Main_ctrl.before_mission == 1){ // Nâng
						Main_ctrl.liftTask = Main_ctrl.liftUp;				
						if (Main_ctrl.lift_status.status.data == 4){  // Hoàn thành
							Main_ctrl.liftTask = Main_ctrl.liftStop;
							Main_ctrl.flag_Auto_to_Byhand = 0;
							Main_ctrl.completed_checkLift = 0;
                        }
                    }
                }

				else if (Main_ctrl.completed_after_mission == 1 && Main_ctrl.completed_before_mission == 1){
					if (Main_ctrl.after_mission == Main_ctrl.serverMission_unknown){
						Main_ctrl.flag_Auto_to_Byhand = 0;
                    }
							
					else if (Main_ctrl.after_mission == Main_ctrl.serverMission_liftDown || Main_ctrl.before_mission == 2){ // Hạ
						Main_ctrl.liftTask = Main_ctrl.liftDown;				
						if (Main_ctrl.lift_status.status.data == 3){  // Hoàn thành
							Main_ctrl.liftTask = Main_ctrl.liftStop;
							Main_ctrl.flag_Auto_to_Byhand = 0;
                        }
                    }

					else if (Main_ctrl.after_mission == Main_ctrl.serverMission_liftUp || Main_ctrl.before_mission == 1){ // Nâng
						Main_ctrl.liftTask = Main_ctrl.liftUp;				
						if (Main_ctrl.lift_status.status.data == 4){  // Hoàn thành
							Main_ctrl.liftTask = Main_ctrl.liftStop;
							Main_ctrl.flag_Auto_to_Byhand = 0;
                        }
                    }

					else if (Main_ctrl.after_mission == Main_ctrl.serverMission_liftDown_charger || Main_ctrl.after_mission == Main_ctrl.serverMission_charger){ // Hạ - Sac
						Main_ctrl.liftTask = Main_ctrl.liftDown;				
						if (Main_ctrl.lift_status.status.data == 3){  // Hoàn thành
							Main_ctrl.charger_requir = Main_ctrl.charger_on;
							Main_ctrl.liftTask = Main_ctrl.liftStop;
							Main_ctrl.flag_Auto_to_Byhand = 0;
                        }

					// else if Main_ctrl.after_mission == Main_ctrl.serverMission_charger: // Sac
					// 	Main_ctrl.charger_requir = Main_ctrl.charger_on
					// 	Main_ctrl.flag_Auto_to_Byhand = 0
                    }

					else{
						Main_ctrl.flag_Auto_to_Byhand = 0;
                    }
                }

				// -- add 18/11
				else{
					Main_ctrl.flag_Auto_to_Byhand = 0;
                }

				Main_ctrl.process = 2;
            }
			else{
				Main_ctrl.process = 43;
            }
        }

		else if (Main_ctrl.process == 43){ 	// Thực hiện nhiệm vụ trước.
			if (Main_ctrl.completed_before_mission == 0){ // chua thuc hien
				
				Main_ctrl.job_doing = 2; 
				if (Main_ctrl.before_mission == 0){
					Main_ctrl.charger_requir = Main_ctrl.charger_off;
					// Main_ctrl.log_mess("info", "Before mission Not have", Main_ctrl.before_mission);
                    ROS_INFO("Before mission Not have %d", Main_ctrl.before_mission);
					Main_ctrl.completed_before_mission = 1;
					// -- add new
					Main_ctrl.completed_checkLift = 1;
                }
					
				else if (Main_ctrl.before_mission == Main_ctrl.serverMission_liftDown || Main_ctrl.before_mission == 2){ // Hạ
					Main_ctrl.charger_requir = Main_ctrl.charger_off;
					Main_ctrl.liftTask = Main_ctrl.liftDown;
					
					if (Main_ctrl.lift_status.status.data == 3){  // Hoàn thành
						// Main_ctrl.log_mess("info", "Before mission completed", Main_ctrl.serverMission_liftDown);
                        ROS_INFO("Before mission completed process 43xx: %d", Main_ctrl.serverMission_liftDown);
						Main_ctrl.liftTask = Main_ctrl.liftStop;
						Main_ctrl.completed_before_mission = 1;
                    }
                }


				else if (Main_ctrl.before_mission == Main_ctrl.serverMission_liftUp || Main_ctrl.before_mission == 1){ // Nâng
					Main_ctrl.charger_requir = Main_ctrl.charger_off;
					Main_ctrl.liftTask = Main_ctrl.liftUp;		
					if (Main_ctrl.lift_status.status.data == 4){  // Hoàn thành
						// Main_ctrl.log_mess("info", "Before mission completed", Main_ctrl.serverMission_liftUp);
                        ROS_INFO("Before mission completed process 43: %d", Main_ctrl.serverMission_liftUp);
						Main_ctrl.liftTask = Main_ctrl.liftStop;
						Main_ctrl.completed_before_mission = 1;
						Main_ctrl.completed_checkLift = 0;
                    }
                }
					
				Main_ctrl.process = 2;
            }
			else{
				Main_ctrl.process = 44;
            }
        }

		else if (Main_ctrl.process == 44){	// Thuc hien kiểm tra kệ có trên bàn nâng ko.
			if (Main_ctrl.completed_checkLift == 0){
				Main_ctrl.job_doing = 3;
				if (Main_ctrl.before_mission == Main_ctrl.serverMission_liftUp || Main_ctrl.before_mission == 1){ // Nâng
					if (Main_ctrl.lift_status.sensorLift.data == 0){
						Main_ctrl.lastTime_checkLift = time(NULL);
                    }

					int t = (int) (time(NULL) - Main_ctrl.lastTime_checkLift)%60;
					if (t > 2){ // 2 s
						Main_ctrl.flag_checkLiftError = 0;
						Main_ctrl.completed_checkLift = 1;
                    }
					else{
						Main_ctrl.flag_checkLiftError = 1; // báo không có kệ.
                    }
                }
				else{
					Main_ctrl.completed_checkLift = 1;
                }
				Main_ctrl.process = 2;
            }
			else{
				Main_ctrl.process = 45;
            }
        }

		else if (Main_ctrl.process == 45){	// Thuc hien di chuyen lui.
			if (Main_ctrl.completed_backward == 1){
				Main_ctrl.process = 46;
				// -- add 15/04/2022
				Main_ctrl.flag_listPointEmpty = 0;
            }
			else{
				Main_ctrl.job_doing = 4;
				// -- add 15/04/2022
				if (Main_ctrl.check_listPoints(Main_ctrl.NN_cmdRequest.list_id) == 1){
					if (Main_ctrl.flag_requirBackward == 1){
						Main_ctrl.move_req.target_x = Main_ctrl.backward_x;
						Main_ctrl.move_req.target_y = Main_ctrl.backward_y;
						Main_ctrl.move_req.target_z = Main_ctrl.backward_z;
						// -- edit: 26/02/2022
						if (Main_ctrl.status_goalControl.misson == 2 && Main_ctrl.status_goalControl.complete_misson == 1){  // Hoan thanh di chuyen lui.
						// if Main_ctrl.status_goalControl.misson == 1 and Main_ctrl.status_goalControl.complete_misson == 1:  // Hoan thanh di chuyen lui.
							Main_ctrl.flag_requirBackward = 0;
							Main_ctrl.enb_move = 0;
							Main_ctrl.completed_backward = 1;
                        }
						else{
							// -- edit: 26/02/2022
							Main_ctrl.enb_move = 2;
                        }
                    }
					else{
						Main_ctrl.completed_backward = 1;	
						Main_ctrl.enb_move = 0;
                    }
					// -- add 15/04/2022
					Main_ctrl.flag_listPointEmpty = 0;
					Main_ctrl.process = 2;
                }
				else{
					Main_ctrl.enb_move = 0;
					Main_ctrl.flag_listPointEmpty = 1;
					// Main_ctrl.log_mess("info", "process = 43 - Buoc lui ra, Danh sach diem Null -> ko chay", 0);
                    ROS_INFO("process = 43 - Buoc lui ra, Danh sach diem Null -> ko chay");
					Main_ctrl.process = 2;
                }
            }
        }


		else if (Main_ctrl.process == 46){	// Thuc hien di chuyen diem thuong.
			if (Main_ctrl.completed_moveSimple == 1){      // 
				Main_ctrl.process = 47;
				Main_ctrl.enb_move = 0;
            }
			else{
				Main_ctrl.job_doing = 5;
				if ((Main_ctrl.NN_cmdRequest.list_x.size() != 0) && (Main_ctrl.NN_cmdRequest.list_y.size() != 0) && (Main_ctrl.target_x < 500) && (Main_ctrl.target_y < 500)){
					Main_ctrl.move_req.target_x = Main_ctrl.target_x;
					Main_ctrl.move_req.target_y = Main_ctrl.target_y;
					Main_ctrl.move_req.target_z = Main_ctrl.target_z;
					
					Main_ctrl.parking_offset = Main_ctrl.NN_cmdRequest.offset;

					Main_ctrl.move_req.tag = Main_ctrl.NN_cmdRequest.tag;
					Main_ctrl.move_req.offset = Main_ctrl.NN_cmdRequest.offset;

					Main_ctrl.move_req.list_id = Main_ctrl.NN_cmdRequest.list_id;
					Main_ctrl.move_req.list_x = Main_ctrl.NN_cmdRequest.list_x;
					Main_ctrl.move_req.list_y = Main_ctrl.NN_cmdRequest.list_y;
					Main_ctrl.move_req.list_speed = Main_ctrl.NN_cmdRequest.list_speed;

					if (Main_ctrl.before_mission == Main_ctrl.serverMission_liftUp || Main_ctrl.before_mission == 1){ // Nâng
						Main_ctrl.move_req.mission = 1;
                    }
					else{
						Main_ctrl.move_req.mission = 0;
                    }

					// -- add 19/01/2022 : chuyen vung sick.
					if (Main_ctrl.before_mission == Main_ctrl.serverMission_liftUp || Main_ctrl.before_mission == 1){ // Nâng
						Main_ctrl.enb_move = 1; // -- vung To
                    }
					else{
						Main_ctrl.enb_move = 3; // -- vung Nho
                    }
                }
				else{
					// Main_ctrl.log_mess("warn", "ERROR: Target of List point wrong !!!", 0);
                    ROS_INFO("ERROR: Target of List point wrong !!!");
                }

				// -- add 19/01/2022 : chuyen vung sick.
				if (Main_ctrl.status_goalControl.misson == 1 || Main_ctrl.status_goalControl.misson == 3){
					if (Main_ctrl.status_goalControl.complete_misson == 1){  // Hoan thanh di chuyen.
						Main_ctrl.completed_moveSimple = 1;
						Main_ctrl.enb_move = 0;
						// Main_ctrl.log_mess("info", "Move completed", 0);
                        ROS_INFO("Move completed");
                    }
                }
				Main_ctrl.process = 2;
            }
        }

	 	// -- Parking	
		else if (Main_ctrl.process == 47){   //  
		  	// print "46 --"
			if (Main_ctrl.completed_moveSpecial == 1){
				Main_ctrl.completed_move = 1;
				Main_ctrl.process = 34;
            }
			else{
				Main_ctrl.process = 48; // kiem tra diem vao ke
            }
        }

		else if (Main_ctrl.process == 48){   // Parking
			if (Main_ctrl.completed_moveSpecial == 0){ // chua hoan thanh di chuyen
				Main_ctrl.job_doing = 6;
				// sau sẽ thêm phần khi đổi mã tag thì tự động reset paking.
				if (Main_ctrl.parking_status.status == 1){ // Free
					Main_ctrl.process = 50;
					// Main_ctrl.log_mess("info", "Special point: readly", Main_ctrl.parking_status.status)
                }
				else if (Main_ctrl.parking_status.status == 51){ // -- Completed Run
					// Main_ctrl.log_mess("info", "Special point: Completed to point: ", Main_ctrl.parking_status.status)
					Main_ctrl.enb_parking = 0;
					Main_ctrl.completed_moveSpecial = 1;
					Main_ctrl.flag_requirBackward = 1;   // 1 - yeu cau 
					Main_ctrl.backward_x = Main_ctrl.target_x;
					Main_ctrl.backward_y = Main_ctrl.target_y;
					Main_ctrl.backward_z = Main_ctrl.target_z;

					Main_ctrl.process = 2;

					// -- add 12/11/2021
					Main_ctrl.poseWait = Main_ctrl.robotPose_nav.pose;

					// -- add 27/12/2021
					Main_ctrl.cancelbackward_pose = Main_ctrl.robotPose_nav.pose;
					Main_ctrl.cancelbackward_offset = Main_ctrl.parking_offset;
                }
				else{
					Main_ctrl.process = 2;
                }

				// -- add 23/12/2021:
				Main_ctrl.pose_parkingRuning = Main_ctrl.robotPose_nav.pose;
            }
			else{
				Main_ctrl.process = 2;
            }
        }

		else if (Main_ctrl.process == 50){ 	// Yêu cầu di chuyển.
			if (Main_ctrl.after_mission == Main_ctrl.serverMission_liftDown_charger || Main_ctrl.after_mission == Main_ctrl.serverMission_charger){
				Main_ctrl.enb_parking = 2;
            }
			else{                 // Nếu after_mission != 5 && != 6
				Main_ctrl.enb_parking = 1;
            }

			Main_ctrl.parking_offset = Main_ctrl.NN_cmdRequest.offset;
			// -
			Main_ctrl.parking_poseBefore.position.x = Main_ctrl.NN_cmdRequest.target_x;
			Main_ctrl.parking_poseBefore.position.y = Main_ctrl.NN_cmdRequest.target_y;
			Main_ctrl.parking_poseBefore.orientation = Main_ctrl.euler_to_quaternion(Main_ctrl.NN_cmdRequest.target_z);
			Main_ctrl.parking_poseTarget = Main_ctrl.getPose_from_offset(Main_ctrl.parking_poseBefore, Main_ctrl.parking_offset);

			// Main_ctrl.log_mess("info", "Tag offset requir: ", Main_ctrl.parking_offset);
            ROS_INFO("Tag offset requir: %f", Main_ctrl.parking_offset);
			Main_ctrl.process = 2;
        } 

	// ------------------------------------------------------------------------------------
		else if (Main_ctrl.process == 34){	// -- Thực hiện nhiệm vụ sau.
			if (Main_ctrl.completed_after_mission == 0){ // chua thuc hien
				Main_ctrl.job_doing = 7;
				if (Main_ctrl.after_mission == 0){
					cout << "Last mission completed 0 : " << Main_ctrl.after_mission << endl;
					Main_ctrl.completed_after_mission = 1;
                }
				else if (Main_ctrl.after_mission == Main_ctrl.serverMission_liftDown || Main_ctrl.before_mission == 2){ // Hạ
					Main_ctrl.liftTask = Main_ctrl.liftDown;
					if (Main_ctrl.lift_status.status.data == 3){ // Hoàn thành
						cout << "Last mission completed 2: " << Main_ctrl.serverMission_liftDown << endl;
						Main_ctrl.liftTask = Main_ctrl.liftStop;
						Main_ctrl.completed_after_mission = 1;
                    }
                }

				else if (Main_ctrl.after_mission == Main_ctrl.serverMission_liftUp || Main_ctrl.before_mission == 1){ // Nâng
					Main_ctrl.liftTask = Main_ctrl.liftUp;				
					if (Main_ctrl.lift_status.status.data == 4){ // Hoàn thành
						cout << "Last mission completed 1 : " << Main_ctrl.serverMission_liftUp << endl;
						Main_ctrl.liftTask = Main_ctrl.liftStop;
						Main_ctrl.completed_after_mission = 1;
                    }
                }
						
				else if (Main_ctrl.after_mission == Main_ctrl.serverMission_charger){ // sac
					Main_ctrl.charger_requir = Main_ctrl.charger_on;
					cout << "Last mission completed 6: " << Main_ctrl.serverMission_charger << endl;
					Main_ctrl.completed_after_mission = 1;
                }

				else if (Main_ctrl.after_mission == Main_ctrl.serverMission_liftDown_charger){ // sac
					Main_ctrl.liftTask = Main_ctrl.liftDown;
					if (Main_ctrl.lift_status.status.data == 3){ // Hoàn thành
						cout << "Last mission completed 5: " << Main_ctrl.serverMission_liftDown_charger << endl;
						Main_ctrl.liftTask = Main_ctrl.liftStop;
						Main_ctrl.completed_after_mission = 1;
						Main_ctrl.charger_requir = Main_ctrl.charger_on; // turn on charger				
                    }
                }
				cout << "AFter mission not done!" << endl;
				Main_ctrl.process = 2;
            }
			else{
				cout << "AFter mission done" << endl;
				Main_ctrl.process = 35;
            }
        }

		else if (Main_ctrl.process == 35){
			Main_ctrl.job_doing = 8;
			Main_ctrl.process = 2;
			// Main_ctrl.log_mess("warn", "Wating new Target ...", 0);
            ROS_INFO("Wating new Target ...");
        }

		// -- Tag + Offset:
		if (Main_ctrl.mode_operate == Main_ctrl.mode_auto){
			if (Main_ctrl.completed_move == 1){
				Main_ctrl.NN_infoRespond.tag = Main_ctrl.NN_cmdRequest.tag;
				Main_ctrl.NN_infoRespond.offset = Main_ctrl.NN_cmdRequest.offset;
            }
			else{
				Main_ctrl.NN_infoRespond.tag = 0;
				Main_ctrl.NN_infoRespond.offset = 0;
            }

			if (Main_ctrl.completed_before_mission == 1 && Main_ctrl.completed_after_mission == 0 && Main_ctrl.flag_checkLiftError == 0){
				Main_ctrl.NN_infoRespond.task_status = Main_ctrl.before_mission;
            }
			else if (Main_ctrl.completed_before_mission == 1 && Main_ctrl.completed_after_mission == 0 && Main_ctrl.flag_checkLiftError == 1){
				Main_ctrl.NN_infoRespond.task_status = Main_ctrl.statusTask_liftError;
            }
			if (Main_ctrl.completed_before_mission == 1 && Main_ctrl.completed_after_mission == 1){
				Main_ctrl.NN_infoRespond.task_status = Main_ctrl.after_mission;
            }
        }
		Main_ctrl.NN_infoRespond.status = Main_ctrl.statusU300L;    // Status: Error
		Main_ctrl.NN_infoRespond.error_perform = Main_ctrl.process;
		Main_ctrl.NN_infoRespond.error_moving = Main_ctrl.flag_error;
		Main_ctrl.NN_infoRespond.error_device = Main_ctrl.numberError;
		Main_ctrl.NN_infoRespond.listError = Main_ctrl.listError;
		Main_ctrl.NN_infoRespond.process = Main_ctrl.job_doing;  

		// -- Battery - ok
		Main_ctrl.readbatteryVoltage();
		Main_ctrl.NN_infoRespond.battery = int(Main_ctrl.valueVoltage);
		
		if (Main_ctrl.flag_cancelMission == 0){
			// -- mode respond server
			if (Main_ctrl.mode_operate == Main_ctrl.mode_by_hand){         // Che do by Hand
				Main_ctrl.NN_infoRespond.mode = 1;
            }

			else if (Main_ctrl.mode_operate == Main_ctrl.mode_auto){          // Che do Auto
				Main_ctrl.NN_infoRespond.mode = 2;
            }
        }
		else{
			Main_ctrl.NN_infoRespond.mode = 5;
        }

		// -- Respond Client
		Main_ctrl.pub_infoRespond.publish(Main_ctrl.NN_infoRespond);    // Pub Client

		// -- Request Navigation
		Main_ctrl.pub_move_req(Main_ctrl.pub_moveReq, Main_ctrl.enb_move, Main_ctrl.move_req);  // Pub Navigation

		// -- Speaker
		if (Main_ctrl.flag_error == 1 && Main_ctrl.flag_warning == 1){
			Main_ctrl.speaker_requir = Main_ctrl.spk_error;
        }
		else if (Main_ctrl.flag_error == 1 && Main_ctrl.flag_warning == 0){
			Main_ctrl.speaker_requir = Main_ctrl.spk_error;
        }
		else if (Main_ctrl.flag_error == 0 && Main_ctrl.flag_warning == 1){
			Main_ctrl.speaker_requir = Main_ctrl.spk_warn;
        }			
		else{
			if (Main_ctrl.completed_backward == 0 && Main_ctrl.completed_before_mission == 0 && Main_ctrl.completed_moveSimple == 0 && Main_ctrl.completed_moveSpecial == 0 && Main_ctrl.completed_after_mission == 0){
				Main_ctrl.speaker_requir = Main_ctrl.spk_move;
            }
			else if (Main_ctrl.completed_backward == 1 && Main_ctrl.completed_before_mission == 0 && Main_ctrl.completed_moveSimple == 0 && Main_ctrl.completed_moveSpecial == 0 && Main_ctrl.completed_after_mission == 0){
				Main_ctrl.speaker_requir = Main_ctrl.spk_move;
            }
			else if (Main_ctrl.completed_backward == 1 && Main_ctrl.completed_before_mission == 1 && Main_ctrl.completed_moveSimple == 0 && Main_ctrl.completed_moveSpecial == 0 && Main_ctrl.completed_after_mission == 0){
				Main_ctrl.speaker_requir = Main_ctrl.spk_move;
            }
			else if (Main_ctrl.completed_backward == 1 && Main_ctrl.completed_before_mission == 1  && Main_ctrl.completed_moveSimple == 1 && Main_ctrl.completed_moveSpecial == 0 && Main_ctrl.completed_after_mission == 0){	
				Main_ctrl.speaker_requir = Main_ctrl.spk_move;
            }
			else if (Main_ctrl.completed_backward == 1 && Main_ctrl.completed_before_mission == 1  && Main_ctrl.completed_moveSimple == 1 && Main_ctrl.completed_moveSpecial == 1 && Main_ctrl.completed_after_mission == 0){	
				Main_ctrl.speaker_requir = Main_ctrl.spk_move;
            }
			else if (Main_ctrl.completed_backward == 1 && Main_ctrl.completed_before_mission == 1  && Main_ctrl.completed_moveSimple == 1 && Main_ctrl.completed_moveSpecial == 1 && Main_ctrl.completed_after_mission == 1){
				Main_ctrl.speaker_requir = Main_ctrl.spk_move;
            }
			else{
				Main_ctrl.speaker_requir = Main_ctrl.spk_move;
            }
        }

		// -- LED
		if (Main_ctrl.flag_error == 1){
			Main_ctrl.led = Main_ctrl.led_error;
        }
		else{
			if (Main_ctrl.completed_before_mission == 0 && Main_ctrl.completed_backward == 0 && Main_ctrl.completed_moveSimple == 0 && Main_ctrl.completed_moveSpecial == 0 && Main_ctrl.completed_after_mission == 0){
				Main_ctrl.led = Main_ctrl.led_perform;
            }
			else if (Main_ctrl.completed_before_mission == 1 && Main_ctrl.completed_backward == 0 && Main_ctrl.completed_moveSimple == 0 && Main_ctrl.completed_moveSpecial == 0 && Main_ctrl.completed_after_mission == 0){
				if (Main_ctrl.status_goalControl.safety == 1){
					Main_ctrl.led = Main_ctrl.led_stopBarrier;
                }
				else{
					Main_ctrl.led = Main_ctrl.led_simpleRun;
                }
            }

			else if (Main_ctrl.completed_before_mission == 1 && Main_ctrl.completed_backward == 1 && Main_ctrl.completed_moveSimple == 0 && Main_ctrl.completed_moveSpecial == 0 && Main_ctrl.completed_after_mission == 0){
				if (Main_ctrl.status_goalControl.safety == 1){
					Main_ctrl.led = Main_ctrl.led_stopBarrier;
                }
				else{
					Main_ctrl.led = Main_ctrl.led_simpleRun;
                }
            }

			else if (Main_ctrl.completed_before_mission == 1 && Main_ctrl.completed_backward == 1  && Main_ctrl.completed_moveSimple == 1 && Main_ctrl.completed_moveSpecial == 0 && Main_ctrl.completed_after_mission == 0){
				if (Main_ctrl.parking_status.warning == 1){
					Main_ctrl.led = Main_ctrl.led_stopBarrier;
                }
				else{
					Main_ctrl.led = Main_ctrl.led_specialRun;
                }
            }
				
			else if (Main_ctrl.completed_before_mission == 1 && Main_ctrl.completed_backward == 1  && Main_ctrl.completed_moveSimple == 1 && Main_ctrl.completed_moveSpecial == 1 && Main_ctrl.completed_after_mission == 0){
				Main_ctrl.led = Main_ctrl.led_perform;
            }

			else if (Main_ctrl.completed_before_mission == 1 && Main_ctrl.completed_backward == 1  && Main_ctrl.completed_moveSimple == 1 && Main_ctrl.completed_moveSpecial == 1 && Main_ctrl.completed_after_mission == 1){
				Main_ctrl.led = Main_ctrl.led_completed;
            }

			else{
				Main_ctrl.led = Main_ctrl.led_simpleRun;
            }
        }

		// -- -- -- pub Board
		double time_curr = ros::Time::now().toSec();
		double d = (time_curr - Main_ctrl.pre_timeBoard);
		if (d > float(1/Main_ctrl.FrequencePubBoard)){ // < 20hz 
			Main_ctrl.pre_timeBoard = time_curr;
			// -- Request OC:
			Main_ctrl.lift_control.control.data = Main_ctrl.liftTask;
			Main_ctrl.lift_control.reset.data = Main_ctrl.liftReset;
			Main_ctrl.pub_OC.publish(Main_ctrl.lift_control);

			// -- Request Main: charger, sound, EMC_write, EMC_reset
			if (Main_ctrl.enb_spk == 1){
				// tat Loa khi sac thanh cong!
				if (Main_ctrl.charger_write == Main_ctrl.charger_on && Main_ctrl.main_info.charge_current >= Main_ctrl.charger_valueOrigin && Main_ctrl.flag_error == 0){
					Main_ctrl.speaker = Main_ctrl.spk_off;
                }
				else{
					Main_ctrl.speaker = Main_ctrl.speaker_requir;
                }
            }
			else{
				Main_ctrl.speaker = Main_ctrl.spk_off;
            }

			Main_ctrl.Main_pub(Main_ctrl.pub_requestMain, Main_ctrl.charger_write, Main_ctrl.speaker, Main_ctrl.EMC_write, Main_ctrl.EMC_reset);  // MISSION

			// -- Request HC:
			Main_ctrl.HC_request.RBG1 = Main_ctrl.led; 
			// if Main_ctrl.charger_write == Main_ctrl.charger_on && Main_ctrl.main_info.charge_current >= 0.5: 
			// 	Main_ctrl.HC_request.RBG1 = 0
			// else:
				// Main_ctrl.HC_request.RBG1 = Main_ctrl.led

			Main_ctrl.HC_request.RBG2 = 0;
			Main_ctrl.pub_HC.publish(Main_ctrl.HC_request);

			// -- Request task Driver:
			Main_ctrl.pub_taskDriver.publish(Main_ctrl.task_driver);

			// ---------------- Brake ---------------- //
			Main_ctrl.pub_disableBrake.publish(Main_ctrl.disable_brake);
        }

	  	// -- cancel mission
		if (Main_ctrl.cancelMission_control.data == 1){
			Main_ctrl.flag_cancelMission = 1;
			Main_ctrl.cancelMission_status.data = 1;
        }

		if (Main_ctrl.NN_cmdRequest.id_command == 0){
			Main_ctrl.flag_cancelMission = 0;
			Main_ctrl.cancelMission_status.data = 0;
        }

		Main_ctrl.pub_cancelMission.publish(Main_ctrl.cancelMission_status);
		
		Main_ctrl.pub_park(Main_ctrl.pub_parking, Main_ctrl.enb_parking, Main_ctrl.parking_poseBefore, Main_ctrl.parking_poseTarget, Main_ctrl.parking_offset);

        ros::spinOnce();     // allow receiving callbacks function
        loop_rate.sleep();   //ros pause in 10Hz
    }

    return 0;
}