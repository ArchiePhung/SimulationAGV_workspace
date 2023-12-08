#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <iostream>

#define PI 3.14159265358979323846264338327950288419
#define INF 10000 // Define Infinite (Using INT_MAX caused overflow problems)

using namespace std;
using namespace sensor_msgs;
using namespace geometry_msgs;

int check1=0;
int check2=0;
int check3=0;

ros::Publisher zone;
ros::Publisher buffer;

struct Point_new { 
    float x; 
    float y; 
}; 

void getField(const Point ){

}

Point_new polygon_zone1[]={{0 , 0},{-0.14 , 0},{-0.6 , 0.5} ,{0 , 1},{0.4 , 0.4}};

int corners1 = sizeof(polygon_zone1)/sizeof(polygon_zone1[0]);
Point_new polygon_zone2[]={{0 , 0},{0 , -0.14},{0.5 , -0.6},{1 , 0},{0.5 , 0.4}};
int corners2 = sizeof(polygon_zone2)/sizeof(polygon_zone2[0]);
Point_new polygon_zone3[]={{0 , 0}};
int corners3 = sizeof(polygon_zone3)/sizeof(polygon_zone3[0]);
  
// Given three colinear points p, q, r, the function checks if 
// point q lies on line segment 'pr' 
bool onSegment(Point_new p, Point_new q, Point_new r) { 
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) && 
            q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y)) 
        return true; 
    return false; 
} 
  
// To find orientation of ordered triplet (p, q, r). 
// The function returns following values 
// 0 --> p, q and r are colinear 
// 1 --> Clockwise 
// 2 --> Counterclockwise 
int orientation(Point_new p, Point_new q, Point_new r) { 
    float val = (q.y - p.y) * (r.x - q.x) - 
              (q.x - p.x) * (r.y - q.y); 
  
    if (val == 0) return 0;  // colinear 
    return (val > 0)? 1: 2; // clock or counterclock wise 
} 
  
// The function that returns true if line segment 'p1q1' 
// and 'p2q2' intersect. 
bool doIntersect(Point_new p1, Point_new q1, Point_new p2, Point_new q2) { 
    // Find the four orientations needed for general and 
    // special cases 
    int o1 = orientation(p1, q1, p2); 
    int o2 = orientation(p1, q1, q2); 
    int o3 = orientation(p2, q2, p1); 
    int o4 = orientation(p2, q2, q1); 
  
    // General case 
    if (o1 != o2 && o3 != o4) 
        return true; 
  
    // Special Cases 
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1 
    if (o1 == 0 && onSegment(p1, p2, q1)) return true; 
  
    // p1, q1 and p2 are colinear and q2 lies on segment p1q1 
    if (o2 == 0 && onSegment(p1, q2, q1)) return true; 
  
    // p2, q2 and p1 are colinear and p1 lies on segment p2q2 
    if (o3 == 0 && onSegment(p2, p1, q2)) return true; 
  
     // p2, q2 and q1 are colinear and q1 lies on segment p2q2 
    if (o4 == 0 && onSegment(p2, q1, q2)) return true; 
  
    return false; // Doesn't fall in any of the above cases 
} 
  
// Returns true if the point p lies inside the polygon[] with n vertices 
bool isInside(Point_new polygon[], int n, Point_new p) { 
    // There must be at least 3 vertices in polygon[] 
    if (n < 3)  return false; 
  
    // Create a point for line segment from p to infinite 
    Point_new extreme = {INF, p.y}; 
  
    // Count intersections of the above line with sides of polygon 
    int count = 0, i = 0; 
    do
    { 
        int next = (i+1)%n; 
  
        // Check if the line segment from 'p' to 'extreme' intersects 
        // with the line segment from 'polygon[i]' to 'polygon[next]' 
        if (doIntersect(polygon[i], polygon[next], p, extreme)) 
        { 
            // If the point 'p' is colinear with line segment 'i-next', 
            // then check if it lies on segment. If it lies, return true, 
            // otherwise false 
            if (orientation(polygon[i], p, polygon[next]) == 0) 
               return onSegment(polygon[i], p, polygon[next]); 
  
            count++; 
        } 
        i = next; 
    } while (i != 0); 
  
    // Return true if count is odd, false otherwise 
    return count&1;  // Same as (count%2 == 1) 
}   

void getString(const LaserScan &scan){
    size_t size = scan.ranges.size();    
    float data[size];
    int count_zone1=0;
    int count_zone2=0;
    int count_zone3=0;
    Point zone_msg;
    for (size_t i = 0; i < size; ++i){
        float dist = scan.ranges[i];

        float x = -dist * cos(2*i*PI/size);
        float y = -dist * sin(2*i*PI/size);
        Point_new p ={x,y};
        
        if (isInside(polygon_zone1,corners1,p)) {
            check1 = 1;
            count_zone1++;
        }
        if (isInside(polygon_zone2,corners2,p)) {
            check2 = 1;
            count_zone2++;
        }
        if (isInside(polygon_zone3,corners3,p)) {
            check3 = 1;
            count_zone3++;
        }
    }
    if (count_zone1 == 0) check1=0;
    if (count_zone2 == 0) check2=0;
    if (count_zone3 == 0) check3=0;
    zone_msg.x=check1;
    zone_msg.y=check2;
    zone_msg.z=check3;
    zone.publish(zone_msg);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "laser_zone");
    ros::NodeHandle nh("~");   
    int queue_size = 10;
    ros::Subscriber laser_in = nh.subscribe("/scan_a2", queue_size, getString);
    zone = nh.advertise<geometry_msgs::Point>("/zone_laser",queue_size);
    // buffer = nh.advertise<sensor_msgs::LaserScan>("/buffer",queue_size);
    ros::spin();
    return 0;
}